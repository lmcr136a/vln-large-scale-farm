#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class VerticalStructureFilter(Node):
    def __init__(self):
        super().__init__("vertical_structure_filter")
        self.declare_parameter("in_topic", "/rslidar_points")
        self.declare_parameter("out_topic", "/rslidar_points_vstruct")
        self.declare_parameter("cell_size", 0.2)         # XY cell for column detection (m)
        self.declare_parameter("z_min_extent", 0.05)      # min vertical span in a cell to call it structure (m)
        self.declare_parameter("min_points", 2)         # min points in a cell to call it structure
        self.declare_parameter("ground_keep_rate", 1.00)  # keep fraction of non-structure points
        self.declare_parameter("max_range", 200.0)
        self.declare_parameter("min_range", 0.7)         # drop self/near points (m)
        self.declare_parameter("ground_z_percentile", 0.1)
        self.declare_parameter("z_amplify", 5.0)
        self.declare_parameter("z_amplify_cap", 0.7)     # raw-z meters, pre-amplification
        self.declare_parameter("roughness_threshold", 0.003)  # m^2 variance of raw z_res

        gp = self.get_parameter
        self.cell_size        = float(gp("cell_size").value)
        self.z_min_extent     = float(gp("z_min_extent").value)
        self.min_points       = int(gp("min_points").value)
        self.ground_keep_rate = float(gp("ground_keep_rate").value)
        self.min_range        = float(gp("min_range").value)
        self.max_range        = float(gp("max_range").value)
        self.ground_z_pct     = float(gp("ground_z_percentile").value)
        self.z_amplify        = float(gp("z_amplify").value)
        self.z_amplify_cap    = float(gp("z_amplify_cap").value)
        self.roughness_thresh = float(gp("roughness_threshold").value)

        # z_min_extent and roughness_threshold are specified in raw-z space;
        # structure detection runs on z_res (also raw-z space, pre-amplification)
        # so no scale correction needed here.

        self._z_offset: int = -1
        self._z_dtype = np.float32

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(PointCloud2, gp("out_topic").value, qos)
        self.create_subscription(PointCloud2, gp("in_topic").value, self.cb, qos)
        self.get_logger().info("vertical_structure_filter up: %s -> %s"
                               % (gp("in_topic").value, gp("out_topic").value))

    def _resolve_z_offset(self, msg):
        for f in msg.fields:
            if f.name == "z":
                self._z_offset = f.offset
                self._z_dtype = {7: np.float32, 8: np.float64}.get(f.datatype, np.float32)
                return

    def _compute(self, cells_cur, z_cur):
        """Returns (struct_mask_cur, z_res_cur, ground_z_cur) all indexed to cells_cur points.

        struct_mask_cur: bool (N,) — True if point belongs to a structure cell
        z_res_cur:       float (N,) — ground-relative z, clipped to z_amplify_cap
        ground_z_cur:    float (N,) — estimated ground height per point
        """
        # Current frame only — no window accumulation for speed
        _, inv = np.unique(cells_cur, axis=0, return_inverse=True)
        m = inv.max() + 1
        cnt = np.bincount(inv, minlength=m)

        # Vectorized per-cell low-percentile ground height
        order      = np.argsort(inv, kind="stable")
        z_sorted   = z_cur[order]
        cell_start = np.zeros(m, dtype=np.int64)
        cell_start[1:] = np.cumsum(cnt[:-1])
        pct_idx    = np.clip(
            (cnt * (self.ground_z_pct / 100.0)).astype(np.int64),
            0, np.maximum(cnt - 1, 0)
        )
        ground_z_cell = z_sorted[cell_start + pct_idx]

        z_res_all = np.clip(z_cur - ground_z_cell[inv], 0.0, self.z_amplify_cap)

        # Structure detection on z_res (raw-z space, NOT amplified)
        zmax = np.full(m, -np.inf)
        zmin = np.full(m,  np.inf)
        np.maximum.at(zmax, inv, z_res_all)
        np.minimum.at(zmin, inv, z_res_all)

        z_mean = np.bincount(inv, weights=z_res_all, minlength=m) / np.maximum(cnt, 1)
        z_var  = np.bincount(inv, weights=(z_res_all - z_mean[inv]) ** 2, minlength=m) / np.maximum(cnt, 1)

        cell_is_struct = (
            ((zmax - zmin) > self.z_min_extent) |
            (z_var > self.roughness_thresh)
        ) & (cnt >= self.min_points)

        return cell_is_struct[inv], z_res_all, ground_z_cell[inv]

    def cb(self, msg):
        t0 = self.get_clock().now().nanoseconds
        n = msg.width * msg.height
        if n == 0:
            return

        if self._z_offset < 0:
            self._resolve_z_offset(msg)

        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)

        xyz = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
        x = np.asarray(xyz["x"], dtype=np.float64)
        y = np.asarray(xyz["y"], dtype=np.float64)
        z = np.asarray(xyz["z"], dtype=np.float64)

        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        r2 = x * x + y * y
        valid &= (r2 >= self.min_range ** 2) & (r2 <= self.max_range ** 2)

        is_structure = np.zeros(n, dtype=bool)
        z_out = z.copy()

        if valid.any():
            cells_cur = np.floor(np.stack([x[valid], y[valid]], axis=1) / self.cell_size).astype(np.int64)
            z_cur     = z[valid]

            struct_cur, z_res_cur, ground_z_cur = self._compute(cells_cur, z_cur)
            is_structure[valid] = struct_cur

            # Amplify z_res, reconstruct absolute z for output
            z_amp_cur = np.clip(z_res_cur * self.z_amplify, 0.0, self.z_amplify_cap * self.z_amplify)
            z_out[valid] = ground_z_cur + z_amp_cur


        keep = valid & (is_structure | (np.random.random(n) < self.ground_keep_rate))

        # Sanity check: verify no points closer than min_range leak through
        r2_keep = (x[keep] ** 2 + y[keep] ** 2)
        n_leaked = (r2_keep < self.min_range ** 2).sum()
        if n_leaked > 0:
            self.get_logger().warn(f"[leak] {n_leaked} points closer than min_range in output")
        if not keep.any():
            return

        out = raw[keep].copy()

        # Patch z field bytes in output buffer
        if self._z_offset >= 0:
            z_keep = z_out[keep].astype(self._z_dtype)
            off    = self._z_offset
            nb     = self._z_dtype().itemsize
            out[:, off:off + nb] = z_keep.view(np.uint8).reshape(-1, nb)

        msg_out = PointCloud2()
        msg_out.header       = msg.header
        msg_out.height       = 1
        msg_out.width        = int(out.shape[0])
        msg_out.fields       = msg.fields
        msg_out.is_bigendian = msg.is_bigendian
        msg_out.point_step   = msg.point_step
        msg_out.row_step     = msg.point_step * msg_out.width
        msg_out.is_dense     = True
        msg_out.data         = out.tobytes()
        self.pub.publish(msg_out)
        dt = (self.get_clock().now().nanoseconds - t0) * 1e-6
        self.get_logger().info(f"[timing] {dt:.1f} ms")


def main():
    rclpy.init()
    node = VerticalStructureFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()