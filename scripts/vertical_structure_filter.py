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
        self.declare_parameter("cell_size", 0.4)         # XY cell for column detection (m)
        self.declare_parameter("z_min_extent", 0.1)      # min vertical span in a cell to call it structure (m)
        self.declare_parameter("min_points", 2)         # min points in a cell to call it structure
        self.declare_parameter("ground_keep_rate", 0.00)  # keep fraction of non-structure points
        self.declare_parameter("max_range", 1000.0)
        self.declare_parameter("min_range", 0.7)         # drop self/near points (m)
        self.declare_parameter("z_amplify", 2.0)
        self.declare_parameter("z_amplify_cap", 999.9)     # raw-z meters, pre-amplification
        self.declare_parameter("roughness_threshold", 0.003)  # m^2 variance of raw z_res
        # Strong vertical features: cells whose RAW z span >= features_z_extent get
        # their points duplicated features_num_mul times to up-weight them in SLAM.
        self.declare_parameter("features_z_extent", 0.4)  # raw-z span (m) to call a cell a strong feature
        self.declare_parameter("features_num_mul", 20)    # duplication factor for those points
        self.declare_parameter("features_jitter", 0.005)  # gaussian jitter (m) on duplicated copies; 0 disables
        # Far points are sparse; duplicate those beyond far_dist to up-weight them.
        # Multiplier ramps linearly with range: x1 at far_dist -> x far_points_mul at far_full_dist.
        self.declare_parameter("far_dist", 30.0)          # XY range (m) where the ramp starts (x1)
        self.declare_parameter("far_full_dist", 150.0)      # XY range (m) where the ramp hits far_points_mul; 0 = max_range
        self.declare_parameter("far_points_mul", 100)      # max duplication factor for far points
        self.declare_parameter("far_jitter", 0.05)        # gaussian jitter (m) on far duplicates; 0 disables

        gp = self.get_parameter
        self.cell_size        = float(gp("cell_size").value)
        self.z_min_extent     = float(gp("z_min_extent").value)
        self.min_points       = int(gp("min_points").value)
        self.ground_keep_rate = float(gp("ground_keep_rate").value)
        self.min_range        = float(gp("min_range").value)
        self.max_range        = float(gp("max_range").value)
        self.z_amplify        = float(gp("z_amplify").value)
        self.z_amplify_cap    = float(gp("z_amplify_cap").value)
        self.roughness_thresh = float(gp("roughness_threshold").value)
        self.feat_z_extent    = float(gp("features_z_extent").value)
        self.feat_num_mul     = int(gp("features_num_mul").value)
        self.feat_jitter      = float(gp("features_jitter").value)
        self.far_dist         = float(gp("far_dist").value)
        ffd                   = float(gp("far_full_dist").value)
        self.far_full         = ffd if ffd > 0.0 else self.max_range
        self.far_mul          = int(gp("far_points_mul").value)
        self.far_jitter       = float(gp("far_jitter").value)

        # z_min_extent and roughness_threshold are specified in raw-z space;
        # structure detection runs on z_res (also raw-z space, pre-amplification)
        # so no scale correction needed here.

        self._z_offset: int = -1
        self._z_dtype = np.float32
        self._xyz_off = None   # (off, dtype) per x,y,z, resolved lazily

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(PointCloud2, gp("out_topic").value, qos)
        self.create_subscription(PointCloud2, gp("in_topic").value, self.cb, qos)
        self.get_logger().info("vertical_structure_filter up: %s -> %s"
                               % (gp("in_topic").value, gp("out_topic").value))

    def _resolve_z_offset(self, msg):
        dt_map = {7: np.float32, 8: np.float64}
        off_by_name = {}
        for f in msg.fields:
            if f.name in ("x", "y", "z"):
                off_by_name[f.name] = (f.offset, dt_map.get(f.datatype, np.float32))
            if f.name == "z":
                self._z_offset = f.offset
                self._z_dtype = dt_map.get(f.datatype, np.float32)
        if {"x", "y", "z"} <= off_by_name.keys():
            self._xyz_off = [off_by_name["x"], off_by_name["y"], off_by_name["z"]]

    def _compute(self, cells_cur, z_cur):
        """Returns (struct_mask_cur, z_res_cur, ground_z_cur, tall_mask_cur).

        struct_mask_cur: bool (N,) — True if point belongs to a structure cell
        z_res_cur:       float (N,) — ground-relative z, clipped to z_amplify_cap
        ground_z_cur:    float (N,) — estimated ground height per point
        tall_mask_cur:   bool (N,) — True if point's cell raw-z span >= features_z_extent
        """
        # Current frame only — no window accumulation for speed.
        # Bit-pack the 2D cell index into one int64 key; np.unique on a 1D key is
        # much faster than axis=0 on a 2D array (cell indices fit safely in int64).
        key = cells_cur[:, 0] * np.int64(1 << 32) + cells_cur[:, 1]
        _, inv = np.unique(key, return_inverse=True)
        inv = inv.reshape(-1)
        m = inv.max() + 1
        cnt = np.bincount(inv, minlength=m)

        # Ground height per cell = lowest point in the cell (per-cell min raw z).
        order      = np.argsort(inv, kind="stable")
        z_sorted   = z_cur[order]
        cell_start = np.zeros(m, dtype=np.int64)
        cell_start[1:] = np.cumsum(cnt[:-1])
        zmin_raw = np.minimum.reduceat(z_sorted, cell_start)
        zmax_raw = np.maximum.reduceat(z_sorted, cell_start)
        ground_z_cell = zmin_raw

        z_res_all = np.clip(z_cur - ground_z_cell[inv], 0.0, self.z_amplify_cap)

        # Structure detection on z_res (raw-z space, NOT amplified).
        # reduceat over the cell-grouped (order-sorted) array is far faster than ufunc.at.
        z_res_sorted = z_res_all[order]
        zmax = np.maximum.reduceat(z_res_sorted, cell_start)
        zmin = np.minimum.reduceat(z_res_sorted, cell_start)

        z_mean = np.bincount(inv, weights=z_res_all, minlength=m) / np.maximum(cnt, 1)
        z_var  = np.bincount(inv, weights=(z_res_all - z_mean[inv]) ** 2, minlength=m) / np.maximum(cnt, 1)

        cell_is_struct = (
            ((zmax - zmin) > self.z_min_extent) |
            (z_var > self.roughness_thresh)
        ) & (cnt >= self.min_points)

        # Strong vertical features: RAW z span per cell (zmin_raw/zmax_raw above).
        cell_is_tall = ((zmax_raw - zmin_raw) >= self.feat_z_extent) & (cnt >= self.min_points)

        return cell_is_struct[inv], z_res_all, ground_z_cell[inv], cell_is_tall[inv]

    def cb(self, msg):
        t0 = self.get_clock().now().nanoseconds
        n = msg.width * msg.height
        if n == 0:
            return

        if self._z_offset < 0:
            self._resolve_z_offset(msg)

        # frombuffer (no bytes() copy); raw stays read-only, we only copy on `raw[keep]`
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

        if self._xyz_off is not None:
            # Read x/y/z straight from the packed buffer — much faster than pc2.read_points
            def _col(off, dt):
                nb = np.dtype(dt).itemsize
                return raw[:, off:off + nb].copy().view(dt).reshape(-1).astype(np.float64)
            (xo, xt), (yo, yt), (zo, zt) = self._xyz_off
            x = _col(xo, xt); y = _col(yo, yt); z = _col(zo, zt)
        else:
            xyz = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
            x = np.asarray(xyz["x"], dtype=np.float64)
            y = np.asarray(xyz["y"], dtype=np.float64)
            z = np.asarray(xyz["z"], dtype=np.float64)

        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        r2 = x * x + y * y
        valid &= (r2 >= self.min_range ** 2) & (r2 <= self.max_range ** 2)

        is_structure = np.zeros(n, dtype=bool)
        is_tall      = np.zeros(n, dtype=bool)
        z_out = z.copy()

        if valid.any():
            cells_cur = np.floor(np.stack([x[valid], y[valid]], axis=1) / self.cell_size).astype(np.int64)
            z_cur     = z[valid]

            struct_cur, z_res_cur, ground_z_cur, tall_cur = self._compute(cells_cur, z_cur)
            is_structure[valid] = struct_cur
            is_tall[valid]      = tall_cur

            # Amplify z_res, reconstruct absolute z for output
            z_amp_cur = np.clip(z_res_cur * self.z_amplify, 0.0, self.z_amplify_cap * self.z_amplify)
            z_out[valid] = ground_z_cur + z_amp_cur


        # Graded far multiplier: linear ramp x1 (at far_dist) -> far_mul (at far_full).
        r = np.sqrt(r2)
        span = max(self.far_full - self.far_dist, 1e-6)
        frac = np.clip((r - self.far_dist) / span, 0.0, 1.0)
        far_count = np.rint(1.0 + frac * (self.far_mul - 1)).astype(np.int64)
        far_count[~valid] = 1
        is_far = valid & (r2 >= self.far_dist ** 2)   # far region: kept + uses far_jitter

        keep = valid & (is_structure | is_tall | is_far | (np.random.random(n) < self.ground_keep_rate))

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

        # Duplicate up-weighted points (strong vertical features + far points).
        # A point in both groups takes the larger multiplier, not the product.
        n_dup = 0
        counts = np.ones(int(out.shape[0]), dtype=np.int64)
        if self.feat_num_mul > 1:
            counts = np.where(is_tall[keep], np.maximum(counts, self.feat_num_mul), counts)
        if self.far_mul > 1:
            counts = np.maximum(counts, far_count[keep])   # graded ramp, >=1 everywhere
        if (counts > 1).any():
            # Per-kept-point jitter scale: far points spread by far_jitter (large enough
            # to survive voxel downsampling), others by features_jitter.
            scale_keep = np.where(is_far[keep], self.far_jitter, self.feat_jitter)
            out        = np.repeat(out, counts, axis=0)
            scale_rows = np.repeat(scale_keep, counts)
            n_dup = int(out.shape[0] - counts.shape[0])
            # Jitter the duplicated copies (all but the first in each group) so they
            # survive GLIM voxel-downsampling and don't make per-point covariance
            # estimation degenerate from identical points.
            if (self.feat_jitter > 0 or self.far_jitter > 0) and self._xyz_off is not None:
                starts = np.zeros(out.shape[0], dtype=bool)
                starts[np.r_[0, np.cumsum(counts)[:-1]]] = True
                jmask = ~starts
                s     = scale_rows[jmask]
                nj    = int(jmask.sum())
                for off_f, dt_f in self._xyz_off:
                    nb_f = np.dtype(dt_f).itemsize
                    col  = out[:, off_f:off_f + nb_f].copy().view(dt_f).reshape(-1)
                    col[jmask] = col[jmask] + (np.random.randn(nj) * s).astype(dt_f)
                    out[:, off_f:off_f + nb_f] = col.view(np.uint8).reshape(-1, nb_f)

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
        self.get_logger().info(f"[timing] {dt:.1f} ms  (+{n_dup} dup feature pts)")


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