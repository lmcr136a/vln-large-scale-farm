#!/usr/bin/env python3
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

INIT_NUM = 4  # sliding window size for structure detection


class VerticalStructureFilter(Node):
    def __init__(self):
        super().__init__("vertical_structure_filter")
        self.declare_parameter("in_topic", "/rslidar_points")
        self.declare_parameter("out_topic", "/rslidar_points_vstruct")
        self.declare_parameter("cell_size", 0.5)         # XY cell for column detection (m)
        self.declare_parameter("z_min_extent", 0.4)      # min vertical span in a cell to call it structure (m)
        self.declare_parameter("min_points", 10)          # min points in a cell to call it structure
        self.declare_parameter("ground_keep_rate", 0.001) # keep fraction of non-structure points
        self.declare_parameter("max_range", 2000.0)   
        self.declare_parameter("min_range", 0.4)   # drop self/near points (m)

        gp = self.get_parameter
        self.cell_size = float(gp("cell_size").value)
        self.z_min_extent = float(gp("z_min_extent").value)
        self.min_points = int(gp("min_points").value)
        self.ground_keep_rate = float(gp("ground_keep_rate").value)
        self.min_range = float(gp("min_range").value)
        self.max_range = float(gp("max_range").value)

        # sliding window: stores (cells_n2, z_n) arrays for the last INIT_NUM frames
        self._window: deque = deque(maxlen=INIT_NUM)
 
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(PointCloud2, gp("out_topic").value, qos)
        self.create_subscription(PointCloud2, gp("in_topic").value, self.cb, qos)
        self.get_logger().info("vertical_structure_filter up: %s -> %s"
                               % (gp("in_topic").value, gp("out_topic").value))
 
    def _structure_mask_from_window(self, cells_cur, z_cur):
        """Compute per-point structure flags using the sliding window (previous INIT_NUM frames).
        cells_cur: (N, 2) int64, z_cur: (N,) float64 — valid points of the current frame only."""
        if len(self._window) == 0:
            # no history yet: fall back to current frame only
            all_cells = cells_cur
            all_z     = z_cur
        else:
            all_cells = np.concatenate([c for c, _ in self._window] + [cells_cur], axis=0)
            all_z     = np.concatenate([z for _, z in self._window] + [z_cur],     axis=0)

        _, inv = np.unique(all_cells, axis=0, return_inverse=True)
        m = inv.max() + 1

        zmax = np.full(m, -np.inf)
        zmin = np.full(m,  np.inf)
        cnt  = np.zeros(m, dtype=np.int64)
        np.maximum.at(zmax, inv, all_z)
        np.minimum.at(zmin, inv, all_z)
        np.add.at(cnt, inv, 1)

        cell_is_struct = ((zmax - zmin) > self.z_min_extent) & (cnt >= self.min_points)

        # return mask only for current-frame points (last len(cells_cur) entries)
        inv_cur = inv[-len(cells_cur):]
        return cell_is_struct[inv_cur]

    def cb(self, msg):
        n = msg.width * msg.height
        if n == 0:
            return
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)

        xyz = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
        x = np.asarray(xyz["x"], dtype=np.float64)
        y = np.asarray(xyz["y"], dtype=np.float64)
        z = np.asarray(xyz["z"], dtype=np.float64)

        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        r2 = x * x + y * y
        valid &= (r2 >= self.min_range * self.min_range) & (r2 <= self.max_range * self.max_range)

        is_structure = np.zeros(n, dtype=bool)
        if valid.any():
            cells_cur = np.floor(np.stack([x[valid], y[valid]], axis=1) / self.cell_size).astype(np.int64)
            z_cur     = z[valid]

            struct_cur = self._structure_mask_from_window(cells_cur, z_cur)
            is_structure[valid] = struct_cur

            # push current frame into window after computing (so window = previous INIT_NUM frames)
            self._window.append((cells_cur, z_cur))
 
        # Keep all structure points (strong weight); decimate everything else hard
        keep = valid & (is_structure | (np.random.random(n) < self.ground_keep_rate))
        if not keep.any():
            return
        out = raw[keep]
 
        msg_out = PointCloud2()
        msg_out.header = msg.header
        msg_out.height = 1
        msg_out.width = int(out.shape[0])
        msg_out.fields = msg.fields
        msg_out.is_bigendian = msg.is_bigendian
        msg_out.point_step = msg.point_step
        msg_out.row_step = msg.point_step * msg_out.width
        msg_out.is_dense = True
        msg_out.data = out.tobytes()
        self.pub.publish(msg_out)
 
 
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