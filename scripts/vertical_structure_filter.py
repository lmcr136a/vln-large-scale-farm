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
        self.declare_parameter("cell_size", 0.5)         # XY cell for column detection (m)
        self.declare_parameter("z_min_extent", 0.15)      # min vertical span in a cell to call it structure (m)
        self.declare_parameter("min_points", 4)          # min points in a cell to call it structure
        self.declare_parameter("ground_keep_rate", 0.001) # keep fraction of non-structure points
        self.declare_parameter("max_range", 200.0)   
        self.declare_parameter("min_range", 0.4)   # drop self/near points (m)

        gp = self.get_parameter
        self.cell_size = float(gp("cell_size").value)
        self.z_min_extent = float(gp("z_min_extent").value)
        self.min_points = int(gp("min_points").value)
        self.ground_keep_rate = float(gp("ground_keep_rate").value)
        self.min_range = float(gp("min_range").value)
        self.max_range = float(gp("max_range").value)
 
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(PointCloud2, gp("out_topic").value, qos)
        self.create_subscription(PointCloud2, gp("in_topic").value, self.cb, qos)
        self.get_logger().info("vertical_structure_filter up: %s -> %s"
                               % (gp("in_topic").value, gp("out_topic").value))
 
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
            cells = np.floor(np.stack([x[valid], y[valid]], axis=1) / self.cell_size).astype(np.int64)
            _, inv = np.unique(cells, axis=0, return_inverse=True)
            m = inv.max() + 1
 
            zmax = np.full(m, -np.inf)
            zmin = np.full(m, np.inf)
            cnt = np.zeros(m, dtype=np.int64)
            np.maximum.at(zmax, inv, z[valid])
            np.minimum.at(zmin, inv, z[valid])
            np.add.at(cnt, inv, 1)
 
            # A vertical structure = points stacked over a tall z range within one XY cell
            cell_is_struct = ((zmax - zmin) > self.z_min_extent) & (cnt >= self.min_points)
            is_structure[valid] = cell_is_struct[inv]
 
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