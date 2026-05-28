import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class Viewer3D(Node):
    def __init__(self):
        super().__init__('viewer_3d')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.sub = self.create_subscription(
            PointCloud2, '/rslidar_points', self._cb, qos)
        self.done = False

    def _cb(self, msg):
        if self.done:
            return
        gen = point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True)
        pts = np.array([(p[0], p[1], p[2]) for p in gen], dtype=np.float32)
        if len(pts) == 0:
            return

        idx = np.random.choice(len(pts), min(1000, len(pts)), replace=False)
        pts = pts[idx]

        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5), facecolor='#111')

        def setup(ax, title, xlabel, ylabel):
            ax.set_facecolor('#111')
            ax.set_title(title, color='white')
            ax.set_xlabel(xlabel, color='white')
            ax.set_ylabel(ylabel, color='white')
            ax.tick_params(colors='white')
            ax.set_aspect('equal')

        # Top-down X-Y
        ax1.scatter(pts[:,0], pts[:,1], s=1.5, c=pts[:,2], cmap='RdYlGn', alpha=0.7)
        ax1.plot(0, 0, 'r+', ms=12, mew=2)
        ax1.annotate('', xy=(1.5,0), xytext=(0,0), arrowprops=dict(arrowstyle='->', color='red', lw=2))
        ax1.annotate('', xy=(0,1.5), xytext=(0,0), arrowprops=dict(arrowstyle='->', color='cyan', lw=2))
        ax1.text(1.6, 0, '+X', color='red', va='center')
        ax1.text(0, 1.6, '+Y', color='cyan', ha='center')
        setup(ax1, 'Top-down (X-Y)', 'X', 'Y')

        # Front view X-Z
        ax2.scatter(pts[:,0], pts[:,2], s=1.5, c=pts[:,1], cmap='RdYlGn', alpha=0.7)
        ax2.plot(0, 0, 'r+', ms=12, mew=2)
        ax2.annotate('', xy=(1.5,0), xytext=(0,0), arrowprops=dict(arrowstyle='->', color='red', lw=2))
        ax2.annotate('', xy=(0,1.5), xytext=(0,0), arrowprops=dict(arrowstyle='->', color='lime', lw=2))
        ax2.text(1.6, 0, '+X', color='red', va='center')
        ax2.text(0, 1.6, '+Z', color='lime', ha='center')
        setup(ax2, 'Front view (X-Z)', 'X', 'Z')

        # Side view Y-Z
        ax3.scatter(pts[:,1], pts[:,2], s=1.5, c=pts[:,0], cmap='RdYlGn', alpha=0.7)
        ax3.plot(0, 0, 'r+', ms=12, mew=2)
        ax3.annotate('', xy=(1.5,0), xytext=(0,0), arrowprops=dict(arrowstyle='->', color='cyan', lw=2))
        ax3.annotate('', xy=(0,1.5), xytext=(0,0), arrowprops=dict(arrowstyle='->', color='lime', lw=2))
        ax3.text(1.6, 0, '+Y', color='cyan', va='center')
        ax3.text(0, 1.6, '+Z', color='lime', ha='center')
        setup(ax3, 'Side view (Y-Z)', 'Y', 'Z')

        plt.tight_layout()
        plt.savefig('./lidar_3d.png', dpi=120, facecolor='#111')
        plt.close()
        print('Saved: ./lidar_3d.png')
        self.done = True

def main():
    rclpy.init()
    node = Viewer3D()
    while not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()