import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
import numpy as np
import matplotlib
matplotlib.use('Agg')   # headless backend (no display needed on Jetson)
import matplotlib.pyplot as plt
import os

WGS84_A = 6378137.0   # earth equatorial radius (m)
OUTPUT_PATH = os.path.expanduser('./gps_xy.png')
SAVE_EVERY_N = 20      # redraw PNG every N new fixes


def latlon_to_local(lat, lon, lat0, lon0):
    """Equirectangular projection around origin — good enough for a small farm."""
    lat0_rad = np.radians(lat0)
    east  = np.radians(lon - lon0) * WGS84_A * np.cos(lat0_rad)
    north = np.radians(lat - lat0) * WGS84_A
    return east, north


class GpsXYPlotter(Node):

    def __init__(self):
        super().__init__('gps_xy_plotter')

        self.lat0 = None
        self.lon0 = None
        self.xs = []
        self.ys = []
        self.count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.fix_callback, qos)

        print(f'[GpsXY] Subscribed to /gps/fix  output={OUTPUT_PATH}')

    def fix_callback(self, msg: NavSatFix):
        lat, lon = msg.latitude, msg.longitude

        if not np.isfinite(lat) or not np.isfinite(lon):
            return
        if lat == 0.0 and lon == 0.0:   # no fix yet
            return

        if self.lat0 is None:
            self.lat0, self.lon0 = lat, lon
            print(f'[GpsXY] Origin set: lat={lat:.7f} lon={lon:.7f}')

        east, north = latlon_to_local(lat, lon, self.lat0, self.lon0)
        self.xs.append(east)
        self.ys.append(north)
        self.count += 1

        if self.count % SAVE_EVERY_N == 0:
            self.save_plot()

    def save_plot(self):
        if len(self.xs) < 2:
            return
        xs = np.array(self.xs)
        ys = np.array(self.ys)

        plt.figure(figsize=(8, 8))
        plt.plot(xs, ys, '-', color='tab:blue', linewidth=1.0, zorder=1)
        plt.scatter(xs, ys, c='tab:red', s=6, zorder=2)
        plt.scatter([xs[0]], [ys[0]], c='green', s=80, marker='o', label='start', zorder=3)
        plt.scatter([xs[-1]], [ys[-1]], c='black', s=80, marker='*', label='latest', zorder=3)

        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.xlabel('East (m)')
        plt.ylabel('North (m)')
        plt.title(f'/gps/fix XY  (n={len(xs)})')
        plt.legend()
        plt.tight_layout()
        plt.savefig(OUTPUT_PATH, dpi=120)
        plt.close()
        print(f'[GpsXY] Saved {OUTPUT_PATH}  (n={len(xs)})')


def main():
    rclpy.init()
    node = GpsXYPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_plot()   # final save on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()