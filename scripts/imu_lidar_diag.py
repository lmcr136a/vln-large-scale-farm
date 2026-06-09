#!/usr/bin/env python3
import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt


def rodrigues(rotvec):
    # Rotation matrix from a rotation vector (axis * angle)
    theta = np.linalg.norm(rotvec)
    if theta < 1e-9:
        return np.eye(3)
    k = rotvec / theta
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def align_gravity(a_body):
    # Rotation world<-body mapping the measured specific force (up at rest) to world +Z
    a = a_body / (np.linalg.norm(a_body) + 1e-12)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(a, z)
    s = np.linalg.norm(v)
    c = float(np.dot(a, z))
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]],
                   [v[2], 0, -v[0]],
                   [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))


def rot_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


class ImuLidarDiag(Node):
    def __init__(self):
        super().__init__("imu_lidar_diag")
        self.declare_parameter("imu_topic", "/xsens/imu/data")
        self.declare_parameter("points_topic", "/rslidar_points")
        self.declare_parameter("out_dir", "./imu_lidar_diag")
        self.declare_parameter("duration", 30.0)
        self.declare_parameter("save_interval", 5.0)
        self.declare_parameter("crop_radius", 20.0)
        self.declare_parameter("max_points", 20000)
        self.declare_parameter("static_init", 1.0)  # stationary window to estimate gravity/bias

        gp = self.get_parameter
        self.imu_topic = gp("imu_topic").value
        self.points_topic = gp("points_topic").value
        self.out_dir = gp("out_dir").value
        self.duration = float(gp("duration").value)
        self.save_interval = float(gp("save_interval").value)
        self.crop_radius = float(gp("crop_radius").value)
        self.max_points = int(gp("max_points").value)
        self.static_init = float(gp("static_init").value)
        os.makedirs(self.out_dir, exist_ok=True)

        qos = QoSProfile(depth=200)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT  # match sensor_data publishers
        qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, qos)
        self.create_subscription(PointCloud2, self.points_topic, self.cloud_cb, qos)

        self.t0 = None
        self.last_t = None
        self.R = np.eye(3)
        self.v = np.zeros(3)
        self.p = np.zeros(3)
        self.g_mag = 9.81
        self.gyro_bias = np.zeros(3)

        self.init_done = False
        self.init_acc = []
        self.init_gyro = []

        self.traj = []  # elapsed, x, y, z, roll, pitch, yaw
        self.next_save = 0.0
        self.finished = False
        self.latest_cloud = None
        self.get_logger().info("Recording for %.0fs ..." % self.duration)

    def stamp_sec(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def imu_cb(self, msg):
        if self.finished:
            return
        t = self.stamp_sec(msg)
        if self.t0 is None:
            self.t0 = t
            self.last_t = t
            self.next_save = self.save_interval
        acc = np.array([msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])
        elapsed = t - self.t0

        if not self.init_done:
            self.init_acc.append(acc)
            self.init_gyro.append(gyro)
            if elapsed >= self.static_init:
                a0 = np.mean(self.init_acc, axis=0)
                self.g_mag = float(np.linalg.norm(a0))
                self.R = align_gravity(a0)
                self.gyro_bias = np.mean(self.init_gyro, axis=0)
                self.init_done = True
                self.get_logger().info("init: g=%.3f gyro_bias=[%.4f %.4f %.4f]"
                                       % (self.g_mag, *self.gyro_bias))
            self.last_t = t
            return

        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 0.5:  # skip duplicate / huge gaps
            return

        self.R = self.R @ rodrigues((gyro - self.gyro_bias) * dt)
        a_world = self.R @ acc - np.array([0.0, 0.0, self.g_mag])
        self.p = self.p + self.v * dt + 0.5 * a_world * dt * dt
        self.v = self.v + a_world * dt

        roll, pitch, yaw = rot_to_rpy(self.R)
        self.traj.append((elapsed, self.p[0], self.p[1], self.p[2], roll, pitch, yaw))

        if elapsed >= self.next_save:
            self.save_lidar_frame(elapsed)
            self.next_save += self.save_interval
        if elapsed >= self.duration:
            self.finalize()

    def cloud_cb(self, msg):
        if not self.finished:
            self.latest_cloud = msg

    def save_lidar_frame(self, elapsed):
        if self.latest_cloud is None:
            self.get_logger().warn("no cloud yet at t=%.1f" % elapsed)
            return
        pts = list(pc2.read_points(self.latest_cloud,
                                   field_names=("x", "y", "z"), skip_nans=True))
        if len(pts) == 0:
            return
        arr = np.array([(p[0], p[1], p[2]) for p in pts], dtype=np.float32)
        r = np.sqrt(arr[:, 0] ** 2 + arr[:, 1] ** 2)
        arr = arr[r <= self.crop_radius]
        if arr.shape[0] == 0:
            return
        if arr.shape[0] > self.max_points:
            idx = np.random.choice(arr.shape[0], self.max_points, replace=False)
            arr = arr[idx]

        x, y = arr[:, 0], arr[:, 1]  # z ignored
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.scatter(y, x, s=1, c="0.45")

        # Explicit sensor axes: +X forward (red), +Y left (blue)
        L = 0.45 * self.crop_radius
        ax.annotate("", xy=(0, L), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color="red", lw=2.5))
        ax.annotate("", xy=(L, 0), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color="blue", lw=2.5))
        ax.text(0, L * 1.05, "+X (forward)", color="red",
                ha="center", va="bottom", fontsize=11, fontweight="bold")
        ax.text(L * 1.05, 0, "+Y (left)", color="blue",
                ha="left", va="center", fontsize=11, fontweight="bold")
        ax.plot(0, 0, "k+", ms=12, mew=2)

        ax.set_aspect("equal")
        ax.set_xlim(self.crop_radius, -self.crop_radius)  # +Y (left) on the left
        ax.set_ylim(-self.crop_radius, self.crop_radius)
        ax.axhline(0, color="0.8", lw=0.8, zorder=0)
        ax.axvline(0, color="0.8", lw=0.8, zorder=0)
        ax.set_xlabel("Y [m] (+left)")
        ax.set_ylabel("X [m] (+forward)")
        cur = self.traj[-1] if self.traj else (elapsed, 0, 0, 0, 0, 0, 0)
        ax.set_title("t=%.1fs  IMU x=%.2f y=%.2f yaw=%.1f  (%d pts)"
                     % (elapsed, cur[1], cur[2], math.degrees(cur[6]), arr.shape[0]))
        ax.grid(True, alpha=0.3)
        fn = os.path.join(self.out_dir, "lidar_top_%05.1fs.png" % elapsed)
        fig.savefig(fn, dpi=120, bbox_inches="tight")
        plt.close(fig)
        self.get_logger().info("saved %s" % fn)

    def finalize(self):
        if self.finished:
            return
        self.finished = True
        if self.traj:
            data = np.array(self.traj)
            np.savetxt(os.path.join(self.out_dir, "imu_pose.csv"), data,
                       header="t,x,y,z,roll,pitch,yaw", delimiter=",", comments="")
            fig, ax = plt.subplots(figsize=(8, 8))
            sc = ax.scatter(data[:, 2], data[:, 1], c=data[:, 0], s=4, cmap="plasma")
            ax.invert_xaxis()
            ax.set_aspect("equal")
            ax.set_xlabel("Y [m] (+left)")
            ax.set_ylabel("X [m] (+forward)")
            ax.set_title("IMU-only trajectory (drift expected)")
            ax.grid(True, alpha=0.3)
            fig.colorbar(sc, ax=ax, label="time [s]")
            fig.savefig(os.path.join(self.out_dir, "imu_trajectory.png"),
                        dpi=120, bbox_inches="tight")
            plt.close(fig)
            self.get_logger().info("saved imu_pose.csv and imu_trajectory.png")
        self.get_logger().info("done.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ImuLidarDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finalize()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()