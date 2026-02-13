#!/usr/bin/env python3

import os
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, Imu, PointField
from std_msgs.msg import Header

NS = 1_000_000_000

def rebuild_timestamp_from_nsec(out, unix_anchor_ns):

    if not out:
        return out

    NS = 1_000_000_000

    HI = 900_000_000
    LO = 100_000_000

    sec_counter = 0
    last_nsec = int(out[0]["_orig_nsec"])

    out[0]["_t"] = unix_anchor_ns + last_nsec

    for i in range(1, len(out)):
        nsec = int(out[i]["_orig_nsec"])

        # 진짜 초 경계일 때만 rollover
        if last_nsec > HI and nsec < LO:
            sec_counter += 1

        # Unix 기반 timestamp
        out[i]["_t"] = unix_anchor_ns + sec_counter * NS + nsec

        last_nsec = nsec

    return out


def ts_ns(sec: int, nsec: int) -> int:
    return int(sec) * NS + int(nsec)

def split_ns(t: int):
    s = int(t // NS)
    n = int(t % NS)
    return s, n

class DataPublisher(Node):
    def __init__(self, dataset_dir):
        super().__init__("data_publisher")

        self.declare_parameter("lidar_topic", "/lidar3d")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("publish_hz", 1000.0)
        self.declare_parameter("max_per_tick", 1)
        self.declare_parameter("max_dt_multiplier", 5.0) 

        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.max_per_tick = int(self.get_parameter("max_per_tick").value)
        self.max_dt_multiplier = float(self.get_parameter("max_dt_multiplier").value)

        self.clock_pub = self.create_publisher(Clock, "/clock", 200)
        self.lidar_pub = self.create_publisher(PointCloud2, self.lidar_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 200)

        self.lidar_data = self.load_lidar_data(dataset_dir)
        self.imu_data = self.load_imu_data(dataset_dir)

        if len(self.imu_data) == 0 or len(self.lidar_data) == 0:
            raise RuntimeError("Empty IMU or LiDAR data")

        self.prepare_streams()

        self.events = self.merge_events()
        self.ev_idx = 0

        self.log_file = open('/tmp/timestamp_log.txt', 'w')
        self.log_file.write("TYPE\tTIMESTAMP_SEC\tTIMESTAMP_NSEC\tINDEX\tORIG_SEC\tORIG_NSEC\n")

        period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(f"IMU kept: {len(self.imu_data)}")
        self.get_logger().info(f"LiDAR kept: {len(self.lidar_data)}")
        self.get_logger().info(f"Events: {len(self.events)}")
        self.get_logger().info(f"Logging to /tmp/timestamp_log.txt")

    def validate_timestamps(self, data, data_type="data"):
        if len(data) < 2:
            return data
        
        valid_data = []
        for i, d in enumerate(data):
            if d["_t"] <= 0:
                self.get_logger().warn(f"{data_type}[{i}]: Invalid timestamp (<=0): {d['_t']}")
                continue
            if i > 0 and d["_t"] < valid_data[-1]["_t"]:
                self.get_logger().warn(f"{data_type}[{i}]: Timestamp going backwards: {d['_t']} <= {valid_data[-1]['_t']}")
                continue
            if i > 0 and d["_t"] == valid_data[-1]["_t"]:
                self.get_logger().warn(f"{data_type}[{i}]: Timestamp same: {d['_t']} == {valid_data[-1]['_t']}")
                continue
            valid_data.append(d)
        
        if len(valid_data) < 2:
            return valid_data
        
        initial_samples = int(len(valid_data))-1
        initial_dts = [valid_data[i+1]["_t"] - valid_data[i]["_t"] for i in range(0,initial_samples)]
        
        expected_dt = np.median(initial_dts)
        max_allowed_dt = expected_dt * self.max_dt_multiplier
        
        self.get_logger().info(
            f"{data_type}: Expected dt: {expected_dt/1e6:.3f}ms, "
            f"Max allowed: {max_allowed_dt/1e6:.3f}ms, "
        )
        
        filtered_data = [valid_data[0]]
        skipped_count = 0
        segment_jumps = 0
        
        for i in range(1, len(valid_data)):
            dt = valid_data[i]["_t"] - valid_data[i-1]["_t"]
            
            if dt > max_allowed_dt:
                self.get_logger().warn(
                    f"{data_type}[{i}]: Abnormal dt: {dt/1e6:.2f}ms "
                    f"(expected: ~{expected_dt/1e6:.2f}ms, max: {max_allowed_dt/1e6:.2f}ms)"
                )
                skipped_count += 1
                continue
            
            filtered_data.append(valid_data[i])
        
        if skipped_count > 0:
            self.get_logger().info(
                f"{data_type}: Filtered {skipped_count} samples with abnormal timestamps. "
                f"Detected {segment_jumps} segment jumps. "
                f"Kept {len(filtered_data)}/{len(data)} samples"
            )
        
        return filtered_data

    def load_lidar_data(self, dataset_dir):
        path = os.path.join(dataset_dir, "lidar_pointcloud.bin")
        out = []
        with open(path, "rb") as f:
            while True:
                b = f.read(4)
                if len(b) < 4:
                    break
                sec = int(np.frombuffer(b, dtype=np.int32)[0])
                nsec = int(np.frombuffer(f.read(4), dtype=np.int32)[0])
                num = int(np.frombuffer(f.read(4), dtype=np.int32)[0])
                pts = np.frombuffer(f.read(num * 4 * 4), dtype=np.float32).reshape(-1, 4)
                
                t_ns = int(str(sec) + f"{nsec:09d}")
                out.append({
                    "sec": sec, 
                    "nanosec": nsec, 
                    "points": pts, 
                    "_t": t_ns,
                    "_orig_sec": sec,
                    "_orig_nsec": nsec
                })
        
        out = self.validate_timestamps(out, "LiDAR")
        
        return out

    def load_imu_data(self, dataset_dir):
        path = os.path.join(dataset_dir, "lidar_imu.bin")
        out = []
        with open(path, "rb") as f:
            while True:
                b = f.read(12 * 4)
                if len(b) < 12 * 4:
                    break
                data = np.frombuffer(b, dtype=np.float32)
                sec_raw = int(data[0])
                nsec_raw = int(data[1])
                
                t_ns = int(str(sec_raw) + f"{nsec_raw:09d}")
                accel = data[2:5].astype(np.float32)
                gyro = data[5:8].astype(np.float32)
                quat = data[8:12].astype(np.float32)
                
                out.append({
                    "accel": accel, 
                    "gyro": gyro, 
                    "quat": quat,
                    "_t": t_ns,
                    "_orig_sec": sec_raw,
                    "_orig_nsec": nsec_raw
                })
        
        # out.sort(key=lambda x: x["_t"])

        out = rebuild_timestamp_from_nsec(out, self.lidar_data[0]["_t"])
        out = self.validate_timestamps(out, "IMU")
        
        return out

    def prepare_streams(self):
        if len(self.imu_data) > 1:
            imu_dts = [self.imu_data[i+1]["_t"] - self.imu_data[i]["_t"] 
                    for i in range(min(100, len(self.imu_data)-1))]
            avg_imu_dt = np.mean(imu_dts)
            actual_imu_hz = NS / avg_imu_dt
            self.get_logger().info(f"Actual IMU rate: {actual_imu_hz:.2f} Hz (dt: {avg_imu_dt/1e6:.3f}ms)")
        
        if len(self.lidar_data) > 1:
            lidar_dts = [self.lidar_data[i+1]["_t"] - self.lidar_data[i]["_t"] 
                        for i in range(min(10, len(self.lidar_data)-1))]
            avg_lidar_dt = np.mean(lidar_dts)
            actual_lidar_hz = NS / avg_lidar_dt
            self.get_logger().info(f"Actual LiDAR rate: {actual_lidar_hz:.2f} Hz (dt: {avg_lidar_dt/1e6:.3f}ms)")

        lidar_orig_start = self.lidar_data[0]["_t"]
        imu_orig_start = self.imu_data[0]["_t"]


        # lidar_start = self.lidar_data[0]["_t"]
        # self.imu_data = [d for d in self.imu_data if d["_t"] >= lidar_start]
        common_start = min(lidar_orig_start, imu_orig_start)
        new_start_ns = 0
        
        for i, ld in enumerate(self.lidar_data):
            offset_ns = ld["_t"] - common_start
            new_t = new_start_ns + offset_ns
            ld["_t"] = new_t
            s, n = split_ns(new_t)
            ld["sec"] = s
            ld["nanosec"] = n
        
        for i, imu in enumerate(self.imu_data):
            offset_ns = imu["_t"] - common_start
            new_t = new_start_ns + offset_ns
            imu["_t"] = new_t
            s, n = split_ns(new_t)
            imu["sec"] = s
            imu["nanosec"] = n
        
        # 3️⃣ 나머지 정보 출력
        start_time = self.lidar_data[0]["_t"] if len(self.lidar_data) > 0 else self.imu_data[0]["_t"]
        end_time = max(self.lidar_data[-1]["_t"], self.imu_data[-1]["_t"])
        
        self.get_logger().info(f"Start time: {start_time / 1e9:.3f}s")
        self.get_logger().info(f"End time: {end_time / 1e9:.3f}s")
        self.get_logger().info(f"Duration: {(end_time - start_time) / 1e9:.3f}s")
        self.get_logger().info(f"LiDAR first: {self.lidar_data[0]['_t'] / 1e9:.6f}s")
        self.get_logger().info(f"IMU first: {self.imu_data[0]['_t'] / 1e9:.6f}s")

    def merge_events(self):
        ev = []
        for i, d in enumerate(self.imu_data):
            ev.append((d["_t"], 0, i))
        for i, d in enumerate(self.lidar_data):
            ev.append((d["_t"], 1, i))
        ev.sort(key=lambda x: (x[0], x[1]))
        return ev

    def tick(self):
        if self.ev_idx >= len(self.events):
            self.timer.cancel()
            self.log_file.close()
            self.get_logger().info("Finished publishing. Log saved to /tmp/timestamp_log.txt")
            rclpy.shutdown()
            return

        n = 0
        while self.ev_idx < len(self.events) and n < self.max_per_tick:
            t, typ, idx = self.events[self.ev_idx]
            self.publish_clock(t)
            if typ == 0:
                self.publish_imu(idx, t)
                print("Published imu")
            else:
                self.publish_lidar(idx, t)
                print("Published lidar")
            self.ev_idx += 1
            n += 1

    def publish_clock(self, t_ns):
        s, n = split_ns(t_ns)
        c = Clock()
        c.clock = Time(sec=s, nanosec=n)
        self.clock_pub.publish(c)

    def publish_lidar(self, idx, t_ns):
        d = self.lidar_data[idx]
        s, n = split_ns(t_ns)
        
        self.log_file.write(f"LIDAR\t{s}\t{n}\t{idx}\t{d['_orig_sec']}\t{d['_orig_nsec']}\n")
        self.log_file.flush()
        
        h = Header()
        h.stamp = Time(sec=s, nanosec=n)
        h.frame_id = self.frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        pts = d["points"]
        msg = PointCloud2()
        msg.header = h
        msg.height = 1
        msg.width = int(len(pts))
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = pts.tobytes()

        self.lidar_pub.publish(msg)

    def publish_imu(self, idx, t_ns):
        d = self.imu_data[idx]
        s, n = split_ns(t_ns)

        self.log_file.write(f"IMU\t{s}\t{n}\t{idx}\t{d['_orig_sec']}\t{d['_orig_nsec']}\n")
        self.log_file.flush()

        msg = Imu()
        msg.header.stamp = Time(sec=s, nanosec=n)
        msg.header.frame_id = self.frame_id

        q = d["quat"]
        msg.orientation.x = float(q[0])
        msg.orientation.y = float(q[1])
        msg.orientation.z = float(q[2])
        msg.orientation.w = float(q[3])

        g = d["gyro"]
        msg.angular_velocity.x = float(g[0])
        msg.angular_velocity.y = float(g[1])
        msg.angular_velocity.z = float(g[2])

        a = d["accel"]
        msg.linear_acceleration.x = float(a[0])
        msg.linear_acceleration.y = float(a[1])
        msg.linear_acceleration.z = float(a[2])

        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    dataset_dir = "1/20260210_1239"
    dataset_dir = os.path.join("/home/nahyeon/box/vln-large-scale-farm/data", dataset_dir)

    node = DataPublisher(dataset_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log_file.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()