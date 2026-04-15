#!/usr/bin/env python3
"""
YOLO semantic perception node.

Runs YOLOv8 on RGB + depth images and publishes SemanticObservation payloads
on /semantic_observations for consumption by the SafetyInterceptor node.

Topic remapping for this robot (ZED cameras, Livox LiDAR)
==========================================================
The node declares generic parameter defaults; override them at launch time:

  ros2 run ... yolo_node --ros-args \
    -p rgb_topic:=/zed/rgb_front \
    -p depth_topic:=/zed/depth_front \
    -p camera_info_topic:=/zed/camera_info_front \
    -p scan_topic:=/scan          # leave as /scan or omit if no LaserScan available
    -p model_path:=/path/to/yolov8n-seg.pt \
    -p device:=cuda:0

Note: the Livox MID360 does NOT publish a standard LaserScan.  If no /scan
source is available the fusion layer will use depth-only distance estimation.
The SafetyInterceptor geometric layer (safety_checker.py) provides independent
LiDAR-based obstacle avoidance and does not depend on this node being active.
"""
from __future__ import annotations

import json
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String

from .fusion import CameraIntrinsics, detections_to_observations
from .runtime import ensure_ros_log_dir
from .types import BoundingBox2D, SemanticDetection, observations_to_payload


class YoloSemanticNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_semantic_node")

        self.declare_parameter("rgb_topic", "/camera/rgb/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/rgb/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_topic", "/semantic_observations")
        self.declare_parameter("model_path", "yolov8n-seg.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("max_depth_m", 20.0)
        self.declare_parameter("publish_debug_detections", False)

        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.device = str(self.get_parameter("device").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.publish_debug_detections = bool(self.get_parameter("publish_debug_detections").value)

        self.model = self._load_model(self.model_path)
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_intrinsics: Optional[CameraIntrinsics] = None
        self.latest_scan: Optional[LaserScan] = None

        self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 20)
        self.create_subscription(Image, self.rgb_topic, self.on_rgb, 10)

        self.semantic_pub = self.create_publisher(String, self.output_topic, 20)
        self.debug_pub = self.create_publisher(String, f"{self.output_topic}/debug", 20)

        self.get_logger().info(
            f"YOLO semantic node ready: rgb={self.rgb_topic}, depth={self.depth_topic}, model={self.model_path}",
        )

    def _load_model(self, model_path: str):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is required for the YOLO node. Install it in the robot env, "
                "then export the model to TensorRT for deployment."
            ) from exc

        model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLO model from {model_path}")
        return model

    def on_depth(self, msg: Image) -> None:
        self.latest_depth = self._decode_depth(msg)

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.latest_intrinsics = CameraIntrinsics(
            fx=float(msg.k[0]),
            fy=float(msg.k[4]),
            cx=float(msg.k[2]),
            cy=float(msg.k[5]),
            width=int(msg.width),
            height=int(msg.height),
        )

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def on_rgb(self, msg: Image) -> None:
        if self.latest_depth is None or self.latest_intrinsics is None:
            return

        rgb_image = self._decode_rgb(msg)
        detections = self._run_yolo(rgb_image)
        observations = detections_to_observations(
            detections=detections,
            depth_image=self.latest_depth,
            intrinsics=self.latest_intrinsics,
            lidar_ranges=self.latest_scan.ranges if self.latest_scan is not None else None,
            lidar_angle_min=self.latest_scan.angle_min if self.latest_scan is not None else 0.0,
            lidar_angle_increment=self.latest_scan.angle_increment if self.latest_scan is not None else 0.0,
            max_depth_m=self.max_depth_m,
        )

        self.semantic_pub.publish(String(data=json.dumps(observations_to_payload(observations))))
        if self.publish_debug_detections:
            self.debug_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "detections": [
                                {
                                    "label": item.label,
                                    "confidence": item.confidence,
                                    "bbox": item.bbox.__dict__,
                                }
                                for item in detections
                            ]
                        }
                    )
                )
            )

    def _run_yolo(self, rgb_image: np.ndarray) -> List[SemanticDetection]:
        results = self.model.predict(
            source=rgb_image,
            conf=self.confidence_threshold,
            device=self.device,
            verbose=False,
        )
        detections: List[SemanticDetection] = []
        for result in results:
            names = result.names
            if result.boxes is None:
                continue
            for box in result.boxes:
                confidence = float(box.conf[0].item())
                class_id = int(box.cls[0].item())
                label = str(names[class_id])
                x_min, y_min, x_max, y_max = [float(item) for item in box.xyxy[0].tolist()]
                detections.append(
                    SemanticDetection(
                        label=label,
                        confidence=confidence,
                        bbox=BoundingBox2D(x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max),
                    )
                )
        return detections

    def _decode_rgb(self, msg: Image) -> np.ndarray:
        if msg.encoding not in {"rgb8", "bgr8"}:
            raise ValueError(f"Unsupported RGB encoding: {msg.encoding}")
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            image = image[:, :, ::-1]
        return image

    def _decode_depth(self, msg: Image) -> np.ndarray:
        if msg.encoding == "32FC1":
            return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        if msg.encoding == "16UC1":
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            return raw.astype(np.float32) * 0.001
        raise ValueError(f"Unsupported depth encoding: {msg.encoding}")


def main() -> None:
    ensure_ros_log_dir()
    rclpy.init()
    node = YoloSemanticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
