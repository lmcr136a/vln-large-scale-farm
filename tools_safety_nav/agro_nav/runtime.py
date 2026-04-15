from __future__ import annotations

import os
from pathlib import Path


def ensure_ros_log_dir() -> None:
    log_dir = os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")
    Path(log_dir).mkdir(parents=True, exist_ok=True)
