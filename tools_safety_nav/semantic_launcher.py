#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import yaml


RETRY_SECONDS = 5.0


def _load_config() -> dict:
    repo_root = Path(__file__).resolve().parents[1]
    config_path = repo_root / "tools_control_panel" / "control_config.yaml"
    with config_path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _candidate_model_paths(configured: str | None) -> list[Path]:
    candidates: list[Path] = []
    env_path = os.environ.get("YOLO_MODEL_PATH", "").strip()
    if env_path:
        candidates.append(Path(env_path).expanduser())
    if configured:
        candidates.append(Path(configured).expanduser())

    repo_root = Path(__file__).resolve().parents[1]
    candidates.extend([
        repo_root / "models" / "yolov8n-seg.pt",
        Path.home() / "box" / "vln-large-scale-farm" / "models" / "yolov8n-seg.pt",
        Path.home() / "models" / "yolov8n-seg.pt",
    ])

    unique: list[Path] = []
    seen: set[str] = set()
    for item in candidates:
        resolved = str(item)
        if resolved in seen:
            continue
        seen.add(resolved)
        unique.append(item)
    return unique


def _find_model_path(configured: str | None) -> Path | None:
    for candidate in _candidate_model_paths(configured):
        if candidate.is_file():
            return candidate
    return None


def main() -> None:
    config = _load_config()
    semantics = config.get("semantics", {}) or {}
    if not semantics.get("enabled", True):
        print("[semantic_launcher] semantics disabled in control_config.yaml", flush=True)
        return

    configured_model = str(semantics.get("model_path", "")).strip() or None
    rgb_topic = str(semantics.get("rgb_topic", "/zed/rgb_front")).strip()
    depth_topic = str(semantics.get("depth_topic", "")).strip()
    camera_info_topic = str(semantics.get("camera_info_topic", "")).strip()
    scan_topic = str(semantics.get("scan_topic", "")).strip()
    device = str(semantics.get("device", "cuda:0")).strip()
    confidence_threshold = float(semantics.get("confidence_threshold", 0.35))
    publish_debug = bool(semantics.get("publish_debug_detections", False))

    if not rgb_topic:
        raise RuntimeError("semantics.rgb_topic must be set in control_config.yaml")

    announced_missing = False
    while True:
        model_path = _find_model_path(configured_model)
        if model_path is None:
            if not announced_missing:
                print("[semantic_launcher] YOLO model not found.", flush=True)
                for candidate in _candidate_model_paths(configured_model):
                    print(f"[semantic_launcher] checked: {candidate}", flush=True)
                print(
                    "[semantic_launcher] Set YOLO_MODEL_PATH or update semantics.model_path; "
                    f"retrying every {int(RETRY_SECONDS)}s.",
                    flush=True,
                )
                announced_missing = True
            time.sleep(RETRY_SECONDS)
            continue

        print(f"[semantic_launcher] using model: {model_path}", flush=True)
        cmd = [
            sys.executable,
            "-m",
            "tools_safety_nav.agro_nav.yolo_node",
            "--ros-args",
            "-p",
            f"rgb_topic:={rgb_topic}",
            "-p",
            f"model_path:={model_path}",
            "-p",
            f"device:={device}",
            "-p",
            f"confidence_threshold:={confidence_threshold}",
            "-p",
            f"publish_debug_detections:={'true' if publish_debug else 'false'}",
        ]
        if depth_topic:
            cmd.extend(["-p", f"depth_topic:={depth_topic}"])
        if camera_info_topic:
            cmd.extend(["-p", f"camera_info_topic:={camera_info_topic}"])
        if scan_topic:
            cmd.extend(["-p", f"scan_topic:={scan_topic}"])
        os.execvp(cmd[0], cmd)


if __name__ == "__main__":
    main()
