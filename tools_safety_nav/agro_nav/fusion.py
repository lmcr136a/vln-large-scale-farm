from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .types import BoundingBox2D, SemanticDetection, SemanticObservation


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


REFERENCE_OBJECT_HEIGHT_M = {
    "human": 1.70,
    "person": 1.70,
    "animal": 0.90,
    "dog": 0.60,
    "horse": 1.60,
    "tree": 2.50,
    "branch": 0.30,
    "crop": 0.50,
    "grass": 0.20,
    "bench": 0.85,
    "chair": 0.90,
    "potted plant": 0.60,
    "refrigerator": 1.70,
    "sink": 0.90,
    "suitcase": 0.70,
    "vehicle": 1.60,
    "car": 1.50,
    "truck": 2.80,
    "tractor": 2.20,
    "fence": 1.20,
    "pole": 2.00,
    "rock": 0.50,
    "unknown": 1.00,
}


def canonicalize_label(raw_label: str) -> str:
    label = raw_label.strip().lower()
    aliases = {
        "person": "human",
        "worker": "human",
        "pedestrian": "human",
        "tractor": "tractor",
        "harvester": "harvester",
        "combine": "harvester",
        "forklift": "vehicle",
        "truck": "vehicle",
        "car": "vehicle",
        "utility_vehicle": "vehicle",
        "utv": "vehicle",
        "cow": "animal",
        "dog": "animal",
        "horse": "animal",
        "rock": "rock",
        "stone": "rock",
        "pole": "pole",
        "post": "pole",
        "fence": "fence",
        "gate": "fence",
        "branch": "branch",
        "bush": "crop",
        "plant": "crop",
        "sapling": "tree",
        "tree": "tree",
        "trunk": "tree",
        "grass": "grass",
        "crop": "crop",
    }
    return aliases.get(label, label or "unknown")


def heuristic_distance_from_bbox(
    label: str,
    bbox: BoundingBox2D,
    intrinsics: CameraIntrinsics,
    max_depth_m: float,
) -> Optional[float]:
    # Monocular fallback for sparse depth frames. This is approximate, but it
    # preserves semantic identity and gives the safety layer a usable range.
    box_height_px = max(bbox.height, 1.0)
    canonical = canonicalize_label(label)
    ref_height_m = REFERENCE_OBJECT_HEIGHT_M.get(canonical, REFERENCE_OBJECT_HEIGHT_M["unknown"])
    distance = (intrinsics.fy * ref_height_m) / box_height_px
    if not np.isfinite(distance) or distance <= 0.0:
        return None
    return float(np.clip(distance, 0.2, max_depth_m))


def median_depth_in_bbox(depth_image: Optional[np.ndarray], bbox: BoundingBox2D, max_depth_m: float) -> Optional[float]:
    if depth_image is None:
        return None
    height, width = depth_image.shape
    x0 = max(0, min(width - 1, int(bbox.x_min)))
    x1 = max(0, min(width, int(np.ceil(bbox.x_max))))
    y0 = max(0, min(height - 1, int(bbox.y_min)))
    y1 = max(0, min(height, int(np.ceil(bbox.y_max))))
    if x1 <= x0 or y1 <= y0:
        return None

    patch = depth_image[y0:y1, x0:x1]
    valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < max_depth_m)]
    if valid.size == 0:
        # Fallback for sparse/noisy depth: search around the bbox center with an
        # expanding window and accept the nearest valid depth cluster we can find.
        cx = int(round((x0 + x1) * 0.5))
        cy = int(round((y0 + y1) * 0.5))
        for radius in (3, 6, 12, 24):
            sx0 = max(0, cx - radius)
            sx1 = min(width, cx + radius + 1)
            sy0 = max(0, cy - radius)
            sy1 = min(height, cy + radius + 1)
            sample = depth_image[sy0:sy1, sx0:sx1]
            valid = sample[np.isfinite(sample) & (sample > 0.05) & (sample < max_depth_m)]
            if valid.size:
                # Use a lower quantile to bias toward the foreground subject rather
                # than background pixels that may appear in the search window.
                return float(np.quantile(valid, 0.35))
        return None
    return float(np.median(valid))


def depth_valid_ratio_in_bbox(depth_image: Optional[np.ndarray], bbox: BoundingBox2D, max_depth_m: float) -> float:
    if depth_image is None:
        return 0.0
    height, width = depth_image.shape
    x0 = max(0, min(width - 1, int(bbox.x_min)))
    x1 = max(0, min(width, int(np.ceil(bbox.x_max))))
    y0 = max(0, min(height - 1, int(bbox.y_min)))
    y1 = max(0, min(height, int(np.ceil(bbox.y_max))))
    if x1 <= x0 or y1 <= y0:
        return 0.0

    patch = depth_image[y0:y1, x0:x1]
    total = patch.size
    if total == 0:
        return 0.0
    valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < max_depth_m)]
    return float(valid.size / total)


def bearing_from_bbox(bbox: BoundingBox2D, intrinsics: CameraIntrinsics) -> float:
    x_offset = bbox.center_x - intrinsics.cx
    return float(degrees(atan2(x_offset, intrinsics.fx)))


def angular_width_from_bbox(bbox: BoundingBox2D, intrinsics: CameraIntrinsics) -> float:
    left_bearing = degrees(atan2(bbox.x_min - intrinsics.cx, intrinsics.fx))
    right_bearing = degrees(atan2(bbox.x_max - intrinsics.cx, intrinsics.fx))
    return float(abs(right_bearing - left_bearing))


def lidar_support_in_bearing_window(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    center_bearing_deg: float,
    half_window_deg: float,
) -> Tuple[Optional[float], int, float, float]:
    if not ranges:
        return None, 0, 0.0, 0.0

    min_range: Optional[float] = None
    center_rad = np.radians(center_bearing_deg)
    half_window_rad = np.radians(half_window_deg)
    valid_count = 0
    total_count = 0
    first_angle: Optional[float] = None
    last_angle: Optional[float] = None

    for index, distance in enumerate(ranges):
        angle = angle_min + (index * angle_increment)
        if abs(angle - center_rad) > half_window_rad:
            continue
        total_count += 1
        if distance <= 0.0 or distance == float("inf") or distance != distance:
            continue
        valid_count += 1
        if first_angle is None:
            first_angle = angle
        last_angle = angle
        if min_range is None or distance < min_range:
            min_range = float(distance)

    density = float(valid_count / total_count) if total_count else 0.0
    span_deg = 0.0
    if first_angle is not None and last_angle is not None:
        span_deg = float(abs(np.degrees(last_angle - first_angle)))
    return min_range, valid_count, density, span_deg


def in_path_from_bbox(bbox: BoundingBox2D, intrinsics: CameraIntrinsics, path_band_ratio: float = 0.35) -> bool:
    image_center = intrinsics.width * 0.5
    band_half_width = intrinsics.width * path_band_ratio * 0.5
    return abs(bbox.center_x - image_center) <= band_half_width


def detections_to_observations(
    detections: Iterable[SemanticDetection],
    depth_image: Optional[np.ndarray],
    intrinsics: CameraIntrinsics,
    lidar_ranges: Optional[Sequence[float]] = None,
    lidar_angle_min: float = 0.0,
    lidar_angle_increment: float = 0.0,
    max_depth_m: float = 20.0,
    lidar_window_deg: float = 6.0,
    lidar_consistency_margin_m: float = 1.5,
) -> List[SemanticObservation]:
    observations: List[SemanticObservation] = []

    for detection in detections:
        label = canonicalize_label(detection.label)
        depth_distance = median_depth_in_bbox(depth_image, detection.bbox, max_depth_m=max_depth_m)
        depth_valid_ratio = depth_valid_ratio_in_bbox(depth_image, detection.bbox, max_depth_m=max_depth_m)
        bearing_deg = bearing_from_bbox(detection.bbox, intrinsics)
        bbox_width_ratio = detection.bbox.width / max(float(intrinsics.width), 1.0)
        bbox_height_ratio = detection.bbox.height / max(float(intrinsics.height), 1.0)
        lidar_support_points = 0
        support_density = depth_valid_ratio
        structure_span_deg = angular_width_from_bbox(detection.bbox, intrinsics)

        fused_distance = depth_distance
        if lidar_ranges is not None and lidar_angle_increment > 0.0:
            lidar_half_window_deg = max(lidar_window_deg, structure_span_deg * 0.5)
            lidar_distance, lidar_support_points, lidar_density, lidar_span_deg = lidar_support_in_bearing_window(
                ranges=lidar_ranges,
                angle_min=lidar_angle_min,
                angle_increment=lidar_angle_increment,
                center_bearing_deg=bearing_deg,
                half_window_deg=lidar_half_window_deg,
            )
            if fused_distance is None:
                fused_distance = lidar_distance
            elif lidar_distance is not None and abs(lidar_distance - fused_distance) <= lidar_consistency_margin_m:
                fused_distance = min(fused_distance, lidar_distance)
            support_density = max(depth_valid_ratio, lidar_density)
            structure_span_deg = max(structure_span_deg, lidar_span_deg)

        used_heuristic = False
        if fused_distance is None:
            fused_distance = heuristic_distance_from_bbox(
                label=label,
                bbox=detection.bbox,
                intrinsics=intrinsics,
                max_depth_m=max_depth_m,
            )
            used_heuristic = fused_distance is not None
        if fused_distance is None:
            continue

        observations.append(
            SemanticObservation(
                label=label,
                distance=float(fused_distance),
                bearing_deg=bearing_deg,
                confidence=float(detection.confidence),
                in_path=in_path_from_bbox(detection.bbox, intrinsics),
                source="camera_heuristic" if used_heuristic else detection.source,
                bbox=detection.bbox,
                support_density=float(support_density),
                depth_valid_ratio=float(depth_valid_ratio),
                bbox_width_ratio=float(bbox_width_ratio),
                bbox_height_ratio=float(bbox_height_ratio),
                lidar_support_points=int(lidar_support_points),
                structure_span_deg=float(structure_span_deg),
            )
        )

    return observations
