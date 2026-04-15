from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any, Dict, List, Optional


class ReflexStage(str, Enum):
    STATUS_QUO = "STATUS_QUO"
    REACT = "REACT"
    RECOVERY = "RECOVERY"


@dataclass(frozen=True)
class BoundingBox2D:
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def center_x(self) -> float:
        return (self.x_min + self.x_max) * 0.5

    @property
    def center_y(self) -> float:
        return (self.y_min + self.y_max) * 0.5

    @property
    def width(self) -> float:
        return max(0.0, self.x_max - self.x_min)

    @property
    def height(self) -> float:
        return max(0.0, self.y_max - self.y_min)

    @property
    def area(self) -> float:
        return self.width * self.height


@dataclass(frozen=True)
class SemanticDetection:
    label: str
    confidence: float
    bbox: BoundingBox2D
    track_id: Optional[int] = None
    source: str = "yolo"


@dataclass(frozen=True)
class SemanticObservation:
    label: str
    distance: float
    bearing_deg: float
    confidence: float
    in_path: bool
    relative_speed: float = 0.0
    source: str = "camera"
    bbox: Optional[BoundingBox2D] = None
    support_density: float = 0.0
    depth_valid_ratio: float = 0.0
    bbox_width_ratio: float = 0.0
    bbox_height_ratio: float = 0.0
    lidar_support_points: int = 0
    structure_span_deg: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        payload = asdict(self)
        if self.bbox is None:
            payload.pop("bbox", None)
        return payload


@dataclass(frozen=True)
class ControlCommand:
    action: str
    stage: str
    target_linear_x: float
    target_angular_z: float
    stop: bool
    reason: str
    dominant_label: Optional[str]
    threat_distance: Optional[float]
    threat_score: float

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def observations_to_payload(observations: List[SemanticObservation]) -> Dict[str, Any]:
    return {"observations": [item.to_dict() for item in observations]}
