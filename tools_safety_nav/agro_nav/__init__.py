"""Robot-first agricultural semantic navigation stack."""

from .fusion import CameraIntrinsics, canonicalize_label, detections_to_observations
from .policy import RiskProfile, SemanticReflexController, default_risk_profiles
from .types import (
    BoundingBox2D,
    ControlCommand,
    ReflexStage,
    SemanticDetection,
    SemanticObservation,
    observations_to_payload,
)

__all__ = [
    "BoundingBox2D",
    "CameraIntrinsics",
    "ControlCommand",
    "ReflexStage",
    "RiskProfile",
    "SemanticDetection",
    "SemanticObservation",
    "SemanticReflexController",
    "canonicalize_label",
    "default_risk_profiles",
    "detections_to_observations",
    "observations_to_payload",
]
