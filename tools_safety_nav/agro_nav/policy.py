from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

from .types import ControlCommand, ReflexStage, SemanticObservation


@dataclass(frozen=True)
class RiskProfile:
    label: str
    family: str
    caution_distance: float
    stop_distance: float
    nominal_speed_scale: float
    stop_action: str
    steer_away: bool
    priority: float
    base_traversability: float


@dataclass(frozen=True)
class CorridorAssessment:
    score: float
    occupancy: float
    nearest_distance: float
    centroid_bearing_deg: float
    speed_scale: float
    stop: bool
    reason: str


def default_risk_profiles() -> Dict[str, RiskProfile]:
    profiles = [
        RiskProfile("crop", "compliant", 0.9, 0.30, 0.65, "CRAWL", False, 0.20, 0.82),
        RiskProfile("grass", "compliant", 0.7, 0.22, 0.80, "CRAWL", False, 0.15, 0.92),
        RiskProfile("branch", "semi_compliant", 1.2, 0.45, 0.35, "STOP", True, 0.55, 0.25),
        RiskProfile("tree", "rigid", 4.0, 1.2, 0.0, "STOP", True, 0.93, 0.00),
        RiskProfile("human", "vulnerable", 5.0, 1.8, 0.0, "YIELD", True, 1.00, 0.00),
        RiskProfile("animal", "vulnerable", 4.0, 1.5, 0.0, "YIELD", True, 0.97, 0.00),
        RiskProfile("tractor", "machinery", 8.0, 3.0, 0.0, "STOP", True, 0.99, 0.00),
        RiskProfile("harvester", "machinery", 8.0, 3.5, 0.0, "STOP", True, 0.99, 0.00),
        RiskProfile("vehicle", "machinery", 6.0, 2.5, 0.0, "STOP", True, 0.94, 0.00),
        RiskProfile("fence", "rigid", 3.0, 0.9, 0.15, "STOP", True, 0.82, 0.00),
        RiskProfile("rock", "rigid", 2.5, 0.8, 0.20, "STOP", True, 0.78, 0.00),
        RiskProfile("pole", "rigid", 2.5, 0.8, 0.15, "STOP", True, 0.80, 0.00),
        RiskProfile("unknown", "unknown", 3.0, 1.0, 0.15, "STOP", True, 0.88, 0.05),
    ]
    return {item.label: item for item in profiles}


class SemanticReflexController:
    def __init__(
        self,
        nominal_linear_speed: float = 0.8,
        recovery_cycles: int = 4,
        max_turn_rate: float = 0.6,
        frontal_fov_deg: float = 100.0,
        min_confidence: float = 0.25,
        risk_profiles: Optional[Dict[str, RiskProfile]] = None,
    ) -> None:
        self.nominal_linear_speed = nominal_linear_speed
        self.recovery_cycles = recovery_cycles
        self.max_turn_rate = max_turn_rate
        self.frontal_fov_deg = frontal_fov_deg
        self.min_confidence = min_confidence
        self.risk_profiles = risk_profiles or default_risk_profiles()
        self.stage = ReflexStage.STATUS_QUO
        self._recovery_counter = 0
        self._persistent_stop_counter = 0
        self._last_stop_signature: Optional[tuple[str, int]] = None
        self._adjust_direction = 1.0

    def reset(self) -> None:
        self.stage = ReflexStage.STATUS_QUO
        self._recovery_counter = 0
        self._persistent_stop_counter = 0
        self._last_stop_signature = None
        self._adjust_direction = 1.0

    def decide(self, observations: Sequence[SemanticObservation]) -> ControlCommand:
        relevant = self._filter_relevant(list(observations))
        corridor_assessment = self._assess_compliant_corridor(relevant)
        dominant = self._select_dominant([item for item in relevant if not self._passes_compliance_check(item)])

        if dominant is None and corridor_assessment is None:
            self._clear_stop_memory()
            return self._recovery_or_go()

        if dominant is not None:
            profile = self.risk_profiles.get(dominant.label, self.risk_profiles["unknown"])
            threat_score = self._threat_score(dominant, profile)
            self.stage = ReflexStage.REACT
            self._recovery_counter = 0

            # Vulnerable obstacles should force a stop when they are clearly large
            # and centered in the scene, even if monocular/depth distance is noisy.
            if self._should_force_stop(dominant, profile):
                return self._command_from_profile(dominant, profile, threat_score, stop=True)
            if dominant.distance <= profile.stop_distance:
                return self._command_from_profile(dominant, profile, threat_score, stop=True)
            if dominant.distance <= profile.caution_distance:
                self._clear_stop_memory()
                return self._command_from_profile(dominant, profile, threat_score, stop=False)

        if corridor_assessment is not None and corridor_assessment.score >= 0.18:
            self._clear_stop_memory()
            return self._command_from_corridor(corridor_assessment)

        if dominant is None:
            self._clear_stop_memory()
            return self._recovery_or_go()
        self._clear_stop_memory()
        return self._recovery_or_go()

    def _filter_relevant(self, observations: List[SemanticObservation]) -> List[SemanticObservation]:
        filtered: List[SemanticObservation] = []
        half_fov = self.frontal_fov_deg * 0.5
        for item in observations:
            if item.confidence < self.min_confidence:
                continue
            if item.distance <= 0.0:
                continue
            if abs(item.bearing_deg) > half_fov:
                continue
            filtered.append(item)
        return filtered

    def _select_dominant(self, observations: List[SemanticObservation]) -> Optional[SemanticObservation]:
        best_item: Optional[SemanticObservation] = None
        best_score = -1.0
        for item in observations:
            profile = self.risk_profiles.get(item.label, self.risk_profiles["unknown"])
            score = self._threat_score(item, profile)
            if score > best_score:
                best_item = item
                best_score = score
        return best_item

    def _is_compliant(self, label: str) -> bool:
        profile = self.risk_profiles.get(label, self.risk_profiles["unknown"])
        return profile.family == "compliant"

    def _passes_compliance_check(self, observation: SemanticObservation) -> bool:
        if not self._is_compliant(observation.label):
            return False

        # Compliance is not semantic label alone. A "crop"/"grass" observation only
        # counts as traversable if it also looks sparse, narrow, and non-rigid.
        density = max(observation.support_density, observation.depth_valid_ratio)
        too_dense = density >= 0.55
        too_wide = observation.bbox_width_ratio >= 0.32
        too_tall = observation.bbox_height_ratio >= 0.45
        too_structured = observation.structure_span_deg >= 20.0
        too_close = observation.distance <= 0.75

        return not (too_dense or too_wide or too_tall or too_structured or too_close)

    def _threat_score(self, observation: SemanticObservation, profile: RiskProfile) -> float:
        distance_term = max(0.0, profile.caution_distance - observation.distance) / max(profile.caution_distance, 1e-6)
        path_term = 1.0 if observation.in_path else 0.35
        bearing_term = max(0.2, 1.0 - abs(observation.bearing_deg) / max(self.frontal_fov_deg * 0.5, 1e-6))
        return profile.priority * observation.confidence * path_term * bearing_term * (0.35 + distance_term)

    def _should_force_stop(self, observation: SemanticObservation, profile: RiskProfile) -> bool:
        if self._passes_compliance_check(observation):
            return False
        if not observation.in_path:
            return False

        # Any non-compliant obstacle that is centered and clearly near in frame
        # should force a stop, even if depth is noisy.
        close_in_frame = (
            observation.bbox_height_ratio >= 0.35
            or observation.bbox_width_ratio >= 0.22
        )
        centered = abs(observation.bearing_deg) <= 20.0
        high_conf = observation.confidence >= 0.30

        # Also stop if range says it's in a close protection bubble.
        close_in_distance = observation.distance <= max(profile.stop_distance, 1.5)

        return high_conf and centered and (close_in_frame or close_in_distance)

    def _assess_compliant_corridor(self, observations: List[SemanticObservation]) -> Optional[CorridorAssessment]:
        compliant = [item for item in observations if self._passes_compliance_check(item) and item.in_path]
        if not compliant:
            return None

        nearest_distance = min(item.distance for item in compliant)
        weighted_widths = []
        weighted_bearings = []
        density_values = []
        traversabilities = []
        structure_values = []

        for item in compliant:
            profile = self.risk_profiles.get(item.label, self.risk_profiles["unknown"])
            density = max(item.support_density, item.depth_valid_ratio)
            width = min(1.0, item.bbox_width_ratio / 0.35)
            structure = min(1.0, item.structure_span_deg / 18.0)
            closeness = max(0.0, profile.caution_distance - item.distance) / max(profile.caution_distance, 1e-6)
            traversability = profile.base_traversability
            traversability *= 1.0 - 0.55 * density
            traversability *= 1.0 - 0.40 * width
            traversability *= 1.0 - 0.35 * structure
            traversability *= 1.0 - 0.20 * closeness
            traversability = max(0.0, min(1.0, traversability))

            occupancy_contrib = min(0.55, item.bbox_width_ratio * (0.45 + 0.55 * density))
            weighted_widths.append(occupancy_contrib)
            weighted_bearings.append(item.bearing_deg * max(occupancy_contrib, 0.05))
            density_values.append(density)
            traversabilities.append(traversability)
            structure_values.append(structure)

        occupancy = min(1.0, sum(weighted_widths))
        density_score = max(density_values)
        mean_traversability = sum(traversabilities) / max(len(traversabilities), 1)
        structure_score = max(structure_values)
        nearest_profile = self.risk_profiles.get(min(compliant, key=lambda item: item.distance).label, self.risk_profiles["unknown"])
        closeness = max(0.0, nearest_profile.caution_distance - nearest_distance) / max(nearest_profile.caution_distance, 1e-6)
        centroid_bearing_deg = 0.0
        if weighted_widths and sum(abs(value) for value in weighted_widths) > 0.0:
            centroid_bearing_deg = sum(weighted_bearings) / max(sum(weighted_widths), 1e-6)

        blockage_score = (
            0.35 * occupancy
            + 0.20 * density_score
            + 0.15 * structure_score
            + 0.20 * closeness
            + 0.10 * (1.0 - mean_traversability)
        )
        blockage_score = max(0.0, min(1.0, blockage_score))

        stop = blockage_score >= 0.60 or (occupancy >= 0.60 and nearest_distance <= 1.2)
        speed_scale = max(0.0, min(0.65, mean_traversability * (1.0 - 0.70 * blockage_score)))
        if stop:
            speed_scale = 0.0

        reason = (
            f"compliant_blockage score={blockage_score:.2f} occ={occupancy:.2f} "
            f"density={density_score:.2f} dist={nearest_distance:.2f}"
        )
        return CorridorAssessment(
            score=blockage_score,
            occupancy=occupancy,
            nearest_distance=nearest_distance,
            centroid_bearing_deg=centroid_bearing_deg,
            speed_scale=speed_scale,
            stop=stop,
            reason=reason,
        )

    def _command_from_profile(
        self,
        observation: SemanticObservation,
        profile: RiskProfile,
        threat_score: float,
        stop: bool,
    ) -> ControlCommand:
        if stop:
            if self._should_adjust_around(observation, profile):
                return self._adjust_command(observation, profile, threat_score)
            action = profile.stop_action
            linear_x = 0.0
        else:
            self._clear_stop_memory()
            action = "SLOW" if profile.nominal_speed_scale < 0.5 else "CRAWL"
            linear_x = self.nominal_linear_speed * profile.nominal_speed_scale

        angular_z = 0.0
        if profile.steer_away and abs(observation.bearing_deg) > 4.0:
            angular_z = self._steer_away_from_bearing(observation.bearing_deg)

        return ControlCommand(
            action=action,
            stage=self.stage.value,
            target_linear_x=linear_x,
            target_angular_z=angular_z,
            stop=stop or action in {"STOP", "YIELD"},
            reason=f"{profile.label}@{observation.distance:.2f}m",
            dominant_label=profile.label,
            threat_distance=observation.distance,
            threat_score=threat_score,
        )

    def _should_adjust_around(self, observation: SemanticObservation, profile: RiskProfile) -> bool:
        signature = (profile.label, int(round(observation.bearing_deg / 5.0)))
        if signature == self._last_stop_signature:
            self._persistent_stop_counter += 1
        else:
            self._last_stop_signature = signature
            self._persistent_stop_counter = 1

        if profile.family in {"vulnerable", "machinery"}:
            return False
        if observation.relative_speed > 0.15:
            return False
        if observation.distance <= max(profile.stop_distance + 0.25, 0.9):
            return False
        if abs(observation.bearing_deg) > 35.0:
            return False
        if abs(observation.bearing_deg) < 8.0:
            return False
        return 3 <= self._persistent_stop_counter <= 5

    def _adjust_command(
        self,
        observation: SemanticObservation,
        profile: RiskProfile,
        threat_score: float,
    ) -> ControlCommand:
        if abs(observation.bearing_deg) <= 4.0:
            self._adjust_direction *= -1.0
            angular_z = self.max_turn_rate * 0.6 * self._adjust_direction
        else:
            angular_z = self._steer_away_from_bearing(observation.bearing_deg)

        linear_x = min(
            self.nominal_linear_speed * max(profile.nominal_speed_scale, 0.18),
            self.nominal_linear_speed * 0.25,
        )
        self._clear_stop_memory()

        return ControlCommand(
            action="ADJUST",
            stage=self.stage.value,
            target_linear_x=linear_x,
            target_angular_z=angular_z,
            stop=False,
            reason=f"persistent_{profile.label}@{observation.distance:.2f}m",
            dominant_label=profile.label,
            threat_distance=observation.distance,
            threat_score=threat_score,
        )

    def _clear_stop_memory(self) -> None:
        self._persistent_stop_counter = 0
        self._last_stop_signature = None

    def _command_from_corridor(self, assessment: CorridorAssessment) -> ControlCommand:
        self.stage = ReflexStage.REACT
        self._recovery_counter = 0
        self._clear_stop_memory()

        if assessment.stop:
            return ControlCommand(
                action="STOP",
                stage=self.stage.value,
                target_linear_x=0.0,
                target_angular_z=0.0,
                stop=True,
                reason=assessment.reason,
                dominant_label="compliant_corridor",
                threat_distance=assessment.nearest_distance,
                threat_score=assessment.score,
            )

        angular_z = 0.0
        action = "CRAWL"
        if assessment.occupancy >= 0.22 and abs(assessment.centroid_bearing_deg) >= 6.0:
            angular_z = self._steer_away_from_bearing(assessment.centroid_bearing_deg)
            action = "AVOID"

        return ControlCommand(
            action=action,
            stage=self.stage.value,
            target_linear_x=self.nominal_linear_speed * assessment.speed_scale,
            target_angular_z=angular_z,
            stop=False,
            reason=assessment.reason,
            dominant_label="compliant_corridor",
            threat_distance=assessment.nearest_distance,
            threat_score=assessment.score,
        )

    def _recovery_or_go(self) -> ControlCommand:
        if self.stage == ReflexStage.REACT and self._recovery_counter < self.recovery_cycles:
            self.stage = ReflexStage.RECOVERY

        if self.stage == ReflexStage.RECOVERY:
            self._recovery_counter += 1
            speed_scale = min(1.0, 0.35 + (self._recovery_counter / max(self.recovery_cycles, 1)) * 0.65)
            if self._recovery_counter >= self.recovery_cycles:
                self.stage = ReflexStage.STATUS_QUO
            return ControlCommand(
                action="RECOVER",
                stage=ReflexStage.RECOVERY.value,
                target_linear_x=self.nominal_linear_speed * speed_scale,
                target_angular_z=0.0,
                stop=False,
                reason="recovery_ramp",
                dominant_label=None,
                threat_distance=None,
                threat_score=0.0,
            )

        self.stage = ReflexStage.STATUS_QUO
        self._recovery_counter = 0
        return ControlCommand(
            action="GO",
            stage=ReflexStage.STATUS_QUO.value,
            target_linear_x=self.nominal_linear_speed,
            target_angular_z=0.0,
            stop=False,
            reason="path_clear",
            dominant_label=None,
            threat_distance=None,
            threat_score=0.0,
        )

    def _steer_away_from_bearing(self, bearing_deg: float) -> float:
        direction = -1.0 if bearing_deg > 0.0 else 1.0
        magnitude = min(self.max_turn_rate, self.max_turn_rate * abs(bearing_deg) / 45.0)
        return direction * magnitude
