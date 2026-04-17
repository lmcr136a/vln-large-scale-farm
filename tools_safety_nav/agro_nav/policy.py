from __future__ import annotations

from dataclasses import dataclass, replace as _dc_replace
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
    # Minimum confidence required before a class triggers a safety-critical action.
    # General detections use min_confidence (0.25).  Human/animal use a higher floor
    # so a crouching person partially occluded by crops doesn't get misclassified as
    # "crop" and dismissed, but also so a low-confidence ghost detection doesn't
    # permanently block the robot.
    _SAFETY_CRITICAL_FAMILIES   = {"vulnerable", "machinery"}
    _SAFETY_CRITICAL_MIN_CONF   = 0.45   # must be confident before acting on human/tractor

    # EMA alpha for blockage score — smooths the compliance/stop decision across frames.
    # α=0.45 means ~3 frames to settle after a step change (at 30 Hz camera rate).
    # Prevents a single wind-gust frame from flipping the traversal decision.
    _BLOCKAGE_EMA_ALPHA = 0.45

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
        self._blockage_score_ema: float = 0.0  # smoothed corridor blockage score

    def reset(self) -> None:
        self.stage = ReflexStage.STATUS_QUO
        self._recovery_counter = 0
        self._persistent_stop_counter = 0
        self._last_stop_signature = None
        self._adjust_direction = 1.0
        self._blockage_score_ema = 0.0

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
            if item.distance <= 0.0:
                continue
            if abs(item.bearing_deg) > half_fov:
                continue

            profile = self.risk_profiles.get(item.label, self.risk_profiles["unknown"])
            if profile.family in self._SAFETY_CRITICAL_FAMILIES:
                # Safety-critical classes (humans, machinery) require a higher confidence
                # floor before driving any safety response.  A faint/ambiguous detection
                # of a person should not be silently dismissed, but a persistent
                # low-confidence ghost shouldn't freeze the robot either.
                #
                # Two-tier logic:
                #   conf >= _SAFETY_CRITICAL_MIN_CONF  → normal processing
                #   conf in [min_confidence, _SAFETY_CRITICAL_MIN_CONF)
                #       → still admitted but distance is treated conservatively
                #         (halved) so the reflex fires at longer range
                if item.confidence < self.min_confidence:
                    continue
                if item.confidence < self._SAFETY_CRITICAL_MIN_CONF:
                    # Admit with a conservative distance penalty: treat as if twice
                    # as close so caution/stop thresholds trigger earlier.
                    item = _dc_replace(item, distance=item.distance * 0.5)
            else:
                if item.confidence < self.min_confidence:
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

        # In agricultural environments crops legitimately fill the entire camera FOV
        # and have high depth-fill ratios from stem returns.  Using bbox_width_ratio
        # or bbox_height_ratio as disqualifiers would classify every crop row as a
        # rigid obstacle and stop the robot perpetually.
        #
        # Only block compliance for vegetation that shows genuine structural rigidity
        # (angular span consistent with a solid wall) or that remains very dense
        # even at contact distance.  Close vegetation should stay traversable when it
        # still looks like stems/leaves rather than a rigid barrier.
        #   structure_span_deg >= 30°  →  obstacle spans like a solid fence, not stems
        #   density             >= 0.80 →  almost completely solid surface return
        #   contact_rigid                  close + dense + structured, not pliable crop
        density = max(observation.support_density, observation.depth_valid_ratio)
        too_structured = observation.structure_span_deg >= 30.0
        too_rigid      = density >= 0.80
        contact_rigid  = (
            observation.distance <= 0.25
            and density >= 0.85
            and observation.structure_span_deg >= 18.0
        )

        return not (too_structured or too_rigid or contact_rigid)

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
            # No compliant vegetation visible: decay the EMA toward zero so the score
            # doesn't stay artificially elevated after the robot exits a crop row.
            self._blockage_score_ema *= (1.0 - self._BLOCKAGE_EMA_ALPHA)
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

        raw_blockage = (
            0.35 * occupancy
            + 0.20 * density_score
            + 0.15 * structure_score
            + 0.20 * closeness
            + 0.10 * (1.0 - mean_traversability)
        )
        raw_blockage = max(0.0, min(1.0, raw_blockage))

        # EMA across frames: prevents a single wind-gust / shadow frame from flipping
        # the traversal decision.  The smoothed score is what controls stop/speed;
        # the raw score is included in the reason string for debugging.
        self._blockage_score_ema = (
            self._BLOCKAGE_EMA_ALPHA * raw_blockage
            + (1.0 - self._BLOCKAGE_EMA_ALPHA) * self._blockage_score_ema
        )
        blockage_score = self._blockage_score_ema

        # Compliant vegetation should usually be handled as a crawl/avoid case rather
        # than a hard stop.  Reserve STOP for scenes that still look strongly
        # wall-like after the compliance filter has already rejected obvious rigid
        # structures.
        stop = (
            blockage_score >= 0.85
            and density_score >= 0.75
            and structure_score >= 0.45
        )
        speed_scale = max(0.0, min(0.65, mean_traversability * (1.0 - 0.70 * blockage_score)))
        if stop:
            speed_scale = 0.0

        reason = (
            f"compliant_blockage score={blockage_score:.2f}(raw={raw_blockage:.2f}) "
            f"occ={occupancy:.2f} density={density_score:.2f} dist={nearest_distance:.2f}"
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
            # CRAWL = very slow (scale < 0.5); SLOW = reduced but faster (scale >= 0.5).
            action = "CRAWL" if profile.nominal_speed_scale < 0.5 else "SLOW"
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
        # Use 15° bins instead of 5° bins for the stop signature.  At 5° granularity,
        # normal sensor vibration / plant sway shifts the bearing between adjacent bins
        # on consecutive frames, so the counter never accumulates to the trigger
        # threshold and ADJUST never fires in practice.
        signature = (profile.label, int(round(observation.bearing_deg / 15.0)))
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
        # Do NOT exclude centered obstacles: _adjust_command handles bearing ≤ 4° by
        # alternating left/right turns.  No upper bound on the counter — if the robot
        # is still stopped after 5+ cycles it should keep trying to manoeuvre, not
        # give up and sit indefinitely.
        return self._persistent_stop_counter >= 3

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
            # Ramp from ~15% to 100% of nominal speed over recovery_cycles steps.
            # Starting at 0.35 meant a 60% jump immediately after a stop (0.9 m/s at
            # nominal 1.5 m/s).  The gentler base of 0.15 keeps the first step ≈36%
            # so the robot eases back up smoothly rather than lurching forward.
            speed_scale = min(1.0, 0.15 + (self._recovery_counter / max(self.recovery_cycles, 1)) * 0.85)
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
        # Apply a minimum floor so that nearly-centered obstacles still receive a
        # meaningful correction.  With pure linear scaling a 5° offset only produces
        # 0.044 rad/s — essentially zero.  The floor (30% of max_turn_rate) ensures
        # the robot always makes a discernible heading adjustment when steer_away fires.
        normalized = abs(bearing_deg) / 45.0
        magnitude  = self.max_turn_rate * max(0.30, min(1.0, normalized))
        return direction * magnitude
