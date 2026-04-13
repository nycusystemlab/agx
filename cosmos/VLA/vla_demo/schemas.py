from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .landmark_logic import (
    landmark_display_name,
    normalize_grounding_prompt,
    parse_grounding_phrases,
)
from .topics import CAMERA_IMAGE_TOPIC


class SchemaError(ValueError):
    """Raised when a mission schema is invalid."""


def _require_int(data: dict[str, Any], key: str) -> int:
    value = data.get(key)
    if not isinstance(value, int):
        raise SchemaError(f"'{key}' must be an integer")
    return value


def _require_str(data: dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise SchemaError(f"'{key}' must be a non-empty string")
    return value.strip()


def _optional_str(data: dict[str, Any], key: str) -> str:
    value = data.get(key)
    if value is None:
        return ""
    if not isinstance(value, str):
        raise SchemaError(f"'{key}' must be a string")
    return value.strip()


def _optional_str_list(data: dict[str, Any], key: str) -> list[str]:
    value = data.get(key, [])
    if value is None:
        return []
    if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
        raise SchemaError(f"'{key}' must be a list of strings")
    return [item.strip() for item in value if item.strip()]


def _float_with_default(data: dict[str, Any], key: str, default: float) -> float:
    value = data.get(key, default)
    if not isinstance(value, (int, float)):
        raise SchemaError(f"'{key}' must be numeric")
    return float(value)


def _int_with_default(data: dict[str, Any], key: str, default: int) -> int:
    value = data.get(key, default)
    if not isinstance(value, int):
        raise SchemaError(f"'{key}' must be an integer")
    return value


@dataclass(frozen=True)
class StepSpec:
    step_id: int
    instruction: str
    visual_goal: str
    scene_description: str = ""
    expected_landmarks: list[str] = field(default_factory=list)
    primary_landmark: str = ""
    grounding_objects: list[str] = field(default_factory=list)
    grounding_prompt: str = ""
    control_primitive: str = "move_forward_until_recheck"
    votes_needed: int = 3
    confidence_threshold: float = 0.75
    min_dwell_sec: float = 2.0
    timeout_sec: float = 10.0
    fallback: str = "pause"

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "StepSpec":
        if not isinstance(data, dict):
            raise SchemaError("Step must be a dictionary")
        visual_goal = _require_str(data, "visual_goal")
        expected_landmarks = _optional_str_list(data, "expected_landmarks")
        scene_description = _optional_str(data, "scene_description") or visual_goal
        primary_landmark = _optional_str(data, "primary_landmark")
        if not primary_landmark:
            if expected_landmarks:
                primary_landmark = expected_landmarks[0]
            else:
                primary_landmark = landmark_display_name(visual_goal)
        grounding_objects = _optional_str_list(data, "grounding_objects")
        prompt_source = _optional_str(data, "grounding_prompt")
        if not grounding_objects:
            grounding_objects = parse_grounding_phrases(prompt_source, primary_landmark)
        step = cls(
            step_id=_require_int(data, "step_id"),
            instruction=_require_str(data, "instruction"),
            visual_goal=visual_goal,
            scene_description=scene_description,
            expected_landmarks=expected_landmarks,
            primary_landmark=primary_landmark,
            grounding_objects=grounding_objects,
            grounding_prompt=normalize_grounding_prompt(
                prompt_source or ", ".join(grounding_objects),
                primary_landmark,
            ),
            control_primitive=_optional_str(data, "control_primitive")
            or "move_forward_until_recheck",
            votes_needed=_int_with_default(data, "votes_needed", 3),
            confidence_threshold=_float_with_default(data, "confidence_threshold", 0.75),
            min_dwell_sec=_float_with_default(data, "min_dwell_sec", 2.0),
            timeout_sec=_float_with_default(data, "timeout_sec", 10.0),
            fallback=_optional_str(data, "fallback") or "pause",
        )
        if step.votes_needed < 1:
            raise SchemaError("'votes_needed' must be >= 1")
        if not 0.0 <= step.confidence_threshold <= 1.0:
            raise SchemaError("'confidence_threshold' must be between 0 and 1")
        if step.min_dwell_sec < 0 or step.timeout_sec <= 0:
            raise SchemaError("'min_dwell_sec' must be >= 0 and 'timeout_sec' > 0")
        return step


@dataclass(frozen=True)
class MissionSpec:
    mission_id: str
    mission_text: str
    environment_id: str
    camera_source: str
    inference_interval_sec: float
    steps: list[StepSpec]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MissionSpec":
        if not isinstance(data, dict):
            raise SchemaError("Mission file must contain a dictionary at the root")
        steps_raw = data.get("steps")
        if not isinstance(steps_raw, list) or not steps_raw:
            raise SchemaError("'steps' must be a non-empty list")
        steps = [StepSpec.from_dict(item) for item in steps_raw]
        step_ids = [step.step_id for step in steps]
        if len(step_ids) != len(set(step_ids)):
            raise SchemaError("Step IDs must be unique")
        return cls(
            mission_id=_require_str(data, "mission_id"),
            mission_text=_require_str(data, "mission_text"),
            environment_id=_require_str(data, "environment_id"),
            camera_source=_require_str(data, "camera_source"),
            inference_interval_sec=_float_with_default(
                data, "inference_interval_sec", 1.5
            ),
            steps=steps,
        )

    def get_step(self, step_index: int) -> StepSpec:
        return self.steps[step_index]


@dataclass(frozen=True)
class RouteRequestSpec:
    mission_id: str
    goal_text: str
    environment_id: str
    source_mode: str = "video_file"
    planning_mode: str = "default"
    video_uri: str = ""
    camera_source: str = CAMERA_IMAGE_TOPIC
    clip_duration_sec: float = 3.0
    inference_interval_sec: float = 1.5

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RouteRequestSpec":
        if not isinstance(data, dict):
            raise SchemaError("Route request must be a dictionary")
        source_mode = _optional_str(data, "source_mode") or "video_file"
        if source_mode not in {"video_file", "live_camera"}:
            raise SchemaError("'source_mode' must be 'video_file' or 'live_camera'")
        planning_mode = _optional_str(data, "planning_mode") or "default"
        if planning_mode not in {"default", "environment_video_landmarks"}:
            raise SchemaError(
                "'planning_mode' must be 'default' or 'environment_video_landmarks'"
            )
        video_uri = _optional_str(data, "video_uri")
        if source_mode == "video_file" and not video_uri:
            raise SchemaError("'video_uri' is required for source_mode='video_file'")
        clip_duration_sec = _float_with_default(data, "clip_duration_sec", 3.0)
        if clip_duration_sec <= 0:
            raise SchemaError("'clip_duration_sec' must be > 0")
        inference_interval_sec = _float_with_default(
            data, "inference_interval_sec", 1.5
        )
        if inference_interval_sec <= 0:
            raise SchemaError("'inference_interval_sec' must be > 0")
        return cls(
            mission_id=_require_str(data, "mission_id"),
            goal_text=_require_str(data, "goal_text"),
            environment_id=_require_str(data, "environment_id"),
            source_mode=source_mode,
            planning_mode=planning_mode,
            video_uri=video_uri,
            camera_source=_optional_str(data, "camera_source") or CAMERA_IMAGE_TOPIC,
            clip_duration_sec=clip_duration_sec,
            inference_interval_sec=inference_interval_sec,
        )
