from __future__ import annotations

from typing import Any

from .landmark_logic import (
    landmark_display_name,
    normalize_grounding_prompt,
    parse_grounding_phrases,
)
from .schemas import RouteRequestSpec, SchemaError
from .topics import CAMERA_IMAGE_TOPIC

PLACEHOLDER_VALUES = {
    "",
    "...",
    "…",
    "-",
    "--",
    "unknown",
    "n/a",
    "na",
    "none",
    "null",
    "tbd",
    "same as above",
}


def _optional_str(data: dict[str, Any], key: str) -> str:
    value = data.get(key)
    if value is None:
        return ""
    if not isinstance(value, str):
        raise SchemaError(f"'{key}' must be a string when provided")
    return value.strip()


def _optional_str_list(data: dict[str, Any], key: str) -> list[str]:
    value = data.get(key, [])
    if value is None:
        return []
    if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
        raise SchemaError(f"'{key}' must be a list of strings when provided")
    return [item.strip() for item in value if item.strip()]


def _is_placeholder_text(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in PLACEHOLDER_VALUES:
        return True
    if normalized.isdigit():
        return True
    return False


def _meaningful_or_fallback(value: str, fallback: str) -> str:
    return fallback if _is_placeholder_text(value) else value


def _meaningful_list(values: list[str]) -> list[str]:
    return [value for value in values if not _is_placeholder_text(value)]


def coerce_route_plan_payload(
    raw_payload: dict[str, Any],
    request: RouteRequestSpec,
) -> dict[str, Any]:
    if not isinstance(raw_payload, dict):
        raise SchemaError("Route plan output must be a JSON object")
    steps_raw = raw_payload.get("steps")
    if not isinstance(steps_raw, list) or not steps_raw:
        raise SchemaError("'steps' must be a non-empty list")

    mission_text = _meaningful_or_fallback(
        _optional_str(raw_payload, "mission_text"),
        request.goal_text,
    )
    camera_source = request.camera_source
    if not camera_source:
        camera_source = CAMERA_IMAGE_TOPIC

    normalized_steps: list[dict[str, Any]] = []
    for index, step_raw in enumerate(steps_raw, start=1):
        if not isinstance(step_raw, dict):
            raise SchemaError("Each route plan step must be a JSON object")
        instruction = _optional_str(step_raw, "instruction")
        visual_goal = _optional_str(step_raw, "visual_goal")
        if _is_placeholder_text(instruction):
            raise SchemaError(f"Step {index} has placeholder 'instruction'")
        if _is_placeholder_text(visual_goal):
            raise SchemaError(f"Step {index} has placeholder 'visual_goal'")
        if not instruction:
            raise SchemaError(f"Step {index} is missing 'instruction'")
        if not visual_goal:
            raise SchemaError(f"Step {index} is missing 'visual_goal'")
        expected_landmarks = _meaningful_list(_optional_str_list(step_raw, "expected_landmarks"))
        scene_description = _meaningful_or_fallback(
            _optional_str(step_raw, "scene_description"),
            visual_goal,
        )
        primary_landmark = _optional_str(step_raw, "primary_landmark")
        if _is_placeholder_text(primary_landmark):
            primary_landmark = ""
        if not primary_landmark:
            if expected_landmarks:
                primary_landmark = expected_landmarks[0]
            else:
                primary_landmark = landmark_display_name(visual_goal)
        grounding_objects = _meaningful_list(_optional_str_list(step_raw, "grounding_objects"))
        prompt_source = _optional_str(step_raw, "grounding_prompt")
        if not grounding_objects:
            grounding_objects = parse_grounding_phrases(prompt_source, primary_landmark)
        grounding_prompt = normalize_grounding_prompt(
            prompt_source or ", ".join(grounding_objects),
            primary_landmark,
        )
        step_id = step_raw.get("step_id", index)
        if not isinstance(step_id, int):
            raise SchemaError(f"Step {index} has invalid 'step_id'")
        normalized_steps.append(
            {
                "step_id": step_id,
                "instruction": instruction,
                "visual_goal": visual_goal,
                "scene_description": scene_description,
                "expected_landmarks": expected_landmarks,
                "primary_landmark": primary_landmark,
                "grounding_objects": grounding_objects,
                "grounding_prompt": grounding_prompt,
                "control_primitive": _optional_str(step_raw, "control_primitive")
                or "move_forward_until_recheck",
                "votes_needed": step_raw.get("votes_needed", 3),
                "confidence_threshold": step_raw.get("confidence_threshold", 0.75),
                "min_dwell_sec": step_raw.get("min_dwell_sec", 2.0),
                "timeout_sec": step_raw.get("timeout_sec", 12.0),
                "fallback": _optional_str(step_raw, "fallback") or "pause",
            }
        )

    inference_interval_sec = raw_payload.get(
        "inference_interval_sec", request.inference_interval_sec
    )
    if not isinstance(inference_interval_sec, (int, float)):
        raise SchemaError("'inference_interval_sec' must be numeric")

    return {
        "mission_id": request.mission_id,
        "mission_text": mission_text,
        "environment_id": request.environment_id,
        "camera_source": camera_source,
        "planning_mode": request.planning_mode,
        "inference_interval_sec": float(request.inference_interval_sec),
        "steps": normalized_steps,
    }
