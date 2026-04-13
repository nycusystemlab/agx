from __future__ import annotations

import re
from typing import Any


def canonicalize_landmark(value: str) -> str:
    text = value.strip().lower().replace("_", " ").replace("-", " ")
    text = re.sub(r"\b(a|an|the)\b", " ", text)
    text = re.sub(r"[^a-z0-9 ]+", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def landmark_display_name(value: str) -> str:
    text = value.strip().replace("_", " ").replace("-", " ")
    return re.sub(r"\s+", " ", text).strip()


def parse_grounding_phrases(prompt: str | None, fallback_landmark: str = "") -> list[str]:
    source = (prompt or "").strip()
    phrases: list[str] = []
    if source:
        for part in re.split(r"[;\n,]+", source):
            for chunk in part.split("."):
                cleaned = re.sub(r"\s+", " ", chunk).strip(" ,")
                if cleaned:
                    phrases.append(cleaned)
    if not phrases:
        fallback = landmark_display_name(fallback_landmark)
        if fallback:
            phrases.append(fallback)
    return phrases


def normalize_grounding_prompt(prompt: str | None, fallback_landmark: str) -> str:
    phrases = parse_grounding_phrases(prompt, fallback_landmark)
    normalized: list[str] = []
    for phrase in phrases:
        final_phrase = re.sub(r"\s+", " ", phrase).strip()
        if not final_phrase:
            continue
        normalized.append(f"{final_phrase.rstrip('.')}.")
    return " ".join(normalized)


def class_matches_target(class_id: str, target_landmark: str) -> bool:
    class_key = canonicalize_landmark(class_id)
    target_key = canonicalize_landmark(target_landmark)
    if not class_key or not target_key:
        return False
    return class_key == target_key


def select_best_detection(
    detections: list[dict[str, Any]],
    target_landmark: str,
) -> dict[str, Any] | None:
    matches = [
        detection
        for detection in detections
        if class_matches_target(str(detection.get("class_id", "")), target_landmark)
    ]
    if not matches:
        return None
    return max(matches, key=lambda item: float(item.get("score", 0.0)))


def build_landmark_detection_payload(
    *,
    mission_id: str,
    step_id: int,
    target_landmark: str,
    best_detection: dict[str, Any] | None,
    image_frame: str,
    stamp: float,
) -> dict[str, Any]:
    if best_detection is None:
        return {
            "mission_id": mission_id,
            "step_id": step_id,
            "target_landmark": target_landmark,
            "found": False,
            "class_id": "",
            "score": 0.0,
            "bbox": {"cx": 0.0, "cy": 0.0, "w": 0.0, "h": 0.0},
            "image_frame": image_frame,
            "stamp": float(stamp),
        }
    return {
        "mission_id": mission_id,
        "step_id": step_id,
        "target_landmark": target_landmark,
        "found": True,
        "class_id": str(best_detection.get("class_id", "")),
        "score": float(best_detection.get("score", 0.0)),
        "bbox": {
            "cx": float(best_detection.get("cx", 0.0)),
            "cy": float(best_detection.get("cy", 0.0)),
            "w": float(best_detection.get("w", 0.0)),
            "h": float(best_detection.get("h", 0.0)),
        },
        "image_frame": image_frame,
        "stamp": float(stamp),
    }


def evaluate_landmark_detection(
    current_step_payload: dict[str, Any],
    landmark_detection_payload: dict[str, Any],
) -> dict[str, Any]:
    step = current_step_payload.get("step") or {}
    step_id = int(step.get("step_id", current_step_payload.get("step_id", -1)))
    confidence_threshold = float(step.get("confidence_threshold", 0.75))
    found = bool(landmark_detection_payload.get("found", False))
    score = float(landmark_detection_payload.get("score", 0.0))
    target_landmark = str(landmark_detection_payload.get("target_landmark", "")).strip()
    class_id = str(landmark_detection_payload.get("class_id", "")).strip()

    if not found:
        return {
            "step_id": step_id,
            "step_match": False,
            "step_completed": False,
            "observed_landmarks": [],
            "confidence": 0.0,
            "reason": f"landmark_not_found:{target_landmark or 'unknown'}",
        }
    if score < confidence_threshold:
        return {
            "step_id": step_id,
            "step_match": False,
            "step_completed": False,
            "observed_landmarks": [class_id] if class_id else [],
            "confidence": score,
            "reason": (
                f"landmark_score_below_threshold:{target_landmark or class_id or 'unknown'}"
            ),
        }
    observed_landmarks = [class_id or target_landmark] if (class_id or target_landmark) else []
    return {
        "step_id": step_id,
        "step_match": True,
        "step_completed": True,
        "observed_landmarks": observed_landmarks,
        "confidence": score,
        "reason": f"landmark_detected:{target_landmark or class_id or 'unknown'}",
    }
