from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .prompting import build_reasoner_prompt
from .schemas import MissionSpec, StepSpec


TERMINAL_STATES = {"DONE", "ABORTED", "ERROR"}


@dataclass
class MissionStateMachine:
    robot_status_timeout_sec: float = 2.5
    inference_stale_multiplier: float = 3.0

    def __post_init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.mission: MissionSpec | None = None
        self.mission_state = "IDLE"
        self.current_step_index = 0
        self.current_votes = 0
        self.mission_started_at: float | None = None
        self.step_started_at: float | None = None
        self.last_robot_status: dict[str, Any] | None = None
        self.last_robot_status_at: float | None = None
        self.last_inference: dict[str, Any] | None = None
        self.last_inference_at: float | None = None
        self.last_reason = "idle"
        self.command_revision = 0

    @property
    def has_active_mission(self) -> bool:
        return self.mission is not None

    def current_step(self) -> StepSpec | None:
        if self.mission is None:
            return None
        if not 0 <= self.current_step_index < len(self.mission.steps):
            return None
        return self.mission.get_step(self.current_step_index)

    def start_mission(self, mission: MissionSpec, now: float) -> None:
        self.reset()
        self.mission = mission
        self.mission_started_at = now
        self.step_started_at = now
        self.mission_state = "WAITING"
        self.current_votes = 0
        self.last_reason = "mission_started"
        self.command_revision = 1

    def abort(self, reason: str, now: float) -> None:
        if self.mission is None:
            return
        self.mission_state = "ABORTED"
        self.last_reason = reason
        self.command_revision += 1
        if self.step_started_at is None:
            self.step_started_at = now

    def handle_robot_status(self, payload: dict[str, Any], now: float) -> None:
        self.last_robot_status = payload
        self.last_robot_status_at = now
        executor_state = str(payload.get("executor_state", "")).upper()
        if executor_state == "ERROR":
            self.abort(payload.get("reason", "executor_error"), now)
            return
        if self.mission_state == "WAITING" and self._robot_status_fresh(now):
            if self._inference_fresh(now):
                self.mission_state = "ACTIVE"

    def handle_inference(self, payload: dict[str, Any], now: float) -> None:
        step = self.current_step()
        if step is None or self.mission_state in TERMINAL_STATES | {"PAUSED"}:
            return
        self.last_inference = payload
        self.last_inference_at = now
        if self._robot_status_fresh(now):
            self.mission_state = "ACTIVE"
        else:
            self.mission_state = "WAITING"
        if int(payload.get("step_id", -1)) != step.step_id:
            return
        confidence = float(payload.get("confidence", 0.0))
        step_match = bool(payload.get("step_match", False))
        step_completed = bool(payload.get("step_completed", False))
        self.last_reason = str(payload.get("reason", "inference"))
        if not step_match or confidence < step.confidence_threshold:
            self.current_votes = 0
            return
        if step_completed:
            self.current_votes += 1
        else:
            self.current_votes = 0
        if self.step_started_at is None:
            self.step_started_at = now
        dwell_satisfied = (now - self.step_started_at) >= step.min_dwell_sec
        if dwell_satisfied and self.current_votes >= step.votes_needed:
            self._advance_step(now)

    def tick(self, now: float) -> None:
        step = self.current_step()
        if step is None or self.mission_state in TERMINAL_STATES:
            return
        if not self._robot_status_fresh(now):
            self.mission_state = "WAITING"
        elif self._inference_fresh(now):
            if self.mission_state == "WAITING":
                self.mission_state = "ACTIVE"
        else:
            self.mission_state = "WAITING"
        if self.step_started_at is not None and (now - self.step_started_at) > step.timeout_sec:
            self._apply_fallback(step, now)

    def active_control_primitive(self) -> str:
        if self.mission_state in {"PAUSED", "DONE", "ABORTED", "ERROR", "IDLE"}:
            return "stop_and_hold"
        step = self.current_step()
        return step.control_primitive if step else "stop_and_hold"

    def build_current_step_payload(self) -> dict[str, Any]:
        step = self.current_step()
        mission_id = self.mission.mission_id if self.mission else ""
        return {
            "mission_id": mission_id,
            "mission_state": self.mission_state,
            "step_index": self.current_step_index,
            "step": None if step is None else {
                "step_id": step.step_id,
                "instruction": step.instruction,
                "visual_goal": step.visual_goal,
                "scene_description": step.scene_description,
                "expected_landmarks": step.expected_landmarks,
                "primary_landmark": step.primary_landmark,
                "grounding_objects": step.grounding_objects,
                "grounding_prompt": step.grounding_prompt,
                "control_primitive": step.control_primitive,
                "votes_needed": step.votes_needed,
                "confidence_threshold": step.confidence_threshold,
                "min_dwell_sec": step.min_dwell_sec,
                "timeout_sec": step.timeout_sec,
                "fallback": step.fallback,
            },
            "current_votes": self.current_votes,
            "last_reason": self.last_reason,
        }

    def build_prompt_payload(self) -> dict[str, Any]:
        if self.mission is None:
            return {}
        step = self.current_step()
        if step is None:
            return {}
        return {
            "mission_id": self.mission.mission_id,
            "environment_id": self.mission.environment_id,
            "camera_source": self.mission.camera_source,
            "step_id": step.step_id,
            "instruction": step.instruction,
            "visual_goal": step.visual_goal,
            "scene_description": step.scene_description,
            "expected_landmarks": step.expected_landmarks,
            "primary_landmark": step.primary_landmark,
            "grounding_objects": step.grounding_objects,
            "grounding_prompt": step.grounding_prompt,
            "confidence_threshold": step.confidence_threshold,
            "votes_needed": step.votes_needed,
            "prompt_text": build_reasoner_prompt(self.mission, step),
        }

    def build_inference_interval_payload(self) -> dict[str, Any]:
        if self.mission is None:
            return {}
        return {
            "mission_id": self.mission.mission_id,
            "seconds": self.mission.inference_interval_sec,
        }

    def build_control_command_payload(self) -> dict[str, Any]:
        step = self.current_step()
        mission_id = self.mission.mission_id if self.mission else ""
        return {
            "mission_id": mission_id,
            "command_id": f"{mission_id}:{self.command_revision}",
            "mission_state": self.mission_state,
            "step_id": None if step is None else step.step_id,
            "control_primitive": self.active_control_primitive(),
            "timeout_sec": None if step is None else step.timeout_sec,
            "instruction": None if step is None else step.instruction,
        }

    def build_mission_state_payload(self, now: float) -> dict[str, Any]:
        step = self.current_step()
        return {
            "mission_id": self.mission.mission_id if self.mission else "",
            "mission_state": self.mission_state,
            "current_step_id": None if step is None else step.step_id,
            "current_step_index": self.current_step_index,
            "current_votes": self.current_votes,
            "last_reason": self.last_reason,
            "last_robot_status": self.last_robot_status,
            "last_inference": self.last_inference,
            "robot_status_fresh": self._robot_status_fresh(now),
            "inference_fresh": self._inference_fresh(now),
        }

    def _advance_step(self, now: float) -> None:
        assert self.mission is not None
        self.current_votes = 0
        self.command_revision += 1
        if self.current_step_index >= len(self.mission.steps) - 1:
            self.mission_state = "DONE"
            self.last_reason = "mission_completed"
            return
        self.current_step_index += 1
        self.step_started_at = now
        self.last_reason = "step_completed"
        if self._robot_status_fresh(now) and self._inference_fresh(now):
            self.mission_state = "ACTIVE"
        else:
            self.mission_state = "WAITING"

    def _apply_fallback(self, step: StepSpec, now: float) -> None:
        fallback = step.fallback.strip().lower()
        self.current_votes = 0
        self.command_revision += 1
        self.last_reason = f"step_timeout:{step.step_id}"
        if fallback in {"pause", "stop_and_hold", "wait_for_operator", "needs_operator"}:
            self.mission_state = "PAUSED"
        elif fallback in {"abort", "abort_mission"}:
            self.mission_state = "ABORTED"
        else:
            self.mission_state = "PAUSED"
        if self.step_started_at is None:
            self.step_started_at = now

    def _robot_status_fresh(self, now: float) -> bool:
        return (
            self.last_robot_status_at is not None
            and (now - self.last_robot_status_at) <= self.robot_status_timeout_sec
        )

    def _inference_fresh(self, now: float) -> bool:
        if self.mission is None or self.step_started_at is None:
            return False
        stale_timeout = max(
            3.0, self.mission.inference_interval_sec * self.inference_stale_multiplier
        )
        if self.last_inference_at is None:
            return (now - self.step_started_at) <= stale_timeout
        return (now - self.last_inference_at) <= stale_timeout
