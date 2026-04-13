from __future__ import annotations

from .landmark_logic import landmark_display_name
from .schemas import MissionSpec, RouteRequestSpec, StepSpec


def build_reasoner_prompt(mission: MissionSpec, step: StepSpec) -> str:
    landmarks = ", ".join(step.expected_landmarks) if step.expected_landmarks else "none"
    return (
        "You are verifying the current AMR mission step from a recent camera clip. "
        "Judge only the active step and return strict JSON. "
        f"Mission: {mission.mission_text}. "
        f"Current step_id: {step.step_id}. "
        f"Instruction: {step.instruction}. "
        f"Visual goal: {step.visual_goal}. "
        f"Scene description: {step.scene_description}. "
        f"Expected landmarks: {landmarks}. "
        f"Primary landmark: {step.primary_landmark}. "
        "Return exactly one JSON object with keys "
        "step_id, step_match, step_completed, observed_landmarks, confidence, reason. "
        "Do not include markdown fences or extra text."
    )


def build_route_planner_prompt(request: RouteRequestSpec) -> str:
    if request.planning_mode == "environment_video_landmarks":
        return (
            "You are analyzing a prerecorded environment video that covers the whole AMR route. "
            "Study the entire video from the robot start area to the requested goal area. "
            "Plan object-recognition-guided navigation using visually distinctive landmarks that "
            "appear in sequence along the route and are suitable for Grounding DINO zero-shot detection. "
            "Prefer concrete, easy-to-detect objects or scene markers such as doors, signs, elevators, "
            "junction openings, pillars, cabinets, desks, plants, posters, cones, or carts. "
            "Avoid vague placeholders such as 'destination area', 'waypoint', 'turn point', "
            "'unknown', 'same as above', or single numbers unless the corresponding object is visibly real "
            "and concrete in the video. "
            f"Goal: {request.goal_text}. "
            f"Set mission_id exactly to '{request.mission_id}'. "
            f"Set environment_id exactly to '{request.environment_id}'. "
            f"Set camera_source exactly to '{request.camera_source}'. "
            f"Set inference_interval_sec exactly to {request.inference_interval_sec}. "
            "Produce 4 to 8 navigation steps when the route supports it. "
            "Return exactly one JSON object with keys "
            "mission_id, mission_text, environment_id, camera_source, inference_interval_sec, steps. "
            "Each step in steps must be a JSON object with keys "
            "step_id, instruction, visual_goal, scene_description, expected_landmarks, "
            "primary_landmark, grounding_objects, grounding_prompt, control_primitive, "
            "votes_needed, confidence_threshold, min_dwell_sec, timeout_sec, fallback. "
            "grounding_objects must be a list of 1 to 4 short noun phrases naming concrete visible objects "
            "for this step, ordered from most important to less important. "
            "primary_landmark must match the main object the robot should use to decide progress. "
            "grounding_prompt must be a phrase-style prompt derived from grounding_objects, "
            "with each phrase ending in a period for Grounding DINO. "
            "Write mission_text, instruction, visual_goal, and scene_description as concrete scene-aware sentences. "
            "Do not include markdown fences or extra text."
        )
    return (
        "You are planning a short AMR route from the current visual context. "
        "Use only landmarks that are visible and easy to detect in the provided video clip. "
        "Do not use placeholders such as '...', 'unknown', 'same as above', or single numbers. "
        "All text fields must contain concrete content grounded in the observed scene. "
        f"Goal: {request.goal_text}. "
        f"Set mission_id exactly to '{request.mission_id}'. "
        f"Environment ID: {request.environment_id}. "
        f"Set environment_id exactly to '{request.environment_id}'. "
        f"Set camera_source exactly to '{request.camera_source}'. "
        f"Set inference_interval_sec exactly to {request.inference_interval_sec}. "
        "Produce 3 to 6 steps unless the scene clearly supports fewer. "
        "Return exactly one JSON object with keys "
        "mission_id, mission_text, environment_id, camera_source, inference_interval_sec, steps. "
        "Each item in steps must be a JSON object with keys "
        "step_id, instruction, visual_goal, scene_description, expected_landmarks, "
        "primary_landmark, grounding_objects, grounding_prompt, control_primitive, votes_needed, "
        "confidence_threshold, min_dwell_sec, timeout_sec, fallback. "
        "Keep primary_landmark as one short noun phrase and make grounding_prompt include that same phrase. "
        "If possible, also include grounding_objects as a short list of concrete visible objects. "
        "Write mission_text, instruction, visual_goal, and scene_description as concrete scene-aware sentences. "
        "Use phrase-style prompts suitable for open-vocabulary detection, such as "
        f"'{landmark_display_name('traffic_cone')}. {landmark_display_name('person')}.' "
        "Do not include markdown fences or extra text."
    )
