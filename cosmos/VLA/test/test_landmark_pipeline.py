import unittest

from vla_demo.landmark_logic import (
    evaluate_landmark_detection,
    normalize_grounding_prompt,
    parse_grounding_phrases,
    select_best_detection,
)
from vla_demo.route_planning import coerce_route_plan_payload
from vla_demo.schemas import MissionSpec, RouteRequestSpec


class LandmarkPipelineTest(unittest.TestCase):
    def test_select_best_detection_matches_target_only(self) -> None:
        best = select_best_detection(
            [
                {"class_id": "a corridor", "score": 0.55, "cx": 1, "cy": 2, "w": 3, "h": 4},
                {"class_id": "corridor", "score": 0.91, "cx": 5, "cy": 6, "w": 7, "h": 8},
                {"class_id": "person", "score": 0.99, "cx": 9, "cy": 10, "w": 11, "h": 12},
            ],
            "corridor",
        )
        self.assertIsNotNone(best)
        self.assertEqual(best["score"], 0.91)
        self.assertEqual(best["class_id"], "corridor")

    def test_normalize_grounding_prompt_adds_periods(self) -> None:
        self.assertEqual(
            normalize_grounding_prompt("person, traffic cone", "person"),
            "person. traffic cone.",
        )

    def test_parse_grounding_phrases_extracts_multiple_objects(self) -> None:
        self.assertEqual(
            parse_grounding_phrases("a chair. a fire extinguisher.", "chair"),
            ["a chair", "a fire extinguisher"],
        )

    def test_evaluator_returns_positive_inference_when_threshold_met(self) -> None:
        inference = evaluate_landmark_detection(
            {
                "step": {
                    "step_id": 3,
                    "confidence_threshold": 0.75,
                }
            },
            {
                "step_id": 3,
                "target_landmark": "right_branch",
                "found": True,
                "class_id": "right branch",
                "score": 0.82,
            },
        )
        self.assertTrue(inference["step_match"])
        self.assertTrue(inference["step_completed"])
        self.assertEqual(inference["observed_landmarks"], ["right branch"])

    def test_coerce_route_plan_adds_defaults_and_validates_with_mission_schema(self) -> None:
        request = RouteRequestSpec.from_dict(
            {
                "mission_id": "route_demo",
                "goal_text": "Go to the freight elevator",
                "environment_id": "hallway_9f",
                "source_mode": "video_file",
                "video_uri": "/tmp/demo.mp4",
                "camera_source": "/camera/front/image_raw",
            }
        )
        payload = coerce_route_plan_payload(
            {
                "steps": [
                    {
                        "instruction": "Move along the corridor",
                        "visual_goal": "A corridor appears straight ahead",
                        "expected_landmarks": ["corridor"],
                    },
                    {
                        "instruction": "Stop at the elevator zone",
                        "visual_goal": "The freight elevator is centered ahead",
                        "primary_landmark": "freight elevator",
                    },
                ]
            },
            request,
        )
        mission = MissionSpec.from_dict(payload)
        self.assertEqual(mission.mission_id, "route_demo")
        self.assertEqual(mission.steps[0].grounding_objects, ["corridor"])
        self.assertEqual(mission.steps[0].grounding_prompt, "corridor.")
        self.assertEqual(mission.steps[1].primary_landmark, "freight elevator")
        self.assertEqual(mission.steps[1].grounding_objects, ["freight elevator"])
        self.assertEqual(mission.steps[1].grounding_prompt, "freight elevator.")

    def test_route_request_accepts_environment_video_landmark_mode(self) -> None:
        request = RouteRequestSpec.from_dict(
            {
                "mission_id": "route_env_demo",
                "goal_text": "前往貨梯區域",
                "environment_id": "hallway_9f",
                "source_mode": "video_file",
                "planning_mode": "environment_video_landmarks",
                "video_uri": "/tmp/demo.mp4",
            }
        )
        self.assertEqual(request.planning_mode, "environment_video_landmarks")

    def test_coerce_environment_video_landmark_plan_preserves_grounding_objects(self) -> None:
        request = RouteRequestSpec.from_dict(
            {
                "mission_id": "route_env_demo",
                "goal_text": "前往貨梯區域",
                "environment_id": "hallway_9f",
                "source_mode": "video_file",
                "planning_mode": "environment_video_landmarks",
                "video_uri": "/tmp/demo.mp4",
                "camera_source": "/camera/front/image_raw",
            }
        )
        payload = coerce_route_plan_payload(
            {
                "steps": [
                    {
                        "step_id": 1,
                        "instruction": "沿主走廊前進並留意右側的消防箱",
                        "visual_goal": "右側牆面可見紅色消防箱",
                        "scene_description": "主走廊右側牆面有顯眼的紅色消防箱。",
                        "expected_landmarks": ["corridor", "fire cabinet"],
                        "primary_landmark": "fire cabinet",
                        "grounding_objects": ["fire cabinet", "corridor wall"],
                        "grounding_prompt": "fire cabinet. corridor wall.",
                    }
                ]
            },
            request,
        )
        self.assertEqual(payload["planning_mode"], "environment_video_landmarks")
        self.assertEqual(
            payload["steps"][0]["grounding_objects"],
            ["fire cabinet", "corridor wall"],
        )
        self.assertEqual(
            payload["steps"][0]["grounding_prompt"],
            "fire cabinet. corridor wall.",
        )

    def test_coerce_route_plan_rejects_placeholder_step_content(self) -> None:
        request = RouteRequestSpec.from_dict(
            {
                "mission_id": "route_live_demo",
                "goal_text": "離開室內區域",
                "environment_id": "hallway_9f",
                "source_mode": "live_camera",
                "camera_source": "/camera/camera/color/image_raw",
            }
        )
        with self.assertRaisesRegex(Exception, "placeholder"):
            coerce_route_plan_payload(
                {
                    "mission_id": "1",
                    "mission_text": "...",
                    "environment_id": "unknown",
                    "camera_source": "unknown",
                    "inference_interval_sec": 1.0,
                    "steps": [
                        {
                            "step_id": 1,
                            "instruction": "...",
                            "visual_goal": "...",
                        }
                    ],
                },
                request,
            )

    def test_coerce_route_plan_uses_request_identity_fields(self) -> None:
        request = RouteRequestSpec.from_dict(
            {
                "mission_id": "route_live_demo",
                "goal_text": "離開室內區域",
                "environment_id": "hallway_9f",
                "source_mode": "live_camera",
                "camera_source": "/camera/camera/color/image_raw",
                "inference_interval_sec": 1.5,
            }
        )
        payload = coerce_route_plan_payload(
            {
                "mission_id": "1",
                "mission_text": "...",
                "environment_id": "unknown",
                "camera_source": "unknown",
                "inference_interval_sec": 1.0,
                "steps": [
                    {
                        "step_id": 1,
                        "instruction": "離開室內門口，往前走向明亮區域",
                        "visual_goal": "前方出口或較明亮的通道清楚可見",
                    }
                ],
            },
            request,
        )
        self.assertEqual(payload["mission_id"], "route_live_demo")
        self.assertEqual(payload["environment_id"], "hallway_9f")
        self.assertEqual(payload["camera_source"], "/camera/camera/color/image_raw")
        self.assertEqual(payload["inference_interval_sec"], 1.5)
        self.assertEqual(payload["mission_text"], "離開室內區域")


if __name__ == "__main__":
    unittest.main()
