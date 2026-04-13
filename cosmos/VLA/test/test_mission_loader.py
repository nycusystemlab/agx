import unittest

from vla_demo.mission_loader import default_mission_path, load_mission_file
from vla_demo.schemas import MissionSpec, SchemaError


class MissionLoaderTest(unittest.TestCase):
    def test_sample_mission_loads(self) -> None:
        mission = load_mission_file(default_mission_path())
        self.assertIsInstance(mission, MissionSpec)
        self.assertEqual(mission.steps[0].step_id, 1)
        self.assertEqual(mission.steps[0].scene_description.startswith("視野前方"), True)
        self.assertEqual(mission.steps[1].primary_landmark, "right_branch")
        self.assertEqual(mission.steps[1].grounding_objects, ["right branch", "corridor"])
        self.assertEqual(mission.steps[1].grounding_prompt, "right branch.")
        self.assertEqual(mission.steps[-1].control_primitive, "approach_target_zone")

    def test_schema_rejects_duplicate_step_ids(self) -> None:
        with self.assertRaises(SchemaError):
            MissionSpec.from_dict(
                {
                    "mission_id": "bad",
                    "mission_text": "bad mission",
                    "environment_id": "lab",
                    "camera_source": "/camera/front/image_raw",
                    "steps": [
                        {
                            "step_id": 1,
                            "instruction": "a",
                            "visual_goal": "b",
                            "control_primitive": "move_forward_until_recheck",
                            "fallback": "pause",
                        },
                        {
                            "step_id": 1,
                            "instruction": "c",
                            "visual_goal": "d",
                            "control_primitive": "move_forward_until_recheck",
                            "fallback": "pause",
                        },
                    ],
                }
            )

    def test_new_step_fields_get_safe_defaults(self) -> None:
        mission = MissionSpec.from_dict(
            {
                "mission_id": "defaults",
                "mission_text": "demo mission",
                "environment_id": "lab",
                "camera_source": "/camera/front/image_raw",
                "steps": [
                    {
                        "step_id": 1,
                        "instruction": "look for the corridor",
                        "visual_goal": "A corridor opens up ahead",
                    }
                ],
            }
        )
        step = mission.steps[0]
        self.assertEqual(step.scene_description, "A corridor opens up ahead")
        self.assertEqual(step.primary_landmark, "A corridor opens up ahead")
        self.assertEqual(step.grounding_objects, ["A corridor opens up ahead"])
        self.assertEqual(step.grounding_prompt, "A corridor opens up ahead.")
        self.assertEqual(step.control_primitive, "move_forward_until_recheck")
        self.assertEqual(step.fallback, "pause")


if __name__ == "__main__":
    unittest.main()
