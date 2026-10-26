from pathlib import Path
import re
import tempfile
import unittest

import yaml

from piper_control.recording.topics import TopicConfigError, missing_required_topics, topic_plan


ROOT = Path(__file__).resolve().parents[1]


class RecordTopicTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "topics.yaml"
        self.cfg = {
            "robot": {"arms": ["left", "right"], "pairs": {
                "left": {"follower": {"channel": "can0"}, "leader": None},
                "right": {"follower": {"channel": "can1"}, "leader": {"channel": "can2"}},
            }},
            "observation": {"camera_map": {"top": "cam_h", "wrist": "cam_r", "duplicate": "cam_h"}},
            "record": {"topic_prefix": "/test", "topics_config": str(self.path)},
        }

    def write(self, required, optional=None):
        self.path.write_text(yaml.safe_dump({"required": required, "optional": optional or []}))

    def test_templates_expand_only_the_variables_they_use(self):
        self.write(["{prefix}/state", "{prefix}/arms/{side}/follower/joint_states",
                    "{prefix}/arms/{side}/leader/command", "{prefix}/cameras/{camera}/color/image_raw"],
                   ["{prefix}/events"])
        plan = topic_plan(self.cfg)
        self.assertEqual(plan["required_topics"], [
            "/test/state", "/test/arms/left/follower/joint_states",
            "/test/arms/right/follower/joint_states", "/test/arms/right/leader/command",
            "/test/cameras/cam_h/color/image_raw", "/test/cameras/cam_r/color/image_raw",
        ])
        self.assertEqual(plan["optional_topics"], ["/test/events"])
        pattern = re.compile(plan["topic_regex"])
        for name in plan["required_topics"] + plan["optional_topics"]:
            self.assertIsNotNone(pattern.fullmatch(name))
        for name in ("/test/action", "/test/state_extra", "/test/arms/left/leader/command",
                     "/test/cameras/cam_l/color/image_raw"):
            self.assertIsNone(pattern.fullmatch(name))

    def test_templates_using_both_side_and_camera_expand_cartesian_product(self):
        self.write(["{prefix}/{side}/{camera}"])
        self.assertEqual(len(topic_plan(self.cfg)["required_topics"]), 4)

    def test_regex_escapes_literal_names_and_accepts_empty_prefix(self):
        self.cfg["record"]["topic_prefix"] = ""
        self.write(["{prefix}/state", "/external/dotted.topic"])
        pattern = re.compile(topic_plan(self.cfg)["topic_regex"])
        self.assertIsNotNone(pattern.fullmatch("/state"))
        self.assertIsNotNone(pattern.fullmatch("/external/dotted.topic"))
        self.assertIsNone(pattern.fullmatch("/external/dottedXtopic"))

    def test_relative_file_is_resolved_against_configuration_path(self):
        self.write(["{prefix}/state"])
        self.cfg["_config_path"] = str(self.path.parent / "collect.yaml")
        self.cfg["record"]["topics_config"] = "topics.yaml"
        self.assertEqual(topic_plan(self.cfg)["topics_config"], str(self.path.resolve()))

    def test_disabled_boolean_leader_is_not_required(self):
        self.cfg["robot"]["pairs"]["right"]["leader"] = False
        self.write(["{prefix}/state", "{prefix}/arms/{side}/leader/command"])
        self.assertEqual(topic_plan(self.cfg)["required_topics"], ["/test/state"])

    def test_unknown_placeholder_and_format_operations_are_rejected(self):
        for template in ("{prefix}/{unknown}", "{prefix}/{side!r}", "{prefix}/{side:>5}",
                         "{prefix}/{side.foo}", "{prefix}/{side[0]}", "{prefix}/{side"):
            with self.subTest(template=template):
                self.write([template])
                with self.assertRaises(TopicConfigError):
                    topic_plan(self.cfg)

    def test_duplicate_topics_are_rejected_within_or_across_lists(self):
        for required, optional in ((["/test/state", "{prefix}/state"], []),
                                   (["{prefix}/state"], ["/test/state"])):
            self.write(required, optional)
            with self.assertRaisesRegex(TopicConfigError, "Duplicate topic /test/state"):
                topic_plan(self.cfg)

    def test_malformed_yaml_contracts_are_rejected(self):
        for data in ({"required": "/state"}, {"requierd": ["/state"]}, [],
                     {"required": [None]}, {"required": []}, {"required": ["relative/topic"]}):
            with self.subTest(data=data):
                self.path.write_text(yaml.safe_dump(data))
                with self.assertRaises(TopicConfigError):
                    topic_plan(self.cfg)
        self.path.write_text("required: [/state]\nrequired: [/action]\n")
        with self.assertRaisesRegex(TopicConfigError, "Duplicate topic configuration key"):
            topic_plan(self.cfg)

    def test_camera_stream_requirements_are_not_silently_dropped(self):
        self.cfg["observation"].update(depth=False, align_depth=False)
        self.write(["{prefix}/cameras/{camera}/depth/image_raw"])
        plan = topic_plan(self.cfg)
        self.assertEqual(plan["required_topics"], ["/test/cameras/cam_h/depth/image_raw",
                                                  "/test/cameras/cam_r/depth/image_raw"])
        self.assertEqual(missing_required_topics(plan, {plan["required_topics"][0]: 1}),
                         [plan["required_topics"][1]])

    def test_default_profile_covers_all_camera_streams_and_commands(self):
        self.cfg["record"]["topics_config"] = str(ROOT / "configs/recording/topics.yaml")
        plan = topic_plan(self.cfg)
        for camera in ("cam_h", "cam_r"):
            for stream in ("color", "depth", "aligned_depth_to_color"):
                for kind in ("image_raw", "camera_info"):
                    self.assertIn("/test/cameras/%s/%s/%s" % (camera, stream, kind), plan["required_topics"])
        self.assertIn("/test/action", plan["required_topics"])
        self.assertIn("/test/action_metadata", plan["required_topics"])
        self.assertIn("/test/arms/right/leader/command", plan["required_topics"])
        self.assertNotIn("/test/arms/left/leader/command", plan["required_topics"])
        self.assertIn("/test/arms/right/leader/joint_states", plan["optional_topics"])

    def test_only_positive_counts_satisfy_required_topics(self):
        self.write(["/state", "/action"], ["/events"])
        plan = topic_plan(self.cfg)
        self.assertEqual(missing_required_topics(plan, {"/state": 1, "/action": 0}), ["/action"])
        self.assertEqual(missing_required_topics(plan, {"/state": 1, "/action": 2}), [])


if __name__ == "__main__":
    unittest.main()
