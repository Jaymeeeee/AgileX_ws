import copy
from pathlib import Path
import tempfile
import unittest

from piper_control.config import (
    ConfigError,
    component_names,
    describe,
    load_config,
    validate_config,
)
from piper_control.recording.bag_to_lerobot import load_config as load_conversion_config

CONFIGS = Path(__file__).resolve().parents[1] / "configs"


class ConfigTests(unittest.TestCase):
    def test_can_queue_length_defaults_and_explicit_override(self):
        cfg = load_config(CONFIGS / "collect_right.yaml")
        self.assertEqual(cfg["robot"]["can_tx_queue_len"], 64)
        cfg = load_config(CONFIGS / "collect_right.yaml", {"robot": {"can_tx_queue_len": 128}})
        self.assertEqual(cfg["robot"]["can_tx_queue_len"], 128)
        for value in (True, False, 0, -1, 1.5, "64", None):
            with self.subTest(value=value), self.assertRaisesRegex(ConfigError, "can_tx_queue_len"):
                load_config(CONFIGS / "collect_right.yaml", {"robot": {"can_tx_queue_len": value}})

    def test_right_collection_selects_two_cameras_with_all_streams_and_topic_file(self):
        cfg = load_config(CONFIGS / "collect_right.yaml")
        self.assertTrue(cfg["observation"]["depth"])
        self.assertTrue(cfg["observation"]["align_depth"])
        self.assertEqual(set(cfg["observation"]["camera_map"].values()), {"cam_h", "cam_r"})
        self.assertEqual(cfg["record"]["topics_config"], str(CONFIGS / "recording" / "topics.yaml"))

    def test_single_arm_replaces_dual_camera_contract(self):
        cfg = load_config(CONFIGS / "act_right.yaml")
        self.assertEqual(cfg["robot"]["arms"], ["right"])
        self.assertEqual(
            set(cfg["observation"]["camera_map"].values()), {"cam_h", "cam_r"}
        )
        self.assertEqual(cfg["observation"]["action_names"], component_names(["right"]))
        self.assertTrue(Path(cfg["policy"]["checkpoint"]).is_absolute())

    def test_type_must_be_explicit_and_rl_requires_matching_backend(self):
        cfg = load_config(CONFIGS / "base.yaml")
        for value in (None, "", "auto"):
            altered = copy.deepcopy(cfg)
            altered["policy"]["type"] = value
            with self.assertRaisesRegex(ConfigError, "policy.type"):
                validate_config(altered)
        cfg["workflow"].update(mode="online_rl", learner="pi05_rlt")
        with self.assertRaisesRegex(ConfigError, "pi05_rlt"):
            validate_config(cfg)

    def test_independent_channels_and_semantic_names_required(self):
        cfg = load_config(CONFIGS / "act_left.yaml")
        bad = copy.deepcopy(cfg)
        bad["robot"]["pairs"]["left"]["leader"] = copy.deepcopy(
            bad["robot"]["pairs"]["left"]["follower"]
        )
        with self.assertRaisesRegex(ConfigError, "independent CAN"):
            validate_config(bad)
        bad = copy.deepcopy(cfg)
        bad["observation"]["action_names"] = component_names(["right"])
        with self.assertRaisesRegex(ConfigError, "action_names"):
            validate_config(bad)

    def test_nonfinite_rates_and_unbounded_queue_are_rejected(self):
        cfg = load_config(CONFIGS / "base.yaml")
        for section, key, value in (
            ("workflow", "policy_hz", float("nan")),
            ("policy", "update_timeout_sec", float("nan")),
            ("policy", "max_pending_commands", True),
            ("robot", "move_speed_percent", 101),
            ("robot", "startup_enable_timeout_sec", 0),
            ("robot", "startup_enable_timeout_sec", float("inf")),
            ("record", "queue_size", 0.5),
            ("record", "topic_prefix", "/bad/topic name"),
        ):
            bad = copy.deepcopy(cfg)
            bad[section][key] = value
            with self.assertRaises(ConfigError):
                validate_config(bad)

    def test_duplicate_keys_and_inheritance_cycles_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text("policy: {}\npolicy: {}\n")
            with self.assertRaisesRegex(ConfigError, "Duplicate"):
                load_config(path)
            path.write_text("extends: config.yaml\n")
            with self.assertRaisesRegex(ConfigError, "cycle"):
                load_config(path)

    def test_all_example_configs_validate_without_lerobot(self):
        # These two files describe offline bag conversion, not an SDK workflow.
        # Classify by explicit filename so malformed runtime configs still fail.
        conversion_configs = {
            "lerobot_insert_plug_v4.yaml",
            "lerobot_insert_plug_v4_with_motor_state.yaml",
        }
        for path in CONFIGS.glob("*.yaml"):
            with self.subTest(path=path):
                validator = load_conversion_config if path.name in conversion_configs else load_config
                validator(path)

    def test_deployment_selects_only_followers_and_manual_configs_add_leaders(self):
        for name in (
            "base.yaml",
            "act_left.yaml",
            "act_right.yaml",
            "act_dual.yaml",
            "pi05_dual.yaml",
        ):
            cfg = load_config(CONFIGS / name)
            for side in cfg["robot"]["arms"]:
                self.assertIsNone(cfg["robot"]["pairs"][side]["leader"])
                self.assertEqual(set(describe(cfg)["devices"][side]), {"follower"})
        for name in ("teleop.yaml", "collect.yaml", "rlt_online.yaml", "mock.yaml"):
            cfg = load_config(CONFIGS / name)
            for side in cfg["robot"]["arms"]:
                self.assertIn("usb_serial", cfg["robot"]["pairs"][side]["leader"])

    def test_boolean_switches_bind_each_device_to_its_named_serial(self):
        expected = {
            ("left", "leader"): "usb-A",
            ("left", "follower"): "usb-B",
            ("right", "leader"): "usb-C",
            ("right", "follower"): "usb-D",
        }
        with tempfile.TemporaryDirectory() as directory:
            mapping = Path(directory) / "can.yaml"
            mapping.write_text(
                "adapters:\n"
                + "".join(
                    "  %s_%s: {usb_serial: '%s'}\n" % (side, role, serial)
                    for (side, role), serial in expected.items()
                )
            )
            cfg = load_config(
                CONFIGS / "teleop.yaml", {"robot": {"can_config": str(mapping)}}
            )
        for (side, role), serial in expected.items():
            spec = cfg["robot"]["pairs"][side][role]
            self.assertEqual(spec["adapter"], "%s_%s" % (side, role))
            self.assertEqual(spec["usb_serial"], serial)
            self.assertNotIn("channel", spec)

    def test_false_overrides_inherited_true_and_true_restores_named_binding(self):
        cfg = load_config(
            CONFIGS / "mock.yaml", {"robot": {"pairs": {"left": {"leader": False}}}}
        )
        self.assertIsNone(cfg["robot"]["pairs"]["left"]["leader"])
        cfg = load_config(
            CONFIGS / "act_left.yaml", {"robot": {"pairs": {"left": {"leader": True}}}}
        )
        self.assertEqual(
            cfg["robot"]["pairs"]["left"]["leader"]["adapter"], "left_leader"
        )

    def test_disabled_and_unselected_devices_do_not_require_usb_entries(self):
        with tempfile.TemporaryDirectory() as directory:
            mapping = Path(directory) / "can.yaml"
            mapping.write_text(
                "adapters:\n  left_follower:\n    usb_serial: 'offline-arm'\n"
            )
            overrides = {
                "robot": {
                    "can_config": str(mapping),
                    "pairs": {
                        "left": {"follower": True, "leader": False},
                        "right": {"follower": True, "leader": True},
                    },
                }
            }
            cfg = load_config(CONFIGS / "act_left.yaml", overrides)
            self.assertEqual(set(describe(cfg)["devices"]), {"left"})
            self.assertEqual(set(describe(cfg)["devices"]["left"]), {"follower"})
            overrides["robot"]["pairs"]["left"]["leader"] = True
            with self.assertRaisesRegex(
                ConfigError, "Unknown CAN adapter.*left_leader"
            ):
                load_config(CONFIGS / "act_left.yaml", overrides)

    def test_device_switch_requires_yaml_boolean_not_a_string_or_number(self):
        for value in ("true", "false", 1, 0):
            with self.subTest(value=value), self.assertRaises(ConfigError):
                load_config(
                    CONFIGS / "act_left.yaml",
                    {"robot": {"pairs": {"left": {"leader": value}}}},
                )

    def test_missing_or_null_leader_is_valid_for_deploy(self):
        cfg = load_config(
            CONFIGS / "mock.yaml", {"robot": {"pairs": {"left": {"leader": None}}}}
        )
        self.assertIsNone(cfg["robot"]["pairs"]["left"]["leader"])
        del cfg["robot"]["pairs"]["left"]["leader"]
        validate_config(cfg)
        for mode in ("teleop", "collect"):
            cfg["workflow"]["mode"] = mode
            with self.assertRaisesRegex(ConfigError, "requires a leader"):
                validate_config(cfg)

    def test_adapter_file_is_relative_to_declaring_config_and_load_is_offline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "models").mkdir()
            (root / "wiring.yaml").write_text(
                "adapters:\n  arm_usb:\n    usb_serial: 'not-connected'\n"
            )
            (root / "base.yaml").write_text(
                "extends: %s\nrobot:\n  arms: [left]\n  can_config: wiring.yaml\n"
                "  pairs:\n    left:\n      follower: {adapter: arm_usb}\n"
                % (CONFIGS / "base.yaml")
            )
            child = root / "models" / "deploy.yaml"
            child.write_text("extends: ../base.yaml\n")
            cfg = load_config(child)
            self.assertEqual(cfg["robot"]["can_config"], str(root / "wiring.yaml"))
            spec = cfg["robot"]["pairs"]["left"]["follower"]
            self.assertEqual(spec["usb_serial"], "not-connected")
            self.assertNotIn("channel", spec)

    def test_unknown_adapter_and_multiple_selectors_report_configuration_error(self):
        with self.assertRaisesRegex(ConfigError, "Unknown CAN adapter"):
            load_config(
                CONFIGS / "act_left.yaml",
                {"robot": {"pairs": {"left": {"follower": {"adapter": "unknown"}}}}},
            )
        cfg = load_config(CONFIGS / "act_left.yaml")
        cfg["robot"]["pairs"]["left"]["follower"]["channel"] = "can99"
        with self.assertRaisesRegex(ConfigError, "one CAN selector"):
            validate_config(cfg)

    def test_inline_channel_or_usb_path_replaces_inherited_adapter(self):
        for selector in ({"channel": "can7"}, {"usb_path": "1-5.3:1.0"}):
            cfg = load_config(
                CONFIGS / "act_left.yaml",
                {"robot": {"pairs": {"left": {"follower": selector}}}},
            )
            self.assertEqual(cfg["robot"]["pairs"]["left"]["follower"], selector)

    def test_follower_is_required_and_numeric_serial_is_rejected(self):
        for spec in (False, None, {"usb_serial": 12345}):
            with self.assertRaises(ConfigError):
                load_config(
                    CONFIGS / "act_left.yaml",
                    {"robot": {"pairs": {"left": {"follower": spec}}}},
                )


if __name__ == "__main__":
    unittest.main()
