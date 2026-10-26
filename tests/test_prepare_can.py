import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from piper_control.hardware.recover import main as recover_main
from piper_control.hardware.prepare_can import (
    configuration_plan,
    configure_interfaces,
    main,
)


def config():
    return {
        "arms": ["left"],
        "bitrate": 1000000,
        "firmware": "S-V1.9-0",
        "pairs": {
            "left": {
                "follower": {"channel": "can2"},
                "leader": {"channel": "can3"},
            }
        },
    }


class PrepareCANTests(unittest.TestCase):
    def test_plan_sets_queue_length_before_bringing_selected_channels_up(self):
        cfg = config()
        plan = configuration_plan(cfg)
        channels = [
            cfg["pairs"][side][role]["channel"]
            for side in cfg["arms"]
            for role in ("follower", "leader")
        ]
        self.assertEqual(len(plan["commands"]), 4 * len(channels))
        for index, channel in enumerate(channels):
            self.assertEqual(
                plan["commands"][index * 4 : index * 4 + 4],
                [
                    ["ip", "link", "set", "dev", channel, "down"],
                    [
                        "ip",
                        "link",
                        "set",
                        "dev",
                        channel,
                        "type",
                        "can",
                        "bitrate",
                        "1000000",
                    ],
                    ["ip", "link", "set", "dev", channel, "txqueuelen", "64"],
                    ["ip", "link", "set", "dev", channel, "up"],
                ],
            )

    def test_custom_queue_and_right_only_selection_use_resolved_usb_channels(self):
        cfg = config()
        cfg["arms"] = ["right"]
        cfg["can_tx_queue_len"] = 128
        cfg["pairs"]["right"] = {
            "leader": {"usb_serial": "right-leader"},
            "follower": {"usb_serial": "right-follower"},
        }
        plan = configuration_plan(cfg, [
            {"channel": "can10", "usb_serial": "right-leader"},
            {"channel": "can11", "usb_serial": "right-follower"},
        ])
        self.assertEqual(plan["resolved"], {
            "right": {"leader": "can10", "follower": "can11"},
        })
        self.assertEqual({command[4] for command in plan["commands"]}, {"can10", "can11"})
        self.assertEqual([command[-1] for command in plan["commands"] if command[-2] == "txqueuelen"],
                         ["128", "128"])

    def test_invalid_queue_lengths_are_rejected_before_interface_setup(self):
        for value in (True, False, 0, -1, 1.5, "64", None):
            cfg = config()
            cfg["can_tx_queue_len"] = value
            with self.subTest(value=value), patch(
                "piper_control.hardware.prepare_can.subprocess.run"
            ) as execute:
                with self.assertRaisesRegex(ValueError, "can_tx_queue_len"):
                    configure_interfaces(cfg)
                execute.assert_not_called()

    def test_numeric_bitrate_and_channel_are_parsed_without_shell(self):
        cfg = config()
        cfg["bitrate"] = "1000000; anything"
        with self.assertRaises(ValueError):
            configuration_plan(cfg)
        cfg["bitrate"] = 1000000
        cfg["pairs"]["left"]["leader"]["channel"] = "can0; anything"
        with self.assertRaises(ValueError):
            configuration_plan(cfg)

    def test_execute_runs_plan_once_without_status_queries_or_locks(self):
        cfg = config()
        with patch(
            "piper_control.hardware.prepare_can.os.geteuid", return_value=1000
        ), patch("piper_control.hardware.prepare_can.subprocess.run") as execute:
            result = configure_interfaces(cfg)
        self.assertEqual(execute.call_count, 8)
        for call, command in zip(execute.call_args_list, result["commands"]):
            self.assertEqual(call.args[0], ["sudo", "-n"] + command)
            self.assertEqual(call.kwargs, {"check": True})

    def test_preview_never_configures_interfaces(self):
        path = Path(__file__).resolve().parents[1] / "configs" / "mock.yaml"
        with patch(
            "piper_control.hardware.prepare_can.configure_interfaces"
        ) as configure, patch(
            "piper_control.hardware.prepare_can.subprocess.run"
        ) as execute, patch(
            "piper_control.hardware.prepare_can.load_config",
            return_value={"robot": config()},
        ), patch(
            "sys.stdout", new=io.StringIO()
        ):
            self.assertEqual(main(["--config", str(path)]), 0)
        configure.assert_not_called()
        execute.assert_not_called()

    def test_usb_resolution_and_optional_leader_in_plan(self):
        cfg = config()
        cfg["pairs"]["left"] = {"follower": {"usb_serial": "adapter-a"}, "leader": None}
        plan = configuration_plan(cfg, [{"channel": "can8", "usb_serial": "adapter-a"}])
        self.assertEqual(plan["resolved"], {"left": {"follower": "can8"}})
        self.assertEqual(len(plan["commands"]), 4)
        self.assertTrue(all(command[4] == "can8" for command in plan["commands"]))

    def test_list_needs_no_config_and_never_changes_interfaces(self):
        inventory = [{"channel": "can8", "usb_serial": "adapter-a"}]
        output = io.StringIO()
        with patch(
            "piper_control.hardware.prepare_can.discover_interfaces",
            return_value=inventory,
        ), patch("piper_control.hardware.prepare_can.load_config") as load, patch(
            "piper_control.hardware.prepare_can.subprocess.run"
        ) as execute, patch(
            "sys.stdout", new=output
        ):
            self.assertEqual(main(["--list"]), 0)
        self.assertEqual(json.loads(output.getvalue()), inventory)
        load.assert_not_called()
        execute.assert_not_called()

    def test_execute_uses_the_exact_previewed_resolution(self):
        cfg = config()
        cfg["pairs"]["left"] = {"follower": {"usb_serial": "adapter-a"}}
        output = io.StringIO()
        with patch(
            "piper_control.hardware.prepare_can.load_config",
            return_value={"robot": cfg},
        ), patch(
            "piper_control.hardware.interfaces.discover_interfaces",
            return_value=[{"channel": "can8", "usb_serial": "adapter-a"}],
        ) as discover, patch(
            "piper_control.hardware.prepare_can.os.geteuid", return_value=0
        ), patch(
            "piper_control.hardware.prepare_can.subprocess.run"
        ) as execute, patch(
            "sys.stdout", new=output
        ):
            self.assertEqual(main(["--config", "unused.yaml", "--execute"]), 0)
        discover.assert_called_once_with()
        printed = json.loads(output.getvalue())
        self.assertEqual(
            [call.args[0] for call in execute.call_args_list], printed["commands"]
        )

    def test_recover_preview_prints_selectors_without_discovery_or_sdk(self):
        cfg = config()
        cfg["pairs"]["left"] = {"follower": {"usb_serial": "unplugged"}}
        output = io.StringIO()
        with patch(
            "piper_control.hardware.recover.load_config", return_value={"robot": cfg}
        ), patch("piper_control.hardware.recover.SDKRobot") as sdk, patch(
            "piper_control.hardware.interfaces.discover_interfaces"
        ) as discover, patch(
            "sys.stdout", new=output
        ):
            self.assertEqual(recover_main(["--config", "unused.yaml"]), 0)
        self.assertEqual(json.loads(output.getvalue())["devices"], cfg["pairs"])
        sdk.assert_not_called()
        discover.assert_not_called()


if __name__ == "__main__":
    unittest.main()
