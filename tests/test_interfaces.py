import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from piper_control.hardware.interfaces import (
    configured_devices,
    discover_interfaces,
    resolve_robot_config,
)


def config(follower=None, leader=None):
    return {
        "arms": ["left"],
        "pairs": {
            "left": {
                "follower": follower or {"channel": "can2"},
                "leader": leader,
            }
        },
    }


def interface(channel="can2", serial="arm-a", path="1-5.3", endpoint="1-5.3:1.0"):
    return dict(
        channel=channel, usb_serial=serial, usb_path=path, usb_interface=endpoint
    )


class InterfaceTests(unittest.TestCase):
    def test_fixed_channels_do_not_discover_and_config_is_copied(self):
        cfg = config(leader={"channel": "can3"})
        before = copy.deepcopy(cfg)
        with patch("piper_control.hardware.interfaces.discover_interfaces") as discover:
            resolved = resolve_robot_config(cfg)
        discover.assert_not_called()
        self.assertEqual(resolved, before)
        resolved["pairs"]["left"]["follower"]["channel"] = "can8"
        self.assertEqual(cfg, before)

    def test_renumbering_follows_serial_without_mutating_source(self):
        cfg = config({"usb_serial": "arm-a"}, {"usb_serial": "arm-b"})
        first = [interface("can0", "arm-a"), interface("can1", "arm-b")]
        second = [interface("can1", "arm-a"), interface("can0", "arm-b")]
        resolved = resolve_robot_config(cfg, first)
        self.assertEqual(resolved["pairs"]["left"]["follower"]["channel"], "can0")
        # Reconnection resolves identities again, even on an already resolved config.
        reconnected = resolve_robot_config(resolved, second)
        self.assertEqual(reconnected["pairs"]["left"]["follower"]["channel"], "can1")
        self.assertEqual(reconnected["pairs"]["left"]["leader"]["channel"], "can0")
        self.assertNotIn("channel", cfg["pairs"]["left"]["follower"])

    def test_missing_identity_never_falls_back_to_cached_channel(self):
        cfg = config({"usb_serial": "unplugged", "channel": "can2"})
        with self.assertRaisesRegex(ValueError, "No SocketCAN.*left/follower"):
            resolve_robot_config(cfg, [interface()])

    def test_serial_and_device_path_ambiguity_requires_interface_path(self):
        inventory = [interface(), interface("can3", endpoint="1-5.3:1.1")]
        for spec in ({"usb_serial": "arm-a"}, {"usb_path": "1-5.3"}):
            with self.subTest(spec=spec):
                with self.assertRaisesRegex(ValueError, "Ambiguous.*can2, can3"):
                    resolve_robot_config(config(spec), inventory)
        resolved = resolve_robot_config(config({"usb_path": "1-5.3:1.1"}), inventory)
        self.assertEqual(resolved["pairs"]["left"]["follower"]["channel"], "can3")

    def test_usb_device_path_selects_the_current_name(self):
        resolved = resolve_robot_config(
            config({"usb_path": "1-5.3"}), [interface("can7")]
        )
        self.assertEqual(resolved["pairs"]["left"]["follower"]["channel"], "can7")

    def test_only_selected_devices_are_resolved(self):
        cfg = config()
        cfg["pairs"]["right"] = {"follower": {"usb_serial": "not-connected"}}
        with patch("piper_control.hardware.interfaces.discover_interfaces") as discover:
            resolve_robot_config(cfg)
        discover.assert_not_called()
        self.assertEqual(
            [(side, role) for side, role, _ in configured_devices(cfg)],
            [("left", "follower")],
        )
        del cfg["pairs"]["left"]["leader"]
        self.assertEqual(len(list(configured_devices(cfg))), 1)

    def test_selected_devices_share_one_discovery_snapshot(self):
        cfg = config({"usb_serial": "arm-a"}, {"usb_serial": "arm-b"})
        inventory = [interface("can0", "arm-a"), interface("can1", "arm-b")]
        with patch(
            "piper_control.hardware.interfaces.discover_interfaces",
            return_value=inventory,
        ) as discover:
            resolve_robot_config(cfg)
        discover.assert_called_once_with()

    def test_two_devices_cannot_resolve_to_the_same_channel(self):
        cfg = config({"usb_serial": "arm-a"}, {"channel": "can2"})
        with self.assertRaisesRegex(ValueError, "same SocketCAN channel can2"):
            resolve_robot_config(cfg, [interface()])

    def test_follower_is_required_and_selectors_must_be_unambiguous(self):
        cfg = config()
        cfg["pairs"]["left"]["follower"] = None
        with self.assertRaisesRegex(ValueError, "follower is required"):
            resolve_robot_config(cfg)
        with self.assertRaisesRegex(ValueError, "either usb_serial or usb_path"):
            resolve_robot_config(config({"usb_serial": "a", "usb_path": "1-5"}), [])

    def test_sysfs_filters_can_and_reads_nearest_usb_device(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            net = root / "class" / "net"
            net.mkdir(parents=True)
            hub = root / "devices" / "usb1" / "1-5"
            hub.mkdir(parents=True)
            (hub / "serial").write_text("hub-serial\n")
            adapter = hub / "1-5.3"
            endpoint = adapter / "1-5.3:1.0"
            endpoint.mkdir(parents=True)
            (adapter / "serial").write_text("adapter-serial\n")
            for name, kind, device in (
                ("can7", "280", endpoint),
                ("ethernet0", "1", endpoint),
                ("can_virtual", "280", root / "devices" / "virtual"),
            ):
                (net / name).mkdir()
                (net / name / "type").write_text(kind + "\n")
                (net / name / "device").symlink_to(device)
            self.assertEqual(
                discover_interfaces(net),
                [
                    interface("can7", "adapter-serial"),
                    interface("can_virtual", None, None, None),
                ],
            )
            # A serial-less adapter must not inherit an upstream hub's serial.
            (adapter / "serial").unlink()
            self.assertIsNone(discover_interfaces(net)[0]["usb_serial"])


if __name__ == "__main__":
    unittest.main()
