import io
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from scripts import right_leader_move_j as jog


class FakeClock:
    def __init__(self):
        self.now = 1000.0

    def time(self):
        return self.now

    monotonic = time

    def sleep(self, duration):
        self.now += duration


class FakeSDK:
    """One arm only; intentionally no gripper, forwarder or disable methods."""

    def __init__(self, clock):
        self.clock = clock
        self.calls = []
        self.initial = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        self.joints = list(self.initial)
        self.enabled = [False] * 6
        self.faults = {}
        self.arm_status = 0
        self.freeze_after_enable = False
        self.frozen_stamp = None
        self.fault_on_enable = False
        self.fault_on_move = False
        self.disable_on_speed = False

    def connect(self):
        self.calls.append(("connect",))

    def disconnect(self):
        self.calls.append(("disconnect",))

    def get_joint_angles(self):
        return SimpleNamespace(msg=list(self.joints), timestamp=self.clock.time())

    def get_driver_states(self, index):
        flags = dict(self.faults) if index == 5 else {}
        flags["driver_enable_status"] = self.enabled[index - 1]
        stamp = self.clock.time() if self.frozen_stamp is None else self.frozen_stamp
        return SimpleNamespace(
            timestamp=stamp,
            msg=SimpleNamespace(foc_status=SimpleNamespace(**flags)),
        )

    def get_arm_status(self):
        return SimpleNamespace(msg=SimpleNamespace(arm_status=self.arm_status))

    def set_follower_mode(self):
        self.calls.append(("follower_mode",))

    def enable(self):
        self.calls.append(("enable",))
        self.enabled = [True] * 6
        if self.freeze_after_enable and self.frozen_stamp is None:
            self.frozen_stamp = self.clock.time()
        if self.fault_on_enable:
            self.faults = {"driver_overcurrent": True}
            self.enabled[4] = False
        return True  # Cached SDK return value is deliberately not proof of freshness.

    def reset(self):
        self.calls.append(("reset",))
        self.enabled = [False] * 6
        self.arm_status = 0

    def set_speed_percent(self, speed):
        self.calls.append(("speed", speed))
        if self.disable_on_speed:
            self.enabled[4] = False

    def move_j(self, target):
        self.calls.append(("move_j", list(target)))
        self.joints = list(target)
        if self.fault_on_move:
            self.faults = {"driver_overcurrent": True, "stall_status": True}
            self.enabled[4] = False

    def electronic_emergency_stop(self):
        self.calls.append(("stop",))


class RightLeaderMoveJTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.sdk = FakeSDK(self.clock)
        self.output = io.StringIO()
        clock_patch = patch.object(jog, "time", self.clock)
        output_patch = patch("sys.stdout", self.output)
        clock_patch.start()
        output_patch.start()
        self.addCleanup(clock_patch.stop)
        self.addCleanup(output_patch.stop)

    def commands(self, name):
        return [entry for entry in self.sdk.calls if entry[0] == name]

    def test_single_target_changes_only_j5_then_stops_without_disabling(self):
        jog.run_test(self.sdk)
        moves = self.commands("move_j")
        self.assertEqual(len(moves), 1)
        target = moves[0][1]
        self.assertAlmostEqual(target[4], self.sdk.initial[4] + math.radians(2))
        for index in (0, 1, 2, 3, 5):
            self.assertEqual(target[index], self.sdk.initial[index])
        self.assertEqual(self.commands("speed"), [("speed", 10)])
        self.assertEqual(self.sdk.calls[-2:], [("stop",), ("disconnect",)])
        self.assertEqual(self.sdk.enabled, [True] * 6)
        self.assertEqual(self.commands("reset"), [])
        self.assertIn('"stage": "observation_finished"', self.output.getvalue())

    def test_signed_increment_is_used_without_return_motion(self):
        jog.run_test(self.sdk, delta_deg=-1.0, speed=7)
        self.assertEqual(len(self.commands("move_j")), 1)
        self.assertAlmostEqual(
            self.commands("move_j")[0][1][4], self.sdk.initial[4] - math.radians(1)
        )
        self.assertEqual(self.commands("speed"), [("speed", 7)])

    def test_existing_driver_fault_blocks_reset_enable_and_motion(self):
        self.sdk.faults = {
            "driver_overcurrent": True,
            "driver_error_status": True,
            "stall_status": True,
        }
        with self.assertRaisesRegex(RuntimeError, "Driver fault"):
            jog.run_test(self.sdk, reset=True)
        self.assertEqual(self.sdk.calls, [("connect",), ("disconnect",)])
        for flag in self.sdk.faults:
            self.assertIn(flag, self.output.getvalue())
        self.assertIn("joint_5", self.output.getvalue())

    def test_fault_after_enable_stops_before_any_target_or_enable_retry(self):
        self.sdk.fault_on_enable = True
        with self.assertRaisesRegex(RuntimeError, "Driver fault"):
            jog.run_test(self.sdk)
        self.assertEqual(self.commands("enable"), [("enable",)])
        self.assertEqual(self.commands("move_j"), [])
        self.assertEqual(self.sdk.calls[-2:], [("stop",), ("disconnect",)])

    def test_cached_true_enable_feedback_cannot_allow_motion(self):
        self.sdk.freeze_after_enable = True
        with self.assertRaisesRegex(RuntimeError, "Timed out waiting for enabled"):
            jog.run_test(self.sdk)
        self.assertGreater(len(self.commands("enable")), 1)
        self.assertEqual(self.commands("move_j"), [])
        self.assertEqual(self.sdk.calls[-2:], [("stop",), ("disconnect",)])

    def test_enable_loss_after_speed_setting_blocks_target(self):
        self.sdk.disable_on_speed = True
        with self.assertRaisesRegex(RuntimeError, "Motor enable lost before move_j"):
            jog.run_test(self.sdk)
        self.assertEqual(self.commands("move_j"), [])
        self.assertEqual(self.commands("speed"), [("speed", 10)])
        self.assertEqual(self.sdk.calls[-2:], [("stop",), ("disconnect",)])

    def test_electronic_stop_requires_explicit_reset(self):
        self.sdk.arm_status = 1
        with self.assertRaisesRegex(RuntimeError, "Electronic stop is active"):
            jog.run_test(self.sdk)
        self.assertEqual(self.sdk.calls, [("connect",), ("disconnect",)])

    def test_requested_reset_is_once_and_precedes_enable(self):
        self.sdk.arm_status = 1
        self.sdk.enabled = [True] * 6
        jog.run_test(self.sdk, reset=True)
        self.assertEqual(self.commands("reset"), [("reset",)])
        self.assertLess(self.sdk.calls.index(("reset",)), self.sdk.calls.index(("enable",)))
        self.assertEqual(len(self.commands("move_j")), 1)
        self.assertIn('"stage": "reset_confirmed"', self.output.getvalue())

    def test_fault_after_target_stops_without_resending_target_or_enabling(self):
        self.sdk.fault_on_move = True
        with self.assertRaisesRegex(RuntimeError, "Fault, enable loss"):
            jog.run_test(self.sdk)
        self.assertEqual(len(self.commands("move_j")), 1)
        last_enable = max(i for i, entry in enumerate(self.sdk.calls) if entry[0] == "enable")
        move = next(i for i, entry in enumerate(self.sdk.calls) if entry[0] == "move_j")
        self.assertLess(last_enable, move)
        self.assertEqual(self.sdk.calls[-2:], [("stop",), ("disconnect",)])
        self.assertIn('"stage": "after_move_j:fault"', self.output.getvalue())

    def test_preview_does_not_import_sdk_or_discover_hardware(self):
        with patch.dict("sys.modules", {"pyAgxArm": None}), patch.object(
            jog, "discover_interfaces"
        ) as discover, patch.object(jog, "run_test") as run:
            self.assertEqual(jog.main([]), 0)
        discover.assert_not_called()
        run.assert_not_called()
        self.assertIn('"device": "right/leader"', self.output.getvalue())

    def test_execute_constructs_only_right_leader_sdk(self):
        create = Mock(side_effect=lambda **kwargs: kwargs)
        factory = Mock()
        factory.create_arm.return_value = self.sdk
        module = SimpleNamespace(
            AgxArmFactory=factory,
            create_agx_arm_config=create,
            resolve_firmware_profile=Mock(return_value="resolved-firmware"),
        )
        interfaces = [{"usb_serial": "003400164648570220393831", "channel": "can41"}]
        with patch.dict("sys.modules", {"pyAgxArm": module}), patch.object(
            jog, "discover_interfaces", return_value=interfaces
        ), patch.object(jog, "run_test") as run:
            self.assertEqual(jog.main(["--execute"]), 0)
        factory.create_arm.assert_called_once()
        create.assert_called_once_with(
            robot="piper", firmeware_version="resolved-firmware",
            interface="socketcan", channel="can41", bitrate=1000000,
        )
        run.assert_called_once_with(self.sdk, 2.0, 10, False)


if __name__ == "__main__":
    unittest.main()
