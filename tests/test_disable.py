import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from piper_control.hardware.disable import disable_motors, main
from piper_control.hardware.robot import RobotError


CONFIG = Path(__file__).resolve().parents[1] / "configs" / "collect.yaml"


def right_robot():
    devices = {}
    for role in ("follower", "leader"):
        sdk, gripper = Mock(), Mock()
        # These are pre-command cached feedback, not command acknowledgements.
        sdk.disable.return_value = False
        gripper.disable_gripper.return_value = False
        devices[("right", role)] = dict(sdk=sdk, gripper=gripper)
    return SimpleNamespace(devices=devices)


class DisableTests(unittest.TestCase):
    def test_only_selected_devices_receive_disable_without_motion_or_reset(self):
        robot = right_robot()
        result = disable_motors(robot)
        self.assertEqual(
            result,
            {
                "joint_disable_commands": ["right/follower", "right/leader"],
                "gripper_disable_commands": ["right/follower", "right/leader"],
                "feedback_verified": False,
            },
        )
        for device in robot.devices.values():
            self.assertEqual(device["sdk"].method_calls, [call.disable()])
            self.assertEqual(
                device["gripper"].method_calls, [call.disable_gripper()]
            )

    def test_failures_do_not_skip_other_commands_or_devices(self):
        robot = right_robot()
        robot.devices[("right", "follower")]["sdk"].disable.side_effect = (
            RuntimeError("joint send failed")
        )
        robot.devices[("right", "leader")]["gripper"].disable_gripper.side_effect = (
            RuntimeError("gripper send failed")
        )
        with self.assertRaises(RobotError) as raised:
            disable_motors(robot)
        message = str(raised.exception)
        for detail in ("right/follower", "joints", "right/leader", "gripper"):
            self.assertIn(detail, message)
        for device in robot.devices.values():
            device["sdk"].disable.assert_called_once_with()
            device["gripper"].disable_gripper.assert_called_once_with()

    def test_real_robot_close_after_manual_disable_only_disconnects(self):
        from tests.test_sdk import fake_robot

        robot, calls, made = fake_robot(arms=["right"])
        for sdk in made:
            sdk.disable = Mock(return_value=False)
            sdk.disable_gripper = Mock(return_value=False)
        try:
            disable_motors(robot)
            self.assertFalse(robot.touched)
        finally:
            robot.close()
        for sdk in made:
            sdk.disable.assert_called_once_with()
            sdk.disable_gripper.assert_called_once_with()
            self.assertEqual(sdk.sent, [])
        self.assertEqual(
            calls,
            [(sdk.name, "connect") for sdk in made]
            + [(sdk.name, "disconnect") for sdk in made],
        )
        self.assertFalse(robot.connected)

    def test_preview_never_constructs_sdk_or_discovers_hardware(self):
        with patch("piper_control.hardware.disable.SDKRobot") as sdk, patch(
            "piper_control.hardware.disable.disable_motors"
        ) as disable, patch(
            "piper_control.hardware.interfaces.discover_interfaces"
        ) as discover, patch("sys.stdout", new=io.StringIO()):
            self.assertEqual(main(["--config", str(CONFIG)]), 0)
        sdk.assert_not_called()
        disable.assert_not_called()
        discover.assert_not_called()

    def test_execute_connects_disables_then_closes(self):
        robot = Mock()
        order = Mock()
        order.attach_mock(robot, "robot")
        with patch(
            "piper_control.hardware.disable.SDKRobot", return_value=robot
        ) as sdk, patch(
            "piper_control.hardware.disable.disable_motors",
            return_value={"feedback_verified": False},
        ) as disable, patch("sys.stdout", new=io.StringIO()):
            order.attach_mock(disable, "disable")
            self.assertEqual(main(["--config", str(CONFIG), "--execute"]), 0)
        sdk.assert_called_once()
        self.assertEqual(
            order.mock_calls,
            [call.robot.connect(), call.disable(robot), call.robot.close()],
        )

    def test_disable_error_still_closes_and_returns_failure(self):
        robot = Mock()
        with patch(
            "piper_control.hardware.disable.SDKRobot", return_value=robot
        ), patch(
            "piper_control.hardware.disable.disable_motors",
            side_effect=RobotError("right/follower joints: unavailable"),
        ), patch("sys.stdout", new=io.StringIO()), patch(
            "sys.stderr", new=io.StringIO()
        ):
            self.assertEqual(main(["--config", str(CONFIG), "--execute"]), 1)
        robot.connect.assert_called_once_with()
        robot.close.assert_called_once_with()

    def test_connect_error_still_closes_without_disabling(self):
        robot = Mock()
        robot.connect.side_effect = RuntimeError("CAN unavailable")
        with patch(
            "piper_control.hardware.disable.SDKRobot", return_value=robot
        ), patch(
            "piper_control.hardware.disable.disable_motors"
        ) as disable, patch("sys.stdout", new=io.StringIO()), patch(
            "sys.stderr", new=io.StringIO()
        ):
            self.assertEqual(main(["--config", str(CONFIG), "--execute"]), 1)
        disable.assert_not_called()
        robot.close.assert_called_once_with()

    def test_close_error_returns_failure(self):
        robot = Mock()
        robot.close.side_effect = RuntimeError("disconnect failed")
        with patch(
            "piper_control.hardware.disable.SDKRobot", return_value=robot
        ), patch(
            "piper_control.hardware.disable.disable_motors",
            return_value={"feedback_verified": False},
        ) as disable, patch("sys.stdout", new=io.StringIO()), patch(
            "sys.stderr", new=io.StringIO()
        ):
            self.assertEqual(main(["--config", str(CONFIG), "--execute"]), 1)
        disable.assert_called_once_with(robot)
        robot.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
