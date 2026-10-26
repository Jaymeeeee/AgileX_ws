import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from piper_control.config import component_names, load_config
from piper_control.hardware.operations import (ACTION_SEMANTICS, execute_trajectory,
                                              home_trajectory, load_trajectory, main)
from piper_control.hardware.recover import recover_software_stop, main as recover_main


def config():
    return load_config(Path(__file__).resolve().parents[1] / "configs" / "mock.yaml")["robot"]


def document(cfg, actions=None, stamps=None):
    return dict(action_names=component_names(cfg["arms"]), action_semantics=ACTION_SEMANTICS,
                actions=actions if actions is not None else [[.02, 0, 0, 0, 0, 0, .002]],
                timestamps=[0.] if stamps is None else stamps)


class Clock:
    def __init__(self):
        self.now = 10.

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class Commands:
    def __init__(self, clock):
        self.clock, self.calls = clock, []

    def connect(self):
        self.calls.append(("connect",))

    def enable(self):
        self.calls.append(("enable",))

    def command(self, action):
        self.calls.append(("command", self.clock.now, np.asarray(action).copy()))
        return action

    def stop(self):
        self.calls.append(("stop",))

    def close(self):
        self.calls.append(("close",))


class OperationsTests(unittest.TestCase):
    def load(self, values, cfg=None, suffix=".json"):
        cfg = config() if cfg is None else cfg
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / ("trajectory" + suffix)
            if suffix == ".npz":
                np.savez(str(path), **values)
            else:
                path.write_text(json.dumps(values))
            return load_trajectory(path, cfg)

    def test_trajectory_requires_explicit_order_units_shape_and_timestamps(self):
        cfg = config()
        good = document(cfg)
        self.assertEqual(self.load(good)["actions"].shape, (1, 7))
        bad = []
        item = copy.deepcopy(good); item["action_names"].reverse(); bad.append(item)
        item = copy.deepcopy(good); item["action_semantics"] = "degrees"; bad.append(item)
        item = copy.deepcopy(good); item["actions"][0].pop(); bad.append(item)
        item = copy.deepcopy(good); item["actions"][0][0] = float("nan"); bad.append(item)
        item = copy.deepcopy(good); item["timestamps"] = [1.]; bad.append(item)
        bad.append(document(cfg, [[0.] * 7, [0.] * 7], [0., 0.]))
        for item in bad:
            with self.subTest(item=item), self.assertRaises(ValueError):
                self.load(item)

    def test_targets_and_timing_are_not_clipped_or_velocity_limited(self):
        cfg = config()
        actions = [[0.] * 7, [3.5, -4., 0., 0., 0., 0., .2]]
        result = self.load(document(cfg, actions, [0., .001]))
        np.testing.assert_array_equal(result["actions"], actions)

    def test_npz_schema_and_explicit_home_pose(self):
        cfg = config()
        self.assertEqual(self.load(document(cfg), suffix=".npz")["actions"].shape, (1, 7))
        cfg.pop("home_position", None)
        with self.assertRaises(ValueError):
            home_trajectory(cfg)
        cfg["home_position"] = {"left": [.02, 0, 0, 0, 0, 0, .002]}
        np.testing.assert_allclose(home_trajectory(cfg)["actions"], document(cfg)["actions"])

    def test_replay_sends_only_listed_targets_at_timestamps_then_waits_and_closes(self):
        cfg, clock = config(), Clock()
        robot = Commands(clock)
        actions = [[3.5, 0, 0, 0, 0, 0, .2], [-4., 0, 0, 0, 0, 0, .3]]
        path = document(cfg, actions, [0., .04])
        with patch("piper_control.hardware.operations.time.monotonic", clock.monotonic), \
                patch("piper_control.hardware.operations.time.sleep", clock.sleep):
            result = execute_trajectory(cfg, path, robot, hold_seconds=.5)
        self.assertEqual([call[0] for call in robot.calls],
                         ["connect", "enable", "command", "command", "stop", "close"])
        writes = robot.calls[2:4]
        np.testing.assert_allclose([item[1] for item in writes], [10., 10.04])
        np.testing.assert_array_equal([item[2] for item in writes], actions)
        self.assertEqual(result["commands"], 2)
        self.assertAlmostEqual(result["elapsed_sec"], .54)
        np.testing.assert_array_equal(result["final_target"], actions[-1])

    def test_home_is_one_command_without_feedback_reads(self):
        cfg, clock = config(), Clock()
        cfg["home_position"] = {"left": [.02, 0, 0, 0, 0, 0, .002]}
        robot = Commands(clock)
        with patch("piper_control.hardware.operations.time.monotonic", clock.monotonic), \
                patch("piper_control.hardware.operations.time.sleep", clock.sleep):
            result = execute_trajectory(cfg, home_trajectory(cfg), robot)
        self.assertEqual(result["commands"], 1)
        self.assertAlmostEqual(result["elapsed_sec"], 2.)

    def test_command_exception_still_stops_and_closes(self):
        cfg, clock = config(), Clock()
        robot = Commands(clock)
        robot.command = Mock(side_effect=RuntimeError("transport failed"))
        with self.assertRaisesRegex(RuntimeError, "transport failed"):
            execute_trajectory(cfg, document(cfg), robot, hold_seconds=0.)
        self.assertEqual(robot.calls[-2:], [("stop",), ("close",)])

    def test_preview_never_constructs_sdk_robot(self):
        path = Path(__file__).resolve().parents[1] / "configs" / "home.yaml"
        with patch("piper_control.hardware.operations.SDKRobot") as sdk, patch("sys.stdout", new=io.StringIO()):
            self.assertEqual(main(["home", "--config", str(path)]), 0)
        sdk.assert_not_called()

    def test_recovery_only_issues_reset_without_feedback_or_stop(self):
        leader, follower = Mock(), Mock()
        robot = Mock(devices={("left", "leader"): {"sdk": leader},
                              ("left", "follower"): {"sdk": follower}})
        result = recover_software_stop(robot)
        self.assertEqual(result["reset_commands"], ["left/leader", "left/follower"])
        self.assertEqual([call[0] for call in leader.mock_calls], ["reset"])
        self.assertEqual([call[0] for call in follower.mock_calls], ["reset"])
        robot.snapshot.assert_not_called()
        robot.stop.assert_not_called()

    def test_recovery_cli_connects_resets_and_disconnects(self):
        path = Path(__file__).resolve().parents[1] / "configs" / "mock.yaml"
        robot = Mock(devices={("left", "follower"): {"sdk": Mock()}})
        with patch("piper_control.hardware.recover.SDKRobot", return_value=robot), \
                patch("sys.stdout", new=io.StringIO()):
            self.assertEqual(recover_main(["--config", str(path), "--execute"]), 0)
        robot.connect.assert_called_once_with()
        robot.devices[("left", "follower")]["sdk"].reset.assert_called_once_with()
        robot.stop.assert_not_called()
        robot.enable.assert_not_called()
        robot.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
