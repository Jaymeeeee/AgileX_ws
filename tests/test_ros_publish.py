"""Offline checks for manual rosbag recording beside a ROS-only publisher."""

import io
from pathlib import Path
import tempfile
import time
import unittest
from unittest import mock

from piper_control.config import ConfigError, load_config
from piper_control.hardware.cameras import MockCameras
from piper_control.hardware.robot import MockRobot, RobotError
from piper_control.ipc import read_message, write_message
from piper_control.recording.client import CollectorClient
from piper_control.recording.ros_worker import serve
from piper_control.recording.session import RecordingSession
from piper_control.workflows.session import Session


CONFIG = Path(__file__).resolve().parents[1] / "configs" / "collect_right.yaml"


class ROSPublishTests(unittest.TestCase):
    def test_right_collection_automatically_aligns_then_enters_human_without_input(self):
        config = load_config(CONFIG, {"observation": {"width": 8, "height": 6}})
        self.assertTrue(config["workflow"]["auto_start"])
        events = []
        recording = mock.Mock(directory=None)
        recording.write_event.side_effect = events.append
        robot = MockRobot(config["robot"])
        calls = mock.Mock()
        for name in ("prepare_human_start", "enable", "command", "wait_for_alignment", "enter_human"):
            wrapped = mock.Mock(wraps=getattr(robot, name))
            setattr(robot, name, wrapped)
            calls.attach_mock(wrapped, name)
        session = Session(
            config, mock=True, robot=robot,
            cameras=MockCameras(config["observation"]), recording=recording,
        )
        result = session.run(max_steps=2, max_seconds=2.0, interactive=False)
        self.assertEqual(
            [call[0] for call in calls.mock_calls[:5]],
            ["prepare_human_start", "enable", "command", "wait_for_alignment", "enter_human"],
        )
        self.assertEqual(robot.arms, ["right"])
        self.assertEqual(result["steps"], 2)
        self.assertIsNone(result["directory"])
        self.assertTrue(any(e.get("control_state") == "HUMAN" for e in events))
        self.assertEqual(sum(e.get("type") == "human_action_ready" for e in events), 1)
        self.assertTrue(recording.publish_action.called)
        self.assertTrue(all(call.args[1] == "HUMAN" for call in recording.publish_action.call_args_list))
        recording.close.assert_called_once_with(error=None)

    def test_human_mode_without_control_frames_does_not_invent_actions(self):
        config = load_config(CONFIG, {"observation": {"width": 8, "height": 6}})

        class SilentLeader(MockRobot):
            def snapshot(self):
                result = super().snapshot()
                result.update(human_action=None, human_action_seq=None, human_action_stamp=None)
                return result

            def human_action(self):
                return None

        events = []
        recording = mock.Mock(directory=None)
        recording.write_event.side_effect = events.append
        session = Session(config, mock=True, robot=SilentLeader(config["robot"]),
                          cameras=MockCameras(config["observation"]), recording=recording)
        session.run(max_seconds=0.15, interactive=False)
        self.assertTrue(any(e.get("control_state") == "HUMAN" for e in events))
        self.assertFalse(any(e.get("type") == "human_action_ready" for e in events))
        recording.publish_action.assert_not_called()

    def test_failed_startup_alignment_stops_without_entering_human(self):
        config = load_config(CONFIG, {"observation": {"width": 8, "height": 6}})
        robot = MockRobot(config["robot"])
        robot.wait_for_alignment = mock.Mock(side_effect=RobotError("Startup alignment timed out"))
        robot.enter_human = mock.Mock(wraps=robot.enter_human)
        recording = mock.Mock(directory=None)
        session = Session(
            config, mock=True, robot=robot,
            cameras=MockCameras(config["observation"]), recording=recording,
        )
        with self.assertRaisesRegex(RobotError, "Startup alignment timed out"):
            session.run(max_seconds=0.5, interactive=False)
        robot.enter_human.assert_not_called()
        self.assertEqual(robot.mode, "STOPPED")
        self.assertFalse(robot.connected)
        self.assertEqual(session.controller.state, "FAULT")
        recording.close.assert_called_once_with(error="RobotError: Startup alignment timed out")

    def test_failed_enable_never_submits_alignment_target_or_enters_human(self):
        config = load_config(CONFIG, {"observation": {"width": 8, "height": 6}})
        robot = MockRobot(config["robot"])
        robot.enable = mock.Mock(side_effect=RobotError("Startup enable timed out"))
        robot.command = mock.Mock(wraps=robot.command)
        robot.wait_for_alignment = mock.Mock(wraps=robot.wait_for_alignment)
        robot.enter_human = mock.Mock(wraps=robot.enter_human)
        recording = mock.Mock(directory=None)
        session = Session(
            config, mock=True, robot=robot,
            cameras=MockCameras(config["observation"]), recording=recording,
        )
        with self.assertRaisesRegex(RobotError, "Startup enable timed out"):
            session.run(max_seconds=0.5, interactive=False)
        robot.command.assert_not_called()
        robot.wait_for_alignment.assert_not_called()
        robot.enter_human.assert_not_called()
        self.assertEqual(robot.mode, "STOPPED")
        self.assertFalse(robot.connected)
        recording.close.assert_called_once_with(error="RobotError: Startup enable timed out")

    def test_publishing_forwards_messages_without_creating_any_recording_files(self):
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "must-not-be-created"
            config = {
                "record": {"backend": "ros_publish", "output_dir": str(output)}
            }
            artifact_factory, collector_factory = mock.Mock(), mock.Mock()
            collector = collector_factory.return_value
            session = RecordingSession(
                config,
                artifact_factory=artifact_factory,
                collector_factory=collector_factory,
            ).start()
            snapshot = {"state": [0.0] * 7}
            frames = {"cam_r": {"stamp": 10.0}}
            action, metadata = [0.1] * 7, {"timestamp": 10.0}
            event = {"type": "episode_start"}
            session.publish_state(snapshot)
            session.publish_images(frames)
            session.publish_action(action, "HUMAN", metadata)
            session.write_event(event)
            session.write_step({"action": action})
            session.check_health()
            session.close()
            session.close()

            artifact_factory.assert_not_called()
            collector_factory.assert_called_once_with(config, None)
            collector.start.assert_called_once_with()
            collector.publish_state.assert_called_once_with(snapshot)
            collector.publish_images.assert_called_once_with(frames)
            collector.publish_action.assert_called_once_with(action, "HUMAN", metadata)
            collector.publish_event.assert_called_once_with(event)
            collector.check_health.assert_called_once_with()
            collector.close.assert_called_once_with()
            self.assertIsNone(session.directory)
            self.assertFalse(output.exists())

    def test_client_start_preserves_null_directory_in_worker_message(self):
        config = {"record": {"backend": "ros_publish", "ros_setup": None}}
        client = CollectorClient(config, None)
        worker = mock.Mock(stdin=io.BytesIO())
        with mock.patch(
            "piper_control.recording.client.subprocess.Popen", return_value=worker
        ), mock.patch("piper_control.recording.client.threading.Thread"), mock.patch.object(
            client, "_reply", return_value={"kind": "ready", "seq": 0}
        ):
            client.start()
        worker.stdin.seek(0)
        message = read_message(worker.stdin)
        self.assertEqual(message["kind"], "start")
        self.assertIsNone(message["directory"])
        self.assertEqual(message["config"], config)
        self.assertIsNone(client.directory)

    def test_worker_publishes_without_opening_bag_and_preserves_old_bag_backend(self):
        for config, directory, automatic_bag in (
            ({"record": {"backend": "ros_publish"}}, None, False),
            ({"record": {"backend": "ros"}}, "/tmp/unused-bag-test", True),
            ({}, "/tmp/unused-bag-test", True),
        ):
            with self.subTest(config=config):
                incoming, outgoing = io.BytesIO(), io.BytesIO()
                bridge, bridge_factory = mock.Mock(), mock.Mock()
                bridge_factory.return_value = bridge
                state = {"state": [0.0] * 7}
                action = {"action": [0.1] * 7, "source": "HUMAN", "metadata": {}}
                frames, event = {"cam_r": {"stamp": 10.0}}, {"type": "episode_start"}
                messages = [
                    {"kind": "start", "config": config, "directory": directory},
                    {"kind": "state", "payload": state},
                    {"kind": "images", "payload": frames},
                    {"kind": "action", "payload": action},
                    {"kind": "event", "payload": event},
                    {"kind": "close"},
                ]
                for sequence, message in enumerate(messages):
                    write_message(incoming, dict(message, seq=sequence))
                incoming.seek(0)
                serve(incoming, outgoing, bridge_factory=bridge_factory)
                outgoing.seek(0)
                replies = [read_message(outgoing) for _ in messages]
                self.assertEqual(
                    [reply["kind"] for reply in replies],
                    ["ready", "ack", "ack", "ack", "ack", "closed"],
                )
                bridge_factory.assert_called_once_with(config)
                bridge.start.assert_called_once_with()
                if automatic_bag:
                    bridge.start_bag.assert_called_once_with(directory)
                else:
                    bridge.start_bag.assert_not_called()
                bridge.publish_state.assert_called_once_with(state)
                bridge.publish_images.assert_called_once_with(frames)
                bridge.publish_action.assert_called_once_with(
                    action["action"], action["source"], action["metadata"]
                )
                bridge.publish_event.assert_called_once_with(event)
                bridge.close.assert_called_once_with()

    def test_manual_publish_workflows_allow_no_episode_deadline(self):
        for mode in ("collect", "teleop"):
            with self.subTest(mode=mode):
                config = load_config(
                    CONFIG,
                    {
                        "record": {"backend": "ros_publish"},
                        "workflow": {"mode": mode, "max_episode_sec": None},
                    },
                )
                self.assertEqual(config["record"]["backend"], "ros_publish")
                self.assertIsNone(config["workflow"]["max_episode_sec"])

    def test_online_learning_requires_files_and_other_workflows_keep_deadline(self):
        cases = [
            {
                "record": {"backend": "ros_publish"},
                "policy": {"type": "pi05_rlt"},
                "workflow": {
                    "mode": "online_rl",
                    "learner": "pi05_rlt",
                    "max_episode_sec": 120,
                },
            },
            {
                "record": {"backend": "ros_publish"},
                "workflow": {"mode": "deploy", "max_episode_sec": None},
            },
        ]
        for mode in ("collect", "teleop"):
            for backend in ("files", "ros"):
                cases.append(
                    {
                        "record": {"backend": backend},
                        "workflow": {"mode": mode, "max_episode_sec": None},
                    }
                )
        for overrides in cases:
            with self.subTest(overrides=overrides), self.assertRaises(ConfigError):
                load_config(CONFIG, overrides)

    def test_deployment_can_publish_with_a_finite_episode_deadline(self):
        config = load_config(
            CONFIG,
            {
                "record": {"backend": "ros_publish"},
                "workflow": {"mode": "deploy", "max_episode_sec": 120},
            },
        )
        self.assertEqual(config["record"]["backend"], "ros_publish")
        self.assertEqual(config["workflow"]["mode"], "deploy")
        self.assertEqual(config["workflow"]["max_episode_sec"], 120)

    def test_manual_control_remains_active_past_old_deadline_until_user_quits(self):
        class OldEpisodeSession(Session):
            def _begin_episode(self):
                super()._begin_episode()
                self.episode_started = time.monotonic() - 300.0

        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "not-recording"
            config = load_config(
                CONFIG,
                {
                    "record": {"backend": "ros_publish", "output_dir": str(output)},
                    "observation": {"width": 8, "height": 6},
                    "workflow": {"max_episode_sec": None, "auto_start": False},
                },
            )
            events = []
            recording = mock.Mock(directory=None)
            recording.write_event.side_effect = events.append
            session = OldEpisodeSession(
                config,
                mock=True,
                robot=MockRobot(config["robot"]),
                cameras=MockCameras(config["observation"]),
                recording=recording,
                command_schedule=[(0.0, "h"), (0.08, "hold"), (0.14, "h"), (0.25, "q")],
            )
            original_command = session.command
            commands = []

            def command(value, snapshot, frames):
                commands.append(value)
                original_command(value, snapshot, frames)

            session.command = command
            result = session.run(max_seconds=1.0, interactive=False)
            self.assertEqual(commands, ["h", "hold", "h", "q"])
            self.assertEqual(result["episodes"], 1)
            self.assertGreater(result["steps"], 0)
            self.assertIsNone(result["directory"])
            self.assertEqual(len([e for e in events if e.get("type") == "episode_end"]), 1)
            self.assertFalse(output.exists())
            recording.close.assert_called_once_with(error=None)


if __name__ == "__main__":
    unittest.main()
