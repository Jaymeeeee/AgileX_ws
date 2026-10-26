import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import unittest
from unittest import mock

import yaml

from piper_control.recording.manual_bag import main, record_bag, rosbag_command, ros_environment, topic_pattern


MODULE = "piper_control.recording.manual_bag"


def config():
    return {"robot": {"arms": ["right"]},
            "observation": {"camera_map": {"observation.images.top": "cam_h",
                                           "observation.images.wrist": "cam_r"}},
            "record": {"topic_prefix": "/piper", "python": "/usr/bin/python3",
                       "ros_setup": None, "compression": "lz4"}}


class ManualBagTests(unittest.TestCase):
    def test_topic_selection_excludes_other_arm_and_camera(self):
        pattern = re.compile(topic_pattern(config()))
        for topic in ("/piper/state", "/piper/action", "/piper/action_metadata", "/piper/events",
                      "/piper/arms/right/leader/command", "/piper/arms/right/follower/joint_states",
                      "/piper/arms/right/leader/diagnostics", "/piper/cameras/cam_h/color/image_raw",
                      "/piper/cameras/cam_r/aligned_depth_to_color/camera_info"):
            self.assertIsNotNone(pattern.fullmatch(topic), topic)
        for topic in ("/piper/arms/left/leader/command", "/piper/cameras/cam_l/color/image_raw",
                      "/piper/action_extra", "/other/state", "/piper/cameras/cam_right/color/image_raw"):
            self.assertIsNone(pattern.fullmatch(topic), topic)

    def test_regex_escapes_names_and_supports_empty_prefix(self):
        cfg = config()
        cfg["record"]["topic_prefix"] = ""
        cfg["observation"]["camera_map"] = {"x": "cam.r"}
        pattern = re.compile(topic_pattern(cfg))
        self.assertIsNotNone(pattern.fullmatch("/cameras/cam.r/color/image_raw"))
        self.assertIsNone(pattern.fullmatch("/cameras/camXr/color/image_raw"))
        self.assertIsNotNone(pattern.fullmatch("/state"))

    def test_shell_values_are_positional_arguments(self):
        cfg = config()
        cfg["record"]["python"] = "/tmp/python $(not_a_command)"
        argv = rosbag_command(cfg, ["record", "-O", "/tmp/a `not_a_command`.bag"])
        self.assertNotIn(cfg["record"]["python"], argv[4])
        self.assertIn(cfg["record"]["python"], argv)
        self.assertEqual(argv[-1], "/tmp/a `not_a_command`.bag")

    def test_existing_ros_environment_is_retained_without_setup_script(self):
        values = {"PYTHONPATH": "existing-ros-python", "ROS_PACKAGE_PATH": "existing-ros-packages",
                  "CMAKE_PREFIX_PATH": "existing-ros-prefix", "ROS_MASTER_URI": "http://host:11311"}
        with mock.patch.dict(os.environ, values):
            env = ros_environment(config())
        for key, value in values.items():
            self.assertEqual(env[key], value)
        self.assertEqual(env["PYTHONNOUSERSITE"], "1")

    def test_explicit_setup_isolates_python_and_package_paths(self):
        cfg = config()
        cfg["record"]["ros_setup"] = "/opt/ros/noetic/setup.bash"
        values = {"PYTHONPATH": "control-python", "ROS_PACKAGE_PATH": "control-packages",
                  "CMAKE_PREFIX_PATH": "control-prefix", "ROS_MASTER_URI": "http://host:11311"}
        with mock.patch.dict(os.environ, values):
            env = ros_environment(cfg)
        for key in ("PYTHONPATH", "ROS_PACKAGE_PATH", "CMAKE_PREFIX_PATH"):
            self.assertNotIn(key, env)
        self.assertEqual(env["ROS_MASTER_URI"], values["ROS_MASTER_URI"])

    def test_existing_recording_files_are_never_overwritten(self):
        for name in ("data.bag", "data.bag.active", "manifest.json", "rosbag.log"):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as root:
                path = Path(root) / name
                path.write_text("preserve")
                with mock.patch(MODULE + ".subprocess.Popen") as popen:
                    with self.assertRaises(FileExistsError):
                        record_bag(config(), root)
                    popen.assert_not_called()
                self.assertEqual(path.read_text(), "preserve")
                self.assertEqual(len(list(Path(root).iterdir())), 1)

    def test_startup_failure_writes_failed_manifest(self):
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "episode"
            with mock.patch(MODULE + ".subprocess.Popen", side_effect=OSError("cannot execute rosbag")):
                result = record_bag(config(), output)
            self.assertEqual(result["status"], "failed")
            self.assertIn("cannot execute", result["reason"])
            self.assertEqual(json.loads((output / "manifest.json").read_text()), result)
            self.assertTrue((output / "rosbag.log").is_file())

    def test_ctrl_c_only_stops_own_group_and_saves(self):
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "episode with spaces"
            proc = mock.Mock(pid=98765, returncode=0)

            def complete(**kwargs):
                (output / "data.bag").write_bytes(b"synthetic bag")
                return 0

            proc.wait.side_effect = [KeyboardInterrupt(), 0]
            info = subprocess.CompletedProcess([], 0, "topics:\n- topic: /piper/state\n  messages: 3\n", "")
            with mock.patch(MODULE + ".subprocess.Popen", return_value=proc) as popen, \
                    mock.patch(MODULE + ".os.killpg", side_effect=lambda *_: complete()) as killpg, \
                    mock.patch(MODULE + ".subprocess.run", return_value=info) as run, \
                    mock.patch.dict(os.environ, {"PYTHONPATH": "control-only", "ROS_MASTER_URI": "http://host:11311"}):
                result = record_bag(config(), output)
            self.assertEqual(result["status"], "complete")
            self.assertEqual(result["topic_counts"], {"/piper/state": 3})
            killpg.assert_called_once_with(proc.pid, signal.SIGINT)
            self.assertTrue(popen.call_args.kwargs["start_new_session"])
            self.assertEqual(popen.call_args.kwargs["env"]["PYTHONPATH"], "control-only")
            self.assertEqual(popen.call_args.kwargs["env"]["ROS_MASTER_URI"], "http://host:11311")
            self.assertIn("--buffsize", popen.call_args.args[0])
            self.assertIn("512", popen.call_args.args[0])
            self.assertIn("--lz4", popen.call_args.args[0])
            self.assertIn("--regex", popen.call_args.args[0])
            self.assertIn("info", run.call_args.args[0])
            self.assertEqual(json.loads((output / "manifest.json").read_text())["status"], "complete")

    def test_no_bag_is_failure_even_when_process_exits_zero_after_ctrl_c(self):
        with tempfile.TemporaryDirectory() as root:
            proc = mock.Mock(pid=98765, returncode=0)
            proc.wait.side_effect = [KeyboardInterrupt(), 0]
            with mock.patch(MODULE + ".subprocess.Popen", return_value=proc), \
                    mock.patch(MODULE + ".os.killpg"), mock.patch(MODULE + ".subprocess.run") as run:
                result = record_bag(config(), root)
            self.assertEqual(result["status"], "failed")
            self.assertIn("data.bag", result["reason"])
            run.assert_not_called()

    def test_unexpected_exit_is_failed(self):
        with tempfile.TemporaryDirectory() as root:
            proc = mock.Mock(pid=98765, returncode=1)
            with mock.patch(MODULE + ".subprocess.Popen", return_value=proc), \
                    mock.patch(MODULE + ".os.killpg"):
                result = record_bag(config(), root)
            self.assertEqual(result["status"], "failed")
            self.assertIn("before recording was stopped", result["reason"])

    def test_required_topics_must_all_have_messages_before_recording_is_complete(self):
        for counts, expected_missing in (
            ({"/piper/state": 3}, ["/piper/action", "/piper/cameras/cam_h/depth/image_raw"]),
            ({"/piper/state": 3, "/piper/action": 2}, ["/piper/cameras/cam_h/depth/image_raw"]),
            ({"/piper/state": 3, "/piper/action": 2, "/piper/cameras/cam_h/depth/image_raw": 4}, []),
        ):
            with self.subTest(counts=counts), tempfile.TemporaryDirectory() as root:
                output = Path(root) / "bag"
                topics = Path(root) / "topics.yaml"
                required = ["/piper/state", "/piper/action", "/piper/cameras/cam_h/depth/image_raw"]
                topics.write_text(yaml.safe_dump({"required": required, "optional": ["/piper/events"]}))
                cfg = config()
                cfg["record"]["topics_config"] = str(topics)
                proc = mock.Mock(pid=98765, returncode=0)
                proc.wait.side_effect = [KeyboardInterrupt(), 0]
                info = subprocess.CompletedProcess([], 0, yaml.safe_dump({"topics": [
                    {"topic": topic, "messages": count} for topic, count in counts.items()
                ]}), "")
                with mock.patch(MODULE + ".subprocess.Popen", return_value=proc), \
                        mock.patch(MODULE + ".os.killpg", side_effect=lambda *_: (output / "data.bag").write_bytes(b"bag")), \
                        mock.patch(MODULE + ".subprocess.run", return_value=info):
                    result = record_bag(cfg, output)
                self.assertEqual(result["status"], "incomplete" if expected_missing else "complete")
                self.assertEqual(result["missing_required_topics"], expected_missing)
                self.assertEqual(result["required_topics"], required)
                self.assertEqual(result["optional_topics"], ["/piper/events"])
                self.assertEqual(result["topic_counts"], counts)
                self.assertTrue((output / "data.bag").is_file())
                self.assertEqual(json.loads((output / "manifest.json").read_text()), result)

    def test_list_topics_needs_no_output_and_never_starts_recording(self):
        with tempfile.TemporaryDirectory() as root:
            topics = Path(root) / "topics.yaml"
            topics.write_text("required: ['{prefix}/state']\noptional: []\n")
            with mock.patch(MODULE + ".load_config", return_value=config()), \
                    mock.patch(MODULE + ".record_bag") as record, \
                    mock.patch(MODULE + ".print_topic_plan") as show:
                code = main(["--config", "collect.yaml", "--topics-config", str(topics), "--list-topics"])
            self.assertEqual(code, 0)
            record.assert_not_called()
            self.assertEqual(show.call_args.args[0]["required_topics"], ["/piper/state"])

    def test_incomplete_recording_returns_nonzero(self):
        result = {"status": "incomplete", "reason": "Missing required topics: /piper/action"}
        with mock.patch(MODULE + ".load_config", return_value=config()), \
                mock.patch(MODULE + ".record_bag", return_value=result), \
                mock.patch("sys.stderr"):
            self.assertEqual(main(["--config", "collect.yaml", "--output", "bag"]), 1)


if __name__ == "__main__":
    unittest.main()
