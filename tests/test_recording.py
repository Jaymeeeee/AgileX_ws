import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

from piper_control.hardware.cameras import CameraRig, MockCameras, _capture_stamp
from piper_control.recording.artifacts import ArtifactRecorder as EpisodeRecorder
from piper_control.recording.ros_bridge import ROSBridge, _BagWriter


HAS_NUMPY = importlib.util.find_spec("numpy") is not None
HAS_ROS = importlib.util.find_spec("rosbag") is not None


class CameraTests(unittest.TestCase):
    def test_sdk_hardware_clock_is_not_replaced_with_receive_time(self):
        rs = types.SimpleNamespace(timestamp_domain=types.SimpleNamespace(hardware_clock="hardware"))
        frame = types.SimpleNamespace(get_frame_timestamp_domain=lambda: "hardware",
                                      get_timestamp=lambda: 12345.0)
        with self.assertRaisesRegex(ValueError, "global clock"):
            _capture_stamp(frame, rs)
        frame.get_frame_timestamp_domain = lambda: "global"
        frame.get_timestamp = lambda: 99500.0
        with mock.patch("piper_control.hardware.cameras.time.time", return_value=110.):
            self.assertEqual(_capture_stamp(frame, rs), 99.5)
        frame.get_timestamp = lambda: float("nan")
        with self.assertRaisesRegex(ValueError, "invalid"):
            _capture_stamp(frame, rs)

    def test_camera_cache_retains_capture_stamp_without_age_limit(self):
        rig = CameraRig({"camera_map": {"observation.images.top": "top"},
                         "cameras": {"top": {"serial": "123"}}})
        rig._started = True
        rig._frames = {"top": {"stamp": 100., "monotonic": 10., "rgb": b"image"}}
        with mock.patch("piper_control.hardware.cameras.time.monotonic", return_value=10.1):
            self.assertEqual(rig.snapshot()["top"]["stamp"], 100.)
        with mock.patch("piper_control.hardware.cameras.time.monotonic", return_value=11.):
            self.assertEqual(rig.snapshot()["top"]["stamp"], 100.)
            self.assertEqual(rig.snapshot()["top"]["monotonic"], 10.)
        rig._frames.clear()
        with self.assertRaisesRegex(RuntimeError, "no captured frame"):
            rig.snapshot()
        rig._errors["top"] = IOError("USB disconnected")
        with self.assertRaisesRegex(RuntimeError, "USB disconnected"):
            rig.check_health()

    @unittest.skipUnless(HAS_NUMPY, "numpy is needed for synthetic images")
    def test_mock_only_opens_selected_camera_names(self):
        rig = MockCameras({"camera_map": {"observation.images.top": "top"},
                           "cameras": {"top": {}, "unused": {}}, "width": 4, "height": 3,
                           "depth": True})
        rig.start()
        try:
            images = rig.snapshot()
            self.assertEqual(list(images), ["top"])
            self.assertEqual(images["top"]["rgb"].shape, (3, 4, 3))
            self.assertFalse(images["top"]["rgb"].flags.writeable)
            self.assertEqual(images["top"]["depth_scale_m"], .001)
        finally:
            rig.close()


class RecorderTests(unittest.TestCase):
    def _recorder(self, directory, **options):
        config = {"output_dir": directory, "min_free_gb": 0, "queue_size": 10}
        config.update(options)
        return EpisodeRecorder(config, {"policy": {"type": "act"}, "robot": {"arms": ["left"]}})

    @unittest.skipUnless(HAS_NUMPY, "numpy is needed for image recording")
    def test_trajectory_retains_actual_action_and_images_and_terminal_flags(self):
        import numpy as np
        with tempfile.TemporaryDirectory() as directory:
            recorder = self._recorder(directory).start()
            frame = {"rgb": np.full((2, 3, 3), 7, dtype=np.uint8),
                     "depth": np.full((2, 3), 100, dtype=np.uint16),
                     "stamp": 20., "monotonic": 10., "depth_stamp": 20.,
                     "depth_scale_m": .001, "camera_info": {"serial": "mock"}}
            recorder.write_event({"event": "episode_start", "timestamp": 20.})
            recorder.write_step({"step_id": 0, "action": np.array([.1, .2]),
                                 "predicted_action": [.3, .4], "source": "HUMAN",
                                 "observation": {"state": [0., 1.], "frames": {"top": frame}},
                                 "next_observation": {"frames": {"top": frame}},
                                 "reward": 1., "terminated": False, "truncated": True})
            # A queued action/frame snapshot must not follow caller mutation.
            frame["rgb"][:] = 0
            recorder.close()
            rows = [json.loads(line) for line in (recorder.directory / "trajectory.jsonl").read_text().splitlines()]
            step = rows[1]
            self.assertEqual(step["action"], [.1, .2])
            self.assertEqual(step["predicted_action"], [.3, .4])
            self.assertTrue(step["truncated"])
            saved = step["observation"]["frames"]["top"]
            self.assertNotIn("rgb", saved)
            self.assertNotIn("aligned_depth_path", saved)
            self.assertNotIn("aligned_depth_stamp", saved)
            self.assertEqual(saved["rgb_path"], step["next_observation"]["frames"]["top"]["rgb_path"])
            self.assertEqual((recorder.directory / saved["rgb_path"]).read_bytes(), b"P6\n3 2\n255\n" + bytes([7]) * 18)
            self.assertTrue(np.all(np.load(str(recorder.directory / saved["depth_path"]), allow_pickle=False) == 100))
            manifest = json.loads((recorder.directory / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "complete")
            self.assertEqual(manifest["steps"], 1)
            self.assertNotIn("aligned_depth", manifest["image_format"])
            second = self._recorder(directory).start()
            self.assertNotEqual(recorder.directory, second.directory)
            second.close(error=RuntimeError("interrupted"))
            self.assertEqual(json.loads((second.directory / "manifest.json").read_text())["status"], "failed")

    @unittest.skipUnless(HAS_NUMPY, "numpy is needed for image recording")
    def test_aligned_depth_is_saved_as_owned_raw_npy_with_source_metadata(self):
        import numpy as np

        with tempfile.TemporaryDirectory() as directory:
            recorder = self._recorder(directory).start()
            calibration = {
                "width": 3,
                "height": 2,
                "fx": 3.0,
                "fy": 2.0,
                "ppx": 1.5,
                "ppy": 1.0,
            }
            raw = np.array([[0, 100], [200, 300]], dtype=np.uint16)
            aligned = np.array([[0, 200, 300], [400, 500, 600]], dtype=np.uint16)
            frame = {
                "rgb": np.full((2, 3, 3), 7, dtype=np.uint8),
                "stamp": 20.0,
                "depth": raw,
                "depth_stamp": 19.9,
                "depth_scale_m": 0.001,
                "aligned_depth": aligned.copy(),
                "aligned_depth_stamp": 19.8,
                "aligned_depth_monotonic": 9.8,
                "aligned_depth_camera_info": calibration,
            }
            recorder.write_step(
                {
                    "step_id": 0,
                    "observation": {"frames": {"top": frame}},
                    "next_observation": {"frames": {"top": frame}},
                }
            )
            # Updating only the aligned source timestamp must store a new frame.
            frame["aligned_depth"][:] = 999
            frame["aligned_depth_stamp"] = 19.81
            recorder.write_step(
                {
                    "step_id": 1,
                    "observation": {"frames": {"top": frame}},
                }
            )
            frame["aligned_depth"][:] = 0
            recorder.close()

            rows = [
                json.loads(line)
                for line in (recorder.directory / "trajectory.jsonl")
                .read_text()
                .splitlines()
            ]
            saved = rows[0]["observation"]["frames"]["top"]
            next_saved = rows[0]["next_observation"]["frames"]["top"]
            updated = rows[1]["observation"]["frames"]["top"]
            self.assertNotIn("aligned_depth", saved)
            self.assertEqual(
                saved["aligned_depth_path"], next_saved["aligned_depth_path"]
            )
            self.assertNotEqual(
                saved["aligned_depth_path"], updated["aligned_depth_path"]
            )
            self.assertEqual(saved["aligned_depth_stamp"], 19.8)
            self.assertEqual(updated["aligned_depth_stamp"], 19.81)
            self.assertEqual(saved["aligned_depth_monotonic"], 9.8)
            self.assertEqual(saved["aligned_depth_camera_info"], calibration)
            self.assertEqual(saved["depth_scale_m"], 0.001)
            loaded = np.load(
                recorder.directory / saved["aligned_depth_path"], allow_pickle=False
            )
            self.assertEqual(loaded.dtype, np.uint16)
            np.testing.assert_array_equal(loaded, aligned)
            np.testing.assert_array_equal(
                np.load(recorder.directory / saved["depth_path"], allow_pickle=False),
                raw,
            )
            self.assertTrue(
                np.all(
                    np.load(
                        recorder.directory / updated["aligned_depth_path"],
                        allow_pickle=False,
                    )
                    == 999
                )
            )
            self.assertEqual(
                len(
                    list(
                        (recorder.directory / "images/top").glob("*.aligned_depth.npy")
                    )
                ),
                2,
            )
            manifest = json.loads((recorder.directory / "manifest.json").read_text())
            self.assertIn("aligned_depth", manifest["image_format"])

    def test_nonfinite_values_cannot_enter_json(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = self._recorder(directory).start()
            try:
                with self.assertRaisesRegex(ValueError, "Nonfinite"):
                    recorder.write_step({"action": [float("nan")]})
            finally:
                recorder.close()

    def test_worker_io_error_is_reported_and_manifest_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = self._recorder(directory)
            with mock.patch.object(recorder, "_extract_frames", side_effect=OSError("disk failed")):
                recorder.start()
                recorder.write_step({"action": [0.]})
                recorder._thread.join(timeout=2.)
                with self.assertRaisesRegex(RuntimeError, "disk failed"):
                    recorder.check_health()
                with self.assertRaisesRegex(RuntimeError, "disk failed"):
                    recorder.close()
            manifest = json.loads((recorder.directory / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "failed")
            self.assertEqual(manifest["steps"], 0)

    def test_queue_overflow_is_explicit_and_existing_rows_are_finalized(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = self._recorder(directory, queue_size=1)
            entered, release = threading.Event(), threading.Event()
            original = recorder._extract_frames

            def blocked(row, path=()):
                entered.set()
                release.wait(timeout=2.)
                return original(row, path)

            with mock.patch.object(recorder, "_extract_frames", side_effect=blocked):
                recorder.start()
                recorder.write_step({"step_id": 0})
                self.assertTrue(entered.wait(timeout=2.))
                recorder.write_step({"step_id": 1})
                with self.assertRaisesRegex(RuntimeError, "queue is full"):
                    recorder.write_step({"step_id": 2})
                release.set()
                with self.assertRaisesRegex(RuntimeError, "queue is full"):
                    recorder.close()
            rows = (recorder.directory / "trajectory.jsonl").read_text().splitlines()
            self.assertEqual(len(rows), 2)


class _Message:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.values = []
        self.status = []


class BridgeTests(unittest.TestCase):
    def bridge(self, config=None):
        bridge = ROSBridge(config or {"ros": True})
        bridge._started = True
        bridge.rospy = types.SimpleNamespace(Time=types.SimpleNamespace(from_sec=lambda value: value),
                                            is_shutdown=lambda: False)
        bridge.messages = dict.fromkeys(("joint", "image", "info", "string", "diagnostics", "status", "keyvalue"), _Message)
        bridge._emit = mock.Mock()
        return bridge

    def test_state_uses_sdk_stamp_and_separates_leader_command(self):
        bridge = self.bridge()
        follower = {"position": [0.] * 7, "stamp": 100., "monotonic": 10., "valid": True, "source": "feedback"}
        leader = dict(follower, source="command")
        with mock.patch("piper_control.recording.ros_bridge.time.monotonic", return_value=10.1):
            bridge.publish_state({"arms": {"left": {"follower": follower, "leader": leader}}})
            calls = bridge._emit.call_args_list
            self.assertEqual([call.args[0] for call in calls],
                             ["arms/left/follower/joint_states", "arms/left/leader/command", "state"])
            self.assertTrue(all(call.args[2].header.stamp == 100. for call in calls))
            bridge._emit.reset_mock()
            bridge.publish_state({"arms": {"left": {"follower": follower, "leader": leader}}})
            self.assertFalse(bridge._emit.called, "Reused SDK cache must not create new samples")
        with mock.patch("piper_control.recording.ros_bridge.time.monotonic", return_value=12.):
            bridge.publish_state({"arms": {"left": {"follower": dict(follower, stamp=101.)}}})
            self.assertEqual(bridge._emit.call_count, 2)
            self.assertTrue(all(call.args[2].header.stamp == 101. for call in bridge._emit.call_args_list))

    def test_diagnostics_preserve_source_and_timestamps_without_age_filter(self):
        bridge = self.bridge()
        follower = {"stamp": 100., "source": "feedback",
                    "gripper_feedback": {"stamp": 99.99, "valid": True, "mode": "width", "value": .02},
                    "arm_status": {"stamp": 100., "valid": True, "err_code": 0},
                    "motor_feedback": [{"stamp": 99.98, "valid": True, "torque": 1.}]}
        leader = {"stamp": 100., "source": "command",
                  "gripper_feedback": {"stamp": 100.01, "valid": True, "status_code": 1, "value": .03},
                  "arm_status": {"stamp": 100.02, "valid": True, "mode_feedback": 1}}
        unavailable = {"stamp": 100., "source": "feedback",
                       "gripper_feedback": {"stamp": 100., "valid": False, "value": .04},
                       "arm_status": {"stamp": 90., "valid": True, "err_code": 0}}
        with mock.patch("piper_control.recording.ros_bridge.time.time", return_value=100.1):
            bridge._publish_feedback("left", "follower", follower)
            bridge._publish_feedback("left", "leader", leader)
            bridge._publish_feedback("right", "follower", unavailable)
        self.assertEqual(bridge._emit.call_count, 3)
        statuses = {status.name: status for call in bridge._emit.call_args_list
                    for status in call.args[2].status}
        self.assertEqual(set(statuses), {"left/follower/motor_feedback/1", "left/follower/gripper_feedback",
                                         "left/follower/arm_status", "left/leader/gripper_command",
                                         "left/leader/arm_status", "right/follower/gripper_feedback",
                                         "right/follower/arm_status"})
        values = {name: {item.key: json.loads(item.value) for item in status.values}
                  for name, status in statuses.items()}
        self.assertEqual(values["left/follower/gripper_feedback"]["stamp"], 99.99)
        self.assertEqual(values["left/follower/gripper_feedback"]["value"], .02)
        self.assertEqual(values["left/leader/gripper_command"]["source"], "command")
        self.assertEqual(values["left/leader/gripper_command"]["stamp"], 100.01)
        self.assertEqual(values["left/leader/arm_status"]["source"], "feedback")
        self.assertEqual(values["left/leader/arm_status"]["stamp"], 100.02)
        self.assertEqual(values["right/follower/arm_status"]["stamp"], 90.)
        self.assertFalse(values["right/follower/gripper_feedback"]["valid"])
        self.assertIn("SDK command", statuses["left/leader/gripper_command"].message)

    def test_missing_optional_leader_and_diagnostics_do_not_drop_follower(self):
        follower = {"position": [.1] * 7, "stamp": 100., "valid": False,
                    "source": "feedback", "gripper_feedback": {"stamp": None, "valid": False},
                    "arm_status": None, "motor_feedback": [None, {"stamp": None, "valid": False}],
                    "driver_feedback": None}
        for leader in ({"position": None, "stamp": None, "valid": False},
                       {"position": None, "stamp": 100., "valid": False},
                       {"position": [0.] * 7, "stamp": None, "valid": False}):
            with self.subTest(leader=leader):
                bridge = self.bridge()
                bridge.publish_state({"arms": {"left": {"follower": follower, "leader": leader}}})
                calls = bridge._emit.call_args_list
                self.assertEqual([call.args[0] for call in calls],
                                 ["arms/left/follower/joint_states", "state"])
                self.assertTrue(all(call.args[2].header.stamp == 100. for call in calls))
                self.assertTrue(all(call.args[2].position == [.1] * 7 for call in calls))

    @unittest.skipUnless(HAS_NUMPY, "numpy is needed for image messages")
    def test_image_stamp_depth_units_and_duplicate_suppression(self):
        import numpy as np
        bridge = self.bridge()
        frame = {"rgb": np.zeros((2, 3, 3), dtype=np.uint8), "stamp": 100., "monotonic": 10.,
                 "camera_info": {"width": 3, "height": 2, "fx": 3., "fy": 2., "ppx": 1.5, "ppy": 1.},
                 "depth": np.array([[0, 100, 200], [300, 400, 500]], dtype=np.uint16),
                 "depth_stamp": 99.99, "depth_monotonic": 9.99, "depth_scale_m": .01}
        frame["depth_camera_info"] = frame["camera_info"]
        with mock.patch("piper_control.recording.ros_bridge.time.monotonic", return_value=1000.):
            bridge.publish_images({"top": frame})
            messages = {call.args[0]: call.args[2] for call in bridge._emit.call_args_list}
            color = messages["cameras/top/color/image_raw"]
            depth = messages["cameras/top/depth/image_raw"]
            self.assertEqual(color.header.stamp, 100.)
            self.assertEqual(depth.header.stamp, 99.99)
            self.assertEqual(depth.encoding, "32FC1")
            values = np.frombuffer(depth.data, dtype="<f4")
            self.assertTrue(np.isnan(values[0]))
            self.assertEqual(values[1], 1.)
            bridge._emit.reset_mock()
            bridge.publish_images({"top": frame})
            self.assertFalse(bridge._emit.called)

    def test_bag_failure_surfaces_and_finalizes(self):
        bag = mock.Mock()
        bag.write.side_effect = OSError("bag disk full")
        fake_module = types.SimpleNamespace(Bag=mock.Mock(return_value=bag))
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict("sys.modules", {"rosbag": fake_module}):
            writer = _BagWriter(Path(directory) / "capture.bag", {"min_free_gb": 0})
            writer.put("/test", "message", 100.)
            writer.thread.join(timeout=2.)
            with self.assertRaisesRegex(RuntimeError, "bag disk full"):
                writer.check_health()
            with self.assertRaisesRegex(RuntimeError, "bag disk full"):
                writer.close()
            bag.close.assert_called_once()
            report = json.loads((Path(directory) / "capture_report.json").read_text())
            self.assertEqual(report["status"], "failed")
            self.assertEqual(report["topic_counts"], {})

    def test_bag_topic_selection_does_not_filter_ros_publication(self):
        bag = mock.Mock()
        fake_module = types.SimpleNamespace(Bag=mock.Mock(return_value=bag))
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict("sys.modules", {"rosbag": fake_module}):
            topics = Path(directory) / "topics.yaml"
            topics.write_text("required: ['{prefix}/action']\noptional: ['{prefix}/state']\n")
            bridge = self.bridge({
                "robot": {"arms": ["right"]}, "observation": {"camera_map": {}},
                "record": {"topics_config": str(topics), "topic_prefix": "/piper", "min_free_gb": 0},
            })
            # Exercise the real emitter with fake ROS publishers and bag I/O.
            del bridge._emit
            publications = {}

            def publisher(topic, kind, **kwargs):
                sink = mock.Mock()
                sink.publish.side_effect = lambda message: publications.setdefault(topic, []).append(message)
                return sink

            bridge.rospy.Publisher = publisher
            bridge.start_bag(directory)
            bridge.publish_action([.1] * 7, "HUMAN", {"timestamp": 100.0})
            bridge._emit("state", "joint", _Message(), 100.0)
            bridge.publish_event({"type": "mode", "timestamp": 100.0})
            bridge.close()
            self.assertEqual(set(publications), {"/piper/action", "/piper/action_metadata",
                                                "/piper/state", "/piper/events"})
            self.assertEqual([call.args[0] for call in bag.write.call_args_list], ["/piper/action", "/piper/state"])
            report = json.loads((Path(directory) / "capture_report.json").read_text())
            self.assertEqual(report["status"], "complete")
            self.assertEqual(report["required_topics"], ["/piper/action"])
            self.assertEqual(report["optional_topics"], ["/piper/state"])
            self.assertEqual(report["topic_counts"], {"/piper/action": 1, "/piper/state": 1})
            self.assertEqual(report["missing_required_topics"], [])

    def test_bag_missing_required_action_writes_incomplete_report_and_raises(self):
        bag = mock.Mock()
        fake_module = types.SimpleNamespace(Bag=mock.Mock(return_value=bag))
        plan = {"topics_config": "topics.yaml", "required_topics": ["/state", "/action"], "optional_topics": []}
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict("sys.modules", {"rosbag": fake_module}):
            writer = _BagWriter(Path(directory) / "capture.bag", {"min_free_gb": 0}, plan=plan)
            writer.put("/state", "message", 100.0)
            with self.assertRaisesRegex(RuntimeError, "ROS bag incomplete: Missing required topics: /action"):
                writer.close()
            report = json.loads((Path(directory) / "capture_report.json").read_text())
            self.assertEqual(report["status"], "incomplete")
            self.assertEqual(report["topic_counts"], {"/state": 1})
            self.assertEqual(report["missing_required_topics"], ["/action"])
            bag.close.assert_called_once()

    def test_bag_counts_only_successful_writes_even_when_later_write_fails(self):
        bag = mock.Mock()
        bag.write.side_effect = [None, OSError("second write failed")]
        fake_module = types.SimpleNamespace(Bag=mock.Mock(return_value=bag))
        plan = {"topics_config": "topics.yaml", "required_topics": ["/state", "/action"], "optional_topics": []}
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict("sys.modules", {"rosbag": fake_module}):
            writer = _BagWriter(Path(directory) / "capture.bag", {"min_free_gb": 0}, plan=plan)
            writer.put("/state", "first", 100.0)
            writer.put("/action", "second", 100.0)
            writer.thread.join(timeout=2.)
            with self.assertRaisesRegex(RuntimeError, "second write failed"):
                writer.close()
            report = json.loads((Path(directory) / "capture_report.json").read_text())
            self.assertEqual(report["status"], "failed")
            self.assertEqual(report["topic_counts"], {"/state": 1})
            self.assertEqual(report["missing_required_topics"], ["/action"])

    def test_bag_without_topic_config_keeps_all_topics(self):
        bag = mock.Mock()
        fake_module = types.SimpleNamespace(Bag=mock.Mock(return_value=bag))
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict("sys.modules", {"rosbag": fake_module}):
            writer = _BagWriter(Path(directory) / "capture.bag", {"min_free_gb": 0})
            writer.put("/external/custom", "message", 100.0)
            writer.close()
            report = json.loads((Path(directory) / "capture_report.json").read_text())
            self.assertEqual(report["status"], "complete")
            self.assertEqual(report["required_topics"], [])
            self.assertEqual(report["topic_counts"], {"/external/custom": 1})

    @unittest.skipUnless(HAS_NUMPY, "numpy is needed for image messages")
    def test_three_cameras_publish_independent_raw_and_aligned_depth(self):
        import numpy as np

        bridge = self.bridge()
        color_info = {
            "width": 3,
            "height": 2,
            "fx": 3.0,
            "fy": 2.0,
            "ppx": 1.5,
            "ppy": 1.0,
        }
        depth_info = {
            "width": 2,
            "height": 2,
            "fx": 1.0,
            "fy": 1.5,
            "ppx": 1.0,
            "ppy": 0.5,
        }
        frames = {
            name: {
                "rgb": np.full((2, 3, 3), 17, dtype=np.uint8),
                "stamp": 100.0 + index,
                "camera_info": color_info,
                "depth": np.array([[0, 100], [200, 300]], dtype=np.uint16),
                "depth_stamp": 99.9 + index,
                "depth_camera_info": depth_info,
                "depth_scale_m": 0.01,
                "aligned_depth": np.array(
                    [[0, 200, 300], [400, 500, 600]], dtype=np.uint16
                ),
                "aligned_depth_stamp": 99.8 + index,
                "aligned_depth_camera_info": color_info,
            }
            for index, name in enumerate(("cam_h", "cam_l", "cam_r"))
        }
        bridge.publish_images(frames)
        messages = {call.args[0]: call.args[2] for call in bridge._emit.call_args_list}
        self.assertEqual(len(messages), 18)
        for index, name in enumerate(frames):
            prefix = "cameras/{}/".format(name)
            for kind, stamp, info, optical_frame in (
                ("color", 100.0 + index, color_info, "color"),
                ("depth", 99.9 + index, depth_info, "depth"),
                ("aligned_depth_to_color", 99.8 + index, color_info, "color"),
            ):
                with self.subTest(camera=name, stream=kind):
                    image = messages[prefix + kind + "/image_raw"]
                    calibration = messages[prefix + kind + "/camera_info"]
                    self.assertEqual(image.header.stamp, stamp)
                    self.assertEqual(calibration.header.stamp, stamp)
                    frame_id = name + "_" + optical_frame + "_optical_frame"
                    self.assertEqual(image.header.frame_id, frame_id)
                    self.assertEqual(calibration.header.frame_id, frame_id)
                    self.assertEqual(
                        (image.width, image.height), (info["width"], info["height"])
                    )
                    self.assertEqual(calibration.width, image.width)
                    self.assertEqual(calibration.height, image.height)
                    self.assertEqual(calibration.K[0], info["fx"])
                    self.assertEqual(calibration.K[4], info["fy"])
            raw = messages[prefix + "depth/image_raw"]
            aligned = messages[prefix + "aligned_depth_to_color/image_raw"]
            self.assertEqual(raw.encoding, "32FC1")
            self.assertEqual(aligned.encoding, "32FC1")
            raw_values = np.frombuffer(raw.data, dtype="<f4")
            aligned_values = np.frombuffer(aligned.data, dtype="<f4")
            self.assertTrue(np.isnan(raw_values[0]))
            self.assertTrue(np.isnan(aligned_values[0]))
            self.assertEqual(raw_values[1], 1.0)
            self.assertEqual(aligned_values[1], 2.0)

        bridge._emit.reset_mock()
        bridge.publish_images(frames)
        bridge._emit.assert_not_called()

        frames["cam_l"]["aligned_depth_stamp"] += 0.01
        bridge.publish_images(frames)
        self.assertEqual(
            [call.args[0] for call in bridge._emit.call_args_list],
            [
                "cameras/cam_l/aligned_depth_to_color/image_raw",
                "cameras/cam_l/aligned_depth_to_color/camera_info",
            ],
        )


@unittest.skipUnless(
    HAS_ROS and HAS_NUMPY, "ROS Noetic messages and numpy are optional"
)
class NoeticSerializationTests(unittest.TestCase):
    def test_real_bag_roundtrip_with_isolated_publishers_and_capture_stamps(self):
        import numpy as np
        import rosbag
        import rospy

        publications = {}

        def publisher(topic, message_type, **kwargs):
            sink = mock.Mock()
            sink.publish.side_effect = lambda message: publications.setdefault(
                topic, []
            ).append(message)
            return sink

        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            rospy, "get_master", return_value=mock.Mock()
        ), mock.patch.object(
            rospy.core, "is_initialized", return_value=True
        ), mock.patch.object(
            rospy, "get_param", return_value=False
        ), mock.patch.object(
            rospy, "is_shutdown", return_value=False
        ), mock.patch.object(
            rospy, "Publisher", side_effect=publisher
        ):
            bridge = ROSBridge(
                {
                    "record": {
                        "topic_prefix": "/piper",
                        "min_free_gb": 0,
                        "compression": "lz4",
                    }
                }
            ).start()
            bridge.start_bag(directory)
            # Worker/queue delay does not change or discard source timestamps.
            follower = {
                "position": [0.1] * 6 + [0.02],
                "stamp": 100.0,
                "monotonic": 10.0,
                "valid": True,
                "source": "feedback",
                "gripper_feedback": {"stamp": 99.99, "valid": True, "value": 0.02},
                "arm_status": {"stamp": 100.0, "valid": True, "err_code": 0},
            }
            leader = dict(follower, source="command")
            bridge.publish_state(
                {"arms": {"left": {"follower": follower, "leader": leader}}}
            )
            calibration = {
                "width": 3,
                "height": 2,
                "fx": 3.0,
                "fy": 2.0,
                "ppx": 1.5,
                "ppy": 1.0,
            }
            bridge.publish_images(
                {
                    "top": {
                        "rgb": np.full((2, 3, 3), 17, dtype=np.uint8),
                        "stamp": 99.98,
                        "monotonic": 9.98,
                        "camera_info": calibration,
                        "depth": np.full((2, 3), 100, dtype=np.uint16),
                        "depth_stamp": 99.97,
                        "depth_monotonic": 9.97,
                        "depth_scale_m": 0.001,
                        "depth_camera_info": calibration,
                        "aligned_depth": np.full((2, 3), 200, dtype=np.uint16),
                        "aligned_depth_stamp": 99.96,
                        "aligned_depth_camera_info": calibration,
                    }
                }
            )
            bridge.publish_action([0.2] * 6 + [0.03], "HUMAN", {"timestamp": 100.02})
            bridge.publish_event({"event": "episode_end", "timestamp": 100.03})
            bridge.close()
            with rosbag.Bag(str(Path(directory) / "capture.bag"), "r") as bag:
                rows = {
                    topic: (message, stamp.to_sec())
                    for topic, message, stamp in bag.read_messages()
                }
            self.assertEqual(set(rows), set(publications))
            image, stamp = rows["/piper/cameras/top/color/image_raw"]
            self.assertAlmostEqual(stamp, 99.98)
            self.assertAlmostEqual(image.header.stamp.to_sec(), 99.98)
            self.assertEqual(image.data, bytes([17]) * 18)
            depth = rows["/piper/cameras/top/depth/image_raw"][0]
            np.testing.assert_allclose(np.frombuffer(depth.data, dtype="<f4"), 0.1)
            self.assertEqual(depth.encoding, "32FC1")
            aligned, stamp = rows["/piper/cameras/top/aligned_depth_to_color/image_raw"]
            aligned_info, info_stamp = rows[
                "/piper/cameras/top/aligned_depth_to_color/camera_info"
            ]
            self.assertAlmostEqual(stamp, 99.96)
            self.assertAlmostEqual(info_stamp, 99.96)
            self.assertAlmostEqual(aligned.header.stamp.to_sec(), 99.96)
            self.assertAlmostEqual(aligned_info.header.stamp.to_sec(), 99.96)
            self.assertEqual(aligned.header.frame_id, "top_color_optical_frame")
            self.assertEqual(aligned_info.header.frame_id, "top_color_optical_frame")
            self.assertEqual(aligned.encoding, "32FC1")
            np.testing.assert_allclose(np.frombuffer(aligned.data, dtype="<f4"), 0.2)
            self.assertEqual(rows["/piper/action"][0].position, (0.2,) * 6 + (0.03,))
            self.assertEqual(
                json.loads(rows["/piper/action_metadata"][0].data)["source"], "HUMAN"
            )
            diagnostics = rows["/piper/arms/left/leader/diagnostics"][0]
            self.assertIn(
                "left/leader/gripper_command",
                [entry.name for entry in diagnostics.status],
            )
            self.assertEqual(
                rows["/piper/arms/left/leader/command"][0].position[-1], 0.02
            )


if __name__ == "__main__":
    unittest.main()
