"""Run with the LeRobot Python environment; ROS is not required."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import bag_to_lerobot as converter


class TimingAndDecodingTests(unittest.TestCase):
    def test_actions_are_causal_and_images_choose_nearest(self):
        stamps = np.array([10, 20, 30])
        grid = np.array([9, 15, 19, 30, 40])
        np.testing.assert_array_equal(converter.sample_indices(stamps, grid, "previous"), [-1, 0, 0, 2, 2])
        np.testing.assert_array_equal(converter.sample_indices(stamps, grid, "nearest"), [0, 0, 1, 2, 2])

    def test_joint_names_reorder_and_validate(self):
        msg = SimpleNamespace(position=[2., 1.], name=["j2", "j1"])
        spec = {"order_field": "name", "order": ["j1", "j2"]}
        np.testing.assert_array_equal(converter.extract_vector(msg, "position", spec, 2), [1, 2])
        msg.name = ["j1", "j1"]
        with self.assertRaises(ValueError):
            converter.extract_vector(msg, "position", spec, 2)

    def test_same_stamp_with_conflicting_values_is_rejected(self):
        s = converter.Series(stamps=[10, 10], bag_stamps=[11, 12], ordinals=[0, 1],
                             values={"position": [np.array([1]), np.array([2])]})
        with self.assertRaisesRegex(ValueError, "相同时间戳"):
            s.finish()

    def test_depth_preserves_endianness_padding_and_invalid_zero(self):
        data = np.array([[0, 65535, 99], [1234, 4321, 99]], dtype=">u2")
        msg = SimpleNamespace(height=2, width=2, step=6, encoding="16UC1", is_bigendian=True,
                              data=np.frombuffer(data.tobytes(), dtype=np.uint8))
        arr = converter.decode_image(msg, "depth")
        self.assertEqual(arr.dtype, np.uint16)
        np.testing.assert_array_equal(arr[..., 0], [[0, 65535], [1234, 4321]])

    def test_rgb_padding_and_bgr_conversion(self):
        msg = SimpleNamespace(height=1, width=1, step=4, encoding="bgr8", is_bigendian=False,
                              data=np.array([10, 20, 30, 99], dtype=np.uint8))
        np.testing.assert_array_equal(converter.decode_image(msg, "rgb"), [[[30, 20, 10]]])

    def test_interior_gap_is_not_removed_from_time_axis(self):
        s = converter.Series(stamps=np.array([1, 2, 10], dtype=np.int64) * converter.NS)
        cfg = {"dataset": {"fps": 1}, "sources": {"state": {"sample": "previous", "max_age_sec": 0.5}},
               "sync": {"min_frames": 2}}
        with self.assertRaisesRegex(ValueError, "拒绝跨缺失段"):
            converter.build_plan({"state": s}, {"bag_start_ns": converter.NS, "bag_end_ns": 10*converter.NS}, cfg)

    def test_latched_target_keeps_original_stamp(self):
        series = {"action": converter.Series(stamps=np.array([1]) * converter.NS),
                  "state": converter.Series(stamps=np.array([10, 11, 12]) * converter.NS)}
        cfg = {"dataset": {"fps": 1}, "sources": {
            "action": {"topic": "/action", "sample": "previous", "max_age_sec": None, "hold_last": True},
            "state": {"topic": "/state", "sample": "previous", "max_age_sec": 0.1}}, "sync": {}}
        grid, ids, summary = converter.build_plan(series, {"bag_start_ns": 10*converter.NS, "bag_end_ns": 12*converter.NS}, cfg)
        np.testing.assert_array_equal(ids["action"], [0, 0, 0])
        self.assertEqual(summary["action"]["max_abs_delta_ms"], 11000)
        np.testing.assert_array_equal(series["action"].stamps, [converter.NS])


class SyntheticBagRoundtrip(unittest.TestCase):
    def test_real_rosbag_to_lerobot_including_uint16_depth(self):
        from rosbags.rosbag1 import Writer
        from rosbags.typesys import Stores, get_typestore
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        with tempfile.TemporaryDirectory(prefix="bag_lerobot_test_") as tmp:
            root = Path(tmp)
            episode = root / "task_0001"
            episode.mkdir()
            bag = episode / "data.bag"
            (episode / "manifest.json").write_text(json.dumps({"state": "complete", "calibration": "fixture"}))
            ts = get_typestore(Stores.ROS1_NOETIC)
            Time = ts.types["builtin_interfaces/msg/Time"]
            Header = ts.types["std_msgs/msg/Header"]
            Joint = ts.types["sensor_msgs/msg/JointState"]
            Image = ts.types["sensor_msgs/msg/Image"]

            def header(stamp):
                return Header(seq=0, stamp=Time(sec=stamp//converter.NS, nanosec=stamp%converter.NS), frame_id="camera")

            def joints(stamp, value):
                return Joint(header(stamp), ["j2", "j1"], np.array([value+1, value]), np.array([], dtype=float), np.array([], dtype=float))

            rgb = np.tile(np.array([[[17, 53, 91]]], dtype=np.uint8), (4, 4, 1))
            depth = np.array([[0, 1, 5000, 65535]]*4, dtype=np.uint16)
            with Writer(bag) as writer:
                conns = {topic: writer.add_connection(topic, kind, typestore=ts)
                         for topic, kind in [("/state", "sensor_msgs/msg/JointState"),
                                             ("/action", "sensor_msgs/msg/JointState"),
                                             ("/rgb", "sensor_msgs/msg/Image"), ("/depth", "sensor_msgs/msg/Image")]}
                def write(topic, stamp, msg):
                    c = conns[topic]
                    writer.write(c, stamp, ts.serialize_ros1(msg, c.msgtype))
                # Latched command is received now but retains a header 1 second old.
                write("/action", 10*converter.NS, joints(9*converter.NS, 100.))
                for i in range(5):
                    stamp = 10*converter.NS + i*100_000_000
                    write("/state", stamp, joints(stamp, float(i)))
                    if i == 2:
                        write("/action", stamp, joints(stamp, 200.))
                    write("/rgb", stamp, Image(header(stamp), 4, 4, "rgb8", 0, 12, rgb.reshape(-1)))
                    write("/depth", stamp, Image(header(stamp), 4, 4, "16UC1", 0, 8, depth.view(np.uint8).reshape(-1)))
            cfg = {
                "schema_version": 1,
                "input": {"bag_glob": str(root / "task_*" / "data.bag"), "require_complete_manifest": True},
                "dataset": {"root": str(root / "dataset"), "repo_id": "local/fixture", "fps": 10, "task": "insert plug"},
                "sync": {"max_clock_skew_sec": 2.0},
                "sources": {
                    "state": {"topic": "/state", "sample": "previous", "max_age_sec": 0.11, "order_field": "name", "order": ["j1", "j2"]},
                    "action": {"topic": "/action", "sample": "previous", "max_age_sec": None, "hold_last": True, "order_field": "name", "order": ["j1", "j2"]},
                    "rgb": {"topic": "/rgb", "sample": "nearest", "max_age_sec": 0.05},
                    "depth": {"topic": "/depth", "sample": "nearest", "max_age_sec": 0.05}},
                "vectors": {"observation.state": [{"source": "state", "field": "position", "names": ["j1", "j2"]}],
                            "action": [{"source": "action", "field": "position", "names": ["j1", "j2"]}]},
                "images": {"observation.images.cam": {"source": "rgb", "kind": "rgb", "storage": "image"},
                           "observation.depth.cam": {"source": "depth", "kind": "depth", "storage": "image"}},
                "video": {"vcodec": "h264", "crf": 18}}
            config = root / "config.yaml"
            config.write_text(yaml.safe_dump(cfg))
            args = argparse.Namespace(config=str(config), output=None, episodes=None, limit_episodes=None, max_frames=None, dry_run=False)
            converter.run(args)
            ds = LeRobotDataset("local/fixture", root=root / "dataset", video_backend="pyav")
            self.assertEqual(len(ds), 5)
            for i in range(5):
                frame = ds[i]
                np.testing.assert_array_equal(frame["observation.state"], [i, i+1])
                target = 100 if i < 2 else 200
                np.testing.assert_array_equal(frame["action"], [target, target+1])
                np.testing.assert_array_equal(frame["observation.depth.cam"].numpy()[0], depth)
                np.testing.assert_allclose(frame["observation.images.cam"].numpy(), rgb.transpose(2,0,1)/255, atol=1e-7)
            timing = np.load(root / "dataset/meta/source/episode_000000/timing.npz")
            self.assertEqual(timing["action__source_timestamp_ns"][0], 9*converter.NS)
            self.assertTrue(bag.exists())
            with self.assertRaisesRegex(ValueError, "输出目录已存在"):
                converter.run(args)
            # Mixed RGB video + lossless depth image uses a different writer path.
            cfg["images"]["observation.images.cam"]["storage"] = "video"
            cfg["dataset"]["root"] = str(root / "mixed_dataset")
            config.write_text(yaml.safe_dump(cfg))
            converter.run(args)
            mixed = LeRobotDataset("local/fixture", root=root / "mixed_dataset", video_backend="pyav")
            np.testing.assert_array_equal(mixed[4]["observation.depth.cam"].numpy()[0], depth)
            self.assertEqual(tuple(mixed[4]["observation.images.cam"].shape), (3, 4, 4))
            # Missing topics must fail before creating a dataset.
            cfg["sources"]["state"]["topic"] = "/missing"
            cfg["dataset"]["root"] = str(root / "missing_dataset")
            config.write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError, "缺失话题"):
                converter.run(args)
            self.assertFalse((root / "missing_dataset").exists())


if __name__ == "__main__":
    unittest.main()
