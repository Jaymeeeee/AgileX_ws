"""Exercise SDK alignment separately from raw-depth capture, without cameras."""

import importlib.util
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from piper_control.hardware.cameras import CameraRig, MockCameras


def config(**options):
    return {
        "camera_map": {"observation.images.top": "top"},
        "cameras": {"top": {"serial": "123"}},
        "width": 4,
        "height": 3,
        **options,
    }


class DepthOptionsTests(unittest.TestCase):
    def test_alignment_requires_depth_for_both_backends(self):
        for backend in (CameraRig, MockCameras):
            with self.subTest(backend=backend):
                with self.assertRaisesRegex(ValueError, "align_depth requires depth"):
                    backend(config(align_depth=True))

    def test_camera_overrides_global_depth_options(self):
        cfg = config(depth=True, align_depth=True)
        cfg["cameras"]["top"].update(depth=False, align_depth=False)
        for backend in (CameraRig, MockCameras):
            backend(cfg)
        cfg["cameras"]["top"].pop("align_depth")
        for backend in (CameraRig, MockCameras):
            with self.assertRaisesRegex(ValueError, "align_depth requires depth"):
                backend(cfg)


@unittest.skipUnless(importlib.util.find_spec("numpy"), "numpy is needed for images")
class CameraAlignmentTests(unittest.TestCase):
    def test_sdk_align_retains_raw_depth_and_sdk_timestamps(self):
        import numpy as np

        rig = CameraRig(config(depth=True, align_depth=True))

        def frame(array, timestamp_ms, focal):
            intrinsics = NS(
                width=array.shape[1],
                height=array.shape[0],
                fx=focal,
                fy=focal,
                ppx=2.0,
                ppy=1.5,
                coeffs=[0.0] * 5,
                model="brown_conrady",
            )
            return NS(
                get_data=lambda: array,
                get_timestamp=lambda: timestamp_ms,
                get_frame_timestamp_domain=lambda: "global",
                get_frame_number=lambda: 10,
                get_profile=lambda: NS(
                    as_video_stream_profile=lambda: NS(
                        get_intrinsics=lambda: intrinsics
                    )
                ),
            )

        rgb = np.zeros((3, 4, 3), dtype=np.uint8)
        raw = np.full((3, 4), 1000, dtype=np.uint16)
        projected = np.full((3, 4), 2000, dtype=np.uint16)
        color_frame = frame(rgb, 100250.0, 400.0)
        depth_frame = frame(raw, 100200.0, 300.0)
        aligned_frame = frame(projected, 100201.0, 400.0)
        frames = NS(
            get_color_frame=lambda: color_frame,
            get_depth_frame=lambda: depth_frame,
        )
        aligner = NS(
            process=Mock(return_value=NS(get_depth_frame=lambda: aligned_frame))
        )
        device = NS(
            get_info=lambda key: "123",
            query_sensors=lambda: [],
            first_depth_sensor=lambda: NS(get_depth_scale=lambda: 0.001),
        )
        profile = NS(get_device=lambda: device)
        sdk_config = NS(
            enable_device=Mock(), enable_stream=Mock(), resolve=lambda wrapper: profile
        )

        calls = 0

        def next_frames(timeout_ms):
            nonlocal calls
            calls += 1
            if calls > 1:
                rig._stop.set()
                return False, None
            return True, frames

        pipeline = NS(
            start=Mock(return_value=profile),
            try_wait_for_frames=next_frames,
            stop=Mock(),
        )
        rs = NS(
            pipeline=lambda: pipeline,
            config=lambda: sdk_config,
            pipeline_wrapper=lambda item: item,
            stream=NS(color="color", depth="depth"),
            format=NS(rgb8="rgb8", z16="z16"),
            camera_info=NS(serial_number="serial_number"),
            timestamp_domain=NS(hardware_clock="hardware"),
            align=Mock(return_value=aligner),
        )
        rig._capture("top", rig.selected["top"], rs, np)
        rig.check_health()
        rs.align.assert_called_once_with("color")
        aligner.process.assert_called_once_with(frames)
        pipeline.stop.assert_called_once_with()
        result = rig._frames["top"]
        np.testing.assert_array_equal(result["depth"], raw)
        np.testing.assert_array_equal(result["aligned_depth"], projected)
        self.assertEqual(result["camera_info"]["fx"], 400.0)
        self.assertEqual(result["depth_camera_info"]["fx"], 300.0)
        self.assertEqual(result["aligned_depth_camera_info"]["fx"], 400.0)
        self.assertAlmostEqual(result["stamp"], 100.250)
        self.assertAlmostEqual(result["depth_stamp"], 100.200)
        self.assertAlmostEqual(result["aligned_depth_stamp"], 100.201)
        self.assertEqual(result["depth_scale_m"], 0.001)
        for key in ("rgb", "depth", "aligned_depth"):
            self.assertFalse(result[key].flags.writeable)
        raw[:] = 0
        projected[:] = 0
        self.assertTrue((result["depth"] == 1000).all())
        self.assertTrue((result["aligned_depth"] == 2000).all())

    def test_mock_alignment_is_explicitly_simulated(self):
        rig = MockCameras(config(depth=True, align_depth=True))
        try:
            frame = rig.start().snapshot()["top"]
            self.assertEqual(frame["timestamp_source"], "mock_capture")
            self.assertEqual(frame["aligned_depth_camera_info"]["serial"], "MOCK-top")
            self.assertEqual(frame["aligned_depth"].shape, frame["rgb"].shape[:2])
            self.assertFalse(frame["aligned_depth"].flags.writeable)
            self.assertIsNot(frame["aligned_depth"], frame["depth"])
        finally:
            rig.close()


if __name__ == "__main__":
    unittest.main()
