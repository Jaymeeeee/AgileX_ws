import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from piper_control.recording.artifacts import ArtifactRecorder
from piper_control.recording.export_dataset import ExportError, export_dataset


class FakeDataset:
    created = []

    @classmethod
    def create(cls, **options):
        obj = cls()
        obj.options = options
        obj.frames = []
        obj.episodes = 0
        obj.finalized = False
        Path(options["root"]).mkdir(parents=True)
        cls.created.append(obj)
        return obj

    def add_frame(self, frame):
        self.frames.append(frame)

    def save_episode(self, **kwargs):
        self.episodes += 1

    def finalize(self):
        self.finalized = True


def recording(root, terminal=True, error=None, arms=None):
    arms = arms or ["left"]
    dimension = 7 * len(arms)
    config = {"record": {"backend": "files", "output_dir": str(root), "min_free_gb": 0},
              "robot": {"arms": arms}, "workflow": {"policy_hz": 10},
              "observation": {"task": "insert the plug", "camera_map": {"observation.images.top": "top"}}}
    writer = ArtifactRecorder(config["record"], config).start()
    def observation(stamp, value):
        return {"state": [value] * dimension, "stamp": stamp, "frames": {"top": {
            "rgb": np.full((4, 5, 3), 17, dtype=np.uint8), "stamp": stamp, "monotonic": stamp,
            "camera_info": {"width": 5, "height": 4}}}}
    for index, source in enumerate(("BASE", "HUMAN")):
        writer.write_step({"episode_id": 9, "step_id": index, "timestamp": float(index + 2),
                           "observation": observation(float(index + 1), .01 * index),
                           "next_observation": observation(float(index + 2), .01 * (index + 1)),
                           "action": [.02 + .01 * index] * dimension,
                           "predicted_action": [9.] * dimension if source == "BASE" else None,
                           "source": source, "intervention": source == "HUMAN", "generation": index,
                           "execution_boundary": True, "segment_start": True,
                           "reward": 0., "duration_sec": .1})
    writer.write_event({"type": "episode_end", "episode_id": 9, "steps": 2,
                        "terminal": terminal, "truncated": not terminal,
                        "reward": 1., "success": terminal, "reward_in_final_step": False,
                        "final_observation": observation(4., .05)})
    writer.close(error=error)
    return writer.directory


class DatasetExportTests(unittest.TestCase):
    def test_executed_action_sources_and_final_observation_survive_export(self):
        with tempfile.TemporaryDirectory() as root:
            session = recording(Path(root) / "sessions")
            output = Path(root) / "dataset"
            result = export_dataset([session], output, "local/recording", dataset_factory=FakeDataset)
            dataset = FakeDataset.created[-1]
            self.assertEqual(result["frames"], 2)
            self.assertEqual(dataset.episodes, 1)
            self.assertTrue(dataset.finalized)
            np.testing.assert_allclose(dataset.frames[0]["action"], .02)
            np.testing.assert_allclose(dataset.frames[0]["metadata.predicted_action"], 9.)
            np.testing.assert_allclose(dataset.frames[1]["action"], .03)
            np.testing.assert_allclose(dataset.frames[1]["next.observation.state"], .05)
            self.assertEqual(dataset.frames[1]["metadata.next_observation_timestamp"][0], 4.)
            self.assertEqual([frame["metadata.action_source"][0] for frame in dataset.frames], [0, 2])
            self.assertEqual([frame["next.reward"][0] for frame in dataset.frames], [0., 1.])
            self.assertFalse(dataset.frames[1]["metadata.prediction_available"][0])
            self.assertEqual(dataset.options["features"]["action"]["names"][-1], "left_gripper_width")
            self.assertTrue((output / "meta/conversion.json").is_file())

    def test_truncation_and_failed_sessions_require_explicit_options(self):
        with tempfile.TemporaryDirectory() as root:
            session = recording(Path(root) / "sessions", terminal=False, error="recording stopped after complete episode")
            output = Path(root) / "dataset"
            with self.assertRaisesRegex(ExportError, "allow-failed"):
                export_dataset([session], output, "local/recording", dataset_factory=FakeDataset)
            with self.assertRaisesRegex(ExportError, "include-truncated"):
                export_dataset([session], output, "local/recording", allow_failed=True, dataset_factory=FakeDataset)
            result = export_dataset([session], output, "local/recording", allow_failed=True,
                                     include_truncated=True, dataset_factory=FakeDataset)
            self.assertEqual(result["frames"], 2)
            self.assertTrue(FakeDataset.created[-1].frames[-1]["next.truncated"][0])
            self.assertFalse(FakeDataset.created[-1].frames[-1]["next.done"][0])

    def test_missing_steps_and_images_fail_before_output_creation(self):
        with tempfile.TemporaryDirectory() as root:
            session = recording(Path(root) / "sessions")
            trajectory = session / "trajectory.jsonl"
            rows = [json.loads(line) for line in trajectory.read_text().splitlines()]
            rows[1]["step_id"] = 3
            trajectory.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
            output = Path(root) / "dataset"
            with self.assertRaisesRegex(ExportError, "gap"):
                export_dataset([session], output, "local/recording", dataset_factory=FakeDataset)
            self.assertFalse(output.exists())
            rows[1]["step_id"] = 1
            rows[0]["observation"]["frames"]["top"]["rgb_path"] = "../outside.ppm"
            trajectory.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
            with self.assertRaisesRegex(ExportError, "escapes"):
                export_dataset([session], output, "local/recording", dataset_factory=FakeDataset)
            self.assertFalse(output.exists())

    def test_equal_dimension_different_arm_contract_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            left = recording(Path(root) / "sessions", arms=["left"])
            right = recording(Path(root) / "sessions", arms=["right"])
            with self.assertRaisesRegex(ExportError, "different arm"):
                export_dataset([left, right], Path(root) / "dataset", "local/recording", dataset_factory=FakeDataset)


try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    HAS_LEROBOT = True
except (ImportError, SyntaxError):
    HAS_LEROBOT = False


@unittest.skipUnless(HAS_LEROBOT, "Requires the model environment's LeRobot dataset dependencies")
class RealDatasetTests(unittest.TestCase):
    def test_local_lerobot_roundtrip_actual_actions_images_and_metadata(self):
        with tempfile.TemporaryDirectory() as root:
            session = recording(Path(root) / "sessions")
            output = Path(root) / "dataset"
            result = export_dataset([session], output, "local/piper-test")
            dataset = LeRobotDataset("local/piper-test", root=output)
            self.assertEqual(len(dataset), 2)
            self.assertEqual(dataset.meta.total_episodes, 1)
            self.assertEqual(result["frames"], 2)
            first, final = dataset[0], dataset[1]
            np.testing.assert_allclose(first["action"].numpy(), .02)
            np.testing.assert_allclose(final["action"].numpy(), .03)
            np.testing.assert_allclose(final["next.observation.state"].numpy(), .05)
            self.assertEqual(final["metadata.action_source"].item(), 2)
            self.assertEqual(final["metadata.next_observation_timestamp"].item(), 4.)
            self.assertEqual(final["next.reward"].item(), 1.)
            self.assertEqual(tuple(first["observation.images.top"].shape), (3, 4, 5))
            np.testing.assert_allclose(first["observation.images.top"].numpy(), 17. / 255., atol=1e-6)


if __name__ == "__main__":
    unittest.main()
