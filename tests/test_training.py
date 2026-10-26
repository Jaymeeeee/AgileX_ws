"""Training launcher checks do not load model weights or start real training."""
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from piper_control.learning.train import launch_training


class TrainingTests(unittest.TestCase):
    def fixture(self, directory):
        root = Path(directory)
        (root / "dataset" / "meta").mkdir(parents=True)
        (root / "dataset" / "meta" / "info.json").write_text("{}")
        cfg = {"policy": {"type": "act", "pretrained_backbone_weights": None},
               "dataset": {"repo_id": "local/test", "root": "dataset"},
               "output_dir": "training output", "steps": 2}
        path = root / "train.json"
        path.write_text(json.dumps(cfg))
        return root, path, cfg

    def test_dry_run_resolves_local_paths_without_starting_training(self):
        with tempfile.TemporaryDirectory() as directory:
            root, path, cfg = self.fixture(directory)
            with patch("piper_control.learning.train.subprocess.run") as run:
                result = launch_training(path, dry_run=True, extra_args=["--batch_size=2"])
            run.assert_not_called()
            self.assertEqual(result["policy_type"], "act")
            self.assertEqual(result["dataset_root"], str(root / "dataset"))
            self.assertEqual(result["output_dir"], str(root / "training output"))
            self.assertIn("--batch_size=2", result["command"])
            self.assertEqual(json.loads(path.read_text()), cfg)

    def test_missing_local_data_and_model_type_mismatch_fail_before_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            root, path, cfg = self.fixture(directory)
            with self.assertRaisesRegex(ValueError, "prepared LeRobot dataset"):
                launch_training(path, dry_run=True, extra_args=["--dataset.root=missing"])
            checkpoint = root / "checkpoint"
            checkpoint.mkdir()
            (checkpoint / "config.json").write_text(json.dumps({"type": "diffusion"}))
            with self.assertRaisesRegex(ValueError, "disagrees"):
                launch_training(path, dry_run=True, extra_args=["--policy.path=checkpoint"])
            with self.assertRaisesRegex(ValueError, "explicit policy.type"):
                launch_training(path, dry_run=True, extra_args=["--policy.type=auto"])

    def test_resume_preserves_original_checkpoint_config_location(self):
        with tempfile.TemporaryDirectory() as directory:
            root, path, cfg = self.fixture(directory)
            (root / "training output").mkdir()
            result = launch_training(path, resume=True, dry_run=True)
            self.assertIn("--config_path=" + str(path), result["command"])
            self.assertIn("--resume=true", result["command"])
            self.assertTrue(result["resume"])
            with self.assertRaisesRegex(ValueError, "already exists"):
                launch_training(path, dry_run=True)

    def test_pretrained_paths_forward_expanded_values_without_conflicting_type(self):
        with tempfile.TemporaryDirectory() as directory:
            root, path, cfg = self.fixture(directory)
            checkpoint = root / "local checkpoint"
            checkpoint.mkdir()
            (checkpoint / "config.json").write_text(json.dumps({"type": "act"}))
            values = ("${PIPER_TEST_CHECKPOINT}", "~/" + os.path.relpath(checkpoint, Path.home()))
            with patch.dict(os.environ, {"PIPER_TEST_CHECKPOINT": str(checkpoint)}):
                for field in ("path", "pretrained_path"):
                    for value in values:
                        with self.subTest(field=field, value=value):
                            cfg["policy"] = {"type": "act", field: value}
                            path.write_text(json.dumps(cfg))
                            result = launch_training(path, dry_run=True,
                                extra_args=["--policy.type", "act", "--policy." + field, value])
                            forwarded = [a for a in result["command"] if a.startswith("--policy." + field + "=")]
                            self.assertEqual(forwarded, ["--policy.%s=%s" % (field, checkpoint)])
                            self.assertNotIn(value, result["command"])
                            if field == "path":
                                self.assertNotIn("--policy.type", result["command"])
                            self.assertEqual(json.loads(path.read_text()), cfg)

    def test_checkpoint_path_and_type_are_accepted_by_upstream_parser(self):
        try:
            from lerobot.configs import parser
            from lerobot.configs.train import TrainPipelineConfig
            from lerobot.policies.act.configuration_act import ACTConfig
        except (ImportError, SyntaxError):
            self.skipTest("Requires configured LeRobot Python environment")
        with tempfile.TemporaryDirectory() as directory:
            root, path, cfg = self.fixture(directory)
            checkpoint = root / "local checkpoint"
            ACTConfig(device="cpu", chunk_size=4, n_action_steps=4,
                      pretrained_backbone_weights=None).save_pretrained(checkpoint)
            cfg["policy"]["path"] = "${PIPER_TEST_CHECKPOINT}"
            path.write_text(json.dumps(cfg))
            with patch.dict(os.environ, {"PIPER_TEST_CHECKPOINT": str(checkpoint)}):
                result = launch_training(path, dry_run=True,
                    extra_args=["--policy.type=act", "--policy.n_action_steps=2"])

            def inspect_config(config: TrainPipelineConfig):
                config.validate()
                return config

            with patch.object(sys, "argv", ["lerobot_train"] + result["command"][3:]), \
                    patch.dict(parser._config_path_args, {}, clear=True), \
                    patch.dict(parser._config_yaml_overrides, {}, clear=True):
                parsed = parser.wrap()(inspect_config)()
            self.assertEqual(parsed.policy.type, "act")
            self.assertEqual(parsed.policy.pretrained_path, checkpoint)
            self.assertEqual(parsed.policy.n_action_steps, 2)

    def test_worker_interpreter_source_and_local_environment_are_used(self):
        with tempfile.TemporaryDirectory() as directory:
            root, path, cfg = self.fixture(directory)
            package = root / "model source" / "src" / "lerobot"
            scripts = package / "scripts"
            scripts.mkdir(parents=True)
            (package / "__init__.py").write_text("")
            (scripts / "__init__.py").write_text("")
            (scripts / "lerobot_train.py").write_text(
                "import json, os, sys\nfrom pathlib import Path\n"
                "Path('invocation.json').write_text(json.dumps({'argv':sys.argv[1:],"
                "'offline':os.environ.get('HF_HUB_OFFLINE'),'source':os.environ.get('PYTHONPATH')}))\n")
            with patch.dict(os.environ, {"PYTHONPATH": "/unrelated/host/packages"}):
                result = launch_training(path, python=sys.executable, lerobot_source=package.parent.parent)
            invocation = json.loads((root / "invocation.json").read_text())
            self.assertEqual(result["returncode"], 0)
            self.assertEqual(invocation["offline"], "1")
            self.assertEqual(invocation["source"], str(package.parent))
            self.assertIn("--config_path=" + str(path), invocation["argv"])
            self.assertIn("--policy.push_to_hub=false", invocation["argv"])


if __name__ == "__main__":
    unittest.main()
