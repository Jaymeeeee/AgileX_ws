import io
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import types
import unittest

import numpy as np

from piper_control.config import component_names as canonical_names
from piper_control.ipc import read_message, write_message
from piper_control.policies.adapter import PolicyRunner, _component_order
from piper_control.policies.client import PolicyClient
from piper_control.policies.interfaces import PolicyError


def config():
    names = canonical_names(["left"])
    return {"policy": {"backend": "mock", "type": "act", "python": sys.executable,
                       "startup_timeout_sec": 10, "inference_timeout_sec": 3},
            "robot": {"arms": ["left"], "profile": "piper", "action_space": "joint_position"},
            "observation": {"state_names": names, "action_names": names, "camera_map": {}},
            "workflow": {"mode": "deploy", "learner": "none"}}


class PolicyTests(unittest.TestCase):
    def test_transport_roundtrip_copies_numpy_without_pickle(self):
        stream = io.BytesIO()
        image = np.arange(36, dtype=np.uint8).reshape(3, 4, 3)
        write_message(stream, {"image": image, "action": np.zeros(7, dtype=np.float32)})
        stream.seek(0)
        result = read_message(stream)
        np.testing.assert_array_equal(result["image"], image)
        self.assertEqual(result["image"].dtype, np.uint8)
        with self.assertRaises(ValueError):
            write_message(io.BytesIO(), {"array": np.array([object()], dtype=object)})

    def test_semantics_reject_equal_dim_wrong_arm(self):
        left, right = canonical_names(["left"]), canonical_names(["right"])
        with self.assertRaisesRegex(PolicyError, "selected robot joints"):
            _component_order(left, None, right, "action_names")
        with self.assertRaisesRegex(PolicyError, "TRAINING order"):
            _component_order(None, None, left, "action_names")
        with self.assertRaisesRegex(PolicyError, "checkpoint metadata"):
            _component_order(left, left[::-1], left, "action_names")

    def test_type_mismatch_is_rejected_before_any_model_import(self):
        cfg = config()
        cfg["policy"]["backend"] = "lerobot"
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, "config.json").write_text(json.dumps({"type": "diffusion"}))
            cfg["policy"]["checkpoint"] = directory
            with self.assertRaisesRegex(PolicyError, "disagrees"):
                PolicyRunner(cfg).load()

    def test_capabilities_keep_history_and_temporal_ensembling(self):
        runner = PolicyRunner(config())
        runner.policy = SimpleNamespace(config=SimpleNamespace(n_obs_steps=2),
                                        chunk_inference_spec=lambda: self.fail("History must not request a chunk spec"))
        runner._select_execution()
        self.assertEqual(runner.mode, "step")
        runner.options["execution"] = "chunk"
        with self.assertRaisesRegex(PolicyError, "observation history"):
            runner._select_execution()

    def test_auto_requires_positive_chunk_contract(self):
        runner = PolicyRunner(config())
        runner.policy = SimpleNamespace(config=SimpleNamespace(n_obs_steps=1), predict_action_chunk=lambda b: b)
        runner._select_execution()
        self.assertEqual(runner.mode, "step")
        runner.options["execution"] = "chunk"
        with self.assertRaisesRegex(PolicyError, "validated chunk"):
            runner._select_execution()

    def test_mock_subprocess_and_invalidated_reply(self):
        client = PolicyClient(config()).load()
        try:
            obs = {"observation.state": np.arange(7, dtype=np.float32)}
            client.submit(obs, generation=0, seq=1)
            client.invalidate(generation=1)
            client.submit(obs, generation=1, seq=2)
            limit = time.monotonic() + 4
            replies = []
            while time.monotonic() < limit:
                reply = client.poll()
                if reply:
                    replies.append(reply)
                    if reply.get("seq") == 2:
                        break
                time.sleep(.005)
            valid = [r for r in replies if r.get("kind") == "infer"]
            self.assertEqual([r["seq"] for r in valid], [2])
            np.testing.assert_array_equal(valid[0]["action"], obs["observation.state"])
        finally:
            client.close()

    def test_reset_clears_model_and_both_saved_processors(self):
        calls = []
        runner = PolicyRunner(config())
        runner.policy = SimpleNamespace(reset=lambda: calls.append("policy"))
        runner.pre = SimpleNamespace(reset=lambda: calls.append("pre"))
        runner.post = SimpleNamespace(reset=lambda: calls.append("post"))
        runner.invalidate()
        self.assertEqual(calls, ["policy", "pre", "post"])

    def test_invalidated_inference_drops_actions_but_surfaces_worker_errors(self):
        client = PolicyClient(config())
        client.generation = 2
        client.responses.put({"kind": "infer", "generation": 1, "seq": 1,
                              "action": np.zeros(7, dtype=np.float32)})
        self.assertIsNone(client.poll())
        client.responses.put({"kind": "infer", "generation": 1, "seq": 1,
                              "error": "model execution failed during intervention"})
        with self.assertRaisesRegex(PolicyError, "model execution failed"):
            client.poll()

    def test_all_worker_commands_have_timeouts_and_bounded_owned_queue(self):
        cfg = config()
        cfg["policy"]["max_pending_commands"] = 2
        client = PolicyClient(cfg)
        client.process = object()  # no subprocess: exercise the command admission contract
        source = np.zeros(7, dtype=np.float32)
        client.record_step(source, "BASE", next_observation={"observation.state": source})
        source[:] = 99
        np.testing.assert_array_equal(client.commands[0]["action"], np.zeros(7))
        np.testing.assert_array_equal(client.commands[0]["next_observation"]["observation.state"], np.zeros(7))
        client.boundary({"observation.state": source})
        with self.assertRaisesRegex(PolicyError, "queue overflow"):
            client.boundary({"observation.state": source})
        client.command_kind = "boundary"
        client.command_since = time.monotonic() - 10
        with self.assertRaisesRegex(PolicyError, "boundary timed out"):
            client.poll()

    def test_parent_cleans_snapshot_spool_after_worker_is_killed(self):
        cfg = config()
        cfg["workflow"]["mode"] = "online_rl"
        client = PolicyClient(cfg).load()
        scratch = Path(client._snapshot_workspace.name)
        try:
            # Simulate a child spool left behind by a worker that cannot run its
            # own cleanup handler. Parent ownership covers this failure path.
            child = scratch / "unfinished_episode"
            child.mkdir()
            (child / "snapshot.npz").write_bytes(b"unfinished")
            client.process.kill()
            client.process.wait(timeout=3)
        finally:
            client.close()
        self.assertFalse(scratch.exists())


try:
    import torch
    from dataclasses import dataclass
    from lerobot.configs import FeatureType, PolicyFeature, PreTrainedConfig
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.pretrained import PreTrainedPolicy
    from lerobot.processor import make_default_pre_post_processors
    from safetensors.torch import save_file
    LEROBOT_AVAILABLE = True
except (ImportError, SyntaxError):
    LEROBOT_AVAILABLE = False


@unittest.skipUnless(LEROBOT_AVAILABLE, "Requires configured LeRobot Python environment")
class RegisteredPolicyTests(unittest.TestCase):
    def test_registered_class_checkpoint_architecture_saved_processors_and_physical_order(self):
        # A real registry/weights/processor load with a tiny model: no downloads,
        # backbone or image encoder needed to verify the deployment contract.
        @PreTrainedConfig.register_subclass("hil_tiny")
        @dataclass
        class TinyConfig(ACTConfig):
            def validate_features(self):
                pass

        TinyConfig.__module__ = "piper_control.policies.configuration_test_tiny"

        class TinyPolicy(PreTrainedPolicy):
            config_class = TinyConfig
            name = "hil_tiny"

            def __init__(self, cfg, **kwargs):
                super().__init__(cfg)
                self.bias = torch.nn.Parameter(torch.full((cfg.dim_model,), .25))

            def get_optim_params(self):
                return self.parameters()

            def reset(self):
                pass

            def forward(self, batch):
                return self.bias.sum(), {}

            def predict_action_chunk(self, batch, **kwargs):
                self.seen_image_shape = tuple(batch["observation.images.top"].shape)
                return (batch["observation.state"] + self.bias[:7])[:, None, :].expand(-1, self.config.chunk_size, -1)

            def select_action(self, batch, **kwargs):
                return self.predict_action_chunk(batch)[:, 0]

        configuration_module = types.ModuleType(TinyConfig.__module__)
        configuration_module.TinyConfig = TinyConfig
        modeling_module = types.ModuleType("piper_control.policies.modeling_test_tiny")
        modeling_module.TinyPolicy = TinyPolicy
        processor_module = types.ModuleType("piper_control.policies.processor_test_tiny")
        modules = (configuration_module, modeling_module, processor_module)
        for module in modules:
            sys.modules[module.__name__] = module
        try:
            with tempfile.TemporaryDirectory() as directory:
                checkpoint = Path(directory)
                cfg = TinyConfig(device="cpu", dim_model=17, chunk_size=3, n_action_steps=2,
                    input_features={"observation.state": PolicyFeature(type=FeatureType.STATE, shape=(7,)),
                                    "observation.images.top": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 4, 4))},
                    output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(7,))})
                cfg._save_pretrained(checkpoint)
                save_file(TinyPolicy(cfg).state_dict(), str(checkpoint / "model.safetensors"))
                stats = {"observation.state": {"mean": torch.zeros(7), "std": torch.ones(7)},
                         "action": {"mean": torch.arange(7, dtype=torch.float32), "std": torch.full((7,), 2.)}}
                pre, post = make_default_pre_post_processors(cfg, stats, normalizer_device="cpu")
                pre.save_pretrained(checkpoint)
                post.save_pretrained(checkpoint)
                deployment = config()
                deployment["policy"].update(backend="lerobot", type="hil_tiny", checkpoint=str(checkpoint), device="cpu")
                deployment["observation"].update(state_names=canonical_names(["left"])[::-1],
                    action_names=canonical_names(["left"])[::-1], camera_map={"observation.images.top": "cam_h"})
                runner = PolicyRunner(deployment).load()
                observation = {"observation.state": np.arange(7, dtype=np.float32),
                               "observation.images.top": np.zeros((8, 8, 3), dtype=np.uint8)}
                result = runner.infer(observation)
                self.assertEqual(runner.policy.bias.numel(), 17)
                self.assertEqual(result["actions"].shape, (2, 7))
                expected = ((np.arange(7)[::-1] + .25) * 2 + np.arange(7))[::-1]
                np.testing.assert_allclose(result["actions"], np.stack([expected, expected]), atol=1e-5)
                self.assertEqual(runner.policy.seen_image_shape, (1, 3, 8, 8))
                runner.cfg["observation"]["image_resize"] = "checkpoint"
                runner.infer(observation)
                self.assertEqual(runner.policy.seen_image_shape, (1, 3, 4, 4))
                runner.close()
                save_file({"unexpected": torch.zeros(1)}, str(checkpoint / "model.safetensors"))
                with self.assertRaises(RuntimeError):
                    PolicyRunner(deployment).load()
        finally:
            for module in modules:
                sys.modules.pop(module.__name__, None)


if __name__ == "__main__":
    unittest.main()
