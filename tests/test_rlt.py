"""Integration checks use tiny real RLT heads; no backbone, camera or downloads.

Run with the configured LeRobot Python and source on PYTHONPATH. The control-side Python
3.8 suite skips these optional-dependency tests instead of importing torch there.
"""
from pathlib import Path
import contextlib
import io
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from piper_control.config import component_names as canonical_names
from piper_control.policies.interfaces import PolicyError
from piper_control.learning.rlt import RLTSession

try:
    import torch
    from lerobot.configs import FeatureType, NormalizationMode, PolicyFeature
    from lerobot.policies.pi05_rlt.configuration_pi05_rlt import PI05RLTConfig
    from lerobot.policies.pi05_rlt.modeling_pi05_rlt import PI05RLTPolicy
    from lerobot.processor import NormalizerProcessorStep
    AVAILABLE = True
except (ImportError, SyntaxError):
    AVAILABLE = False


@unittest.skipUnless(AVAILABLE, "Requires configured LeRobot Python environment")
class RLTTests(unittest.TestCase):
    def make_session(self, options=None, horizon=3, stride=2):
        cfg = PI05RLTConfig(device="cpu", mode="inference", vlm_hidden_dim=8,
                           rlt_hidden_dim=8, rlt_num_heads=2, rlt_encoder_layers=1,
                           rlt_decoder_layers=1, rlt_dropout=0., actor_hidden_dim=16,
                           critic_hidden_dim=16, chunk_size=max(6, horizon), n_action_steps_rl=horizon,
                           action_stride=stride,
                           state_dim=7, action_dim=7)
        policy = PI05RLTPolicy(cfg)
        normalizer = NormalizerProcessorStep(
            features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(7,))},
            norm_map={FeatureType.ACTION: NormalizationMode.MEAN_STD},
            stats={"action": {"mean": torch.ones(7), "std": torch.full((7,), 2.)}})
        names = canonical_names(["left"])
        runner = SimpleNamespace(policy=policy, options={"type": "pi05_rlt", "device": "cpu"},
                                 cfg={"workflow": {"learner": "pi05_rlt"}}, torch=torch,
                                 pre=SimpleNamespace(steps=[normalizer]), names=names,
                                 state_names=names, action_names=names, reset=policy.reset)
        opts = {"batch_size": 1, "replay_capacity": 8, "learner": {"utd_ratio": 1},
                "replay_windows": "execution_chunks"}
        opts.update(options or {})
        session = RLTSession(runner, opts)
        session.snapshot = lambda observation: self.boundary(horizon)
        return session

    @staticmethod
    def boundary(horizon=3):
        return {"z_rl": np.zeros(8, dtype=np.float32), "proprio": np.zeros(7, dtype=np.float32),
                "ref_chunk": np.zeros((horizon, 7), dtype=np.float32)}

    def test_interrupted_chunk_records_only_accepted_actions_and_real_update(self):
        session = self.make_session()
        session.record_step(np.full(7, 3.), "RL", boundary=self.boundary(), next_observation={})
        # Human correction cuts the predicted chunk after one action.
        session.boundary({})
        session.record_step(np.full(7, 5.), "HUMAN", intervention=True, next_observation={})
        before = [p.detach().clone() for p in session.policy.critic.parameters()]
        result = session.end_episode({}, terminal=True, reward=1., success=True)
        self.assertEqual(result["transitions"], 2)
        self.assertEqual(session.learner.updates, 2)
        self.assertTrue(any(not torch.equal(a, b) for a, b in zip(before, session.policy.critic.parameters())))
        storage = session.replay._storage
        np.testing.assert_array_equal(storage["valid_mask"][:2], [[True, False, False]] * 2)
        np.testing.assert_allclose(storage["action_chunk"][0, 0], 1.)
        np.testing.assert_allclose(storage["action_chunk"][1, 0], 2.)
        self.assertFalse(storage["done"][0])
        self.assertTrue(storage["done"][1])
        self.assertEqual(storage["rewards"][1, 0], 1.)

    def test_full_window_terminal_reward_and_timeout_bootstrap(self):
        session = self.make_session({"warmup_transitions": 8})
        for index in range(3):
            session.record_step(np.ones(7), "BASE", boundary=self.boundary() if index == 0 else None,
                                next_observation={})
        session.end_episode({}, truncated=True, reward=-.25)
        storage = session.replay._storage
        self.assertEqual(len(session.replay), 1)
        self.assertFalse(storage["done"][0])
        np.testing.assert_array_equal(storage["rewards"][0], [0, 0, -.25])
        self.assertEqual(session.learner.updates, 0)

    def test_consistent_save_resume_preserves_updates_and_replay(self):
        session = self.make_session()
        session.record_step(np.ones(7), "BASE", boundary=self.boundary(), next_observation={})
        session.end_episode({}, terminal=True)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "snapshot"
            session.save(path)
            restored = self.make_session({"resume": str(path)})
            self.assertEqual(restored.learner.updates, session.learner.updates)
            self.assertEqual(len(restored.replay), len(session.replay))
            self.assertEqual(restored.episode_id, 1)
            for a, b in zip(restored.policy.actor.parameters(), session.policy.actor.parameters()):
                torch.testing.assert_close(a, b)

    def test_offline_cli_continues_real_optimizer_from_consistent_snapshot(self):
        from piper_control.learning.train_online import main
        session = self.make_session()
        session.record_step(np.ones(7), "BASE", boundary=self.boundary(), next_observation={})
        session.end_episode({}, terminal=True)
        with tempfile.TemporaryDirectory() as directory:
            old, new = Path(directory) / "old", Path(directory) / "new"
            session.save(old)
            with patch.object(sys, "argv", ["train_online", "--resume", str(old), "--output", str(new),
                                            "--updates", "2", "--batch-size", "1", "--device", "cpu"]):
                with contextlib.redirect_stdout(io.StringIO()):
                    main()
            restored = self.make_session({"resume": str(new)})
            self.assertEqual(restored.learner.updates, session.learner.updates + 2)
            self.assertEqual(len(restored.replay), 1)

    def test_export_uses_executed_actions_and_adds_terminal_reward_once(self):
        from piper_control.learning.replay_export import export_session
        from lerobot.policies.pi05_rlt.replay import ReplayBuffer
        session = self.make_session({"warmup_transitions": 8})
        runner = SimpleNamespace(rlt=session, close=lambda: None)
        observation = {"state": [0.] * 7, "stamp": 123., "frames": {}}
        cfg = {"robot": {"arms": ["left"]}, "policy": {"backend": "lerobot", "type": "pi05_rlt"},
               "observation": {"camera_map": {}, "task": "test"}, "workflow": {"rlt": {}}}
        rows = [
            {"record_type": "step", "episode_id": 0, "step_id": 0, "observation": observation,
             "next_observation": observation, "action": [3.] * 7, "predicted_action": [99.] * 7,
             "source": "RL", "reward": 0., "boundary": self.boundary()["z_rl"].tolist()},
            {"record_type": "event", "type": "episode_end", "episode_id": 0,
             "terminal": True, "truncated": False, "reward": 1., "success": True}]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "manifest.json").write_text(json.dumps({"status": "complete", "config": cfg, "steps": 1}))
            (root / "trajectory.jsonl").write_text("\n".join(json.dumps(row) for row in rows))
            with patch("piper_control.learning.replay_export.PolicyRunner") as factory:
                factory.return_value.load.return_value = runner
                result = export_session(root, root / "replay.npz")
            replay = ReplayBuffer.load(root / "replay.npz")
            self.assertEqual(result["executed_steps"], 1)
            np.testing.assert_allclose(replay._storage["action_chunk"][0, 0], 1.)
            np.testing.assert_array_equal(replay._storage["valid_mask"][0], [True, False, False])
            np.testing.assert_array_equal(replay._storage["rewards"][0], [1., 0., 0.])

    def test_same_source_human_pause_resume_export_matches_online_windows(self):
        from piper_control.learning.replay_export import export_session
        from lerobot.policies.pi05_rlt.replay import ReplayBuffer
        cfg = {"robot": {"arms": ["left"]}, "policy": {"backend": "lerobot", "type": "pi05_rlt"},
               "observation": {"camera_map": {}, "task": "test"}, "workflow": {"rlt": {}}}
        observation = {"state": [0.] * 7, "stamp": 123., "frames": {}}
        for marker in ("execution_boundary", "generation"):
            with self.subTest(marker=marker):
                online = self.make_session({"warmup_transitions": 8})
                rows = []
                for index, action in enumerate((3., 5., 7.)):
                    if index in (0, 1):
                        # First human interval, then HUMAN -> hold -> HUMAN.
                        online.boundary({}, episode_id=0, step_id=index)
                    online.record_step(np.full(7, action), "HUMAN", intervention=True,
                                       next_observation={})
                    row = {"record_type": "step", "episode_id": 0, "step_id": index,
                           "observation": observation, "next_observation": observation,
                           "action": [action] * 7, "source": "HUMAN", "intervention": True,
                           "reward": 0., "generation": 1}
                    if marker == "execution_boundary":
                        row["execution_boundary"] = index in (0, 1)
                    else:
                        row["generation"] = 1 if index == 0 else 3
                    rows.append(row)
                online.end_episode({}, terminal=True, reward=1., success=True)
                rows.append({"record_type": "event", "type": "episode_end", "episode_id": 0,
                             "terminal": True, "truncated": False, "reward": 1., "success": True})
                offline = self.make_session({"warmup_transitions": 8})
                runner = SimpleNamespace(rlt=offline, close=lambda: None)
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    (root / "manifest.json").write_text(json.dumps({"status": "complete", "config": cfg, "steps": 3}))
                    (root / "trajectory.jsonl").write_text("\n".join(json.dumps(row) for row in rows))
                    with patch("piper_control.learning.replay_export.PolicyRunner") as factory:
                        factory.return_value.load.return_value = runner
                        result = export_session(root, root / "replay.npz")
                    replay = ReplayBuffer.load(root / "replay.npz")
                    self.assertEqual(result["transitions"], 2)
                    for key in ("valid_mask", "executed_steps", "action_chunk", "source", "source_chunk",
                                "rewards", "done", "episode_id", "step_id"):
                        np.testing.assert_array_equal(replay._storage[key][:2], online.replay._storage[key][:2])
                    np.testing.assert_array_equal(replay._storage["executed_steps"][:2], [1, 2])

    def test_learner_only_runs_after_end_and_late_steps_do_not_cross_episodes(self):
        session = self.make_session()
        session.record_step(np.full(7, 3.), "RL", boundary=self.boundary(), next_observation={})
        session.record_step(np.full(7, 5.), "HUMAN", intervention=True, next_observation={})
        with self.assertRaisesRegex(PolicyError, "paused, completed episode"):
            session.update()
        self.assertEqual(session.learner.updates, 0)
        session.end_episode({}, terminal=True, reward=1.)
        self.assertEqual(session.replay._storage["source"][0], 3)  # MIXED
        self.assertEqual(session.learner.updates, 1)
        with self.assertRaisesRegex(PolicyError, "boundary is required"):
            session.record_step(np.full(7, 7.), "HUMAN", next_observation={})
        with self.assertRaisesRegex(PolicyError, "episode id disagrees"):
            session.boundary({}, episode_id=0, step_id=0)
        self.assertEqual(session.step_id, 0)
        self.assertEqual(len(session.replay), 1)

    def test_full_horizon_then_hold_uses_actual_final_observation_online_and_offline(self):
        from piper_control.learning.replay_export import export_session
        from lerobot.policies.pi05_rlt.replay import ReplayBuffer

        def snapshot(observation):
            state = np.asarray(observation["observation.state"], dtype=np.float32)
            return {"z_rl": np.full(8, state[0], dtype=np.float32), "proprio": state,
                    "ref_chunk": np.full((3, 7), state[0], dtype=np.float32)}

        cfg = {"robot": {"arms": ["left"]}, "policy": {"backend": "lerobot", "type": "pi05_rlt"},
               "observation": {"camera_map": {}, "task": "test"}, "workflow": {"rlt": {}}}
        step_observation = {"state": [1.] * 7, "stamp": 123., "frames": {}}
        final_observation = {"state": [4.] * 7, "stamp": 124., "frames": {}}
        online = self.make_session({"warmup_transitions": 8})
        online.snapshot = snapshot
        online.boundary({"observation.state": np.ones(7)})
        rows = []
        for index in range(3):
            online.record_step(np.full(7, 3.), "HUMAN", intervention=True,
                               next_observation={"observation.state": np.ones(7)})
            rows.append({"record_type": "step", "episode_id": 0, "step_id": index,
                         "observation": step_observation, "next_observation": step_observation,
                         "action": [3.] * 7, "source": "HUMAN", "intervention": True,
                         "reward": 0., "generation": 1, "execution_boundary": index == 0})
        np.testing.assert_array_equal(online.transitions[-1].next_proprio, np.ones(7))
        online.end_episode({"observation.state": np.full(7, 4.)}, truncated=True, reward=-.25)
        rows.append({"record_type": "event", "type": "episode_end", "episode_id": 0,
                     "terminal": False, "truncated": True, "reward": -.25, "success": False,
                     "final_observation": final_observation})
        offline = self.make_session({"warmup_transitions": 8})
        offline.snapshot = snapshot
        runner = SimpleNamespace(rlt=offline, close=lambda: None)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "manifest.json").write_text(json.dumps({"status": "complete", "config": cfg, "steps": 3}))
            (root / "trajectory.jsonl").write_text("\n".join(json.dumps(row) for row in rows))
            with patch("piper_control.learning.replay_export.PolicyRunner") as factory:
                factory.return_value.load.return_value = runner
                result = export_session(root, root / "replay.npz")
            replay = ReplayBuffer.load(root / "replay.npz")
            self.assertEqual(result["transitions"], 1)
            self.assertEqual(replay._storage["executed_steps"][0], 3)
            self.assertFalse(replay._storage["done"][0])
            np.testing.assert_array_equal(replay._storage["rewards"][0], [0., 0., -.25])
            for key in ("next_z_rl", "next_proprio", "next_ref_chunk"):
                np.testing.assert_array_equal(replay._storage[key][0], online.replay._storage[key][0])
                np.testing.assert_array_equal(replay._storage[key][0], np.full_like(replay._storage[key][0], 4.))

    def test_empty_episode_end_does_not_compute_vla_boundary(self):
        session = self.make_session()
        with patch.object(session, "snapshot", side_effect=AssertionError("Unexpected VLA")):
            result = session.end_episode({}, truncated=True)
        self.assertEqual(result["transitions"], 0)

    @staticmethod
    def stride_snapshot(observation):
        state = np.asarray(observation["observation.state"], dtype=np.float32)
        return {"z_rl": np.full(8, state[0], dtype=np.float32), "proprio": state,
                "ref_chunk": np.full((4, 7), -1000., dtype=np.float32)}

    def test_checkpoint_stride_matches_baseline_overlap_refs_and_export(self):
        from piper_control.learning.replay_export import export_session
        from lerobot.policies.pi05_rlt.replay import ReplayBuffer
        options = {"replay_windows": "checkpoint_stride", "warmup_transitions": 8}
        online = self.make_session(options, horizon=4, stride=2)
        online.snapshot = self.stride_snapshot
        sources = ("RL", "RL", "HUMAN", "BASE", "BASE", "RL")
        rows = []
        with patch.object(online, "snapshot", wraps=online.snapshot) as encode:
            for index, source in enumerate(sources):
                before = {"observation.state": np.full(7, index, dtype=np.float32)}
                after = {"observation.state": np.full(7, index + 1, dtype=np.float32)}
                if index in (0, 4):
                    online.boundary(before, segment_start=index == 0)
                online.record_step(np.full(7, 1. + 2 * index), source, intervention=source == "HUMAN",
                                   next_observation=after, ref_row=np.full(7, 100. + index))
                rows.append({"record_type": "step", "episode_id": 0, "step_id": index,
                    "observation": {"state": before["observation.state"].tolist(), "frames": {}},
                    "next_observation": {"state": after["observation.state"].tolist(), "frames": {}},
                    "action": [1. + 2 * index] * 7, "source": source, "intervention": source == "HUMAN",
                    "reward": 0., "generation": 1, "execution_boundary": index in (0, 4),
                    "segment_start": index == 0, "boundary": {} if index in (0, 4) else None,
                    "metadata": {"rlt_ref_row": [100. + index] * 7}})
            self.assertEqual(encode.call_count, 0, "Movement-time collection must not re-encode frozen VLA")
            scratch = Path(online._spool.directory.name)
            online.end_episode({"observation.state": np.full(7, 99., dtype=np.float32)},
                               truncated=True, reward=-.5)
            self.assertGreater(encode.call_count, 0)
            self.assertFalse(scratch.exists())
        data = online.replay._storage
        self.assertEqual(len(online.replay), 2)
        np.testing.assert_array_equal(data["step_id"][:2], [0, 2])
        np.testing.assert_array_equal(data["source_chunk"][:2], [[1, 1, 2, 0], [2, 0, 0, 1]])
        np.testing.assert_array_equal(data["source"][:2], [3, 3])
        np.testing.assert_allclose(data["ref_chunk"][:2, :, 0], [[100, 101, 2, 103], [2, 103, 104, 105]], atol=1e-5)
        np.testing.assert_allclose(data["action_chunk"][:2, :, 0], [[0, 1, 2, 3], [2, 3, 4, 5]], atol=1e-5)
        np.testing.assert_array_equal(data["next_proprio"][:2, 0], [4, 99])
        np.testing.assert_array_equal(data["done"][:2], [False, False])
        np.testing.assert_array_equal(data["rewards"][:2], [[0, 0, 0, 0], [0, 0, 0, -.5]])
        rows.append({"record_type": "event", "type": "episode_end", "episode_id": 0,
                     "terminal": False, "truncated": True, "reward": -.5,
                     "final_observation": {"state": [99.] * 7, "frames": {}}})
        offline = self.make_session(options, horizon=4, stride=2)
        offline.snapshot = self.stride_snapshot
        cfg = {"robot": {"arms": ["left"]}, "policy": {"backend": "lerobot", "type": "pi05_rlt"},
               "observation": {"camera_map": {}}, "workflow": {"rlt": options}}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "manifest.json").write_text(json.dumps({"status": "complete", "config": cfg, "steps": 6}))
            (root / "trajectory.jsonl").write_text("\n".join(json.dumps(row) for row in rows))
            with patch("piper_control.learning.replay_export.PolicyRunner") as factory:
                factory.return_value.load.return_value = SimpleNamespace(rlt=offline, close=offline.close)
                export_session(root, root / "replay.npz")
            rebuilt = ReplayBuffer.load(root / "replay.npz")
            for key in data:
                np.testing.assert_array_equal(rebuilt._storage[key][:2], data[key][:2])

    def test_checkpoint_stride_does_not_cross_pause_segments_and_marks_terminal_tails(self):
        session = self.make_session({"replay_windows": "checkpoint_stride", "warmup_transitions": 8}, horizon=4)
        session.snapshot = self.stride_snapshot
        try:
            for index in range(6):
                if index in (0, 3):
                    session.boundary({"observation.state": np.full(7, 0. if index == 0 else 30.)}, segment_start=True)
                session.record_step(np.full(7, 1. + 2 * index), "HUMAN", intervention=True,
                                    next_observation={"observation.state": np.full(7, index + 1.)})
            session.end_episode({"observation.state": np.full(7, 99.)}, terminal=True, reward=1.)
            data = session.replay._storage
            np.testing.assert_array_equal(data["step_id"][:4], [0, 2, 3, 5])
            np.testing.assert_array_equal(data["executed_steps"][:4], [3, 1, 3, 1])
            np.testing.assert_array_equal(data["next_proprio"][:4, 0], [3, 3, 99, 99])
            np.testing.assert_array_equal(data["done"][:4], [False, False, True, True])
            np.testing.assert_array_equal(data["rewards"][:4].sum(axis=1), [0, 0, 1, 1])
        finally:
            session.close()

    def test_stride_spool_limits_and_resume_collector_contract(self):
        session = self.make_session({"replay_windows": "checkpoint_stride", "snapshot_max_bytes": 4096}, horizon=4)
        try:
            with self.assertRaisesRegex(PolicyError, "spool limit"):
                session.boundary({"observation.state": np.zeros(7, dtype=np.float32)})
            scratch = Path(session._spool.directory.name)
        finally:
            session.close()
        self.assertFalse(scratch.exists())
        session = self.make_session()
        session.record_step(np.ones(7), "BASE", boundary=self.boundary(), next_observation={})
        session.end_episode({}, terminal=True)
        with tempfile.TemporaryDirectory() as directory:
            saved = Path(directory) / "saved"
            session.save(saved)
            restored = RLTSession(session.runner, {"resume": str(saved), "batch_size": 1, "replay_capacity": 8})
            self.assertEqual(restored.replay_windows, "execution_chunks")
            with self.assertRaisesRegex(PolicyError, "collector contract"):
                RLTSession(session.runner, {"resume": str(saved), "replay_windows": "checkpoint_stride"})

    def test_snapshot_encoding_preserves_rng_and_reconstructs_same_observation(self):
        session = self.make_session()
        session.runner._prepare = lambda observation: observation
        session.policy._infer_window = lambda batch: (torch.rand(1, 3, 7), torch.rand(1, 8), torch.rand(1, 7))
        before = torch.get_rng_state().clone()
        first = RLTSession.snapshot(session, {"observation.state": np.ones(7, dtype=np.float32), "task": "test"})
        torch.testing.assert_close(before, torch.get_rng_state())
        second = RLTSession.snapshot(session, {"observation.state": np.ones(7, dtype=np.float32), "task": "test"})
        for key in first:
            np.testing.assert_array_equal(first[key], second[key])


if __name__ == "__main__":
    unittest.main()
