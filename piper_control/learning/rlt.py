"""Episodic online RLT using the existing LeRobot learner, not a new algorithm.

All methods execute serially in the policy worker. Weights update only after the
robot has paused at an episode boundary. Inference results do not enter replay:
only hardware-accepted, policy-rate targets supplied by record_step do.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile

import numpy as np

from ..policies.interfaces import PolicyError


def _array(value):
    if hasattr(value, "detach"):
        value = value.detach().float().cpu().numpy()
    return np.asarray(value, dtype=np.float32)


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise PolicyError("%s must be a positive integer" % name)
    return value


class _ObservationSpool:
    """Bounded disk-backed snapshots; camera arrays are never retained per episode."""
    def __init__(self, directory, limit, reserve):
        if directory:
            Path(directory).mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(prefix="piper-rlt-", dir=directory)
        self.limit = limit
        self.reserve = reserve
        self.bytes = 0
        self.count = 0

    def put(self, observation):
        arrays, metadata = {}, {}
        for key, value in observation.items():
            if isinstance(value, np.ndarray):
                if value.dtype.hasobject:
                    raise PolicyError("Snapshot arrays cannot contain Python objects")
                arrays[key] = value
            elif isinstance(value, np.generic):
                metadata[key] = value.item()
            else:
                metadata[key] = value
        arrays["__metadata_json__"] = np.asarray(json.dumps(metadata, allow_nan=False))
        estimate = sum(value.nbytes for value in arrays.values()) + 4096
        if self.bytes + estimate > self.limit:
            raise PolicyError("RLT snapshot spool limit reached; end shorter episodes or increase snapshot_max_bytes")
        if shutil.disk_usage(self.directory.name).free - estimate < self.reserve:
            raise PolicyError("RLT snapshot disk free-space reserve reached; stop collection")
        path = Path(self.directory.name) / ("%08d.npz" % self.count)
        with path.open("xb") as stream:
            np.savez(stream, **arrays)
        size = path.stat().st_size
        if self.bytes + size > self.limit:
            path.unlink()
            raise PolicyError("RLT snapshot spool limit reached")
        self.bytes += size
        self.count += 1
        return str(path)

    @staticmethod
    def read(path):
        with np.load(path, allow_pickle=False) as archive:
            result = json.loads(str(archive["__metadata_json__"]))
            result.update({key: archive[key].copy() for key in archive.files if key != "__metadata_json__"})
        return result

    def close(self):
        self.directory.cleanup()


def rlt_prediction(runner, predicted):
    """Return metadata from the REAL model boundary and optional exploration."""
    policy = runner.policy
    cfg = policy.config
    gated = cfg.deploy_gating or cfg.deploy_auto_critical or cfg.deploy_vla_only
    actor = policy._actor_takes_over() if gated else True
    boundary = {"z_rl": _array(policy._last_z_rl).reshape(-1),
                "proprio": _array(policy._last_state_rl).reshape(-1),
                "ref_chunk": _array(policy._last_ref_actions)[0]}
    session = runner.rlt
    explore = session is not None and session.options.get("exploration", True) and actor
    if explore:
        # Actor std is part of the existing algorithm's policy, not a new noise schedule.
        torch = runner.torch
        z = torch.as_tensor(boundary["z_rl"], device=predicted.device).unsqueeze(0)
        state = torch.as_tensor(boundary["proprio"], device=predicted.device).unsqueeze(0)
        ref = torch.as_tensor(boundary["ref_chunk"], device=predicted.device).unsqueeze(0)
        _, std = policy.actor(z, state, ref)
        predicted = predicted + std * torch.randn_like(predicted)
    return predicted, {"source": "RL" if actor else "BASE", "exploration": bool(explore),
                       "rlt_boundary": boundary,
                       "weight_version": session.learner.updates if session else 0}


class RLTSession:
    def __init__(self, runner, options):
        from lerobot.policies.pi05_rlt.learning import RLTLearner, RLTLearnerConfig
        from lerobot.policies.pi05_rlt.replay import ReplayBuffer
        from lerobot.processor import NormalizerProcessorStep, RelativeActionsProcessorStep
        if runner.options["type"] != "pi05_rlt":
            raise PolicyError("workflow.mode=online_rl requires policy.type=pi05_rlt")
        if runner.cfg["workflow"].get("learner") != "pi05_rlt":
            raise PolicyError("online_rl requires workflow.learner=pi05_rlt")
        if runner.state_names != runner.action_names:
            raise PolicyError("RLT state/action component order must align with its trained absolute-action contract")
        self.runner = runner
        self.options = options
        self.policy = runner.policy
        self.replay_windows = options.get("replay_windows", "checkpoint_stride")
        if self.replay_windows not in ("checkpoint_stride", "execution_chunks"):
            raise PolicyError("replay_windows must be checkpoint_stride or execution_chunks")
        self.max_episode_steps = _positive_integer(options.get("max_episode_steps", 10000), "max_episode_steps")
        self.snapshot_max_bytes = _positive_integer(options.get("snapshot_max_bytes", 8 * 1024 ** 3), "snapshot_max_bytes")
        self.snapshot_min_free_bytes = _positive_integer(options.get("snapshot_min_free_bytes", 1024 ** 3), "snapshot_min_free_bytes")
        self.stride = _positive_integer(getattr(self.policy.config, "action_stride", None), "checkpoint action_stride")
        if self.stride > self.horizon:
            raise PolicyError("checkpoint action_stride must not exceed its RLT horizon")
        self._spool = None
        self._segments = []
        self._segment = None
        if any(isinstance(s, RelativeActionsProcessorStep) and s.enabled for s in runner.pre.steps):
            raise PolicyError("Existing RLT baseline requires absolute actions")
        self.normalizer = next((s for s in runner.pre.steps if isinstance(s, NormalizerProcessorStep)), None)
        if self.normalizer is None:
            raise PolicyError("RLT replay requires the checkpoint's action normalizer")
        self.policy.config.mode = "online_rl"
        # Exported inference policies construct frozen heads. Changing the config
        # string alone does not restore autograd on their parameters.
        self.policy.actor.requires_grad_(True)
        self.policy.critic.requires_grad_(True)
        self.policy.rlt_encoder.requires_grad_(False)
        self.policy.rlt_decoder.requires_grad_(False)
        self.learner = RLTLearner(self.policy, RLTLearnerConfig(**options.get("learner", {})))
        capacity = _positive_integer(options.get("replay_capacity", 100000), "replay_capacity")
        self.replay = ReplayBuffer(capacity=capacity, seed=int(options.get("seed", 42)))
        if options.get("prefill_replay"):
            self.replay = ReplayBuffer.load(options["prefill_replay"], capacity=capacity)
            self._validate_replay_contract()
        self.episode_id = 0
        self.step_id = 0
        self.window = None
        self.window_rewards = []
        self.window_ref_rows = []
        self.transitions = []
        self.new_transitions = 0
        self.batch_size = _positive_integer(options.get("batch_size", 256), "batch_size")
        _positive_integer(options.get("warmup_transitions", self.batch_size), "warmup_transitions")
        if self.batch_size < 1 or self.batch_size > capacity:
            raise PolicyError("RLT batch_size must be within replay capacity")
        if options.get("resume"):
            self._resume(Path(options["resume"]))

    @property
    def horizon(self):
        return int(self.policy.config.n_action_steps_rl)

    @property
    def needs_boundary(self):
        return self._segment is None if self.replay_windows == "checkpoint_stride" else self.window is None

    def _validate_replay_contract(self):
        # ReplayBuffer's validated storage schema is fixed by the local RLT
        # library. Inspect shapes without consuming its restored sampler RNG.
        if not len(self.replay):
            return
        shapes = {"z_rl": (self.policy.config.rlt_hidden_dim,),
                  "next_z_rl": (self.policy.config.rlt_hidden_dim,),
                  "proprio": (self.policy.config.state_dim,),
                  "next_proprio": (self.policy.config.state_dim,),
                  "ref_chunk": (self.horizon, self.policy.config.action_dim),
                  "next_ref_chunk": (self.horizon, self.policy.config.action_dim),
                  "action_chunk": (self.horizon, self.policy.config.action_dim)}
        for key, shape in shapes.items():
            if self.replay._storage[key].shape[1:] != shape:
                raise PolicyError("Replay %s shape does not match the selected RLT checkpoint" % key)

    def _context(self, metadata):
        from lerobot.policies.pi05_rlt.replay import BoundaryContext
        torch = self.runner.torch
        arrays = {key: np.asarray(metadata[key], dtype=np.float32) for key in ("z_rl", "proprio", "ref_chunk")}
        shapes = {"z_rl": (self.policy.config.rlt_hidden_dim,),
                  "proprio": (self.policy.config.state_dim,),
                  "ref_chunk": (self.horizon, self.policy.config.action_dim)}
        for key, arr in arrays.items():
            if arr.shape != shapes[key] or not np.isfinite(arr).all():
                raise PolicyError("Invalid RLT boundary %s" % key)
        return BoundaryContext(*(torch.from_numpy(arrays[key].copy()) for key in ("z_rl", "proprio", "ref_chunk")),
                               "online", self.episode_id, self.step_id)

    def snapshot(self, observation):
        # This does not call predict_action_chunk: human replay must not advance
        # deployment gate counters or fabricate an actor action that was never run.
        torch = self.runner.torch
        digest = hashlib.sha256()
        for key, value in sorted(observation.items()):
            if isinstance(value, np.ndarray):
                digest.update(key.encode())
                digest.update(str(value.shape).encode())
                digest.update(value.dtype.str.encode())
                digest.update(np.ascontiguousarray(value).tobytes())
            elif key == "task":
                digest.update(str(value).encode())
        seed = int.from_bytes(digest.digest()[:8], "little") % (2 ** 63)
        device = next(self.policy.parameters()).device
        cuda_devices = [device.index if device.index is not None else torch.cuda.current_device()] if device.type == "cuda" else []
        # Reconstructing the same saved observation yields the same frozen-VLA
        # boundary without consuming policy exploration or learner RNG state.
        with torch.random.fork_rng(devices=cuda_devices), torch.inference_mode():
            torch.random.default_generator.manual_seed(seed)
            if cuda_devices:
                with torch.cuda.device(cuda_devices[0]):
                    torch.cuda.manual_seed(seed)
            prepared = self.runner._prepare(observation)
            ref, z, state = self.policy._infer_window(prepared)
        return {"z_rl": _array(z).reshape(-1), "proprio": _array(state).reshape(-1),
                "ref_chunk": _array(ref)[0]}

    def begin(self, metadata):
        if self.replay_windows == "checkpoint_stride":
            raise PolicyError("checkpoint_stride requires boundary(actual_observation), not inference metadata")
        from lerobot.policies.pi05_rlt.replay import ExecutionWindow
        boundary = self._context(metadata)
        if self.window is not None and self.window.executed_steps:
            self._finish(boundary)
        self.window = ExecutionWindow(boundary)
        self.window_rewards = []
        self.window_ref_rows = []

    def boundary(self, observation, episode_id=None, step_id=None, segment_start=False):
        if episode_id is not None and int(episode_id) != self.episode_id:
            raise PolicyError("RLT episode id disagrees with completed episode count")
        if step_id is not None and int(step_id) != self.step_id:
            raise PolicyError("RLT boundary step id disagrees with accepted action count")
        if self.replay_windows == "checkpoint_stride":
            if self._spool is None:
                self._spool = _ObservationSpool(self.options.get("snapshot_dir"), self.snapshot_max_bytes,
                                              self.snapshot_min_free_bytes)
            if self._segment is None or (segment_start and self._segment["records"]):
                self._segment = {"start": self.step_id, "records": [], "rewards": [], "snapshots": {}, "plans": []}
                self._segments.append(self._segment)
            index = len(self._segment["records"])
            path = self._spool.put(observation)
            self._segment["snapshots"][index] = path
            if self._segment["plans"] and self._segment["plans"][-1][0] == index:
                self._segment["plans"][-1] = (index, path)
            else:
                self._segment["plans"].append((index, path))
            return {"deferred": True, "episode_id": self.episode_id, "step_id": self.step_id,
                    "segment_start": bool(segment_start), "snapshot_bytes": self._spool.bytes}
        metadata = self.snapshot(observation)
        self.begin(metadata)
        return metadata

    def _finish(self, next_boundary):
        from lerobot.policies.pi05_rlt.replay import build_transition
        transition = build_transition(self.window, next_boundary, horizon=self.horizon,
                                      action_dim=self.policy.config.action_dim)
        transition.rewards[:self.window.executed_steps] = self.window_rewards
        for index, ref_row in enumerate(self.window_ref_rows):
            if ref_row is not None and self.window.sources[index] != 2:
                transition.ref_chunk[index] = _array(ref_row)
        transition.validate()
        # Delay commit until episode end: terminal annotation may follow exactly a
        # full window and must be applied to its last REAL step, not a fake action.
        self.transitions.append(transition)
        self.window = None
        self.window_rewards = []
        self.window_ref_rows = []

    def record_step(self, action, source, intervention=False, reward=0., boundary=None,
                    next_observation=None, ref_row=None):
        from lerobot.policies.pi05_rlt.replay import TransitionSource
        if boundary is not None:
            self.begin(boundary)
        if self.needs_boundary:
            raise PolicyError("An actual observation boundary is required before recording an RLT step")
        if isinstance(source, str):
            try:
                source = TransitionSource[source.upper()]
            except KeyError as exc:
                raise PolicyError("Unknown RLT action source") from exc
        if int(source) not in (0, 1, 2, 3) or not np.isfinite(reward):
            raise PolicyError("Invalid RLT source/reward")
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (len(self.runner.names),) or not np.isfinite(action).all():
            raise PolicyError("Invalid executed RLT action")
        if self.step_id >= self.max_episode_steps:
            raise PolicyError("RLT episode exceeds max_episode_steps; end the episode")
        if ref_row is not None:
            ref_row = np.asarray(ref_row, dtype=np.float32)
            if ref_row.shape != (self.policy.config.action_dim,) or not np.isfinite(ref_row).all():
                raise PolicyError("Invalid normalized model-order RLT reference row")
            ref_row = self.runner.torch.from_numpy(ref_row.copy())
        ordered = action[[self.runner.names.index(n) for n in self.runner.action_names]]
        tensor = self.runner.torch.from_numpy(ordered.copy()).unsqueeze(0)
        normalized = self.normalizer._normalize_action(tensor, inverse=False).detach().cpu()[0]
        if self.replay_windows == "checkpoint_stride":
            from lerobot.policies.pi05_rlt.replay import StepRecord
            if next_observation is None:
                raise PolicyError("checkpoint_stride requires the actual next_observation of every action")
            segment = self._segment
            index = len(segment["records"])
            if index - segment["plans"][-1][0] >= self.horizon:
                segment["plans"].append((index, segment["snapshots"][index]))
            after = self._spool.put(next_observation)
            segment["records"].append(StepRecord(normalized, int(source), bool(intervention), self.step_id, ref_row))
            segment["rewards"].append(float(reward))
            segment["snapshots"][index + 1] = after
            self.step_id += 1
            return
        self.window.append(normalized, int(source), intervention)
        self.window_rewards.append(float(reward))
        self.window_ref_rows.append(ref_row)
        self.step_id += 1
        if self.window.executed_steps == self.horizon:
            if next_observation is None:
                raise PolicyError("A full executed window requires its real next_observation")
            self.begin(self.snapshot(next_observation))

    def _materialize_stride(self, observation, terminal, reward):
        from lerobot.policies.pi05_rlt.replay import DeferredBoundarySnapshot, build_stride2_transitions
        segments = [segment for segment in self._segments if segment["records"]]
        if not segments:
            return []
        final_segment = segments[-1]
        final_segment["snapshots"][len(final_segment["records"])] = self._spool.put(observation)
        contexts = {}

        def context_at(path, step_id):
            if path not in contexts:
                value = self._context(self.snapshot(self._spool.read(path)))
                value.step_id = step_id
                contexts[path] = value
            return contexts[path]

        def materialize(snapshot):
            return context_at(snapshot.observation["spool_path"], snapshot.step_id)

        transitions = []
        for segment in segments:
            records = segment["records"]
            snapshots = {
                index: DeferredBoundarySnapshot({"spool_path": path}, self.runner.torch.empty(0),
                                                segment["start"] + index)
                for index, path in segment["snapshots"].items()
            }
            plan_index = 0
            for index, record in enumerate(records):
                while plan_index + 1 < len(segment["plans"]) and segment["plans"][plan_index + 1][0] <= index:
                    plan_index += 1
                # Actual predictor references win. Recorded non-RLT/legacy data
                # lacks these rows, so reconstruct the covering VLA chunk once.
                if record.source != 2 and record.ref_row is None:
                    plan_start, path = segment["plans"][plan_index]
                    plan = context_at(path, segment["start"] + plan_start)
                    record.ref_row = plan.ref_chunk[index - plan_start].detach().clone()
            built = build_stride2_transitions(
                records, snapshots, snapshots[len(records)], horizon=self.horizon,
                action_dim=self.policy.config.action_dim, stride=self.stride,
                materialize_snapshot=materialize, terminal_reward=0.)
            for transition in built:
                offset = transition.step_id - segment["start"]
                end = offset + transition.executed_steps
                transition.rewards[:transition.executed_steps] = segment["rewards"][offset:end]
                at_episode_end = segment is final_segment and end == len(records)
                # The library uses terminal_snapshot to close partial tails;
                # segment breaks and time limits retain their bootstrap target.
                transition.done = bool(terminal and at_episode_end)
                if at_episode_end:
                    transition.rewards[transition.executed_steps - 1] += float(reward)
                transition.validate()
            transitions.extend(built)
        return transitions

    def close(self):
        if self._spool is not None:
            self._spool.close()
            self._spool = None
        self._segments = []
        self._segment = None

    def end_episode(self, observation, terminal=False, reward=0., success=False, truncated=False):
        if terminal and truncated:
            raise PolicyError("An episode cannot be terminal and truncated simultaneously")
        if not terminal and not truncated:
            raise PolicyError("Mark episode end terminal or truncated explicitly")
        if not np.isfinite(reward):
            raise PolicyError("Nonfinite episode reward")
        if self.replay_windows == "checkpoint_stride":
            self.transitions = self._materialize_stride(observation, terminal, reward)
        else:
            # End annotation may arrive after a hold, even at a full horizon.
            final_boundary = None
            if self.transitions or (self.window is not None and self.window.executed_steps):
                final_boundary = self._context(self.snapshot(observation))
            if self.window is not None and self.window.executed_steps:
                self._finish(final_boundary)
            if self.transitions:
                last = self.transitions[-1]
                last.next_z_rl = _array(final_boundary.z_rl).reshape(-1)
                last.next_proprio = _array(final_boundary.proprio).reshape(-1)
                last.next_ref_chunk = _array(final_boundary.ref_chunk)
                last.done = bool(terminal)
                last.rewards[last.executed_steps - 1] += float(reward)
        if self.transitions:
            for transition in self.transitions:
                transition.success = int(bool(success))
                self.replay.add(transition)
            self.new_transitions += len(self.transitions)
        count = len(self.transitions)
        self.transitions = []
        self.window = None
        self.window_rewards = []
        self.window_ref_rows = []
        self.close()
        self.episode_id += 1
        self.step_id = 0
        self.runner.reset()
        metrics = self.update()
        output = self.options.get("output_dir")
        saved = self.save(Path(output) / ("episode_%06d" % self.episode_id)) if output and len(self.replay) else None
        return {"transitions": count, "replay_size": len(self.replay), "metrics": metrics,
                "weight_version": self.learner.updates, "checkpoint": saved}

    def update(self):
        if self.step_id or self.transitions or (self.window is not None and self.window.executed_steps):
            raise PolicyError("Learner updates require a paused, completed episode")
        if len(self.replay) < max(self.batch_size, int(self.options.get("warmup_transitions", self.batch_size))):
            return []
        pending = self.new_transitions
        metrics = self.learner.learn(self.replay, batch_size=self.batch_size, new_transitions=pending)
        self.new_transitions = 0
        self.policy.eval()
        return metrics

    def save(self, path):
        if self.step_id or self.transitions:
            raise PolicyError("Save replay and learner together only at a completed episode boundary")
        path = Path(path)
        if path.exists():
            raise PolicyError("Checkpoint directory already exists: %s" % path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(tempfile.mkdtemp(prefix=".rlt-", dir=str(path.parent)))
        self.replay.save(temporary / "replay.npz")
        self.learner.save(temporary / "learner.pt", episode_id=self.episode_id)
        manifest = {"schema_version": 1, "episode_id": self.episode_id,
                    "new_transitions": self.new_transitions, "updates": self.learner.updates,
                    "policy_type": "pi05_rlt", "action_names": self.runner.action_names,
                    "state_names": self.runner.state_names, "options": self.options,
                    "model_contract_sha256": getattr(self.runner, "contract_sha256", None),
                    "replay_windows": self.replay_windows, "action_stride": self.stride}
        (temporary / "manifest.json").write_text(json.dumps(manifest, indent=2))
        os.rename(str(temporary), str(path))
        return str(path)

    def _resume(self, path):
        from lerobot.policies.pi05_rlt.learning import RLTLearner
        from lerobot.policies.pi05_rlt.replay import ReplayBuffer
        manifest = json.loads((path / "manifest.json").read_text())
        saved_mode = manifest.get("replay_windows", "execution_chunks")
        if saved_mode not in ("checkpoint_stride", "execution_chunks"):
            raise PolicyError("Unknown replay window mode in resume manifest")
        if "replay_windows" in self.options and self.options["replay_windows"] != saved_mode:
            raise PolicyError("Configured replay_windows differs from the saved collector contract")
        if manifest.get("action_stride", self.stride) != self.stride:
            raise PolicyError("Checkpoint action_stride differs from the saved replay contract")
        self.replay_windows = saved_mode
        if manifest.get("schema_version") != 1 or manifest.get("action_names") != self.runner.action_names or manifest.get("state_names") != self.runner.state_names:
            raise PolicyError("Resume manifest does not match deployment semantics")
        if manifest.get("model_contract_sha256") != getattr(self.runner, "contract_sha256", None):
            raise PolicyError("Checkpoint configuration/processors changed since the saved replay; resume rejected")
        learner = RLTLearner.load(path / "learner.pt", device=self.runner.options.get("device", "cpu"))
        for key in ("state_dim", "action_dim", "n_action_steps_rl", "rlt_hidden_dim", "pi05_pretrained_path"):
            if getattr(learner.policy.config, key) != getattr(self.policy.config, key):
                raise PolicyError("Resume checkpoint incompatible with deployment: %s" % key)
        # Reuse the original, already loaded frozen VLA and canonical processors;
        # load only matching learned tensors and optimizer/RNG state from resume.
        self.policy.load_state_dict(learner.policy.state_dict(), strict=True)
        self.learner = type(learner)(self.policy, learner.config)
        self.learner.actor_optimizer.load_state_dict(learner.actor_optimizer.state_dict())
        self.learner.critic_optimizer.load_state_dict(learner.critic_optimizer.state_dict())
        self.learner.updates = learner.updates
        self.replay = ReplayBuffer.load(path / "replay.npz", capacity=self.replay.capacity)
        self._validate_replay_contract()
        self.episode_id = int(manifest["episode_id"])
        self.new_transitions = int(manifest.get("new_transitions", 0))
