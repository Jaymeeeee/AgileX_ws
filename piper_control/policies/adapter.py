"""Registry-based LeRobot loading, observation adaptation and physical actions.

Heavy model dependencies are imported only in ``load``. This module owns model
history and checkpoint processors; transport and hardware live outside it.
"""
from __future__ import annotations

import hashlib
import inspect
import json
from pathlib import Path
import sys

import numpy as np

from ..config import component_names
from .interfaces import PolicyError


def _component_order(explicit, saved, canonical, label):
    if explicit is not None and saved is not None and list(explicit) != list(saved):
        raise PolicyError("Configured %s differ from checkpoint metadata" % label)
    names = list(saved or explicit or [])
    if not names:
        raise PolicyError("Checkpoint has no %s; explicitly declare observation.%s "
                          "using its TRAINING order" % (label, label))
    if len(names) != len(canonical) or len(set(names)) != len(names) or set(names) != set(canonical):
        raise PolicyError("%s must describe exactly the selected robot joints: %s" % (label, canonical))
    return names


class PolicyRunner:
    """One owner of policy weights, history, processors and action semantics."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.options = cfg["policy"]
        self.names = component_names(cfg["robot"]["arms"])
        self.policy = self.pre = self.post = None
        self.mode = "step"
        self.loaded = False
        self.rlt = None

    def load(self):
        if self.loaded:
            return self
        if self.options.get("backend") == "mock":
            self.state_names = self.action_names = self.names
            self.loaded = True
            return self
        if self.options.get("backend", "lerobot") != "lerobot":
            raise PolicyError("Unknown policy backend")
        kind = self.options.get("type")
        if not kind or kind == "auto":
            raise PolicyError("policy.type must explicitly name a registered LeRobot policy")
        checkpoint = Path(self.options.get("checkpoint", "")).expanduser()
        if not (checkpoint / "config.json").is_file():
            raise PolicyError("policy.checkpoint must be a prepared local pretrained_model directory")
        raw = json.loads((checkpoint / "config.json").read_text())
        if raw.get("type") != kind:
            raise PolicyError("policy.type=%s disagrees with checkpoint type=%s" % (kind, raw.get("type")))
        digest = hashlib.sha256()
        contract_files = [checkpoint / "config.json"] + sorted(checkpoint.glob("policy_*.json")) + sorted(checkpoint.glob("policy_*.safetensors"))
        if (checkpoint / "deployment.json").exists():
            contract_files.append(checkpoint / "deployment.json")
        for contract_file in contract_files:
            digest.update(contract_file.name.encode())
            digest.update(contract_file.read_bytes())
        self.contract_sha256 = digest.hexdigest()
        source = self.options.get("lerobot_source")
        if source:
            source = Path(source).expanduser()
            sys.path.insert(0, str(source / "src" if (source / "src").is_dir() else source))
        try:
            import torch
            from lerobot.configs import PreTrainedConfig
            from lerobot.policies.factory import get_policy_class, make_pre_post_processors
        except (ImportError, SyntaxError) as exc:
            raise PolicyError("Install LeRobot and this model's optional dependencies in policy.python "
                              "(LeRobot requires its own supported Python); optionally set policy.lerobot_source. "
                              "No repository or weights are downloaded automatically. Original error: %s" % exc) from exc
        self.torch = torch
        # Select the requested registered class FIRST; architecture comes from the checkpoint.
        policy_class = get_policy_class(kind)
        model_cfg = PreTrainedConfig.from_pretrained(str(checkpoint), local_files_only=True)
        model_cfg.device = self.options.get("device", "cpu")
        overrides = self.options.get("overrides", {})
        allowed = {"num_inference_steps", "use_amp", "deploy_gating", "deploy_auto_critical",
                   "deploy_vla_only", "critical_classifier_path", "pi05_pretrained_path",
                   "text_tokenizer_name", "base_model_path"}
        for key, value in overrides.items():
            if key not in allowed or not hasattr(model_cfg, key):
                raise PolicyError("Unsupported deployment override %s; structural parameters belong to checkpoint" % key)
            setattr(model_cfg, key, value)
        if kind == "pi05_rlt" and getattr(model_cfg, "mode", None) == "rlt_training":
            raise PolicyError("Stage-1 RLT training configuration cannot control hardware; export an inference policy first")
        if (kind == "pi05_rlt" and model_cfg.deploy_gating and not model_cfg.deploy_auto_critical
                and not model_cfg.deploy_vla_only):
            raise PolicyError("Manual-only RLT gating needs an operator gate interface that this runtime does not expose; "
                              "set policy.overrides.deploy_gating=false or configure a trained automatic classifier")
        self._validate_contract(model_cfg, raw, checkpoint)
        load_options = {"config": model_cfg, "local_files_only": True}
        if "strict" in inspect.signature(policy_class.from_pretrained).parameters:
            load_options["strict"] = True
        self.policy = policy_class.from_pretrained(str(checkpoint), **load_options).to(model_cfg.device).eval()
        self.pre, self.post = make_pre_post_processors(
            model_cfg, pretrained_path=str(checkpoint),
            preprocessor_overrides={"device_processor": {"device": model_cfg.device}})
        if kind == "pi05_rlt":
            self.policy.attach_frozen_vla_from_config()
            if model_cfg.deploy_auto_critical and not model_cfg.deploy_vla_only:
                self.policy.set_actor_enabled(True)
        self._select_execution()
        from lerobot.processor import RelativeActionsProcessorStep
        relative = [s for s in self.pre.steps if isinstance(s, RelativeActionsProcessorStep) and s.enabled]
        if relative and self.state_names != self.action_names:
            raise PolicyError("Relative action processors require matching state/action component order")
        for step in relative:
            if step.action_names and list(step.action_names) != self.action_names:
                raise PolicyError("Relative processor action names disagree with deployment order")
            step.action_names = list(self.action_names)
            step.bind_action_queue(self.policy.count_queued_actions if self.mode == "step" else None)
        rtc = getattr(model_cfg, "rtc_config", None)
        if rtc is not None and getattr(rtc, "enabled", False):
            raise PolicyError("This executor uses ordinary step/chunk scheduling; disable checkpoint RTC "
                              "or supply a dedicated RTC scheduler, rather than silently changing its semantics")
        if self.cfg.get("workflow", {}).get("mode") == "online_rl":
            from ..learning.rlt import RLTSession
            self.rlt = RLTSession(self, self.cfg["workflow"].get("rlt", {}))
        self.loaded = True
        self.reset()
        return self

    def _validate_contract(self, model_cfg, raw, checkpoint):
        obs = self.cfg.get("observation", {})
        deployment = checkpoint / "deployment.json"
        metadata = json.loads(deployment.read_text()) if deployment.is_file() else {}
        self.state_names = _component_order(obs.get("state_names"),
            raw.get("state_feature_names") or metadata.get("state_names"), self.names, "state_names")
        self.action_names = _component_order(obs.get("action_names"),
            raw.get("action_feature_names") or metadata.get("action_names"), self.names, "action_names")
        for key, value in (("joint_unit", "rad"), ("gripper_unit", "m"),
                           ("action_space", "joint_position")):
            if metadata.get(key, value) != value:
                raise PolicyError("Checkpoint deployment.%s is incompatible with Piper %s" % (key, value))
        inputs = model_cfg.input_features
        if "observation.state" not in inputs or tuple(inputs["observation.state"].shape) != (len(self.names),):
            raise PolicyError("Checkpoint state shape is incompatible with selected arms")
        output = model_cfg.output_features.get("action")
        if output is None or tuple(output.shape) != (len(self.names),):
            raise PolicyError("Checkpoint action shape is incompatible with selected arms; actions are never sliced")
        images = {key for key in inputs if key.startswith("observation.images.")}
        if images != set(obs.get("camera_map", {})):
            raise PolicyError("observation.camera_map keys must match checkpoint cameras: %s" % sorted(images))
        unsupported = set(inputs) - images - {"observation.state"}
        if unsupported:
            raise PolicyError("Checkpoint requires additional observations without a hardware mapping: %s" % sorted(unsupported))

    def _select_execution(self):
        choice = self.options.get("execution", "auto")
        if choice not in ("auto", "step", "chunk"):
            raise PolicyError("policy.execution must be auto, step, or chunk")
        cfg = self.policy.config
        spec_fn = getattr(self.policy, "chunk_inference_spec", None)
        history = getattr(cfg, "n_obs_steps", 1) > 1
        temporal = getattr(cfg, "temporal_ensemble_coeff", None) is not None
        self.spec = None
        if not history and not temporal and spec_fn:
            try:
                self.spec = spec_fn()
            except ValueError:
                if choice == "chunk":
                    raise
                choice = "step"
        if self.spec is not None:
            history = history or not self.spec.current_observation_only
        relative_groot = self.options["type"] == "groot" and getattr(cfg, "use_relative_actions", False)
        if choice == "auto":
            choice = "step" if history or temporal or self.spec is None else "chunk"
        if choice == "chunk" and (history or temporal):
            raise PolicyError("This checkpoint requires select_action observation history/temporal ensembling; use step")
        if choice == "step" and relative_groot:
            raise PolicyError("Relative GR00T actions require full-chunk postprocessing; use chunk")
        if self.options["type"] == "pi05_rlt" and choice != "chunk":
            raise PolicyError("RLT requires explicit chunk boundaries; use chunk or auto")
        if choice == "chunk" and not callable(getattr(self.policy, "predict_action_chunk", None)):
            raise PolicyError("Policy has no chunk capability; use step")
        if choice == "chunk" and self.spec is None:
            raise PolicyError("Policy does not declare a validated chunk inference contract; use step")
        self.mode = choice

    def _prepare(self, observation):
        torch = self.torch
        state = np.asarray(observation["observation.state"], dtype=np.float32)
        if state.shape != (len(self.names),) or not np.isfinite(state).all():
            raise PolicyError("Invalid finite robot state")
        state = state[[self.names.index(name) for name in self.state_names]]
        batch = {"observation.state": torch.from_numpy(state.copy()).unsqueeze(0),
                 "task": [observation.get("task", self.cfg.get("observation", {}).get("task", ""))],
                 "robot_type": self.cfg["robot"].get("profile", "piper")}
        for key in self.cfg.get("observation", {}).get("camera_map", {}):
            image = np.asarray(observation[key])
            if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
                raise PolicyError("%s must be RGB HWC uint8" % key)
            tensor = torch.from_numpy(image.copy()).permute(2, 0, 1).unsqueeze(0).float() / 255.
            shape = tuple(self.policy.config.input_features[key].shape)
            if len(shape) != 3 or shape[0] != 3:
                raise PolicyError("Checkpoint image feature must be CHW RGB")
            resize = self.cfg.get("observation", {}).get("image_resize", "none")
            if resize not in ("none", "checkpoint"):
                raise PolicyError("observation.image_resize must be none or checkpoint")
            if resize == "checkpoint" and tuple(tensor.shape[-2:]) != shape[-2:]:
                tensor = torch.nn.functional.interpolate(tensor, size=shape[-2:], mode="bilinear", align_corners=False)
            batch[key] = tensor
        return self.pre(batch)

    def infer(self, observation):
        if not self.loaded:
            raise PolicyError("PolicyRunner.load() must be called before inference")
        if self.options.get("backend") == "mock":
            return {"action": np.array(observation["observation.state"], dtype=np.float32, copy=True),
                    "metadata": {"source": "BASE", "mock": True, "execution": "step"}}
        torch = self.torch
        with torch.inference_mode():
            prepared = self._prepare(observation)
            predicted = (self.policy.predict_action_chunk(prepared) if self.mode == "chunk"
                         else self.policy.select_action(prepared))
            metadata = {"source": "BASE", "execution": self.mode}
            if self.options["type"] == "pi05_rlt":
                from ..learning.rlt import rlt_prediction
                predicted, extra = rlt_prediction(self, predicted)
                metadata.update(extra)
            canonical = self.post(predicted).detach().float().cpu().numpy()
        rank = 3 if self.mode == "chunk" else 2
        if canonical.ndim != rank or canonical.shape[0] != 1 or canonical.shape[-1] != len(self.names):
            raise PolicyError("Policy processors produced invalid action shape %s" % (canonical.shape,))
        if not np.isfinite(canonical).all():
            raise PolicyError("Policy produced nonfinite physical actions")
        result = canonical[0][..., [self.action_names.index(n) for n in self.names]]
        if self.mode == "chunk":
            steps = getattr(self.spec, "execution_steps", None) if self.spec is not None else None
            if steps is None:
                steps = getattr(self.policy.config, "n_action_steps_rl", getattr(self.policy.config, "n_action_steps", len(result)))
            result = result[:steps]
            if len(result) == 0:
                raise PolicyError("Policy returned empty action chunk")
        return {"actions" if self.mode == "chunk" else "action": result, "metadata": metadata}

    def reset(self):
        if self.policy is not None:
            self.policy.reset()
            for processor in (self.pre, self.post):
                if callable(getattr(processor, "reset", None)):
                    processor.reset()

    def invalidate(self):
        self.reset()

    def close(self):
        if self.rlt is not None:
            self.rlt.close()
        self.reset()
        self.loaded = False
