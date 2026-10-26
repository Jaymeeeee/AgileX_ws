"""Validated deployment contracts shared by the Python entry points."""

import copy
import math
import os
import re
from pathlib import Path

import yaml

from .workflows.definitions import WORKFLOWS


class ConfigError(ValueError):
    pass


def component_names(arms):
    return [
        name
        for side in arms
        for name in (
            ["%s_joint_%d" % (side, i) for i in range(1, 7)]
            + ["%s_gripper_width" % side]
        )
    ]


def _merge(base, update):
    result = copy.deepcopy(base)
    for key, value in update.items():
        # Camera maps and device selectors replace the inherited mapping.
        if (
            isinstance(value, dict)
            and isinstance(result.get(key), dict)
            and key not in ("camera_map", "leader", "follower")
        ):
            result[key] = _merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


class _Loader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ConfigError("Duplicate YAML key: %s" % key)
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def _path(value, parent):
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ConfigError("Paths must be nonempty strings or null")
    path = Path(os.path.expandvars(value)).expanduser()
    return str((path if path.is_absolute() else parent / path).resolve())


def _read(path, seen):
    path = Path(path).expanduser().resolve()
    if path in seen:
        raise ConfigError("Configuration inheritance cycle at %s" % path)
    with path.open() as stream:
        data = yaml.load(stream, Loader=_Loader)
    if not isinstance(data, dict):
        raise ConfigError("Configuration must be a YAML mapping")
    base_path = data.pop("extends", None)
    base = _read(path.parent / base_path, seen + [path]) if base_path else {}
    for section, keys in (
        ("robot", ("can_config",)),
        ("policy", ("checkpoint", "lerobot_source")),
        ("record", ("output_dir", "ros_setup", "topics_config")),
    ):
        for key in keys:
            if key in data.get(section, {}):
                data[section][key] = _path(data[section][key], path.parent)
    for section in ("policy", "record"):
        executable = data.get(section, {}).get("python")
        if executable and ("/" in executable or executable.startswith("~")):
            data[section]["python"] = _path(executable, path.parent)
    rlt = data.get("workflow", {}).get("rlt", {})
    for key in ("prefill_replay", "resume", "output_dir", "snapshot_dir"):
        if key in rlt:
            rlt[key] = _path(rlt[key], path.parent)
    result = _merge(base, data)
    result["_config_path"] = str(path)
    return result


def _positive(value, name, zero=False):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ConfigError("%s must be a finite number" % name)
    if value < 0 or (value == 0 and not zero):
        raise ConfigError(
            "%s must be %s" % (name, "nonnegative" if zero else "positive")
        )


def _bind_can_adapters(config):
    """Expand device switches and named adapters without accessing hardware."""
    cfg = copy.deepcopy(config)
    robot = cfg.get("robot", {})
    adapters = {}
    if robot.get("can_config"):
        with Path(robot["can_config"]).open() as stream:
            data = yaml.load(stream, Loader=_Loader)
        if not isinstance(data, dict) or not isinstance(data.get("adapters"), dict):
            raise ConfigError("robot.can_config must contain an adapters mapping")
        adapters = data["adapters"]
    for side in robot.get("arms", []):
        pair = robot.get("pairs", {}).get(side, {})
        for role in ("follower", "leader"):
            spec = pair.get(role)
            if isinstance(spec, bool):
                spec = {"adapter": "%s_%s" % (side, role)} if spec else None
                pair[role] = spec
            if not isinstance(spec, dict) or "adapter" not in spec:
                continue
            name = spec["adapter"]
            if not isinstance(name, str) or name not in adapters:
                raise ConfigError(
                    "Unknown CAN adapter for %s %s: %s" % (side, role, name)
                )
            if set(spec) != {"adapter"}:
                raise ConfigError(
                    "Configure the CAN selector in robot.can_config when using adapter"
                )
            if not isinstance(adapters[name], dict):
                raise ConfigError("CAN adapter %s must be a mapping" % name)
            pair[role] = dict(copy.deepcopy(adapters[name]), adapter=name)
    return cfg


def validate_config(config):
    cfg = copy.deepcopy(config)
    for key in ("robot", "policy", "observation", "workflow", "record"):
        if not isinstance(cfg.get(key), dict):
            raise ConfigError("Missing configuration mapping: %s" % key)
    if cfg.get("schema_version", 1) != 1:
        raise ConfigError("Unsupported schema_version")
    unknown = set(cfg) - {
        "schema_version",
        "robot",
        "policy",
        "observation",
        "workflow",
        "record",
        "_config_path",
    }
    if unknown:
        raise ConfigError("Unknown top-level configuration keys: %s" % sorted(unknown))
    robot, policy, obs, flow, record = (
        cfg[k] for k in ("robot", "policy", "observation", "workflow", "record")
    )
    arms = robot.get("arms")
    if arms not in (["left"], ["right"], ["left", "right"]):
        raise ConfigError(
            "robot.arms must be [left], [right], or [left, right] (canonical order)"
        )
    if robot.get("profile") != "piper" or robot.get("action_space") != "joint_position":
        raise ConfigError(
            "Piper executor requires joint_position targets after policy postprocessing"
        )
    identities = []
    for side in arms:
        pair = robot.get("pairs", {}).get(side)
        if not isinstance(pair, dict):
            raise ConfigError("Missing robot.pairs mapping for %s" % side)
        for role in ("follower", "leader"):
            spec = pair.get(role)
            if role == "leader" and spec is None:
                continue
            if not isinstance(spec, dict):
                raise ConfigError("Missing CAN device for %s %s" % (side, role))
            selectors = [
                key for key in ("channel", "usb_serial", "usb_path") if key in spec
            ]
            if len(selectors) != 1:
                raise ConfigError(
                    "Choose one CAN selector (channel, usb_serial, usb_path) for %s %s"
                    % (side, role)
                )
            key = selectors[0]
            value = spec[key]
            if not isinstance(value, str) or not value.strip():
                raise ConfigError(
                    "CAN %s for %s %s must be a nonempty string" % (key, side, role)
                )
            identities.append((key, value))
    if len(identities) != len(set(identities)):
        raise ConfigError("Each arm needs an independent CAN adapter/channel")
    if not isinstance(robot.get("firmware"), str) or not robot["firmware"]:
        raise ConfigError(
            "robot.firmware must explicitly identify the firmware profile"
        )
    for key, default in (
        ("control_hz", 100),
        ("gripper_force", 1),
        ("move_speed_percent", 100),
        ("mode_settle_sec", 0.1),
        ("startup_enable_timeout_sec", 10.0),
        ("startup_alignment_timeout_sec", 10.0),
    ):
        robot.setdefault(key, default)
        _positive(robot[key], "robot." + key, zero=(key == "mode_settle_sec"))
    if (
        type(robot["move_speed_percent"]) is not int
        or robot["move_speed_percent"] > 100
    ):
        raise ConfigError("robot.move_speed_percent must be an integer from 1 to 100")
    robot.setdefault("can_tx_queue_len", 64)
    if type(robot["can_tx_queue_len"]) is not int or robot["can_tx_queue_len"] <= 0:
        raise ConfigError("robot.can_tx_queue_len must be a positive integer")
    policy_type = policy.get("type")
    if (
        not isinstance(policy_type, str)
        or not policy_type.strip()
        or policy_type == "auto"
    ):
        raise ConfigError(
            "policy.type is required and must explicitly select a registered LeRobot model"
        )
    if policy.get("backend", "lerobot") not in ("lerobot", "mock"):
        raise ConfigError("policy.backend must be lerobot or mock")
    policy.setdefault("backend", "lerobot")
    policy.setdefault("execution", "auto")
    if policy["execution"] not in ("auto", "step", "chunk"):
        raise ConfigError("policy.execution must be auto, step, or chunk")
    policy.setdefault("device", "cuda")
    policy.setdefault("startup_timeout_sec", 120.0)
    policy.setdefault("inference_timeout_sec", 2.0)
    policy.setdefault("max_pending_commands", 64)
    for key in ("startup_timeout_sec", "inference_timeout_sec"):
        _positive(policy[key], "policy." + key)
    if (
        type(policy["max_pending_commands"]) is not int
        or policy["max_pending_commands"] < 1
    ):
        raise ConfigError("policy.max_pending_commands must be a positive integer")
    if not isinstance(policy.get("overrides", {}), dict):
        raise ConfigError("policy.overrides must be a mapping")
    canonical = component_names(arms)
    for key in ("state_names", "action_names"):
        obs.setdefault(key, canonical[:])
        if not isinstance(obs[key], list) or sorted(obs[key]) != sorted(canonical):
            raise ConfigError(
                "observation.%s must name each selected Piper component once" % key
            )
    obs.setdefault("action_semantics", "absolute_joint_position_rad_gripper_m")
    if obs["action_semantics"] != "absolute_joint_position_rad_gripper_m":
        raise ConfigError(
            "SDK action contract is absolute joint rad / gripper m; supply the model's matching postprocessor"
        )
    obs.setdefault("task", "")
    obs.setdefault("image_resize", "none")
    if obs["image_resize"] not in ("none", "checkpoint"):
        raise ConfigError("observation.image_resize must be none or checkpoint")
    if not isinstance(obs["task"], str):
        raise ConfigError("observation.task must be a string")
    mapping = obs.get("camera_map")
    if not isinstance(mapping, dict):
        raise ConfigError(
            "observation.camera_map must explicitly map policy features to physical cameras"
        )
    for feature, camera in mapping.items():
        if not isinstance(feature, str) or not feature.startswith(
            "observation.images."
        ):
            raise ConfigError("Camera features must start with observation.images.")
        if camera not in obs.get("cameras", {}):
            raise ConfigError("Unknown camera in camera_map: %s" % camera)
        if not isinstance(camera, str) or not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_]*", camera
        ):
            raise ConfigError("Physical camera names must be valid ROS name components")
    selected = sorted(set(mapping.values()))
    serials = [str(obs["cameras"][name].get("serial", "")) for name in selected]
    if any(not value for value in serials) or len(set(serials)) != len(serials):
        raise ConfigError("Selected cameras require distinct explicit serial numbers")
    for key, default in (("width", 640), ("height", 480), ("fps", 15)):
        obs.setdefault(key, default)
        _positive(obs[key], "observation." + key)
    for key in ("width", "height", "fps"):
        if not isinstance(obs[key], int):
            raise ConfigError("observation.%s must be an integer" % key)
    flow.setdefault("mode", "deploy")
    flow.setdefault("learner", "none")
    flow.setdefault("policy_hz", 15.0)
    flow.setdefault("max_episode_sec", 120.0)
    flow.setdefault("auto_start", False)
    flow.setdefault("success_reward", 1.0)
    flow.setdefault("failure_reward", 0.0)
    if flow["mode"] not in WORKFLOWS:
        raise ConfigError("workflow.mode must be collect, teleop, deploy, or online_rl")
    if flow["mode"] in ("collect", "teleop") and any(
        robot["pairs"][side].get("leader") is None for side in arms
    ):
        raise ConfigError("%s requires a leader for every selected arm" % flow["mode"])
    if flow["learner"] not in ("none", "pi05_rlt"):
        raise ConfigError("workflow.learner must be none or pi05_rlt")
    if flow["mode"] == "online_rl":
        if flow["learner"] != "pi05_rlt" or policy_type != "pi05_rlt":
            raise ConfigError(
                "online_rl requires policy.type=pi05_rlt and workflow.learner=pi05_rlt"
            )
    elif flow["learner"] != "none":
        raise ConfigError("Select mode=online_rl to enable a learner")
    _positive(flow["policy_hz"], "workflow.policy_hz")
    if flow["max_episode_sec"] is None:
        if flow["mode"] not in ("teleop", "collect") or record.get("backend") != "ros_publish":
            raise ConfigError("An unlimited episode requires teleop/collect with record.backend=ros_publish")
    else:
        _positive(flow["max_episode_sec"], "workflow.max_episode_sec")
    for key in ("success_reward", "failure_reward"):
        if not isinstance(flow[key], (int, float)) or not math.isfinite(flow[key]):
            raise ConfigError("workflow rewards must be finite")
    rlt = flow.get("rlt", {})
    if not isinstance(rlt, dict):
        raise ConfigError("workflow.rlt must be a mapping")
    for options, key, label in (
        (policy, "update_timeout_sec", "policy.update_timeout_sec"),
        (rlt, "update_timeout_sec", "workflow.rlt.update_timeout_sec"),
    ):
        if key in options:
            _positive(options[key], label)
    if rlt.get("replay_windows", "checkpoint_stride") not in (
        "checkpoint_stride",
        "execution_chunks",
    ):
        raise ConfigError(
            "workflow.rlt.replay_windows must be checkpoint_stride or execution_chunks"
        )
    if flow["policy_hz"] > robot["control_hz"]:
        raise ConfigError("workflow.policy_hz cannot exceed robot.control_hz")
    if "enabled" in record or "ros" in record:
        raise ConfigError(
            "Use record.backend=none|files|ros|ros_publish; legacy record.enabled/ros flags are not supported"
        )
    record.setdefault("backend", "none")
    if record["backend"] not in ("none", "files", "ros", "ros_publish"):
        raise ConfigError("record.backend must be none, files, ros, or ros_publish")
    if flow["mode"] == "online_rl" and record["backend"] == "ros_publish":
        raise ConfigError("online_rl requires stored trajectories; ros_publish only publishes topics")
    record.setdefault("python", "/usr/bin/python3")
    record.setdefault("ros_setup", "/opt/ros/noetic/setup.bash")
    record.setdefault("startup_timeout_sec", 20.0)
    record.setdefault("command_timeout_sec", 5.0)
    for key in ("startup_timeout_sec", "command_timeout_sec"):
        _positive(record[key], "record." + key)
    for key, default in (
        ("queue_size", 256),
        ("min_free_gb", 2.0),
        ("topic_prefix", "/piper"),
        ("compression", "lz4"),
    ):
        record.setdefault(key, default)
    if not isinstance(record.get("output_dir"), str) or not record["output_dir"]:
        raise ConfigError("record.output_dir is required")
    for section, key in ((flow, "auto_start"), (obs, "depth"), (rlt, "exploration")):
        if key not in section:
            continue
        if not isinstance(section[key], bool):
            raise ConfigError("%s must be a YAML boolean" % key)
    if WORKFLOWS[flow["mode"]].requires_recording and record["backend"] == "none":
        raise ConfigError("Collection and online learning require recording")
    _positive(record["queue_size"], "record.queue_size")
    if not isinstance(record["queue_size"], int):
        raise ConfigError("record.queue_size must be an integer")
    if record["compression"] not in ("none", "lz4", "bz2"):
        raise ConfigError("record.compression must be none, lz4, or bz2")
    if not re.fullmatch(
        r"/?[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*", record["topic_prefix"]
    ):
        raise ConfigError("record.topic_prefix must be a valid ROS namespace")
    _positive(record["min_free_gb"], "record.min_free_gb", zero=True)
    return cfg


def load_config(path, overrides=None):
    return validate_config(_bind_can_adapters(_merge(_read(path, []), overrides or {})))


def describe(config):
    return {
        "arms": config["robot"]["arms"],
        "action_names": component_names(config["robot"]["arms"]),
        "can_config": config["robot"].get("can_config"),
        "devices": {
            side: {
                role: spec
                for role, spec in config["robot"]["pairs"][side].items()
                if role in ("follower", "leader") and spec is not None
            }
            for side in config["robot"]["arms"]
        },
        "policy": config["policy"]["type"],
        "checkpoint": config["policy"].get("checkpoint"),
        "cameras": config["observation"]["camera_map"],
        "mode": config["workflow"]["mode"],
        "learner": config["workflow"]["learner"],
        "policy_hz": config["workflow"]["policy_hz"],
        "control_hz": config["robot"]["control_hz"],
        "record_backend": config["record"]["backend"],
        "record_topics_config": config["record"].get("topics_config"),
        "output_dir": (
            None if config["record"]["backend"] == "ros_publish"
            else config["record"]["output_dir"]
        ),
    }
