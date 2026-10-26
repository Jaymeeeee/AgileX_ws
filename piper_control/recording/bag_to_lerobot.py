"""Offline ROS1 -> LeRobot v3 conversion for the right-arm SDK recordings.

Read bags twice: first index source timestamps and small numeric arrays, then
stream selected RGB frames. Neither ROS nor the hardware SDK is imported.
"""
import argparse
from collections import defaultdict
from dataclasses import dataclass, field
import hashlib
import importlib.metadata
import json
import logging
import os
from pathlib import Path
import sys
import uuid

import numpy as np
import yaml


LOG = logging.getLogger("bag_to_lerobot")
NS = 1_000_000_000
CAMERAS = ("cam_h", "cam_r")
MOTOR_KEYS = tuple("motor_%d" % i for i in range(1, 7))
MOTOR_DIAGNOSTICS_TOPIC = "/piper/arms/right/follower/diagnostics"
MOTOR_STATUS_NAMES = tuple("right/follower/motor_feedback/%d" % i for i in range(1, 7))
DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "configs/lerobot_insert_plug_v4.yaml"


def header_ns(msg):
    stamp = int(msg.header.stamp.sec) * NS + int(msg.header.stamp.nanosec)
    if stamp <= 0:
        raise ValueError("Invalid header timestamp; bag receipt time is not a substitute")
    return stamp


def joint_vector(msg, names):
    actual = list(msg.name)
    values = np.asarray(msg.position, dtype=np.float64)
    if (len(actual) != len(set(actual)) or set(actual) != set(names)
            or values.shape != (len(actual),)):
        raise ValueError("Joint names/positions disagree with the configured joint contract: %r" % actual)
    values = values[[actual.index(name) for name in names]].astype(np.float32)
    if not np.isfinite(values).all():
        raise ValueError("Joint positions must be finite float32 values")
    return values


def motor_samples(msg):
    """Six (embedded source time, [A, N*m]) pairs; never use the outer header.

    The SDK's torque is estimated from current. A present/valid cached sample
    may still be stale, so sampling additionally enforces its age in build_plan.
    """
    entries = {}
    for status in msg.status:
        if status.name not in MOTOR_STATUS_NAMES:
            continue
        if status.name in entries:
            raise ValueError("Duplicate follower motor feedback entry: " + status.name)
        values = {}
        for item in status.values:
            if item.key in values:
                raise ValueError("Duplicate motor feedback field: " + item.key)
            values[item.key] = json.loads(item.value)
        if values.get("valid") is not True or values.get("source") != "feedback":
            raise ValueError("Motor sample must be valid follower feedback: " + status.name)
        for key in ("stamp", "current", "torque"):
            value = values.get(key)
            if type(value) not in (int, float) or not np.isfinite(value):
                raise ValueError("Motor %s must be a finite numeric scalar: %s" % (key, status.name))
        if values["stamp"] <= 0:
            raise ValueError("Motor feedback source timestamp must be positive")
        vector = np.asarray([values["current"], values["torque"]], dtype=np.float32)
        if not np.isfinite(vector).all():
            raise ValueError("Motor current/torque exceed float32 range")
        entries[status.name] = (round(values["stamp"] * NS), vector)
    missing = set(MOTOR_STATUS_NAMES) - set(entries)
    if missing:
        raise ValueError("Missing follower motor feedback entries: " + str(sorted(missing)))
    return [entries[name] for name in MOTOR_STATUS_NAMES]


def state_names(cfg):
    names = list(cfg["joint_names"])
    if "motor_state" in cfg:
        names += ["right_joint_%d_%s" % (i, field)
                  for field in ("current", "torque") for i in range(1, 7)]
    return names


def motor_keys(series):
    present = tuple(key for key in MOTOR_KEYS if key in series)
    if present and present != MOTOR_KEYS:
        raise ValueError("Motor state requires all six follower motor streams")
    return present


def rgb_shape(msg):
    height, width, step = int(msg.height), int(msg.width), int(msg.step)
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError("Expected RGB/BGR uint8, got " + msg.encoding)
    if min(height, width) <= 0 or step < width * 3 or len(msg.data) != height * step:
        raise ValueError("Invalid image dimensions, row stride or payload length")
    return height, width, 3


def decode_rgb(msg):
    height, width, channels = rgb_shape(msg)
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, int(msg.step))
    array = rows[:, :width * channels].reshape(height, width, channels)
    if msg.encoding == "bgr8":
        array = array[:, :, ::-1]
    return np.array(array, dtype=np.uint8, order="C", copy=True)


@dataclass
class Series:
    stamps: list = field(default_factory=list)
    bag_stamps: list = field(default_factory=list)
    ordinals: list = field(default_factory=list)
    values: list = field(default_factory=list)
    shape: tuple = None
    count: int = 0
    reordered: int = 0
    duplicates: int = 0

    def finish(self):
        stamps = np.asarray(self.stamps, dtype=np.int64)
        if not len(stamps):
            raise ValueError("Required topic has no messages")
        if len(self.bag_stamps) != len(stamps) or len(self.ordinals) != len(stamps):
            raise ValueError("Inconsistent source index lengths")
        self.count = len(stamps)
        self.reordered = int(np.sum(np.diff(stamps) < 0))
        order = np.argsort(stamps, kind="stable")
        duplicate = np.flatnonzero(np.diff(stamps[order]) == 0)
        values = np.stack(self.values) if len(self.values) else None
        if values is not None:
            if len(values) != len(stamps):
                raise ValueError("Inconsistent numeric source length")
            if not np.array_equal(values[order[duplicate]], values[order[duplicate + 1]]):
                raise ValueError("Conflicting values at an identical source timestamp")
        elif len(duplicate):
            # Without retaining pixel buffers, equality cannot be proven.
            raise ValueError("Duplicate image timestamps cannot be disambiguated")
        self.duplicates = len(duplicate)
        order = order[np.r_[True, np.diff(stamps[order]) != 0]]
        for name in ("stamps", "bag_stamps", "ordinals"):
            setattr(self, name, np.asarray(getattr(self, name), dtype=np.int64)[order])
        self.values = values[order] if values is not None else []


def sample_indices(stamps, grid, mode="previous"):
    right = np.searchsorted(stamps, grid, side="right")
    previous = right - 1
    if mode == "previous":
        return previous
    if mode != "nearest":
        raise ValueError("Unknown sampling mode: " + mode)
    before = np.maximum(previous, 0)
    after = np.minimum(right, len(stamps) - 1)
    return np.where(abs(stamps[after] - grid) < abs(stamps[before] - grid), after, before)


def load_config(path):
    cfg = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(cfg, dict) or cfg.get("schema_version") != 1:
        raise ValueError("Expected schema_version: 1 conversion config")
    names = ["right_joint_%d" % i for i in range(1, 7)] + ["right_gripper_width"]
    if cfg.get("joint_names") != names:
        raise ValueError("This converter requires the canonical seven-component right-arm contract")
    expected = {"state": "/piper/arms/right/follower/joint_states", "action": "/piper/action",
                **{cam: "/piper/cameras/%s/color/image_raw" % cam for cam in CAMERAS}}
    if {key: value["topic"] for key, value in cfg["sources"].items()} != expected:
        raise ValueError("Source mapping must preserve follower feedback / forwarded leader action / RGB semantics")
    if cfg.get("action_metadata_topic") != "/piper/action_metadata":
        raise ValueError("Forwarded actions require /piper/action_metadata")
    if "motor_state" in cfg:
        motor = cfg["motor_state"]
        if not isinstance(motor, dict) or motor.get("topic") != MOTOR_DIAGNOSTICS_TOPIC:
            raise ValueError("Motor state requires the right follower diagnostics topic")
    dataset, sync = cfg["dataset"], cfg["sync"]
    if type(dataset["fps"]) is not int or dataset["fps"] < 1:
        raise ValueError("FPS must be a positive integer")
    if not isinstance(dataset["task"], str) or not dataset["task"].strip():
        raise ValueError("Task description must not be empty")
    if len(dataset["repo_id"].split("/")) != 2 or not all(dataset["repo_id"].split("/")):
        raise ValueError("repo_id must be namespace/dataset")
    for key in ("max_age_sec", "max_clock_skew_sec"):
        if not np.isfinite(sync[key]) or sync[key] <= 0:
            raise ValueError(key + " must be positive and finite")
    for key in ("min_frames", "max_pending_frames"):
        if type(sync[key]) is not int or sync[key] < 1:
            raise ValueError(key + " must be a positive integer")
    return cfg


def bag_identity(path):
    stat = path.stat()
    return {"path": str(path), "size_bytes": stat.st_size, "mtime_ns": stat.st_mtime_ns}


def check_manifest(bag, cfg):
    manifest = json.loads((bag.parent / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("status") != "complete":
        raise ValueError("Recording manifest is not complete: " + str(bag))
    if (manifest["selection"]["arms"] != ["right"]
            or set(manifest["selection"]["cameras"]) != set(CAMERAS)):
        raise ValueError("Manifest is not right arm + cam_h/cam_r: " + str(bag))
    obs = manifest["config"]["observation"]
    if obs.get("action_semantics") != "absolute_joint_position_rad_gripper_m":
        raise ValueError("Manifest has incompatible action units/semantics")
    if obs.get("state_names") != cfg["joint_names"] or obs.get("action_names") != cfg["joint_names"]:
        raise ValueError("Manifest has incompatible state/action component names")


def verify_actions(action, metadata, names):
    """Join evidence by source timestamp, never by receipt order or nearest step."""
    if not metadata:
        raise ValueError("No action provenance metadata")
    stamps = np.array(sorted(metadata), dtype=np.int64)
    ids = sample_indices(stamps, action.stamps, "nearest")
    matched = abs(stamps[ids] - action.stamps) <= 1000  # float seconds -> ns roundoff only
    good = np.flatnonzero(matched)
    if not len(good):
        raise ValueError("No actions have matching provenance metadata")
    first, last = int(good[0]), int(good[-1])
    if not matched[first:last + 1].all():
        raise ValueError("Action provenance is missing inside the episode; cannot bridge an unverified interval")
    for index in good:
        row = metadata[int(stamps[ids[index]])]
        # A matching timestamp is insufficient: verify the actual values as well.
        actual = row.get("joint_names", [])
        if len(actual) != len(set(actual)) or set(actual) != set(names):
            raise ValueError("Action metadata has inconsistent joint names")
        values = np.asarray(row["action"], dtype=np.float32)
        if values.shape != (len(names),) or not np.isfinite(values).all():
            raise ValueError("Invalid action metadata vector")
        values = values[[actual.index(name) for name in names]]
        if not np.array_equal(values, action.values[index]):
            raise ValueError("/piper/action and its metadata disagree")
    trimmed = {"unverified_prefix_actions": first,
               "unverified_suffix_actions": len(matched) - last - 1,
               "metadata_messages": len(metadata), "verified_actions": len(good)}
    for name in ("stamps", "bag_stamps", "ordinals", "values"):
        setattr(action, name, getattr(action, name)[first:last + 1])
    return trimmed


def read_index(bag, cfg):
    from rosbags.highlevel import AnyReader

    topics = {spec["topic"]: key for key, spec in cfg["sources"].items()}
    evidence_topic = cfg["action_metadata_topic"]
    series = {key: Series() for key in cfg["sources"]}
    motor_topic = cfg["motor_state"]["topic"] if "motor_state" in cfg else None
    if motor_topic:
        series.update({key: Series() for key in MOTOR_KEYS})
    required_topics = set(topics) | {evidence_topic}
    if motor_topic:
        required_topics.add(motor_topic)
    motor_ordinal = 0
    metadata = {}
    with AnyReader([bag]) as reader:
        missing = required_topics - {c.topic for c in reader.connections}
        if missing:
            raise ValueError("Missing required topics: " + str(sorted(missing)))
        connections = [c for c in reader.connections if c.topic in required_topics]
        for connection, bag_stamp, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            if connection.topic == motor_topic:
                for key, (stamp, vector) in zip(MOTOR_KEYS, motor_samples(msg)):
                    if abs(stamp - bag_stamp) > cfg["sync"]["max_clock_skew_sec"] * NS:
                        raise ValueError("Motor source/receipt clock discrepancy exceeds limit: " + key)
                    source = series[key]
                    source.stamps.append(stamp)
                    source.bag_stamps.append(bag_stamp)
                    source.ordinals.append(motor_ordinal)
                    source.values.append(vector)
                motor_ordinal += 1
                continue
            if connection.topic == evidence_topic:
                row = json.loads(msg.data)
                if row.get("source") != "HUMAN" or row.get("action_evidence") != "forwarded_can_target":
                    raise ValueError("Expected HUMAN / forwarded_can_target action provenance")
                stamp = float(row["timestamp"])
                if not np.isfinite(stamp) or stamp <= 0:
                    raise ValueError("Invalid action metadata timestamp")
                stamp = round(stamp * NS)
                if stamp in metadata and metadata[stamp] != row:
                    raise ValueError("Conflicting action metadata at identical timestamp")
                metadata[stamp] = row
                continue
            key = topics[connection.topic]
            source = series[key]
            stamp = header_ns(msg)
            if abs(stamp - bag_stamp) > cfg["sync"]["max_clock_skew_sec"] * NS:
                raise ValueError("Source/receipt clock discrepancy exceeds limit: " + connection.topic)
            if key in CAMERAS:
                shape = rgb_shape(msg)
                if source.shape is not None and source.shape != shape:
                    raise ValueError("Camera resolution changed: " + connection.topic)
                source.shape = shape
            else:
                source.values.append(joint_vector(msg, cfg["joint_names"]))
            source.stamps.append(stamp)
            source.bag_stamps.append(bag_stamp)
            source.ordinals.append(len(source.ordinals))
        bounds = {"bag_start_ns": int(reader.start_time), "bag_end_ns": int(reader.end_time - 1)}
    for source in series.values():
        source.finish()
    evidence = verify_actions(series["action"], metadata, cfg["joint_names"])
    return series, bounds, evidence


def build_plan(series, bounds, cfg):
    fps = cfg["dataset"]["fps"]
    # Keep the original state/action/RGB grid when appending motor observations.
    # Missing or stale motor feedback must fail, not silently move/crop the grid.
    base = [series[key] for key in cfg["sources"]]
    start = max(bounds["bag_start_ns"], *(int(s.stamps[0]) for s in base))
    end = min(bounds["bag_end_ns"], *(int(s.stamps[-1]) for s in base))
    if end < start:
        raise ValueError("Required streams have no common time interval")
    count = (end - start) * fps // NS + 1
    if count < cfg["sync"]["min_frames"]:
        raise ValueError("Not enough synchronized frames")
    grid = start + np.rint(np.arange(count, dtype=np.float64) * NS / fps).astype(np.int64)
    selected, summary = {}, {}
    motors = motor_keys(series)
    for key, source in series.items():
        indices = sample_indices(source.stamps, grid)
        if (indices < 0).any():
            raise ValueError("No past sample for " + key)
        age = grid - source.stamps[indices]
        bad = np.flatnonzero(age > round(cfg["sync"]["max_age_sec"] * NS))
        if len(bad):
            raise ValueError("%s: frame %d sample age %.6fs exceeds limit; refusing to compress a gap" %
                             (key, bad[0], age[bad[0]] / NS))
        selected[key] = indices
        topic = cfg["motor_state"]["topic"] if key in motors else cfg["sources"][key]["topic"]
        summary[key] = {"topic": topic, "messages": source.count,
                        "reordered": source.reordered, "duplicate_stamps": source.duplicates,
                        "max_age_ms": float(age.max() / 1e6),
                        "reused_samples": int(len(indices) - len(np.unique(indices)))}
        if key in motors:
            summary[key].update(status_name=MOTOR_STATUS_NAMES[MOTOR_KEYS.index(key)],
                                timestamp_field="status.values.stamp", fields=["current", "torque"],
                                units=["A", "N*m"])
    return grid, selected, summary


def feature_schema(cfg, series):
    if bool(motor_keys(series)) != ("motor_state" in cfg):
        raise ValueError("Motor streams and configured observation state disagree")
    observation_names = state_names(cfg)
    features = {
        "observation.state": {"dtype": "float32", "shape": (len(observation_names),), "names": observation_names},
        "action": {"dtype": "float32", "shape": (len(cfg["joint_names"]),), "names": cfg["joint_names"]},
    }
    for cam in CAMERAS:
        features["observation.images." + cam] = {"dtype": "video", "shape": series[cam].shape,
                                               "names": ["height", "width", "channels"]}
    return features


def numeric_frame(series, selected, index):
    state = series["state"].values[selected["state"][index]]
    motors = motor_keys(series)
    if motors:
        values = np.stack([series[key].values[selected[key][index]] for key in motors])
        state = np.concatenate((state, values[:, 0], values[:, 1]))
    return {"observation.state": state,
            "action": series["action"].values[selected["action"][index]]}


def numeric_arrays(series, selected):
    state = series["state"].values[selected["state"]]
    motors = motor_keys(series)
    if motors:
        values = np.stack([series[key].values[selected[key]] for key in motors], axis=1)
        state = np.concatenate((state, values[:, :, 0], values[:, :, 1]), axis=1)
    return {"observation.state": state, "action": series["action"].values[selected["action"]]}


def numeric_hash(array):
    return hashlib.sha256(np.asarray(array, dtype="<f4").tobytes()).hexdigest()


def write_episode(dataset, bag, cfg, series, grid, selected):
    from rosbags.highlevel import AnyReader

    wanted = {cam: defaultdict(list) for cam in CAMERAS}
    for cam in CAMERAS:
        for frame, index in enumerate(selected[cam]):
            wanted[cam][int(series[cam].ordinals[index])].append(frame)
    topics = {cfg["sources"][cam]["topic"]: cam for cam in CAMERAS}
    seen, pending, next_frame = defaultdict(int), {}, 0
    with AnyReader([bag]) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        for connection, _, raw in reader.messages(connections=connections):
            cam = topics[connection.topic]
            frames = wanted[cam].get(seen[cam], [])
            seen[cam] += 1
            if not frames:
                continue
            array = decode_rgb(reader.deserialize(raw, connection.msgtype))
            for frame in frames:
                pending.setdefault(frame, {})["observation.images." + cam] = array
            while next_frame in pending and len(pending[next_frame]) == len(CAMERAS):
                frame = numeric_frame(series, selected, next_frame)
                frame.update(pending.pop(next_frame))
                frame["task"] = cfg["dataset"]["task"]
                dataset.add_frame(frame)
                next_frame += 1
            if len(pending) > cfg["sync"]["max_pending_frames"]:
                raise ValueError("Image reordering exceeds bounded pending frame buffer")
            if next_frame == len(grid):
                break
    if next_frame != len(grid):
        raise ValueError("Image readback incomplete: %d/%d" % (next_frame, len(grid)))
    dataset.save_episode(parallel_encoding=False)


def json_write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")


def validate_dataset(root, cfg, plans):
    """Official reader: every numeric label plus first/last video frame per episode."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(cfg["dataset"]["repo_id"], root=root, video_backend="pyav")
    total = sum(len(plan["grid"]) for plan in plans)
    if dataset.num_episodes != len(plans) or len(dataset) != total:
        raise ValueError("Readback episode/frame count mismatch")
    offset = 0
    for episode, plan in enumerate(plans):
        count = len(plan["grid"])
        rows = dataset.hf_dataset.select(range(offset, offset + count)).with_format("numpy")
        for key, expected in numeric_arrays(plan["series"], plan["selected"]).items():
            np.testing.assert_array_equal(np.asarray(rows[key]), expected)
        np.testing.assert_allclose(np.asarray(rows["timestamp"]).reshape(-1), np.arange(count) / cfg["dataset"]["fps"], atol=1e-5)
        np.testing.assert_array_equal(np.asarray(rows["frame_index"]).reshape(-1), np.arange(count))
        np.testing.assert_array_equal(np.asarray(rows["episode_index"]).reshape(-1), np.full(count, episode))
        for index in (offset, offset + count - 1):
            frame = dataset[index]
            for cam in CAMERAS:
                image = frame["observation.images." + cam]
                height, width, _ = plan["series"][cam].shape
                if tuple(image.shape) != (3, height, width) or not np.isfinite(image.numpy()).all():
                    raise ValueError("Invalid decoded video: " + cam)
        offset += count
    return {"frames": total, "episodes": len(plans), "all_numeric_values_exact": True,
            "state_dimension": len(state_names(cfg)), "action_dimension": len(cfg["joint_names"]),
            "episode_timing_checked": True, "video_boundaries_decoded": True}


def run(args):
    config_path = Path(args.config).expanduser().resolve()
    cfg = load_config(config_path)
    resolve = lambda value: (config_path.parent / Path(value).expanduser()).resolve()
    input_dir = Path(args.input).expanduser().resolve() if args.input else resolve(cfg["input_dir"])
    output = Path(args.output).expanduser().resolve() if args.output else resolve(cfg["output_dir"])
    if args.limit is not None and args.limit < 1:
        raise ValueError("--limit must be positive")
    if args.limit and not args.inspect and not args.output:
        raise ValueError("Use an explicit --output for a limited preview conversion")
    if not args.inspect and output.exists():
        raise ValueError("Output already exists; refusing to overwrite: " + str(output))
    if list(input_dir.rglob("*.bag.active")):
        raise ValueError("Input contains an actively recorded bag")
    bags = sorted(input_dir.rglob("*.bag"))
    if args.limit:
        bags = bags[:args.limit]
    if not bags:
        raise ValueError("No .bag files under " + str(input_dir))
    plans, reports, schema = [], [], None
    report = {"status": "inspecting", "config": cfg, "output": str(output), "episodes": reports,
              "timing": "15 Hz by default; causal previous sampling using header stamps and individual motor source stamps; no gap compression",
              "action": "HUMAN forwarded_can_target; absolute joints rad + gripper m",
              "state": {"component_names": state_names(cfg),
                        "units": ["rad"] * 6 + ["m"] + (["A"] * 6 + ["N*m"] * 6 if "motor_state" in cfg else [])},
              "omitted": ["depth", "camera calibration", "other diagnostics" if "motor_state" in cfg else "diagnostics", "empty velocity/effort"],
              "success_labels": "Not inferred from recording completeness",
              "converter_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    if "motor_state" in cfg:
        report["state"]["motor_feedback"] = {
            "topic": cfg["motor_state"]["topic"], "source": "feedback", "role": "right/follower",
            "order": "original 7 positions; currents J1..J6; estimated torques J1..J6",
            "torque_semantics": "SDK estimate from current times fixed joint coefficients, not independent force/torque measurement",
            "timestamp_field": "each motor_feedback entry's values.stamp; not DiagnosticArray.header.stamp",
        }
    # Complete preflight for every bag before creating an output dataset.
    for index, bag in enumerate(bags):
        LOG.info("[%d/%d] Inspect %s", index + 1, len(bags), bag.parent.name)
        identity = bag_identity(bag)
        check_manifest(bag, cfg)
        try:
            series, bounds, evidence = read_index(bag, cfg)
            grid, selected, summary = build_plan(series, bounds, cfg)
        except ValueError as exc:
            raise ValueError("%s: %s" % (bag.parent.name, exc)) from exc
        features = feature_schema(cfg, series)
        if schema is not None and features != schema:
            raise ValueError("Feature schema differs between bags: " + str(bag))
        schema = features
        if bag_identity(bag) != identity:
            raise ValueError("Bag changed during inspection: " + str(bag))
        arrays = numeric_arrays(series, selected)
        item = {"episode_index": index, "source": identity, "frames": len(grid), "sources": summary,
                "evidence": evidence, "grid_start_ns": int(grid[0]), "grid_end_ns": int(grid[-1]),
                "trimmed_start_sec": (int(grid[0]) - bounds["bag_start_ns"]) / NS,
                "trimmed_end_sec": (bounds["bag_end_ns"] - int(grid[-1])) / NS,
                "numeric_sha256": {"state": numeric_hash(arrays["observation.state"]), "action": numeric_hash(arrays["action"])}}
        reports.append(item)
        plans.append(dict(bag=bag, series=series, grid=grid, selected=selected, identity=identity))
    report.update(status="inspected", total_episodes=len(plans), total_frames=sum(len(p["grid"]) for p in plans))
    if args.report:
        report_path = Path(args.report).expanduser().resolve()
        if report_path.exists():
            raise ValueError("Report already exists; refusing to overwrite: " + str(report_path))
        json_write(report_path, report)
    if args.inspect:
        return {key: report[key] for key in ("status", "total_episodes", "total_frames")}

    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["HF_DATASETS_OFFLINE"] = "1"
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import av

    av.logging.set_level(av.logging.ERROR)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.parent / ("." + output.name + ".converting-" + uuid.uuid4().hex[:10])
    dataset = None
    report.update(status="writing", dependencies={key: importlib.metadata.version(key)
                                                 for key in ("lerobot", "rosbags", "numpy", "av")})
    try:
        dataset = LeRobotDataset.create(repo_id=cfg["dataset"]["repo_id"], root=staging,
            fps=cfg["dataset"]["fps"], robot_type=cfg["dataset"]["robot_type"], features=schema,
            use_videos=True, streaming_encoding=True, rgb_encoder=RGBEncoderConfig(**cfg["video"]),
            encoder_threads=2, video_backend="pyav")
        json_write(staging / "meta/conversion.json", report)
        for index, plan in enumerate(plans):
            LOG.info("[%d/%d] Convert %s (%d frames)", index + 1, len(plans), plan["bag"].parent.name, len(plan["grid"]))
            if bag_identity(plan["bag"]) != plan["identity"]:
                raise ValueError("Bag changed after inspection: " + str(plan["bag"]))
            write_episode(dataset, plan["bag"], cfg, plan["series"], plan["grid"], plan["selected"])
            if bag_identity(plan["bag"]) != plan["identity"]:
                raise ValueError("Bag changed during conversion: " + str(plan["bag"]))
            arrays = {"grid_timestamp_ns": plan["grid"]}
            for key, source in plan["series"].items():
                ids = plan["selected"][key]
                arrays[key + "_source_timestamp_ns"] = source.stamps[ids]
                arrays[key + "_bag_timestamp_ns"] = source.bag_stamps[ids]
                arrays[key + "_message_index"] = source.ordinals[ids]
            trace = staging / "meta/source_timestamps"
            trace.mkdir(exist_ok=True)
            np.savez_compressed(trace / ("episode_%06d.npz" % index), **arrays)
        dataset.finalize()
        report["validation"] = validate_dataset(staging, cfg, plans)
        report["status"] = "complete"
        json_write(staging / "meta/conversion.json", report)
        if output.exists():
            raise ValueError("Output appeared during conversion; refusing to overwrite")
        staging.rename(output)
    except BaseException as exc:
        if dataset is not None:
            try:
                dataset.clear_episode_buffer()
                dataset.finalize()
            except Exception:
                LOG.exception("Failed to close unfinished dataset")
        if staging.exists():
            report.update(status="failed", error=str(exc))
            json_write(staging / "meta/conversion.json", report)
            LOG.error("Incomplete output retained at %s", staging)
        raise
    return {"output": str(output), "episodes": len(plans), "frames": report["total_frames"]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--input", help="Directory searched recursively for complete .bag recordings")
    parser.add_argument("--output", help="New output dataset directory; never overwritten")
    parser.add_argument("--inspect", "--dry-run", action="store_true", help="Validate every source and time alignment without writing a dataset")
    parser.add_argument("--report", help="Save inspection/provenance JSON to a new file")
    parser.add_argument("--limit", type=int, help="Use only the first N bags; conversion also requires --output")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        print(json.dumps(run(args), ensure_ascii=False))
        return 0
    except (ValueError, ImportError, OSError) as exc:
        LOG.error("%s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
