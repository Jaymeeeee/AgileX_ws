"""Derive a current-only LeRobot v3 dataset without re-encoding its videos.

The supported source contract is the right-arm 19D position/current/torque
dataset. Retain components 0..12, preserving all episodes and other features.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import uuid

import numpy as np


STATE = "observation.state"
INDICES = tuple(range(13))
BASE_NAMES = ["right_joint_%d" % i for i in range(1, 7)] + ["right_gripper_width"]
SOURCE_NAMES = BASE_NAMES + ["right_joint_%d_%s" % (i, kind)
                           for kind in ("current", "torque") for i in range(1, 7)]
ROOT = Path(__file__).resolve().parents[2]


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                    encoding="utf-8")


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def select_info(info):
    result = deepcopy(info)
    feature = result["features"][STATE]
    if (result.get("codebase_version") != "v3.0" or feature.get("dtype") != "float32"
            or feature.get("shape") != [19] or feature.get("names") != SOURCE_NAMES):
        raise ValueError("Source must be LeRobot v3 with canonical 19D right position/current/torque state")
    action = result["features"]["action"]
    if action.get("shape") != [7] or action.get("names") != BASE_NAMES:
        raise ValueError("Source action must retain the canonical seven-component joint target")
    feature["shape"] = [13]
    feature["names"] = [SOURCE_NAMES[i] for i in INDICES]
    return result


def select_stat(values):
    if not isinstance(values, list) or len(values) != 19:
        raise ValueError("Each non-count state statistic must contain 19 components")
    if not np.isfinite(np.asarray(values, dtype=np.float64)).all():
        raise ValueError("State statistics must be finite")
    return [values[i] for i in INDICES]


def select_stats(stats):
    result = deepcopy(stats)
    for name, values in result[STATE].items():
        if name != "count":
            result[STATE][name] = select_stat(values)
    return result


def _hf_metadata(metadata, lengths):
    """Update embedded datasets Features, avoiding a stale fixed list length."""
    if not metadata or b"huggingface" not in metadata:
        return metadata
    result = dict(metadata)
    hf = json.loads(result[b"huggingface"])
    features = hf["info"]["features"]
    for name, length in lengths.items():
        if name not in features:
            raise ValueError("Hugging Face schema is missing " + name)
        if features[name].get("length", -1) != -1:
            features[name]["length"] = length
    hf["fingerprint"] = hashlib.sha256(
        result[b"huggingface"] + json.dumps(lengths, sort_keys=True).encode()).hexdigest()[:16]
    result[b"huggingface"] = json.dumps(hf).encode()
    return result


def select_table(table, episode_metadata=False):
    import pyarrow as pa

    if episode_metadata:
        columns = [name for name in table.column_names
                   if name.startswith("stats/" + STATE + "/") and not name.endswith("/count")]
        if not columns:
            raise ValueError("Episode metadata lacks state statistics")
    else:
        columns = [STATE]
    for name in columns:
        old_field = table.schema.field(name)
        rows = table[name].to_pylist()
        values = [select_stat(row) for row in rows]
        if episode_metadata:
            old_type = old_field.type
            if pa.types.is_fixed_size_list(old_type):
                value_type = pa.list_(old_type.value_type, len(INDICES))
            elif pa.types.is_list(old_type):
                value_type = old_type
            else:
                raise ValueError("Unsupported episode statistic Arrow type: " + str(old_type))
        else:
            if not pa.types.is_fixed_size_list(old_field.type) or old_field.type.value_type != pa.float32():
                raise ValueError("Expected fixed-size float32 state vectors in source Parquet")
            value_type = pa.list_(pa.float32(), len(INDICES))
        field = pa.field(name, value_type, nullable=old_field.nullable, metadata=old_field.metadata)
        table = table.set_column(table.schema.get_field_index(name), field, pa.array(values, type=value_type))
    return table.replace_schema_metadata(_hf_metadata(table.schema.metadata, {name: 13 for name in columns}))


def _validate(source, target, repo_id, info, stats):
    import pyarrow.parquet as pq
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    files = sorted((source / "data").rglob("*.parquet"))
    original = pq.read_table(files)
    derived = pq.read_table([target / path.relative_to(source) for path in files])
    expected_state = np.asarray(original[STATE].to_pylist(), dtype=np.float32)[:, INDICES]
    np.testing.assert_array_equal(np.asarray(derived[STATE].to_pylist(), dtype=np.float32), expected_state)
    for name in original.column_names:
        if name != STATE and not original[name].equals(derived[name]):
            raise ValueError("A non-state data column changed: " + name)
    for path in sorted((source / "meta/episodes").rglob("*.parquet")):
        before, after = pq.read_table(path), pq.read_table(target / path.relative_to(source))
        for name in before.column_names:
            if name.startswith("stats/" + STATE + "/") and not name.endswith("/count"):
                expected = [[row[i] for i in INDICES] for row in before[name].to_pylist()]
                if after[name].to_pylist() != expected:
                    raise ValueError("Episode state statistic mismatch: " + name)
            elif not before[name].equals(after[name]):
                raise ValueError("Other episode metadata changed: " + name)
    if read_json(target / "meta/info.json") != info or read_json(target / "meta/stats.json") != stats:
        raise ValueError("Derived feature/global-statistics metadata mismatch")

    dataset = LeRobotDataset(repo_id=repo_id, root=target, video_backend="pyav")
    if len(dataset) != info["total_frames"] or dataset.num_episodes != info["total_episodes"]:
        raise ValueError("Official reader frame/episode count mismatch")
    if tuple(dataset.features[STATE]["shape"]) != (13,) or dataset.features[STATE]["names"] != SOURCE_NAMES[:13]:
        raise ValueError("Official reader state schema mismatch")
    rows = dataset.hf_dataset.with_format("numpy")
    np.testing.assert_array_equal(np.asarray(rows[STATE]), expected_state)
    np.testing.assert_array_equal(np.asarray(rows["action"]), np.asarray(original["action"].to_pylist(), np.float32))
    for feature, feature_stats in stats.items():
        for name, value in feature_stats.items():
            np.testing.assert_array_equal(dataset.meta.stats[feature][name], np.asarray(value))
    episode_ids = np.asarray(original["episode_index"])
    boundaries = np.flatnonzero(np.r_[True, episode_ids[1:] != episode_ids[:-1], True])
    checked = 0
    for start, stop in zip(boundaries[:-1], boundaries[1:]):
        for index in sorted({int(start), int(stop - 1)}):
            frame = dataset[index]
            np.testing.assert_array_equal(frame[STATE].numpy(), expected_state[index])
            for key, feature in info["features"].items():
                if feature["dtype"] == "video":
                    height, width, channels = feature["shape"]
                    if tuple(frame[key].shape) != (channels, height, width) or not frame[key].isfinite().all():
                        raise ValueError("Official reader could not decode video frame: " + key)
            checked += 1
    return {"frames": len(dataset), "episodes": dataset.num_episodes, "state_dimension": 13,
            "action_dimension": 7, "all_state_values_match_source_slice": True,
            "all_other_data_columns_unchanged": True, "all_episode_metadata_verified": True,
            "all_global_stats_verified": True, "official_reader_boundary_frames_checked": checked}


def derive(source, output, repo_id="local/piper_insert_plug_v4_with_current"):
    import pyarrow.parquet as pq

    source, output = Path(source).expanduser().resolve(), Path(output).expanduser().resolve()
    if output.exists():
        raise ValueError("Output already exists; refusing to overwrite: " + str(output))
    if source == output or source in output.parents or output in source.parents:
        raise ValueError("Source and output must be separate dataset directories")
    if len(repo_id.split("/")) != 2 or not all(repo_id.split("/")):
        raise ValueError("repo_id must be namespace/dataset")
    info = select_info(read_json(source / "meta/info.json"))
    stats = select_stats(read_json(source / "meta/stats.json"))
    source_files = sorted(path for path in source.rglob("*") if path.is_file())
    if any(path.is_symlink() for path in source.rglob("*")):
        raise ValueError("Source must contain ordinary files, not symbolic links")
    if not list((source / "data").rglob("*.parquet")) or not list((source / "meta/episodes").rglob("*.parquet")):
        raise ValueError("Source dataset lacks data or episode Parquet files")
    identities = {str(path.relative_to(source)): (path.stat().st_size, path.stat().st_mtime_ns)
                  for path in source_files}
    report = {"status": "writing", "created_at": datetime.now(timezone.utc).isoformat(),
              "source": str(source), "output": str(output), "repo_id": repo_id,
              "operation": "Select observation.state components 0..12; preserve all rows and other features",
              "selected_indices": list(INDICES), "source_state_names": SOURCE_NAMES,
              "state_names": SOURCE_NAMES[:13], "state_units": ["rad"] * 6 + ["m"] + ["A"] * 6,
              "video_operation": "Independent byte-for-byte copies; no decoding/re-encoding during derivation",
              "script_sha256": file_hash(Path(__file__)), "source_files": {}, "copied_files": {}}
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.parent / ("." + output.name + ".selecting-" + uuid.uuid4().hex[:10])
    staging.mkdir()
    try:
        for path in source_files:
            relative = path.relative_to(source)
            name = str(relative)
            report["source_files"][name] = {"sha256": file_hash(path), "size_bytes": path.stat().st_size}
            target = staging / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            if name == "meta/info.json":
                write_json(target, info)
            elif name == "meta/stats.json":
                write_json(target, stats)
            elif relative.parts[0] == "data" and path.suffix == ".parquet":
                pq.write_table(select_table(pq.read_table(path)), target, compression="snappy")
            elif relative.parts[:2] == ("meta", "episodes") and path.suffix == ".parquet":
                pq.write_table(select_table(pq.read_table(path), episode_metadata=True), target, compression="snappy")
            else:
                if name == "meta/conversion.json":
                    target = staging / "meta/source_conversion.json"
                    if (source / "meta/source_conversion.json").exists():
                        raise ValueError("Source already contains source_conversion.json; refusing ambiguous provenance")
                shutil.copy2(path, target)
                if file_hash(target) != report["source_files"][name]["sha256"]:
                    raise ValueError("Copied file content mismatch: " + name)
                report["copied_files"][str(target.relative_to(staging))] = report["source_files"][name]
        os.environ["HF_HUB_OFFLINE"] = "1"
        os.environ["HF_DATASETS_OFFLINE"] = "1"
        report["validation"] = _validate(source, staging, repo_id, info, stats)
        for path in source_files:
            if (path.stat().st_size, path.stat().st_mtime_ns) != identities[str(path.relative_to(source))]:
                raise ValueError("Source changed during derivation: " + str(path))
        report["status"] = "complete"
        write_json(staging / "meta/conversion.json", report)
        if output.exists():
            raise ValueError("Output appeared during derivation; refusing to overwrite")
        staging.rename(output)
    except BaseException as exc:
        report.update(status="failed", error=str(exc))
        if staging.exists():
            write_json(staging / "meta/conversion.json", report)
        raise
    return {"output": str(output), "repo_id": repo_id, **report["validation"]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=str(ROOT / "datasets/piper_insert_plug_v4_with_motor_state"))
    parser.add_argument("--output", default=str(ROOT / "datasets/piper_insert_plug_v4_with_current"))
    parser.add_argument("--repo-id", default="local/piper_insert_plug_v4_with_current")
    args = parser.parse_args(argv)
    print(json.dumps(derive(args.source, args.output, args.repo_id), ensure_ascii=False, indent=2))
    return 0
