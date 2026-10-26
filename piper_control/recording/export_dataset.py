"""Export SDK session artifacts to a local LeRobot dataset without ROS.

Every accepted executed step becomes exactly one frame; predictions are auxiliary
metadata, never action labels. Preflight rejects gaps, incomplete transitions,
missing images, mismatched arms/cameras and unapproved failed/truncated sessions.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import uuid

import numpy as np


SOURCE_IDS = {"BASE": 0, "RL": 1, "HUMAN": 2, "MIXED": 3}


class ExportError(ValueError):
    pass


def _names(arms):
    if arms not in (["left"], ["right"], ["left", "right"]):
        raise ExportError("Recorded arms must be left, right, or canonical left/right")
    return [name for side in arms for name in
            [side + "_joint_" + str(index) for index in range(1, 7)] + [side + "_gripper_width"]]


def _inside(root, reference):
    path = (root / reference).resolve()
    try:
        path.relative_to(root.resolve())
    except ValueError as exc:
        raise ExportError("Image reference escapes its recording session") from exc
    return path


def _rgb(root, reference, header_only=False):
    path = _inside(root, reference)
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P6":
            raise ExportError("Expected an archival RGB PPM: " + str(path))
        try:
            width, height = map(int, stream.readline().split())
        except ValueError as exc:
            raise ExportError("Malformed PPM dimensions") from exc
        if min(width, height) < 1 or stream.readline().strip() != b"255":
            raise ExportError("Invalid RGB PPM header")
        if path.stat().st_size - stream.tell() != width * height * 3:
            raise ExportError("Truncated or malformed RGB image: " + str(path))
        if header_only:
            return (height, width, 3)
        return np.frombuffer(stream.read(), dtype=np.uint8).reshape(height, width, 3).copy()


def _vector(value, dimension, field):
    array = np.asarray(value, dtype=np.float32)
    if array.shape != (dimension,) or not np.isfinite(array).all():
        raise ExportError("{} must contain {} finite components".format(field, dimension))
    return array


def _stamp(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ExportError("Invalid source timestamp: " + field)
    return float(value)


def _session(root, allow_failed, include_truncated):
    root = Path(root).expanduser().resolve()
    manifest = json.loads((root / "manifest.json").read_text())
    status = manifest.get("status")
    if status != "complete" and not (allow_failed and status == "failed"):
        raise ExportError("Session is not complete; --allow-failed explicitly permits failed sessions with complete episodes")
    cfg = manifest["config"]
    names = _names(cfg["robot"]["arms"])
    mapping = cfg["observation"].get("camera_map", {})
    if not isinstance(mapping, dict) or any(not key.startswith("observation.images.") for key in mapping):
        raise ExportError("Invalid recorded camera feature contract")
    fps = cfg["workflow"]["policy_hz"]
    if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not math.isfinite(fps) or fps < 1 or int(fps) != fps:
        raise ExportError("LeRobot export requires an integer recorded policy_hz; no implicit resampling is performed")
    episodes, pending = [], []
    active = None
    seen = set()
    shapes = {}
    stamps = []
    path = _inside(root, manifest.get("trajectory", "trajectory.jsonl"))
    with path.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                raise ExportError("Empty/truncated trajectory row at line {}".format(number))
            row = json.loads(line)
            if row.get("record_type") == "step":
                episode = row.get("episode_id")
                if type(episode) is not int or type(row.get("step_id")) is not int:
                    raise ExportError("Episode and step identifiers must be integers")
                if active is None:
                    if episode in seen:
                        raise ExportError("An episode id was reused")
                    active = episode
                if episode != active or row["step_id"] != len(pending):
                    raise ExportError("Trajectory has a gap or changed episode before episode_end")
                if row.get("execution_complete", True) is False or not isinstance(row.get("next_observation"), dict):
                    raise ExportError("Incomplete executed transition cannot be exported, even with --allow-failed")
                _vector(row.get("action"), len(names), "action")
                _vector(row["observation"].get("state"), len(names), "observation.state")
                _vector(row["next_observation"].get("state"), len(names), "next_observation.state")
                if row.get("predicted_action") is not None:
                    _vector(row["predicted_action"], len(names), "predicted_action")
                if row.get("source") not in SOURCE_IDS:
                    raise ExportError("Unknown action source")
                timestamp = _stamp(row.get("timestamp"), "action")
                if stamps and timestamp < stamps[-1]:
                    raise ExportError("Action timestamps moved backwards inside an episode")
                stamps.append(timestamp)
                _stamp(row["observation"].get("stamp"), "observation")
                _stamp(row["next_observation"].get("stamp"), "next_observation")
                for feature, physical in mapping.items():
                    frame = row["observation"].get("frames", {}).get(physical, {})
                    if "rgb_path" not in frame:
                        raise ExportError("Missing recorded camera: " + physical)
                    _stamp(frame.get("stamp"), feature)
                    shape = _rgb(root, frame["rgb_path"], header_only=True)
                    if shapes.setdefault(feature, shape) != shape:
                        raise ExportError("Camera shape changes within a recording")
                pending.append(row)
            elif row.get("record_type") == "event" and row.get("type") == "episode_end":
                episode = row.get("episode_id")
                terminal, truncated = row.get("terminal", False), row.get("truncated", False)
                if bool(terminal) == bool(truncated):
                    raise ExportError("Episode end must be exactly terminal or truncated")
                if truncated and not include_truncated:
                    raise ExportError("Truncated episode requires --include-truncated; no episode was silently skipped")
                if pending:
                    if episode != active or row.get("steps", len(pending)) != len(pending):
                        raise ExportError("Episode end does not match its executed steps")
                    if "final_observation" in row:
                        _vector(row["final_observation"].get("state"), len(names), "episode_end.final_observation.state")
                        _stamp(row["final_observation"].get("stamp"), "episode_end.final_observation")
                    episodes.append({"episode_id": episode, "steps": pending, "end": row})
                    seen.add(episode)
                elif row.get("steps", 0) != 0:
                    raise ExportError("Episode end refers to missing executed steps")
                active, pending, stamps = None, [], []
            elif row.get("record_type") != "event":
                raise ExportError("Unknown trajectory record type")
    if pending:
        raise ExportError("Final episode has no explicit end; incomplete episodes cannot be silently dropped")
    count = sum(len(episode["steps"]) for episode in episodes)
    if count != manifest.get("steps", count):
        raise ExportError("Manifest step count disagrees with trajectory")
    if not count:
        raise ExportError("Session contains no executed steps")
    return {"root": root, "manifest": manifest, "config": cfg, "names": names,
            "mapping": mapping, "shapes": shapes, "fps": int(fps), "episodes": episodes,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def _features(session, video):
    names = session["names"]
    features = {key: {"dtype": "float32", "shape": (len(names),), "names": names}
                for key in ("observation.state", "action", "next.observation.state", "metadata.predicted_action")}
    scalars = {
        "metadata.action_source": "int64", "metadata.original_episode_id": "int64",
        "metadata.original_step_id": "int64", "metadata.session_index": "int64", "metadata.generation": "int64",
        "metadata.intervention": "bool", "metadata.prediction_available": "bool",
        "metadata.execution_boundary": "bool", "metadata.segment_start": "bool",
        "metadata.action_timestamp": "float64", "metadata.observation_timestamp": "float64",
        "metadata.next_observation_timestamp": "float64", "metadata.duration_sec": "float32",
        "next.reward": "float32", "next.done": "bool", "next.truncated": "bool", "next.success": "bool",
    }
    for key, dtype in scalars.items():
        features[key] = {"dtype": dtype, "shape": (1,), "names": [key.rsplit(".", 1)[-1]]}
    for feature, shape in session["shapes"].items():
        features[feature] = {"dtype": "video" if video else "image", "shape": shape,
                             "names": ["height", "width", "channels"]}
        key = "metadata.camera_timestamp." + session["mapping"][feature]
        features[key] = {"dtype": "float64", "shape": (1,), "names": ["acquisition_seconds"]}
    return features


def _frame(session, episode, row, index, session_index, features):
    obs, after = row["observation"], row["next_observation"]
    final = index == len(episode["steps"]) - 1
    end = episode["end"]
    if final and "final_observation" in end:
        # An operator can pause, let the arm settle, then end the episode. Its
        # real final observation is newer than the last step's immediate after.
        after = end["final_observation"]
    reward = float(row.get("reward", 0.))
    if final and not end.get("reward_in_final_step", False):
        reward += float(end.get("reward", 0.))
    values = {
        "observation.state": obs["state"], "action": row["action"],
        "next.observation.state": after["state"],
        "metadata.predicted_action": row.get("predicted_action") if row.get("predicted_action") is not None else [0.] * len(session["names"]),
        "metadata.action_source": SOURCE_IDS[row["source"]], "metadata.original_episode_id": row["episode_id"],
        "metadata.original_step_id": row["step_id"], "metadata.session_index": session_index,
        "metadata.generation": row.get("generation", 0), "metadata.intervention": bool(row.get("intervention", row["source"] == "HUMAN")),
        "metadata.prediction_available": row.get("predicted_action") is not None,
        "metadata.execution_boundary": bool(row.get("execution_boundary", False)),
        "metadata.segment_start": bool(row.get("segment_start", False)),
        "metadata.action_timestamp": row["timestamp"], "metadata.observation_timestamp": obs["stamp"],
        "metadata.next_observation_timestamp": after["stamp"], "metadata.duration_sec": row.get("duration_sec", 1. / session["fps"]),
        "next.reward": reward, "next.done": bool(final and end.get("terminal", False)),
        "next.truncated": bool(final and end.get("truncated", False)), "next.success": bool(end.get("success", False)),
    }
    frame = {key: np.asarray(value, dtype=features[key]["dtype"]).reshape(features[key]["shape"])
             for key, value in values.items()}
    for key, value in frame.items():
        if value.dtype.kind == "f" and not np.isfinite(value).all():
            raise ExportError("Nonfinite exported metadata: " + key)
    frame["task"] = session["config"]["observation"].get("task", "")
    for feature, physical in session["mapping"].items():
        image = obs["frames"][physical]
        frame[feature] = _rgb(session["root"], image["rgb_path"])
        frame["metadata.camera_timestamp." + physical] = np.array([image["stamp"]], dtype=np.float64)
    return frame


def export_dataset(session_dirs, output, repo_id, include_truncated=False, allow_failed=False,
                   images_as_video=False, dataset_factory=None):
    output = Path(output).expanduser().resolve()
    if output.exists():
        raise ExportError("Refusing to overwrite an existing dataset")
    sessions = [_session(root, allow_failed, include_truncated) for root in session_dirs]
    if not sessions:
        raise ExportError("At least one session is required")
    features = _features(sessions[0], images_as_video)
    for session in sessions[1:]:
        if (session["names"] != sessions[0]["names"] or session["mapping"] != sessions[0]["mapping"]
                or session["fps"] != sessions[0]["fps"] or _features(session, images_as_video) != features):
            raise ExportError("Sessions have different arm, camera, FPS, or feature contracts")
    if dataset_factory is None:
        os.environ["HF_HUB_OFFLINE"] = "1"
        os.environ["HF_DATASETS_OFFLINE"] = "1"
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        dataset_factory = LeRobotDataset
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.parent / ("." + output.name + ".partial-" + uuid.uuid4().hex[:10])
    dataset = None
    report = {"schema_version": 1, "status": "writing", "repo_id": repo_id,
              "arms": sessions[0]["config"]["robot"]["arms"], "component_names": sessions[0]["names"],
              "action_source_ids": SOURCE_IDS, "fps": sessions[0]["fps"],
              "timing": "No steps resampled; LeRobot index/fps plus original sensor/action timestamps",
              "source_sessions": [], "frames": 0, "episodes": 0,
              "include_truncated": include_truncated, "allow_failed": allow_failed}
    try:
        dataset = dataset_factory.create(repo_id=repo_id, root=staging, fps=sessions[0]["fps"],
                                         robot_type="piper_" + "_".join(report["arms"]),
                                         features=features, use_videos=images_as_video,
                                         image_writer_threads=2)
        (staging / "meta").mkdir(parents=True, exist_ok=True)
        with (staging / "meta" / "episode_annotations.jsonl").open("w", encoding="utf-8") as annotations:
            for session_index, session in enumerate(sessions):
                report["source_sessions"].append({"path": str(session["root"]), "trajectory_sha256": session["sha256"],
                                                   "status": session["manifest"]["status"]})
                for episode in session["episodes"]:
                    for index, row in enumerate(episode["steps"]):
                        dataset.add_frame(_frame(session, episode, row, index, session_index, features))
                        report["frames"] += 1
                    dataset.save_episode(parallel_encoding=False)
                    annotations.write(json.dumps({"dataset_episode_index": report["episodes"],
                        "session_index": session_index, "source_episode_id": episode["episode_id"],
                        "end": episode["end"]}, ensure_ascii=False, allow_nan=False) + "\n")
                    report["episodes"] += 1
        dataset.finalize()
        report["status"] = "complete"
        (staging / "meta" / "conversion.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        if output.exists():
            raise ExportError("Output dataset appeared during conversion; refusing to overwrite it")
        staging.rename(output)
    except BaseException as exc:
        if dataset is not None:
            try:
                dataset.finalize()
            except Exception:
                pass
        if staging.exists():
            report.update(status="failed", error=str(exc))
            (staging / "conversion_failed.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        raise
    return {"output": str(output), "frames": report["frames"], "episodes": report["episodes"]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", nargs="+")
    parser.add_argument("--output", required=True)
    parser.add_argument("--repo-id", required=True, help="Local dataset identity; nothing is uploaded")
    parser.add_argument("--include-truncated", action="store_true")
    parser.add_argument("--allow-failed", action="store_true")
    parser.add_argument("--images-as-video", action="store_true")
    parser.add_argument("--lerobot-source")
    args = parser.parse_args(argv)
    if args.lerobot_source:
        source = Path(args.lerobot_source).expanduser().resolve()
        sys.path.insert(0, str(source / "src" if (source / "src").is_dir() else source))
    result = export_dataset(args.sessions, args.output, args.repo_id,
                            include_truncated=args.include_truncated, allow_failed=args.allow_failed,
                            images_as_video=args.images_as_video)
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
