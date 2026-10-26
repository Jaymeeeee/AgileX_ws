"""Convert a complete recorded SDK session into existing LeRobot RLT replay.

Run in policy.python. Frozen VLA boundaries are recomputed from the observations
at actual execution time, not from predictions that may have waited in a queue.
Images and raw actions remain in the original session; this adds normalized NPZ.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import numpy as np

from ..policies.adapter import PolicyRunner
from ..policies.interfaces import PolicyError


def _read_rgb(root, reference):
    path = (root / reference).resolve()
    try:
        path.relative_to(root.resolve())
    except ValueError as exc:
        raise PolicyError("Recorded image path escapes the session directory") from exc
    # EpisodeRecorder writes the portable binary PPM format without extra dependencies.
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P6":
            raise PolicyError("Expected recorder RGB PPM")
        width, height = map(int, stream.readline().split())
        if stream.readline().strip() != b"255" or width <= 0 or height <= 0:
            raise PolicyError("Invalid recorded RGB header")
        data = stream.read()
    return np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3).copy()


def observation_from_record(record, cfg, root):
    state = record.get("state", record.get("observation.state"))
    if state is None:
        raise PolicyError("Trajectory observation has no canonical state")
    result = {"observation.state": np.asarray(state, dtype=np.float32),
              "task": cfg["observation"].get("task", ""),
              "timestamp": record.get("stamp", 0.)}
    frames = record.get("frames", {})
    for key, physical in cfg["observation"].get("camera_map", {}).items():
        if physical not in frames or "rgb_path" not in frames[physical]:
            raise PolicyError("Trajectory is missing recorded camera %s" % physical)
        result[key] = _read_rgb(root, frames[physical]["rgb_path"])
    return result


def export_session(session_dir, output, device=None, deployment=None):
    root, output = Path(session_dir), Path(output)
    if output.exists():
        raise PolicyError("Replay output already exists: %s" % output)
    manifest = json.loads((root / "manifest.json").read_text())
    if manifest.get("status") != "complete":
        raise PolicyError("Only complete recording sessions can be exported")
    cfg = copy.deepcopy(deployment or manifest["config"])
    recorded = manifest["config"]
    if cfg["robot"]["arms"] != recorded["robot"]["arms"]:
        raise PolicyError("Replay deployment must use the recorded robot arms and canonical state order")
    if cfg["policy"].get("backend") != "lerobot" or cfg["policy"]["type"] != "pi05_rlt":
        raise PolicyError("RLT replay export requires a real pi05_rlt checkpoint and its saved processors")
    if device:
        cfg["policy"]["device"] = device
    cfg["workflow"].update(mode="online_rl", learner="pi05_rlt")
    rlt = cfg["workflow"].setdefault("rlt", {})
    rlt.pop("resume", None)
    rlt.pop("prefill_replay", None)
    rlt.pop("output_dir", None)
    rlt["exploration"] = False
    rlt["warmup_transitions"] = 2 ** 62  # Export performs zero learner updates.
    # At most one transition per executed step; preserve every recorded row.
    rlt["replay_capacity"] = max(int(manifest.get("steps", 0)), int(rlt.get("batch_size", 256)), 1)
    runner = PolicyRunner(cfg).load()
    session = runner.rlt
    active_episode = None
    last_observation = None
    last_source = None
    last_generation = None
    steps = 0
    try:
        with (root / manifest.get("trajectory", "trajectory.jsonl")).open() as stream:
            for line in stream:
                row = json.loads(line)
                if row.get("record_type") == "step":
                    episode_id = int(row["episode_id"])
                    if active_episode is not None and episode_id != active_episode:
                        raise PolicyError("Episode changed without a recorded episode_end")
                    if active_episode is None:
                        active_episode = episode_id
                        session.episode_id = episode_id
                    if int(row["step_id"]) != session.step_id:
                        raise PolicyError("Trajectory step gap; cannot invent missing executed actions")
                    before = observation_from_record(row["observation"], cfg, root)
                    after = observation_from_record(row["next_observation"], cfg, root)
                    source = row["source"]
                    generation = row.get("generation")
                    new_prediction = row.get("boundary") is not None
                    # A pause/resume can retain the same HUMAN/BASE/RL source.
                    # Preserve the runtime's explicit cut; generation also covers
                    # older sessions that predate execution_boundary recording.
                    changed_generation = (last_generation is not None and generation is not None
                                          and generation != last_generation)
                    segment_start = bool(row.get("segment_start", changed_generation or source != last_source
                                         or (row.get("execution_boundary", False) and not new_prediction)))
                    if (session.needs_boundary or new_prediction or source != last_source
                            or row.get("execution_boundary", False) or changed_generation):
                        session.boundary(before, segment_start=segment_start)
                    session.record_step(row["action"], source, intervention=row.get("intervention", False),
                                        reward=row.get("reward", 0.), next_observation=after,
                                        ref_row=row.get("metadata", {}).get("rlt_ref_row"))
                    last_observation, last_source, last_generation = after, source, generation
                    steps += 1
                elif row.get("record_type") == "event" and row.get("type") == "episode_end":
                    if active_episode is None:
                        continue
                    if int(row["episode_id"]) != active_episode:
                        raise PolicyError("Episode end id does not match its trajectory")
                    final_observation = (observation_from_record(row["final_observation"], cfg, root)
                                         if row.get("final_observation") is not None else last_observation)
                    session.end_episode(final_observation, terminal=bool(row.get("terminal", False)),
                                        truncated=bool(row.get("truncated", False)),
                                        reward=float(row.get("reward", 0.)), success=bool(row.get("success", False)))
                    active_episode, last_source, last_generation = None, None, None
        if active_episode is not None:
            raise PolicyError("Final episode has no terminal/truncated annotation")
        if not len(session.replay):
            raise PolicyError("Recording contains no executed transitions")
        output.parent.mkdir(parents=True, exist_ok=True)
        session.replay.save(output)
        return {"output": str(output), "executed_steps": steps, "transitions": len(session.replay)}
    finally:
        runner.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", help="Complete EpisodeRecorder session directory")
    parser.add_argument("output", help="New replay.npz path")
    parser.add_argument("--device", default=None)
    parser.add_argument("--config", help="RLT deployment YAML to encode a collect/ACT/other-policy session")
    args = parser.parse_args()
    deployment = None
    if args.config:
        from ..config import load_config
        deployment = load_config(args.config)
    print(json.dumps(export_session(args.session, args.output, args.device, deployment), indent=2))


if __name__ == "__main__":
    main()
