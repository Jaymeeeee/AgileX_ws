"""Continue RLT training from a consistent paused-episode snapshot, without hardware.

Live online training is driven by rollout end_episode and uses the same learner.
This CLI consumes an existing learner/replay snapshot for additional offline updates.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--resume", required=True, help="Directory with learner.pt, replay.npz, manifest.json")
    parser.add_argument("--output", required=True, help="New snapshot directory")
    parser.add_argument("--updates", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--lerobot-source")
    args = parser.parse_args()
    if args.updates < 1 or args.batch_size < 1:
        parser.error("updates and batch-size must be positive")
    output, source = Path(args.output), Path(args.resume)
    if output.exists():
        parser.error("output directory already exists")
    if args.lerobot_source:
        path = Path(args.lerobot_source).expanduser()
        sys.path.insert(0, str(path / "src" if (path / "src").is_dir() else path))
    from lerobot.policies.pi05_rlt.learning import RLTLearner
    from lerobot.policies.pi05_rlt.replay import ReplayBuffer
    manifest = json.loads((source / "manifest.json").read_text())
    if manifest.get("schema_version") != 1 or manifest.get("policy_type") != "pi05_rlt":
        parser.error("unsupported RLT snapshot manifest")
    learner = RLTLearner.load(source / "learner.pt", device=args.device)
    replay = ReplayBuffer.load(source / "replay.npz")
    if len(replay) < args.batch_size:
        parser.error("replay has fewer transitions than batch-size")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".rlt-offline-", dir=str(output.parent)))
    with (temporary / "metrics.jsonl").open("w") as stream:
        for _ in range(args.updates):
            metrics = learner.update(replay.sample(args.batch_size))
            stream.write(json.dumps(metrics) + "\n")
    learner.save(temporary / "learner.pt", episode_id=manifest["episode_id"])
    replay.save(temporary / "replay.npz")
    manifest.update(updates=learner.updates, new_transitions=0, offline_source=str(source.resolve()))
    (temporary / "manifest.json").write_text(json.dumps(manifest, indent=2))
    os.rename(str(temporary), str(output))
    print(json.dumps({"checkpoint": str(output), "updates": learner.updates, "replay_size": len(replay)}))


if __name__ == "__main__":
    main()
