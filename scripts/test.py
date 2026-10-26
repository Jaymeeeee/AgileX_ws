#!/usr/bin/env python3
"""Run hardware-isolated checks; optionally test the prepared model environment."""
from _bootstrap import ROOT
import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-python", help="Optional prepared LeRobot interpreter")
    parser.add_argument("--lerobot-source", help="Optional existing LeRobot checkout")
    args = parser.parse_args()
    env = dict(os.environ)
    env.update(HF_HUB_OFFLINE="1", HF_DATASETS_OFFLINE="1", TRANSFORMERS_OFFLINE="1", PYTHONNOUSERSITE="1")
    with tempfile.TemporaryDirectory(prefix="piper-test-cache-") as cache:
        env["HF_HOME"] = cache
        env["HF_DATASETS_CACHE"] = str(Path(cache) / "datasets")
        result = subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-v"],
                                cwd=ROOT, env=env)
        if result.returncode or not args.model_python:
            return result.returncode
        env.pop("PYTHONPATH", None)
        if args.lerobot_source:
            source = Path(args.lerobot_source).expanduser().resolve()
            env["PYTHONPATH"] = str(source / "src" if (source / "src").is_dir() else source)
        return subprocess.run([str(Path(args.model_python).expanduser()), "-m", "unittest",
                               "tests.test_policy", "tests.test_rlt", "tests.test_dataset_export",
                               "tests.test_training", "-v"], cwd=ROOT, env=env).returncode


if __name__ == "__main__":
    raise SystemExit(main())
