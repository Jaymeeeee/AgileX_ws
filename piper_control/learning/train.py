"""Launch local LeRobot training in the selected model Python environment.

The input is an upstream TrainPipelineConfig YAML or JSON document with explicit
policy.type, dataset.repo_id, dataset.root and output_dir. Relative paths resolve
beside that document. The original config path is preserved for optimizer resume.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import sys


def _read_config(path):
    with path.open(encoding="utf-8") as stream:
        if path.suffix.lower() == ".json":
            config = json.load(stream)
        else:
            import yaml
            config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Training configuration must be a YAML/JSON mapping")
    return config


def _override_config(config, arguments):
    """Inspect effective values before forwarding upstream's native CLI options."""
    import yaml

    result = copy.deepcopy(config)
    index = 0
    while index < len(arguments):
        argument = arguments[index]
        if not argument.startswith("--"):
            raise ValueError("LeRobot overrides must use --key=value or --key value")
        key, separator, value = argument[2:].partition("=")
        if key == "config_path":
            raise ValueError("Use --config to select the single training configuration")
        if not separator:
            index += 1
            if index == len(arguments) or arguments[index].startswith("--"):
                raise ValueError("Missing value for --%s" % key)
            value = arguments[index]
        if not key or any(not part for part in key.split(".")):
            raise ValueError("Invalid training override")
        target = result
        parts = key.split(".")
        for part in parts[:-1]:
            if not isinstance(target.get(part), dict):
                target[part] = {}
            target = target[part]
        target[parts[-1]] = yaml.safe_load(value)
        index += 1
    return result


def _local_path(value, parent, field):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("%s must identify a local path" % field)
    path = Path(os.path.expandvars(value)).expanduser()
    return (path if path.is_absolute() else parent / path).resolve()


def _without_options(arguments, keys):
    """Remove selected validated options before forwarding one canonical value."""
    result = []
    index = 0
    while index < len(arguments):
        option = arguments[index]
        key, separator, _ = option[2:].partition("=")
        length = 1 if separator else 2
        if key not in keys:
            result.extend(arguments[index:index + length])
        index += length
    return result


def launch_training(config_path, *, python=None, lerobot_source=None, extra_args=(),
                    resume=False, dry_run=False):
    """Validate and launch upstream training; dry_run performs no subprocess work.

    Checkpoints, the local dataset and model-specific pretrained assets must
    already be prepared. Hub access and external tracking are disabled here.
    """
    path = Path(config_path).expanduser().resolve()
    if not path.is_file():
        raise ValueError("Training configuration does not exist: %s" % path)
    arguments = list(extra_args)
    if arguments and arguments[0] == "--":
        arguments.pop(0)
    config = _override_config(_read_config(path), arguments)
    if resume:
        config["resume"] = True
        arguments.append("--resume=true")
    is_resume = config.get("resume", False)
    if type(is_resume) is not bool:
        raise ValueError("resume must be a boolean")
    policy = config.get("policy", {})
    if not isinstance(policy, dict) or not isinstance(policy.get("type"), str) or policy["type"] in ("", "auto"):
        raise ValueError("Training requires explicit policy.type from the LeRobot registry")
    dataset = config.get("dataset", {})
    if not isinstance(dataset, dict) or not isinstance(dataset.get("repo_id"), str) or not dataset["repo_id"]:
        raise ValueError("Training requires dataset.repo_id for the prepared local dataset")
    dataset_root = _local_path(dataset.get("root"), path.parent, "dataset.root")
    if not (dataset_root / "meta" / "info.json").is_file():
        raise ValueError("dataset.root must contain a prepared LeRobot dataset (meta/info.json)")
    output = _local_path(config.get("output_dir"), path.parent, "output_dir")
    if output.exists() and not is_resume:
        raise ValueError("Training output already exists; use a new output_dir or resume a saved training config")
    if config.get("job", {}).get("target", "local") != "local":
        raise ValueError("This entry point runs local training; remote jobs require the upstream launcher")
    # Validate local pretrained sources and their type without importing torch.
    pretrained_paths = {}
    for field in ("path", "pretrained_path"):
        if policy.get(field):
            checkpoint = _local_path(policy[field], path.parent, "policy." + field)
            saved_config = checkpoint / "config.json"
            if not saved_config.is_file():
                raise ValueError("policy.%s must contain a prepared local checkpoint" % field)
            saved_type = json.loads(saved_config.read_text())["type"]
            if saved_type != policy["type"]:
                raise ValueError("policy.type disagrees with the pretrained checkpoint")
            pretrained_paths["policy." + field] = str(checkpoint)
    executable = str(Path(python).expanduser()) if python else sys.executable
    source = None
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    if lerobot_source:
        source = Path(lerobot_source).expanduser().resolve()
        source_path = source / "src" if (source / "src").is_dir() else source
        if not (source_path / "lerobot").is_dir():
            raise ValueError("lerobot_source must contain the LeRobot Python package")
        env["PYTHONPATH"] = str(source_path)
    env.update(PYTHONNOUSERSITE="1", HF_HUB_OFFLINE="1", HF_DATASETS_OFFLINE="1",
               TRANSFORMERS_OFFLINE="1", WANDB_MODE="offline")
    # Keep the exact checkpoint config location: LeRobot uses its parent to find
    # optimizer/scheduler state when --resume=true.
    command = [executable, "-m", "lerobot.scripts.lerobot_train", "--config_path=" + str(path)]
    replaced = set(pretrained_paths)
    if "policy.path" in pretrained_paths:
        # LeRobot rejects simultaneous CLI --policy.path and --policy.type.
        # The explicit effective type was checked against this checkpoint above;
        # loading that path now supplies the same registered type to LeRobot.
        replaced.add("policy.type")
    command.extend(_without_options(arguments, replaced))
    command.extend("--%s=%s" % item for item in pretrained_paths.items())
    command.extend(["--dataset.root=" + str(dataset_root), "--output_dir=" + str(output),
                    "--job.target=local", "--policy.push_to_hub=false",
                    "--save_checkpoint_to_hub=false", "--wandb.enable=false"])
    result = {"command": command, "cwd": str(path.parent), "policy_type": policy["type"],
              "dataset_root": str(dataset_root), "output_dir": str(output),
              "resume": is_resume, "dry_run": bool(dry_run)}
    if not dry_run:
        completed = subprocess.run(command, cwd=str(path.parent), env=env, check=True)
        result["returncode"] = completed.returncode
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="Prepared LeRobot training YAML/JSON")
    parser.add_argument("--python", help="Model environment Python; defaults to this interpreter")
    parser.add_argument("--lerobot-source", help="Existing LeRobot repository or src directory")
    parser.add_argument("--resume", action="store_true", help="Resume using the original checkpoint train_config.json")
    parser.add_argument("--dry-run", action="store_true", help="Validate local inputs and print the command without training")
    args, extra = parser.parse_known_args(argv)
    try:
        result = launch_training(args.config, python=args.python, lerobot_source=args.lerobot_source,
                                 extra_args=extra, resume=args.resume, dry_run=args.dry_run)
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
