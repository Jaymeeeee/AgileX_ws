"""Manually record one directory of ROS messages without controlling hardware."""

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

from piper_control.config import load_config
from piper_control.recording.topics import missing_required_topics, print_topic_plan, topic_plan


def topic_pattern(config):
    """Return the explicit configured topic selection, or the legacy pattern."""
    return topic_plan(config)["topic_regex"]


def rosbag_command(config, arguments):
    """Source ROS and run its rosbag script using the configured ROS Python."""
    options = config["record"]
    setup = options.get("ros_setup") or ""
    if setup and not Path(setup).is_file():
        raise ValueError("ROS setup script does not exist: " + str(setup))
    # All user/configuration values are positional arguments, never shell code.
    script = ('if [ -n "$1" ]; then source "$1" || exit $?; fi; '
              'record_python="$2"; shift 2; '
              'record_rosbag="$(command -v rosbag)" || exit $?; '
              'exec "$record_python" "$record_rosbag" "$@"')
    return ["bash", "--noprofile", "--norc", "-c", script, "piper-record",
            str(setup), str(options.get("python") or "/usr/bin/python3")] + list(arguments)


def ros_environment(config):
    env = dict(os.environ)
    env["PYTHONNOUSERSITE"] = "1"
    if config["record"].get("ros_setup"):
        for key in ("PYTHONPATH", "ROS_PACKAGE_PATH", "CMAKE_PREFIX_PATH"):
            env.pop(key, None)
    return env


@contextmanager
def _finishing():
    # A second Ctrl+C must not abandon the writer while it closes its bag index.
    previous = {sig: signal.signal(sig, signal.SIG_IGN)
                for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def _signal_group(process, sig):
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def _wait_writer(process, active, timeout):
    deadline = time.monotonic() + timeout
    process.wait(timeout=timeout)
    # rosbag's Python wrapper can exit before its C++ child finishes indexing.
    while active.exists():
        if time.monotonic() >= deadline:
            raise TimeoutError("rosbag did not finish closing data.bag.active")
        time.sleep(0.05)


def stop_writer(process, directory, timeout):
    active = directory / "data.bag.active"
    with _finishing():
        _signal_group(process, signal.SIGINT)
        try:
            _wait_writer(process, active, timeout)
        except (subprocess.TimeoutExpired, TimeoutError):
            _signal_group(process, signal.SIGTERM)
            try:
                _wait_writer(process, active, 3.0)
            except (subprocess.TimeoutExpired, TimeoutError):
                _signal_group(process, signal.SIGKILL)
                process.wait(timeout=3.0)
            raise RuntimeError("rosbag shutdown timed out; recording is incomplete")


def _save_manifest(path, manifest):
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")


def _inspect_bag(config, path, env):
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError("rosbag did not produce a nonempty data.bag")
    result = subprocess.run(rosbag_command(config, ["info", "--yaml", str(path)]),
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, timeout=30.0)
    if result.returncode:
        raise RuntimeError("Cannot read saved bag: " + result.stderr.strip())
    info = yaml.safe_load(result.stdout) or {}
    counts = {row["topic"]: int(row["messages"]) for row in info.get("topics", [])}
    if not sum(counts.values()):
        raise RuntimeError("Saved bag has no messages; check ROS publishers and ROS_MASTER_URI")
    return counts


def record_bag(config, output):
    """Run until Ctrl+C, returning a finalized manifest; no hardware imports."""
    directory = Path(output).expanduser().absolute()
    if directory.suffix == ".bag":
        raise ValueError("--output must name a directory, not a .bag file")
    if directory.is_symlink() or (directory.exists() and not directory.is_dir()):
        raise ValueError("--output must be a directory, not a file or symlink")
    names = ("data.bag", "data.bag.active", "manifest.json", "rosbag.log")
    for name in names:
        if os.path.lexists(directory / name):
            raise FileExistsError("Refusing to overwrite recording: " + str(directory / name))
    options = config["record"]
    compression = options.get("compression", "lz4")
    if compression not in ("none", "lz4", "bz2"):
        raise ValueError("Unsupported rosbag compression: " + str(compression))
    buffer_mb = int(options.get("buffer_size_mb", 512))
    if buffer_mb <= 0:
        raise ValueError("record.buffer_size_mb must be positive")
    bag = directory / "data.bag"
    plan = topic_plan(config)
    pattern = plan["topic_regex"]
    arguments = ["record", "--buffsize", str(buffer_mb), "-O", str(bag)]
    if compression != "none":
        arguments.append("--" + compression)
    arguments += ["--regex", pattern]
    command = rosbag_command(config, arguments)
    env = ros_environment(config)
    directory.mkdir(parents=True, exist_ok=True)
    manifest_path = directory / "manifest.json"
    manifest = {"schema_version": 1, "status": "starting", "started_at": time.time(),
                "selection": {"arms": list(config["robot"]["arms"]),
                              "cameras": sorted(set(config["observation"]["camera_map"].values()))},
                "config": config, **plan, "command": command,
                "topic_counts": {}, "missing_required_topics": [],
                "output_file": str(bag), "ros_master_uri": env.get("ROS_MASTER_URI", "http://localhost:11311"),
                "timing": "Original message headers; bag timestamps are recorder receipt times."}
    # The exclusive manifest also reserves an existing otherwise-empty directory.
    with manifest_path.open("x") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    process = None
    stopped = False
    try:
        print_topic_plan(plan)
        with (directory / "rosbag.log").open("x") as log:
            process = subprocess.Popen(command, env=env, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            manifest["status"] = "recording"
            _save_manifest(manifest_path, manifest)
            print("Recording process started -> {}\nCtrl+C stops and saves this bag; teleoperation continues."
                  .format(bag), flush=True)
            try:
                process.wait()
            except KeyboardInterrupt:
                print("Saving bag...", flush=True)
                stopped = True
                stop_writer(process, directory, float(options.get("shutdown_timeout_sec", 20.0)))
            if not stopped:
                raise RuntimeError("rosbag exited before recording was stopped (exit {})".format(process.returncode))
            if process.returncode != 0:
                raise RuntimeError("rosbag exited with code {}".format(process.returncode))
        counts = _inspect_bag(config, bag, env)
        missing = missing_required_topics(plan, counts)
        manifest.update(topic_counts=counts, size_bytes=bag.stat().st_size,
                        missing_required_topics=missing)
        writer_log = (directory / "rosbag.log").read_text(errors="replace").lower()
        if "buffer exceeded" in writer_log or "dropping oldest" in writer_log:
            raise RuntimeError("rosbag reported buffer overflow/message loss; inspect rosbag.log")
        manifest.update(status="incomplete" if missing else "complete",
                        reason=("Missing required topics: " + ", ".join(missing))
                        if missing else "Ctrl+C / recorder stop", exit_code=process.returncode)
    except BaseException as exc:
        if process is not None and not stopped:
            try:
                stop_writer(process, directory, float(options.get("shutdown_timeout_sec", 20.0)))
            except Exception as cleanup_error:
                exc = RuntimeError("{}; cleanup: {}".format(exc, cleanup_error))
        manifest.update(status="failed", reason=str(exc) or type(exc).__name__,
                        exit_code=process.returncode if process is not None else None)
    finally:
        manifest["stopped_at"] = time.time()
        _save_manifest(manifest_path, manifest)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description="Manually record one ROS bag directory; Ctrl+C saves it.")
    parser.add_argument("--config", required=True, help="Same configuration as the running ROS publisher")
    parser.add_argument("--output", help="Directory for data.bag, manifest.json and rosbag.log")
    parser.add_argument("--topics-config", help="Override the recording topic YAML (relative to this working directory)")
    parser.add_argument("--list-topics", action="store_true", help="Print expanded topics without connecting to ROS or hardware")
    args = parser.parse_args(argv)
    if not args.list_topics and not args.output:
        parser.error("--output is required unless --list-topics is used")
    previous = signal.signal(signal.SIGTERM, _request_stop)
    try:
        config = load_config(args.config)
        if args.topics_config:
            config["record"]["topics_config"] = str(Path(args.topics_config).expanduser().resolve())
        if args.list_topics:
            print_topic_plan(topic_plan(config))
            return 0
        result = record_bag(config, args.output)
        if result["status"] != "complete":
            print("Recording %s: %s" % (result["status"], result["reason"]), file=sys.stderr)
            return 1
        print("Saved: " + result["output_file"])
        return 0
    except (OSError, ValueError) as exc:
        print("Recording failed: " + str(exc), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        signal.signal(signal.SIGTERM, previous)


def _request_stop(_signum, _frame):
    raise KeyboardInterrupt
