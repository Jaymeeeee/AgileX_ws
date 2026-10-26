"""Bounded, lossless session recording independent of ROS and LeRobot.

``write_step`` accepts the runtime's step dictionary. Camera dictionaries with
``rgb`` arrays may occur under observation.frames, next_observation.frames or
frames. Arrays are saved as RGB PPM / raw-depth and aligned-depth NPY, and JSON
contains relative paths plus the original acquisition timestamps and calibration.
"""

import datetime
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import queue
import re
import shutil
import threading
import time
import uuid


def json_value(value):
    """Convert numpy values without importing numpy; reject nonfinite numbers."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("Nonfinite values are forbidden in recorded data")
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    if hasattr(value, "tolist"):
        return json_value(value.tolist())
    raise TypeError("Cannot record value of type " + type(value).__name__)


def _atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".partial")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(json_value(value), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


class ArtifactRecorder:
    def __init__(self, record_cfg, full_config=None):
        self.config = dict(record_cfg)
        self.full_config = full_config or {}
        self.enabled = True
        self.directory = None
        self._queue = queue.Queue(maxsize=int(self.config.get("queue_size", 100)))
        self._thread = None
        self._error = None
        self._started = False
        self._closed = False
        self._stream = None
        self._manifest = {}
        self._records = 0
        self._steps = 0
        self._image_count = 0
        self._image_refs = {}
        self._last_space_check = 0.0

    def _check_space(self):
        minimum = float(self.config.get("min_free_gb", 2)) * 1024 ** 3
        free = shutil.disk_usage(str(self.directory)).free
        if free < minimum:
            raise RuntimeError("Recording disk free space {:.2f} GiB is below {:.2f} GiB".format(
                free / 1024 ** 3, minimum / 1024 ** 3))
        self._last_space_check = time.monotonic()

    def start(self):
        if self._started or self._closed:
            raise RuntimeError("Recorder cannot be restarted")
        self._started = True
        if not self.enabled:
            return self
        output = Path(self.config.get("output_dir", "outputs/sessions/piper")).expanduser().resolve()
        output.mkdir(parents=True, exist_ok=True)
        identifier = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.directory = output / (identifier + "-" + uuid.uuid4().hex[:8])
        self.directory.mkdir(exist_ok=False)
        self._check_space()
        versions = {}
        for package in ("numpy", "pyrealsense2", "pyAgxArm", "lerobot", "torch"):
            try:
                versions[package] = importlib.metadata.version(package)
            except importlib.metadata.PackageNotFoundError:
                continue
        self._manifest = {
            "schema_version": 1, "status": "recording", "created_at": time.time(),
            "session_id": self.directory.name, "config": json_value(self.full_config),
            "runtime": {"python": platform.python_version(), "platform": platform.platform(),
                        "packages": versions},
            "trajectory": "trajectory.jsonl",
            "image_format": {"rgb": "PPM (rgb8)", "depth": "NPY (raw uint16; multiply depth_scale_m)"},
        }
        try:
            _atomic_json(self.directory / "manifest.json", self._manifest)
            self._stream = (self.directory / "trajectory.jsonl").open("x", encoding="utf-8")
            self._thread = threading.Thread(target=self._run, name="trajectory-writer", daemon=True)
            self._thread.start()
        except BaseException:
            if self._stream is not None:
                self._stream.close()
            raise
        return self

    def check_health(self):
        if self._error is not None:
            raise RuntimeError("Trajectory recording failed: {}".format(self._error)) from self._error
        if self.enabled and self._started and not self._closed and self._thread is not None:
            if not self._thread.is_alive():
                raise RuntimeError("Trajectory writer stopped unexpectedly")

    def _freeze(self, value):
        # Copy only mutable image arrays; ordinary numeric arrays become lists at
        # enqueue time so callers cannot mutate a queued action or observation.
        if isinstance(value, dict):
            result = {}
            for key, item in value.items():
                if key in ("rgb", "depth", "aligned_depth") and hasattr(item, "shape"):
                    result[key] = item if not item.flags.writeable else item.copy()
                else:
                    result[key] = self._freeze(item)
            return result
        if isinstance(value, (list, tuple)):
            return [self._freeze(item) for item in value]
        return json_value(value)

    def _enqueue(self, row):
        if not self.enabled:
            return
        if not self._started or self._closed:
            raise RuntimeError("Recorder is not running")
        self.check_health()
        try:
            self._queue.put_nowait(self._freeze(row))
        except queue.Full as exc:
            self._error = RuntimeError("Trajectory writer queue is full; capture must stop")
            raise self._error from exc

    def write_event(self, event):
        row = dict(event)
        row.setdefault("timestamp", time.time())
        row["record_type"] = "event"
        self._enqueue(row)

    def write_step(self, step):
        row = dict(step)
        row["record_type"] = "step"
        self._enqueue(row)

    def _save_frame(self, frame, path):
        import numpy as np

        stamp = json_value(frame["stamp"])
        if not isinstance(stamp, (int, float)) or stamp <= 0:
            raise ValueError("Saved frame requires a positive acquisition timestamp")
        name = re.sub(r"[^a-zA-Z0-9_-]", "_", path[-1] if path else "camera")
        key = (name, stamp, frame.get("depth_stamp"), frame.get("aligned_depth_stamp"))
        if key in self._image_refs:
            refs = self._image_refs[key]
        else:
            rgb = np.asarray(frame["rgb"])
            if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError("Recording expects HWC uint8 RGB images")
            self._image_count += 1
            directory = self.directory / "images" / name
            directory.mkdir(parents=True, exist_ok=True)
            destination = directory / "{:09d}.ppm".format(self._image_count)
            with destination.open("xb") as stream:
                stream.write(
                    "P6\n{} {}\n255\n".format(rgb.shape[1], rgb.shape[0]).encode(
                        "ascii"
                    )
                )
                stream.write(rgb.tobytes(order="C"))
            refs = {"rgb_path": str(destination.relative_to(self.directory))}
            for depth_key, suffix in (
                ("depth", ".npy"),
                ("aligned_depth", ".aligned_depth.npy"),
            ):
                if depth_key not in frame:
                    continue
                depth = np.asarray(frame[depth_key])
                if depth.dtype != np.uint16 or depth.ndim != 2:
                    raise ValueError("Recording expects raw uint16 depth images")
                destination = directory / ("{:09d}".format(self._image_count) + suffix)
                with destination.open("xb") as stream:
                    np.save(stream, depth, allow_pickle=False)
                refs[depth_key + "_path"] = str(destination.relative_to(self.directory))
                if depth_key == "aligned_depth":
                    self._manifest["image_format"][
                        depth_key
                    ] = "NPY (raw uint16; multiply depth_scale_m; aligned to color)"
            # Bounded dedup cache retains frames likely shared with next_observation.
            self._image_refs[key] = refs
            if len(self._image_refs) > 128:
                del self._image_refs[next(iter(self._image_refs))]
        result = {
            key: json_value(value)
            for key, value in frame.items()
            if key not in ("rgb", "depth", "aligned_depth")
        }
        result.update(refs)
        return result

    def _extract_frames(self, value, path=()):
        if isinstance(value, dict):
            if "rgb" in value and hasattr(value["rgb"], "shape"):
                return self._save_frame(value, path)
            return {key: self._extract_frames(item, path + (key,)) for key, item in value.items()}
        if isinstance(value, list):
            return [self._extract_frames(item, path + (str(index),)) for index, item in enumerate(value)]
        return json_value(value)

    def _run(self):
        try:
            while True:
                row = self._queue.get()
                if row is None:
                    break
                if time.monotonic() - self._last_space_check > 1.0:
                    self._check_space()
                extracted = self._extract_frames(row)
                line = json.dumps(extracted, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
                self._stream.write(line + "\n")
                self._stream.flush()
                self._records += 1
                self._steps += row.get("record_type") == "step"
        except BaseException as exc:
            self._error = exc
        finally:
            try:
                self._stream.flush()
                os.fsync(self._stream.fileno())
                self._stream.close()
            except BaseException as exc:
                self._error = exc

    def close(self, error=None):
        if self._closed:
            return
        self._closed = True
        if not self.enabled or self._thread is None:
            return
        deadline = time.monotonic() + 15.0
        while self._thread.is_alive():
            try:
                self._queue.put(None, timeout=0.1)
                break
            except queue.Full:
                if time.monotonic() > deadline:
                    self._error = RuntimeError("Trajectory writer shutdown timed out")
                    break
        self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if self._thread.is_alive():
            self._error = RuntimeError("Trajectory writer shutdown timed out")
        failure = self._error if self._error is not None else error
        self._manifest.update(status="failed" if failure is not None else "complete",
                              closed_at=time.time(), records=self._records, steps=self._steps,
                              image_count=self._image_count)
        if failure is not None:
            self._manifest["error"] = str(failure)
        _atomic_json(self.directory / "manifest.json", self._manifest)
        self.check_health()
