#!/usr/bin/env python3
"""Convert ROS 1 bags offline with the local LeRobot API (Python >= 3.12).

Based on examples/port_datasets/port_droid.py: create/add_frame/save_episode/finalize.
No ROS node, robot SDK connection, Hub upload, or modification of input bags.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
import glob
import hashlib
import importlib.metadata
import json
import logging
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

import numpy as np
import yaml
from rosbags.highlevel import AnyReader

LOG = logging.getLogger("bag_to_lerobot")
NS = 1_000_000_000
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "lerobot_conversion.yaml"
if not DEFAULT_CONFIG.is_file():
    # catkin install layout: <prefix>/lib/data_collection/script.py
    DEFAULT_CONFIG = (Path(__file__).resolve().parents[2] / "share" / "data_collection"
                      / "config" / "lerobot_conversion.yaml")


def resolve_path(value: str, base: Path) -> Path:
    p = Path(os.path.expandvars(value)).expanduser()
    return (p if p.is_absolute() else base / p).resolve()


def load_config(path: Path) -> dict:
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(cfg, dict) or cfg.get("schema_version") != 1:
        raise ValueError("配置必须是 schema_version: 1 的 YAML")
    ds = cfg["dataset"]
    if not isinstance(ds["fps"], int) or isinstance(ds["fps"], bool) or ds["fps"] <= 0:
        raise ValueError("dataset.fps 必须是正整数")
    if not isinstance(ds["task"], str) or not ds["task"].strip():
        raise ValueError("dataset.task 必须填写任务描述")
    if len(ds["repo_id"].split("/")) != 2:
        raise ValueError("dataset.repo_id 使用 local/dataset_name 或 user/dataset_name")
    if not cfg.get("vectors") or not cfg.get("images"):
        raise ValueError("至少配置 vectors 和 images")
    used = set()
    for key, segments in cfg["vectors"].items():
        labels = []
        for seg in segments:
            used.add(seg["source"])
            if not seg.get("names") or not seg.get("field"):
                raise ValueError(f"{key}: 每段需要 field 和 names")
            labels.extend(seg["names"])
        if len(labels) != len(set(labels)):
            raise ValueError(f"{key}: names 不得重复")
    for key, spec in cfg["images"].items():
        if spec.get("enabled", True):
            used.add(spec["source"])
            if spec["kind"] not in ("rgb", "depth"):
                raise ValueError(f"{key}: kind 必须为 rgb 或 depth")
            if spec.get("storage", "video") not in ("video", "image"):
                raise ValueError(f"{key}: storage 必须为 video 或 image")
            if spec["kind"] == "depth" and spec.get("storage") != "image":
                raise ValueError(f"{key}: 深度只支持 image 存储，保留 uint16 毫米值")
    if not any(v.get("enabled", True) for v in cfg["images"].values()):
        raise ValueError("至少启用一路图像")
    topics = []
    for sid in used:
        spec = cfg["sources"][sid]
        topics.append(spec["topic"])
        if spec.get("timestamp", "header") not in ("header", "bag", "json.stamp"):
            raise ValueError(f"{sid}: timestamp 应为 header / bag / json.stamp")
        if spec["sample"] not in ("previous", "nearest"):
            raise ValueError(f"{sid}: sample 应为 previous / nearest")
        age = spec.get("max_age_sec")
        if age is None and not (spec.get("hold_last", False) and spec["sample"] == "previous"):
            raise ValueError(f"{sid}: 无限保持仅用于显式 hold_last: true 的 previous 源")
        if age is not None and (not np.isfinite(age) or age <= 0):
            raise ValueError(f"{sid}: max_age_sec 必须为正数或 null")
        if spec.get("hold_last") and spec["sample"] != "previous":
            raise ValueError(f"{sid}: hold_last 必须使用 previous")
    if len(topics) != len(set(topics)):
        raise ValueError("同一话题请复用同一个 source，不要重复定义")
    return cfg


def active_images(cfg: dict) -> dict:
    return {k: v for k, v in cfg["images"].items() if v.get("enabled", True)}


def requirements(cfg: dict) -> dict:
    """source -> fields and expected lengths; images have no numeric fields."""
    result = {}
    for segments in cfg["vectors"].values():
        for seg in segments:
            fields = result.setdefault(seg["source"], {})
            n = len(seg["names"])
            if seg["field"] in fields and fields[seg["field"]] != n:
                raise ValueError("同一字段配置了不同长度")
            fields[seg["field"]] = n
    for spec in active_images(cfg).values():
        if result.get(spec["source"]):
            raise ValueError("图像源不能同时作为数值源")
        result.setdefault(spec["source"], {})
    return result


def message_stamp(msg, bag_ns: int, mode: str) -> int:
    if mode == "bag":
        return bag_ns
    if mode == "json.stamp":
        val = float(json.loads(msg.data)["stamp"])
        if not np.isfinite(val):
            raise ValueError("JSON 时间戳非有限数")
        stamp = round(val * NS)
    else:
        stamp = int(msg.header.stamp.sec) * NS + int(msg.header.stamp.nanosec)
    if stamp <= 0:
        raise ValueError("无效源时间戳；不自动回退到 bag 接收时间")
    return stamp


def extract_vector(msg, name: str, spec: dict, length: int) -> np.ndarray:
    value = msg
    parts = name.split(".")
    if parts[0] == "json":
        value, parts = json.loads(msg.data), parts[1:]
    for part in parts:
        value = value[part] if isinstance(value, dict) else getattr(value, part)
    array = np.atleast_1d(np.asarray(value, dtype=np.float64))
    if array.ndim != 1:
        raise ValueError(f"{name}: 需要一维数值")
    if "order" in spec:
        actual = list(getattr(msg, spec["order_field"]))
        expected = spec["order"]
        if len(actual) != len(set(actual)) or len(actual) != len(array):
            raise ValueError(f"{name}: 顺序字段与数值长度不匹配")
        if set(actual) != set(expected):
            raise ValueError(f"{name}: 关节标识不匹配: {actual}，预期 {expected}")
        array = array[[actual.index(x) for x in expected]]
    if len(array) != length or not np.isfinite(array).all():
        raise ValueError(f"{name}: 预期 {length} 个有限数，得到 {array}")
    out = array.astype(np.float32)
    if not np.isfinite(out).all():
        raise ValueError(f"{name}: 超出 float32 范围")
    return out


def image_layout(msg, kind: str) -> tuple:
    expected = ("rgb8", "bgr8") if kind == "rgb" else ("16UC1", "mono16")
    if msg.encoding not in expected:
        raise ValueError(f"{kind} 不支持编码 {msg.encoding}，支持 {expected}")
    channels, itemsize = (3, 1) if kind == "rgb" else (1, 2)
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    if h <= 0 or w <= 0 or step < w * channels * itemsize or step % itemsize:
        raise ValueError("Image 的 height/width/step 无效")
    if len(msg.data) != h * step:
        raise ValueError("Image 数据长度与 height * step 不符")
    return h, w, channels


def decode_image(msg, kind: str) -> np.ndarray:
    h, w, c = image_layout(msg, kind)
    dtype = np.dtype("u1") if kind == "rgb" else np.dtype(">u2" if msg.is_bigendian else "<u2")
    # Honor ROS row padding and endianness; do not normalize depth to 8 bits.
    rows = np.frombuffer(msg.data, dtype=dtype).reshape(h, msg.step // dtype.itemsize)
    array = rows[:, :w * c].reshape(h, w, c)
    if kind == "rgb" and msg.encoding == "bgr8":
        array = array[:, :, ::-1]
    return np.array(array, dtype=np.uint8 if kind == "rgb" else np.uint16, order="C", copy=True)


@dataclass
class Series:
    stamps: list = field(default_factory=list)
    bag_stamps: list = field(default_factory=list)
    ordinals: list = field(default_factory=list)
    values: dict = field(default_factory=dict)
    shape: tuple | None = None
    message_count: int = 0
    reordered: int = 0
    duplicates: int = 0

    def finish(self):
        stamps = np.asarray(self.stamps, dtype=np.int64)
        if not len(stamps):
            raise ValueError("话题没有消息")
        self.reordered = int(np.sum(np.diff(stamps) < 0))
        order = np.argsort(stamps, kind="stable")
        sorted_stamps = stamps[order]
        dup = np.flatnonzero(np.diff(sorted_stamps) == 0)
        for values in self.values.values():
            a = np.stack(values)
            if len(dup) and not np.array_equal(a[order[dup]], a[order[dup + 1]]):
                raise ValueError("相同时间戳对应不同数值，不能确定动作顺序")
        self.duplicates = len(dup)
        keep = np.r_[True, np.diff(sorted_stamps) != 0]
        order = order[keep]
        self.stamps = stamps[order]
        self.bag_stamps = np.asarray(self.bag_stamps, dtype=np.int64)[order]
        self.ordinals = np.asarray(self.ordinals, dtype=np.int64)[order]
        self.values = {k: np.stack(v)[order] for k, v in self.values.items()}


def read_index(bag: Path, cfg: dict) -> tuple[dict, dict]:
    req = requirements(cfg)
    result = {sid: Series(values={f: [] for f in fields}) for sid, fields in req.items()}
    topics = {cfg["sources"][sid]["topic"]: sid for sid in req}
    kinds = {s["source"]: s["kind"] for s in active_images(cfg).values()}
    with AnyReader([bag]) as reader:
        available = {c.topic for c in reader.connections}
        missing = set(topics) - available
        if missing:
            raise ValueError(f"缺失话题: {sorted(missing)}")
        clock_limit = int(cfg["sync"]["max_clock_skew_sec"] * NS)
        for conn, bag_ns, raw in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
            sid = topics[conn.topic]
            spec, series = cfg["sources"][sid], result[sid]
            try:
                msg = reader.deserialize(raw, conn.msgtype)
                stamp = message_stamp(msg, bag_ns, spec.get("timestamp", "header"))
                skew = stamp - bag_ns
                if skew > clock_limit or (skew < -clock_limit and not spec.get("hold_last")):
                    raise ValueError(f"源时间与 bag 接收时间差 {skew / NS:.3f}s，超出时钟阈值")
                if sid in kinds:
                    shape = image_layout(msg, kinds[sid])
                    if series.shape is not None and series.shape != shape:
                        raise ValueError("同一话题图像分辨率发生变化")
                    series.shape = shape
                for f, n in req[sid].items():
                    series.values[f].append(extract_vector(msg, f, spec, n))
                series.stamps.append(stamp)
                series.bag_stamps.append(bag_ns)
                series.ordinals.append(series.message_count)
                series.message_count += 1
            except Exception as exc:
                raise ValueError(f"{bag.parent.name} {conn.topic} 第 {series.message_count} 条: {exc}") from exc
        bounds = {"bag_start_ns": int(reader.start_time), "bag_end_ns": int(reader.end_time - 1)}
    for sid, series in result.items():
        try:
            series.finish()
        except ValueError as exc:
            raise ValueError(f"{bag.parent.name} {sid}: {exc}") from exc
    return result, bounds


def sample_indices(stamps: np.ndarray, grid: np.ndarray, mode: str) -> np.ndarray:
    right = np.searchsorted(stamps, grid, side="right")
    previous = right - 1
    if mode == "previous":
        return previous
    left = np.maximum(previous, 0)
    after = np.minimum(right, len(stamps) - 1)
    # In a tie prefer the earlier frame.
    return np.where(abs(stamps[after] - grid) < abs(stamps[left] - grid), after, left)


def build_plan(series: dict, bounds: dict, cfg: dict, max_frames: int | None = None) -> tuple:
    specs, fps = cfg["sources"], cfg["dataset"]["fps"]
    start = max(bounds["bag_start_ns"], *(int(s.stamps[0]) for s in series.values()))
    end = min(bounds["bag_end_ns"], *(int(s.stamps[-1]) for sid, s in series.items()
                                    if not specs[sid].get("hold_last")))
    if end <= start:
        raise ValueError("各话题没有共同时间区间")
    count = (end - start) * fps // NS + 1
    if max_frames is not None:
        count = min(count, max_frames)
    if count < cfg["sync"].get("min_frames", 2):
        raise ValueError("共同时间区间太短")
    grid = start + np.rint(np.arange(count, dtype=np.float64) * NS / fps).astype(np.int64)
    selected, summary = {}, {}
    for sid, s in series.items():
        spec = specs[sid]
        ids = sample_indices(s.stamps, grid, spec["sample"])
        if (ids < 0).any():
            raise ValueError(f"{sid}: 首帧之前没有有效目标，禁止用未来值填充")
        delta = s.stamps[ids] - grid
        limit = spec.get("max_age_sec")
        if limit is not None:
            bad = np.flatnonzero(np.abs(delta) > round(limit * NS))
            if len(bad):
                i = int(bad[0])
                raise ValueError(f"{sid}: 第 {i} 帧同步误差 {delta[i]/NS:.6f}s 超过 {limit}s；"
                                 "拒绝跨缺失段拼接或压缩时间轴")
        selected[sid] = ids
        summary[sid] = {"topic": spec["topic"], "messages": s.message_count,
                        "reordered": s.reordered, "duplicate_stamps": s.duplicates,
                        "max_abs_delta_ms": float(np.max(abs(delta)) / 1e6),
                        "reused_samples": int(len(ids) - len(np.unique(ids))),
                        "hold_last": bool(spec.get("hold_last", False))}
    return grid, selected, summary


def feature_schema(cfg: dict, series: dict) -> dict:
    features = {}
    for key, segments in cfg["vectors"].items():
        names = [n for seg in segments for n in seg["names"]]
        features[key] = {"dtype": "float32", "shape": (len(names),), "names": names}
    for key, spec in active_images(cfg).items():
        features[key] = {"dtype": spec.get("storage", "video"),
                         "shape": series[spec["source"]].shape,
                         "names": ["height", "width", "channels"],
                         "info": {"is_depth_map": spec["kind"] == "depth"}}
        if spec["kind"] == "depth":
            features[key]["info"]["depth_unit"] = "mm"
    return features


def numeric_frame(cfg: dict, series: dict, selected: dict, i: int) -> dict:
    return {key: np.concatenate([series[s["source"]].values[s["field"]][selected[s["source"]][i]]
                                 for s in segments]) for key, segments in cfg["vectors"].items()}


def numeric_hashes(cfg: dict, series: dict, selected: dict) -> dict:
    hashes = {}
    for key, segments in cfg["vectors"].items():
        values = np.concatenate([series[s["source"]].values[s["field"]][selected[s["source"]]]
                                 for s in segments], axis=1)
        hashes[key] = hashlib.sha256(np.asarray(values, dtype="<f4").tobytes()).hexdigest()
    return hashes


def write_episode(dataset, bag: Path, cfg: dict, series: dict, grid: np.ndarray, selected: dict, task: str):
    images = active_images(cfg)
    by_source = defaultdict(list)
    for key, spec in images.items():
        by_source[spec["source"]].append((key, spec["kind"]))
    wanted = {}
    for sid in by_source:
        lookup = defaultdict(list)
        for frame, idx in enumerate(selected[sid]):
            lookup[int(series[sid].ordinals[idx])].append(frame)
        wanted[sid] = lookup
    topics = {cfg["sources"][sid]["topic"]: sid for sid in by_source}
    seen, pending, next_frame = defaultdict(int), {}, 0
    with AnyReader([bag]) as reader:
        for conn, _, raw in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
            sid = topics[conn.topic]
            ordinal = seen[sid]
            seen[sid] += 1
            frame_ids = wanted[sid].get(ordinal)
            if frame_ids is None:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            for key, kind in by_source[sid]:
                array = decode_image(msg, kind)
                for i in frame_ids:
                    pending.setdefault(i, {})[key] = array
            while next_frame in pending and len(pending[next_frame]) == len(images):
                frame = numeric_frame(cfg, series, selected, next_frame)
                frame.update(pending.pop(next_frame))
                frame["task"] = task
                dataset.add_frame(frame)
                next_frame += 1
            if len(pending) > cfg["sync"].get("max_pending_frames", 128):
                raise ValueError("图像严重乱序，超过待组帧上限")
            if next_frame == len(grid):
                break
    if next_frame != len(grid):
        raise ValueError(f"图像读回不完整: {next_frame}/{len(grid)}")
    dataset.save_episode(parallel_encoding=False)


def json_write(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def bag_identity(path: Path) -> dict:
    s = path.stat()
    return {"path": str(path), "size_bytes": s.st_size, "mtime_ns": s.st_mtime_ns}


def source_metadata(root: Path, index: int, bag: Path, cfg: dict, series: dict, grid, selected, report):
    folder = root / "meta" / "source" / f"episode_{index:06d}"
    folder.mkdir(parents=True)
    manifest = bag.parent / "manifest.json"
    if manifest.is_file():
        shutil.copy2(manifest, folder / "recording_manifest.json")
    arrays = {"grid_timestamp_ns": grid}
    for sid, ids in selected.items():
        arrays[f"{sid}__source_timestamp_ns"] = series[sid].stamps[ids]
        arrays[f"{sid}__bag_timestamp_ns"] = series[sid].bag_stamps[ids]
        arrays[f"{sid}__message_index"] = series[sid].ordinals[ids]
    np.savez_compressed(folder / "timing.npz", **arrays)
    json_write(folder / "conversion.json", report)


def validate_dataset(root: Path, cfg: dict, reports: list) -> dict:
    """Read back all numeric rows and first/middle/last RGB/depth frames per episode."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import pyarrow.parquet as pq

    expected = sum(r["frames"] for r in reports)
    dataset = LeRobotDataset(cfg["dataset"]["repo_id"], root=root, video_backend="pyav")
    if len(dataset) != expected or dataset.meta.total_episodes != len(reports):
        raise ValueError("读回帧数或 episode 数不一致")
    table = pq.read_table(sorted((root / "data").rglob("*.parquet")))
    episodes = table["episode_index"].to_numpy()
    timestamps = table["timestamp"].to_numpy()
    frame_ids = table["frame_index"].to_numpy()
    count = 0
    for report in reports:
        ep, n = report["episode_index"], report["frames"]
        rows = np.flatnonzero(episodes == ep)
        if len(rows) != n or not np.array_equal(frame_ids[rows], np.arange(n)):
            raise ValueError(f"episode {ep} 帧编号不连续")
        np.testing.assert_allclose(timestamps[rows], np.arange(n) / cfg["dataset"]["fps"], atol=1e-5)
        for i in sorted({int(rows[0]), int(rows[len(rows)//2]), int(rows[-1])}):
            item = dataset[i]
            for key, spec in active_images(cfg).items():
                image = item[key]
                h, w, c = dataset.features[key]["shape"]
                if tuple(image.shape) != (c, h, w) or not image.isfinite().all():
                    raise ValueError(f"{key}: 读回图像形状或像素无效")
            count += 1
    for key in cfg["vectors"]:
        values = np.asarray(table[key].to_pylist(), dtype=np.float32)
        if not np.isfinite(values).all():
            raise ValueError(f"{key}: 读回非有限数")
        for report in reports:
            rows = values[episodes == report["episode_index"]]
            digest = hashlib.sha256(np.asarray(rows, dtype="<f4").tobytes()).hexdigest()
            if digest != report["numeric_sha256"][key]:
                raise ValueError(f"{key}: episode {report['episode_index']} 与源采样数值不一致")
    return {"episodes": len(reports), "frames": expected, "image_frames_checked": count,
            "all_numeric_rows_finite": True, "all_numeric_rows_match_source": True,
            "frame_indices_and_timestamps_valid": True}


def run(args):
    config_path = Path(args.config).expanduser().resolve()
    cfg = load_config(config_path)
    pattern = str(resolve_path(cfg["input"]["bag_glob"], config_path.parent))
    bags = sorted({Path(p).resolve() for p in glob.glob(pattern)})
    if args.episodes:
        requested = set(args.episodes)
        missing = requested - {p.parent.name for p in bags}
        if missing:
            raise ValueError(f"找不到采集目录: {sorted(missing)}")
        bags = [p for p in bags if p.parent.name in requested]
    if args.limit_episodes:
        bags = bags[:args.limit_episodes]
    if not bags:
        raise ValueError(f"没有匹配的 bag: {pattern}")
    for bag in bags:
        if Path(str(bag) + ".active").exists():
            raise ValueError(f"仍在录制: {bag}")
        if cfg["input"].get("require_complete_manifest", True):
            manifest = json.loads((bag.parent / "manifest.json").read_text())
            if manifest.get("state") != "complete":
                raise ValueError(f"录制未标记 complete: {bag}")
    output = resolve_path(args.output, Path.cwd()) if args.output else resolve_path(cfg["dataset"]["root"], config_path.parent)
    if not args.dry_run and output.exists():
        raise ValueError(f"输出目录已存在，换一个 --output；不会覆盖: {output}")
    staging = output.parent / f".{output.name}.converting-{uuid.uuid4().hex[:8]}"
    dataset, schema, reports = None, None, []
    report = {"state": "running", "created_at": datetime.now(timezone.utc).isoformat(),
              "config": cfg, "config_path": str(config_path), "episodes": reports,
              "preview_max_frames": args.max_frames, "output": str(output)}
    try:
        for ep, bag in enumerate(bags):
            LOG.info("[%d/%d] 索引 %s", ep + 1, len(bags), bag.parent.name)
            identity = bag_identity(bag)
            series, bounds = read_index(bag, cfg)
            grid, selected, summary = build_plan(series, bounds, cfg, args.max_frames)
            features = feature_schema(cfg, series)
            if schema is not None and features != schema:
                raise ValueError(f"{bag}: 特征形状与前一个 episode 不一致")
            schema = features
            task = cfg.get("tasks", {}).get(bag.parent.name, cfg["dataset"]["task"])
            if not isinstance(task, str) or not task.strip():
                raise ValueError(f"{bag}: task 为空")
            item = {"episode_index": ep, "source": identity, "frames": len(grid), "task": task,
                    "grid_start_ns": int(grid[0]), "grid_end_ns": int(grid[-1]),
                    "trimmed_start_sec": float((grid[0] - bounds["bag_start_ns"]) / NS),
                    "trimmed_end_sec": float((bounds["bag_end_ns"] - grid[-1]) / NS),
                    "sources": summary}
            item["numeric_sha256"] = numeric_hashes(cfg, series, selected)
            LOG.info("  %d 帧 / %.2f 秒 @ %d FPS", len(grid), len(grid)/cfg["dataset"]["fps"], cfg["dataset"]["fps"])
            if args.dry_run:
                reports.append(item)
                continue
            if dataset is None:
                from lerobot.datasets.lerobot_dataset import LeRobotDataset
                from lerobot.configs.video import RGBEncoderConfig
                import lerobot
                import av

                av.logging.set_level(av.logging.ERROR)
                source_root = Path(lerobot.__file__).resolve().parents[2]
                version = subprocess.run(["git", "-C", str(source_root), "rev-parse", "HEAD"], capture_output=True, text=True)
                report["lerobot"] = {"version": importlib.metadata.version("lerobot"),
                                     "module": str(lerobot.__file__),
                                     "commit": version.stdout.strip() if version.returncode == 0 else None}
                report["converter_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
                report["dependencies"] = {p: importlib.metadata.version(p) for p in ("rosbags", "numpy", "av", "torch", "datasets")}
                dataset = LeRobotDataset.create(
                    repo_id=cfg["dataset"]["repo_id"], root=staging, fps=cfg["dataset"]["fps"],
                    robot_type=cfg["dataset"].get("robot_type", "piper_dual"), features=schema,
                    use_videos=any(s.get("storage", "video") == "video" for s in active_images(cfg).values()),
                    streaming_encoding=True, rgb_encoder=RGBEncoderConfig(**cfg.get("video", {})),
                    encoder_threads=2, video_backend="pyav")
                (staging / "meta" / "conversion_config.yaml").write_text(
                    yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
                json_write(staging / "meta" / "conversion_report.json", report)
            if bag_identity(bag) != identity:
                raise ValueError(f"索引期间 bag 被修改: {bag}")
            write_episode(dataset, bag, cfg, series, grid, selected, task)
            if bag_identity(bag) != identity:
                raise ValueError(f"转换期间 bag 被修改: {bag}")
            source_metadata(staging, ep, bag, cfg, series, grid, selected, item)
            reports.append(item)
            json_write(staging / "meta" / "conversion_report.json", report)
        if args.dry_run:
            LOG.info("检查通过：%d 个 episode，共 %d 帧；未写数据集", len(reports), sum(r["frames"] for r in reports))
            return
        dataset.finalize()
        LOG.info("读回验证 episode、数值和图像…")
        report["validation"] = validate_dataset(staging, cfg, reports)
        report["state"] = "complete"
        json_write(staging / "meta" / "conversion_report.json", report)
        if output.exists():
            raise ValueError(f"输出目录在转换期间被创建，拒绝覆盖: {output}")
        staging.rename(output)
        LOG.info("完成：%s（%d 个 episode，%d 帧）", output, len(reports), sum(r["frames"] for r in reports))
    except BaseException as exc:
        if dataset is not None:
            try:
                if dataset.has_pending_frames():
                    dataset.clear_episode_buffer()
                dataset.finalize()
            except Exception:
                LOG.exception("清理未完成的写入时出错")
        if staging.exists():
            report.update(state="failed", error=f"{type(exc).__name__}: {exc}")
            json_write(staging / "meta" / "conversion_report.json", report)
            LOG.error("未完成输出保留在 %s，不能作为完整训练数据", staging)
        raise


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default=str(DEFAULT_CONFIG), help="YAML 配置")
    p.add_argument("--output", help="覆盖 YAML 的数据集输出目录（必须不存在）")
    p.add_argument("--episodes", nargs="+", help="只转换这些采集目录，如 task_0001 task_0002")
    p.add_argument("--limit-episodes", type=int, help="只处理前 N 个 episode")
    p.add_argument("--max-frames", type=int, help="每个 episode 只取前 N 帧，用于预览测试")
    p.add_argument("--dry-run", action="store_true", help="扫描话题、字段和时间同步，不写数据集")
    args = p.parse_args()
    for name in ("limit_episodes", "max_frames"):
        if getattr(args, name) is not None and getattr(args, name) <= 0:
            p.error(f"--{name.replace('_', '-')} 必须大于 0")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        run(args)
    except KeyboardInterrupt:
        LOG.error("转换已中断；原始 bag 保持不变")
        return 130
    except Exception:
        LOG.exception("转换失败")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
