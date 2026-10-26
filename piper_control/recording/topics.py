"""Expand the explicit ROS recording list without importing ROS or hardware."""

import itertools
from pathlib import Path
import re
import string

import yaml


class TopicConfigError(ValueError):
    pass


class _Loader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise TopicConfigError("Duplicate topic configuration key: " + str(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def _legacy_pattern(config):
    prefix = config["record"].get("topic_prefix", "/piper").strip("/")
    root = "/" + prefix if prefix else ""
    arms = "|".join(re.escape(side) for side in config["robot"]["arms"])
    cameras = sorted(set(config["observation"]["camera_map"].values()))
    parts = ["state", "action", "action_metadata", "events", "arms/(" + arms + ")/.+"]
    if cameras:
        parts.append("cameras/(" + "|".join(re.escape(name) for name in cameras) + ")/.+")
    return "^" + re.escape(root) + "/(" + "|".join(parts) + ")$"


def _expand(template, config):
    if not isinstance(template, str) or not template.strip():
        raise TopicConfigError("Topic templates must be nonempty strings")
    try:
        pieces = list(string.Formatter().parse(template))
    except ValueError as exc:
        raise TopicConfigError("Invalid topic template %r: %s" % (template, exc)) from exc
    fields = set()
    for _, field, format_spec, conversion in pieces:
        if field is None:
            continue
        if field not in ("prefix", "side", "camera") or format_spec or conversion:
            raise TopicConfigError(
                "Unsupported placeholder in %r: {%s}; use {prefix}, {side}, {camera}"
                % (template, field)
            )
        fields.add(field)
    prefix = config["record"].get("topic_prefix", "/piper").strip("/")
    sides = list(config["robot"]["arms"]) if "side" in fields else [None]
    cameras = sorted(set(config["observation"]["camera_map"].values())) if "camera" in fields else [None]
    for side, camera in itertools.product(sides, cameras):
        # A deployment with no leader must not require that absent device.
        if "/arms/{side}/leader/" in template:
            leader = config["robot"].get("pairs", {}).get(side, {}).get("leader")
            if leader is None or leader is False:
                continue
        topic = template.format(prefix="/" + prefix if prefix else "", side=side, camera=camera)
        if (not topic.startswith("/") or topic == "/" or "//" in topic
                or topic.endswith("/") or any(char.isspace() for char in topic)
                or "{" in topic or "}" in topic):
            raise TopicConfigError("Expected an absolute ROS topic name, got %r" % topic)
        yield topic


def topic_plan(config):
    """Return the expanded required/optional names and exact rosbag regex.

    The explicit list is independent of camera stream switches: requiring a
    disabled depth stream remains a missing-topic error, not a silent omission.
    Older non-collection configs without this file retain their broad selection.
    """
    path = config["record"].get("topics_config")
    if path is None:
        return dict(topics_config=None, required_topics=[], optional_topics=[],
                    topic_regex=_legacy_pattern(config))
    if not isinstance(path, str) or not path.strip():
        raise TopicConfigError("record.topics_config must be a nonempty path or null")
    path = Path(path).expanduser()
    if not path.is_absolute() and config.get("_config_path"):
        path = Path(config["_config_path"]).parent / path
    path = path.resolve()
    try:
        with path.open() as stream:
            data = yaml.load(stream, Loader=_Loader)
    except yaml.YAMLError as exc:
        raise TopicConfigError("Invalid topic configuration %s: %s" % (path, exc)) from exc
    if not isinstance(data, dict) or set(data) - {"required", "optional"}:
        raise TopicConfigError("Topic configuration must contain only required and optional lists")
    expanded = {}
    origins = {}
    for group in ("required", "optional"):
        templates = data.get(group, [])
        if not isinstance(templates, list):
            raise TopicConfigError("Topic configuration %s must be a list" % group)
        names = []
        for template in templates:
            for topic in _expand(template, config):
                if topic in origins:
                    raise TopicConfigError(
                        "Duplicate topic %s in %s; already selected in %s"
                        % (topic, group, origins[topic])
                    )
                origins[topic] = group
                names.append(topic)
        expanded[group + "_topics"] = names
    topics = expanded["required_topics"] + expanded["optional_topics"]
    if not topics:
        raise TopicConfigError("Topic configuration selects no topics for this setup")
    return dict(topics_config=str(path), topic_regex="^(?:" + "|".join(map(re.escape, topics)) + ")$",
                **expanded)


def missing_required_topics(plan, counts):
    return [topic for topic in plan["required_topics"] if counts.get(topic, 0) <= 0]


def print_topic_plan(plan):
    if plan["topics_config"] is None:
        print("Recording legacy topic pattern: " + plan["topic_regex"], flush=True)
        return
    print("Recording topics from: " + plan["topics_config"], flush=True)
    for key, label in (("required_topics", "required"), ("optional_topics", "optional")):
        for topic in plan[key]:
            print("  [%s] %s" % (label, topic), flush=True)
