"""Standard ROS 1 messages and optional asynchronous bag recording.

Hardware and policy execution do not depend on this module. ``start_bag`` opens
the bag before the runtime enables motion; the same messages passed to ROS
publishers are written with their acquisition/command timestamps. No custom
messages, cv_bridge, roslaunch, or rosbag subprocess are required.
"""

import json
import math
from pathlib import Path
import queue
import shutil
import threading
import time

from .artifacts import _atomic_json, json_value
from .topics import missing_required_topics, topic_plan


class _BagWriter:
    def __init__(self, path, config, plan=None):
        import rosbag

        self.path = Path(path)
        self.report_path = self.path.with_name("capture_report.json")
        self.plan = plan
        self.selected_topics = (
            set(plan["required_topics"] + plan["optional_topics"]) if plan is not None else None
        )
        self.topic_counts = {}
        self.queue = queue.Queue(maxsize=int(config.get("queue_size", 100)))
        self.error = None
        self.closed = False
        self.min_free = float(config.get("min_free_gb", 2)) * 1024 ** 3
        self.last_space_check = 0.0
        if self.path.exists():
            raise FileExistsError("Refusing to overwrite bag: " + str(self.path))
        if self.report_path.exists():
            raise FileExistsError("Refusing to overwrite capture report: " + str(self.report_path))
        compression = str(config.get("compression", "lz4"))
        if compression not in ("none", "lz4", "bz2"):
            raise ValueError("Unsupported rosbag compression " + compression)
        self.bag = rosbag.Bag(str(self.path), "w", compression=compression)
        self.thread = threading.Thread(target=self._run, name="rosbag-writer", daemon=True)
        self.thread.start()

    def put(self, topic, message, stamp):
        self.check_health()
        if self.closed:
            raise RuntimeError("Bag writer is closed")
        if self.selected_topics is not None and topic not in self.selected_topics:
            return
        try:
            self.queue.put_nowait((topic, message, stamp))
        except queue.Full as exc:
            self.error = RuntimeError("ROS bag queue is full; capture must stop")
            raise self.error from exc

    def check_health(self):
        if self.error is not None:
            raise RuntimeError("ROS bag recording failed: {}".format(self.error)) from self.error
        if not self.closed and not self.thread.is_alive():
            raise RuntimeError("ROS bag writer stopped unexpectedly")

    def _run(self):
        try:
            while True:
                entry = self.queue.get()
                if entry is None:
                    break
                if time.monotonic() - self.last_space_check > 1.0:
                    if shutil.disk_usage(str(self.path.parent)).free < self.min_free:
                        raise RuntimeError("Insufficient free disk space for ROS bag")
                    self.last_space_check = time.monotonic()
                topic, message, stamp = entry
                self.bag.write(topic, message, t=stamp)
                # A queued message is not evidence that rosbag wrote it.
                self.topic_counts[topic] = self.topic_counts.get(topic, 0) + 1
        except BaseException as exc:
            self.error = exc
        finally:
            try:
                self.bag.close()
            except BaseException as exc:
                self.error = exc

    def close(self):
        if self.closed:
            return
        self.closed = True
        deadline = time.monotonic() + 15.0
        while self.thread.is_alive():
            try:
                self.queue.put(None, timeout=0.1)
                break
            except queue.Full:
                if time.monotonic() > deadline:
                    break
        self.thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if self.thread.is_alive():
            self.error = RuntimeError("ROS bag shutdown timed out; bag may require rosbag reindex")
        counts = dict(self.topic_counts)
        missing = missing_required_topics(self.plan, counts) if self.plan is not None else []
        status = "failed" if self.error is not None else "incomplete" if missing else "complete"
        reason = str(self.error) if self.error is not None else (
            "Missing required topics: " + ", ".join(missing) if missing else "ROS bag closed"
        )
        _atomic_json(self.report_path, {
            "schema_version": 1,
            "status": status,
            "reason": reason,
            "output_file": str(self.path),
            "topics_config": self.plan["topics_config"] if self.plan is not None else None,
            "required_topics": self.plan["required_topics"] if self.plan is not None else [],
            "optional_topics": self.plan["optional_topics"] if self.plan is not None else [],
            "topic_counts": counts,
            "missing_required_topics": missing,
            "size_bytes": self.path.stat().st_size if self.path.exists() else None,
            "closed_at": time.time(),
        })
        if missing and self.error is None:
            self.error = RuntimeError("ROS bag incomplete: " + reason)
        self.check_health()


class ROSBridge:
    def __init__(self, cfg):
        self.full_config = cfg
        self.config = dict(cfg.get("record", cfg))
        self.enabled = True
        namespace = self.config.get("topic_prefix", "/piper").strip("/")
        self.prefix = "/" + namespace if namespace else ""
        self._publishers = {}
        self._last_stamps = {}
        self._bag = None
        self.bag_path = None
        self._started = False
        self._closed = False
        self._joint_names = []

    @property
    def topics(self):
        return sorted(self._publishers)

    def start(self):
        if self._started or self._closed:
            raise RuntimeError("ROS bridge cannot be restarted")
        if not self.enabled:
            self._started = True
            return self
        try:
            import rospy
            from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
            from sensor_msgs.msg import CameraInfo, Image, JointState
            from std_msgs.msg import String
        except ImportError as exc:
            raise RuntimeError("ROS capture requires sourcing ROS Noetic before starting this Python process") from exc
        try:
            rospy.get_master().getPid()
        except Exception as exc:
            raise RuntimeError("ROS master is unavailable; start roscore before enabling capture") from exc
        if not rospy.core.is_initialized():
            rospy.init_node("piper_collection", anonymous=True, disable_signals=True)
        if rospy.get_param("/use_sim_time", False):
            raise RuntimeError("SDK capture requires use_sim_time=false")
        self.rospy = rospy
        self.messages = {"joint": JointState, "image": Image, "info": CameraInfo,
                         "string": String, "diagnostics": DiagnosticArray,
                         "status": DiagnosticStatus, "keyvalue": KeyValue}
        self._started = True
        return self

    def start_bag(self, directory):
        if not self.enabled:
            return None
        if not self._started or self._closed:
            raise RuntimeError("ROS bridge must be started before bag recording")
        if self._bag is not None:
            raise RuntimeError("Bag recording has already started")
        self.bag_path = Path(directory) / "capture.bag"
        plan = topic_plan(self.full_config) if self.config.get("topics_config") is not None else None
        self._bag = _BagWriter(self.bag_path, self.config, plan=plan)
        return self.bag_path

    def check_health(self):
        if self._bag is not None:
            self._bag.check_health()
        if self.enabled and self._started and not self._closed and self.rospy.is_shutdown():
            raise RuntimeError("ROS shutdown requested")

    def _emit(self, suffix, kind, message, stamp):
        if not self.enabled:
            return
        if not self._started or self._closed:
            raise RuntimeError("ROS bridge is not running")
        self.check_health()
        topic = self.prefix + "/" + suffix.strip("/")
        if topic not in self._publishers:
            self._publishers[topic] = self.rospy.Publisher(
                topic, self.messages[kind], queue_size=2 if kind == "image" else 10)
        if self._bag is not None:
            self._bag.put(topic, message, stamp)
        self._publishers[topic].publish(message)

    def _time(self, stamp):
        stamp = float(stamp)
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError("ROS messages require a valid source timestamp")
        return self.rospy.Time.from_sec(stamp)

    def _joint_message(self, positions, stamp, names, velocities=None, efforts=None):
        message = self.messages["joint"]()
        message.header.stamp = self._time(stamp)
        message.name = names
        message.position = json_value(positions)
        if velocities is not None:
            message.velocity = json_value(velocities)
        if efforts is not None:
            message.effort = json_value(efforts)
        return message

    def publish_state(self, snapshot):
        if not self.enabled:
            return
        follower_states = []
        all_names = []
        for side, devices in snapshot.get("arms", {}).items():
            names = [side + "_joint_" + str(i) for i in range(1, 7)] + [side + "_gripper_width"]
            all_names.extend(names)
            for role in ("follower", "leader"):
                data = devices.get(role)
                if not data or data.get("position") is None or data.get("stamp") is None:
                    continue
                suffix = "arms/{}/{}/{}".format(side, role,
                    "command" if data.get("source") == "command" else "joint_states")
                stamp = float(data["stamp"])
                if stamp == self._last_stamps.get(suffix):
                    if role == "follower":
                        follower_states.append(data)
                    continue
                message = self._joint_message(data["position"], stamp, names,
                                              data.get("velocity"), data.get("effort"))
                self._emit(suffix, "joint", message, message.header.stamp)
                self._last_stamps[suffix] = stamp
                if role == "follower":
                    follower_states.append(data)
                self._publish_feedback(side, role, data)
        self._joint_names = all_names
        if follower_states and len(follower_states) == len(snapshot.get("arms", {})):
            stamps = tuple(float(data["stamp"]) for data in follower_states)
            stamp = min(stamps)
            if stamps != self._last_stamps.get("state"):
                positions = [value for data in follower_states for value in data["position"]]
                message = self._joint_message(positions, stamp, all_names)
                self._emit("state", "joint", message, message.header.stamp)
                self._last_stamps["state"] = stamps

    def _publish_feedback(self, side, role, data):
        diagnostics = self.messages["diagnostics"]()
        diagnostics.header.stamp = self._time(data["stamp"])
        entries = []
        for collection in ("motor_feedback", "driver_feedback"):
            for index, feedback in enumerate(data.get(collection) or []):
                entries.append((collection + "/" + str(index + 1), feedback, "feedback"))
        command_gripper = data.get("source") == "command"
        entries.append(("gripper_command" if command_gripper else "gripper_feedback",
                        data.get("gripper_feedback", {}), "command" if command_gripper else "feedback"))
        # Arm status remains a measured SDK status even when the leader's joint
        # and gripper targets come from its command stream.
        entries.append(("arm_status", data.get("arm_status", {}), "feedback"))
        for name, feedback, source in entries:
            if not feedback or feedback.get("stamp") is None:
                continue
            stamp = float(feedback.get("stamp", 0.))
            if not math.isfinite(stamp) or stamp <= 0:
                continue
            status = self.messages["status"]()
            status.name = "{}/{}/{}".format(side, role, name)
            status.hardware_id = "piper_" + side + "_" + role
            status.level = 0
            status.message = "SDK {}; stamp field is its source timestamp".format(source)
            for key, value in dict(feedback, source=source).items():
                field = self.messages["keyvalue"]()
                field.key = str(key)
                field.value = json.dumps(json_value(value), ensure_ascii=False, allow_nan=False)
                status.values.append(field)
            diagnostics.status.append(status)
        if diagnostics.status:
            self._emit("arms/{}/{}/diagnostics".format(side, role), "diagnostics",
                       diagnostics, diagnostics.header.stamp)

    def _camera_info(self, info, stamp, frame_id):
        message = self.messages["info"]()
        message.header.stamp = self._time(stamp)
        message.header.frame_id = frame_id
        message.width, message.height = int(info["width"]), int(info["height"])
        fx, fy, cx, cy = (float(info[key]) for key in ("fx", "fy", "ppx", "ppy"))
        message.K = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
        message.R = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        message.P = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
        coefficients = list(info.get("coeffs", [0.0] * 5))
        model = str(info.get("distortion_model", "brown_conrady")).split(".")[-1]
        if model in ("brown_conrady", "none", "plumb_bob") or not any(coefficients):
            message.distortion_model = "plumb_bob"
            message.D = coefficients
        elif model in ("kannala_brandt4", "equidistant"):
            message.distortion_model = "equidistant"
            message.D = coefficients[:4]
        else:
            # Preserve the SDK model explicitly; do not mislabel inverse Brown as
            # ordinary ROS plumb_bob calibration.
            message.distortion_model = "realsense_" + model
            message.D = coefficients
        return message

    def _image(self, array, stamp, frame_id, depth_scale=None):
        import numpy as np

        array = np.asarray(array)
        message = self.messages["image"]()
        message.header.stamp = self._time(stamp)
        message.header.frame_id = frame_id
        message.height, message.width = array.shape[:2]
        if depth_scale is None:
            if array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3:
                raise ValueError("ROS RGB image requires HWC uint8")
            message.encoding = "rgb8"
            message.step = int(array.shape[1]) * 3
        else:
            if array.dtype != np.uint16 or array.ndim != 2:
                raise ValueError("ROS depth input requires raw uint16")
            if not math.isfinite(float(depth_scale)) or float(depth_scale) <= 0:
                raise ValueError("ROS depth image requires a valid depth scale")
            array = array.astype("<f4") * float(depth_scale)
            # REP 118: float depth is in metres; invalid depth is NaN.
            array[array == 0] = np.nan
            message.encoding = "32FC1"
            message.step = int(array.shape[1]) * 4
        message.is_bigendian = False
        message.data = array.tobytes(order="C")
        return message

    def publish_images(self, frames):
        if not self.enabled:
            return
        for name, frame in frames.items():
            streams = (
                ("color", "rgb", "stamp", "camera_info", "color"),
                ("depth", "depth", "depth_stamp", "depth_camera_info", "depth"),
                (
                    "aligned_depth_to_color",
                    "aligned_depth",
                    "aligned_depth_stamp",
                    "aligned_depth_camera_info",
                    "color",
                ),
            )
            for kind, key, stamp_key, info_key, optical_frame in streams:
                if key not in frame:
                    continue
                stamp = float(frame[stamp_key])
                suffix = "cameras/{}/{}/image_raw".format(name, kind)
                if stamp == self._last_stamps.get(suffix):
                    continue
                frame_id = name + "_" + optical_frame + "_optical_frame"
                message = self._image(
                    frame[key],
                    stamp,
                    frame_id,
                    frame.get("depth_scale_m") if kind != "color" else None,
                )
                self._emit(suffix, "image", message, message.header.stamp)
                info = self._camera_info(frame[info_key], stamp, frame_id)
                self._emit(
                    "cameras/{}/{}/camera_info".format(name, kind),
                    "info",
                    info,
                    info.header.stamp,
                )
                self._last_stamps[suffix] = stamp

    def publish_event(self, event):
        if not self.enabled:
            return
        event = json_value(event)
        stamp = self._time(event.get("timestamp", time.time()))
        message = self.messages["string"]()
        message.data = json.dumps(event, ensure_ascii=False, allow_nan=False)
        self._emit("events", "string", message, stamp)

    def publish_action(self, action, source, metadata=None):
        if not self.enabled:
            return
        metadata = dict(metadata or {})
        stamp = metadata.get("timestamp", metadata.get("stamp", time.time()))
        names = metadata.get("joint_names", self._joint_names)
        message = self._joint_message(action, stamp, names)
        self._emit("action", "joint", message, message.header.stamp)
        metadata.update(source=source, timestamp=stamp, action=json_value(action))
        details = self.messages["string"]()
        details.data = json.dumps(json_value(metadata), ensure_ascii=False, allow_nan=False)
        self._emit("action_metadata", "string", details, message.header.stamp)

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            if self._bag is not None:
                self._bag.close()
        finally:
            for publisher in self._publishers.values():
                publisher.unregister()
