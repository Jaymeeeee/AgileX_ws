"""Recording facade used by SDK workflows; ROS exists only in its child process."""
from .artifacts import ArtifactRecorder
from .client import CollectorClient


class RecordingSession:
    def __init__(self, full_cfg, artifact_factory=ArtifactRecorder, collector_factory=CollectorClient):
        self.config = full_cfg
        self.backend = full_cfg["record"].get("backend", "none")
        if self.backend not in ("none", "files", "ros", "ros_publish"):
            raise ValueError("record.backend must be none, files, ros, or ros_publish")
        self.artifacts = artifact_factory(full_cfg["record"], full_cfg) if self.backend in ("files", "ros") else None
        self.collector_factory = collector_factory
        self.collector = None
        self._started = False
        self._closed = False

    @property
    def directory(self):
        return self.artifacts.directory if self.artifacts is not None else None

    def start(self):
        if self._started or self._closed:
            raise RuntimeError("Recording session cannot be restarted")
        self._started = True
        try:
            if self.artifacts is not None:
                self.artifacts.start()
            if self.backend in ("ros", "ros_publish"):
                self.collector = self.collector_factory(self.config, self.directory)
                self.collector.start()
        except BaseException as exc:
            try:
                self.close(error=exc)
            except Exception:
                pass
            raise
        return self

    def write_step(self, row):
        if self.artifacts is not None:
            self.artifacts.write_step(row)

    def write_event(self, event):
        if self.artifacts is not None:
            self.artifacts.write_event(event)
        if self.collector is not None:
            # Full final frames belong in archival images and image topics; the
            # ROS event carries acquisition stamps and scalar/vector metadata.
            ros_event = event
            if "final_observation" in event:
                final = event["final_observation"]
                ros_event = dict(event)
                ros_event["final_observation"] = {key: value for key, value in final.items() if key != "frames"}
                ros_event["final_observation"]["camera_stamps"] = {
                    name: frame["stamp"] for name, frame in final.get("frames", {}).items()}
            self.collector.publish_event(ros_event)

    def publish_state(self, snapshot):
        if self.collector is not None:
            self.collector.publish_state(snapshot)

    def publish_images(self, frames):
        if self.collector is not None:
            self.collector.publish_images(frames)

    def publish_action(self, action, source, metadata=None):
        if self.collector is not None:
            self.collector.publish_action(action, source, metadata)

    def check_health(self):
        if self.artifacts is not None:
            self.artifacts.check_health()
        if self.collector is not None:
            self.collector.check_health()

    def close(self, error=None):
        if self._closed:
            return
        self._closed = True
        collection_error = None
        try:
            if self.collector is not None:
                self.collector.close()
        except Exception as exc:
            collection_error = exc
        finally:
            if self.artifacts is not None:
                self.artifacts.close(error=error if error is not None else collection_error)
        if collection_error is not None:
            raise collection_error
