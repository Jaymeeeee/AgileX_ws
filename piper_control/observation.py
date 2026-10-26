"""Build immutable, timestamped policy inputs without ROS dependencies."""
import numpy as np


def make_observation(config, snapshot, frames):
    state = np.asarray(snapshot["state"], dtype=np.float32)
    if state.shape != (7 * len(config["robot"]["arms"]),) or not np.isfinite(state).all():
        raise ValueError("Invalid follower state for policy observation")
    result = {"observation.state": state.copy(), "task": config["observation"]["task"],
              "timestamp": snapshot["stamp"]}
    for feature, name in config["observation"]["camera_map"].items():
        if name not in frames:
            raise ValueError("Missing camera: %s" % name)
        frame = frames[name]
        rgb = np.asarray(frame["rgb"])
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("Camera %s must supply HWC RGB uint8" % name)
        result[feature] = rgb.copy()
    return result
