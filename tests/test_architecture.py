"""Executable boundary checks for the replacement SDK framework."""
import ast
import importlib.abc
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
ROS_MODULES = {"rospy", "rosbag", "roslib", "sensor_msgs", "std_msgs", "diagnostic_msgs",
               "geometry_msgs", "cv_bridge", "rosgraph", "rospkg"}


class ArchitectureTests(unittest.TestCase):
    def test_ros_imports_are_confined_to_collection_worker_layer(self):
        for path in (ROOT / "piper_control").rglob("*.py"):
            if path.parent.name == "recording":
                continue
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                names = ([item.name for item in node.names] if isinstance(node, ast.Import)
                         else [node.module or ""] if isinstance(node, ast.ImportFrom) else [])
                self.assertFalse(any(name.split(".")[0] in ROS_MODULES for name in names), str(path))
                self.assertFalse(any("ros_bridge" in name or "ros_worker" in name for name in names), str(path))

    def test_active_repository_has_no_ros_packages_or_parallel_legacy_framework(self):
        self.assertFalse((ROOT / "src").exists())
        self.assertFalse((ROOT / "hil_rl").exists())
        self.assertFalse((ROOT / ".catkin_workspace").exists())
        for base in ("piper_control", "scripts", "configs"):
            self.assertFalse(list((ROOT / base).rglob("package.xml")))
            self.assertFalse(list((ROOT / base).rglob("CMakeLists.txt")))

    def test_deploy_and_teleop_run_when_all_ros_imports_are_blocked(self):
        code = '''
import importlib.abc, sys, tempfile
from pathlib import Path
ROS = %r
class DenyROS(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ROS:
            raise RuntimeError('ROS dependency escaped collection: ' + fullname)
sys.meta_path.insert(0, DenyROS())
from piper_control.config import load_config
from piper_control.workflows.session import Session
for mode in ('deploy', 'teleop'):
    with tempfile.TemporaryDirectory() as output:
        cfg = load_config('configs/mock.yaml', {'workflow': {'mode': mode}, 'record': {'backend': 'none', 'output_dir': output}})
        session = Session(cfg, mock=True)
        result = session.run(max_steps=3, max_seconds=3., interactive=False)
        assert result['steps'] == 3 and result['directory'] is None
        assert not list(Path(output).iterdir())
        assert 'piper_control.recording.ros_bridge' not in sys.modules
        assert 'piper_control.recording.ros_worker' not in sys.modules
        if mode == 'teleop':
            assert session.policy is None
''' % sorted(ROS_MODULES)
        result = subprocess.run([sys.executable, "-c", code], cwd=str(ROOT), text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15.)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
