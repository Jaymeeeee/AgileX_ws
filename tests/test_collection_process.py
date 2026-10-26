import io
import json
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest import mock

import numpy as np

from piper_control.ipc import _decode, read_message, write_message
from piper_control.recording.client import CollectionError, CollectorClient
from piper_control.recording.session import RecordingSession
from piper_control.recording.ros_worker import serve


ROOT = Path(__file__).resolve().parents[1]
FAKE_WORKER = '''
import sys, json, signal, time
from pathlib import Path
sys.path.insert(0, ROOT_PATH)
from piper_control.ipc import _encode
from piper_control.recording.ros_worker import serve
class Bridge:
    def __init__(self, cfg): self.path=None
    def start(self): pass
    def start_bag(self, directory):
        self.path=Path(directory)
        self.log('ready', {})
    def log(self, kind, payload):
        with (self.path/'collector.audit.jsonl').open('a') as f:
            f.write(json.dumps({'kind':kind, 'payload':payload}, default=_encode)+'\\n')
    def publish_state(self, data): self.log('state', data)
    def publish_images(self, data): self.log('images', data)
    def publish_action(self, action, source, metadata): self.log('action', {'action':action,'source':source})
    def publish_event(self, data):
        if data.get('type')=='fail': raise OSError('collector disk failed')
        if data.get('type')=='block': time.sleep(10)
        self.log('event',data)
    def check_health(self): pass
    def close(self):
        manifest=json.loads((self.path/'manifest.json').read_text()) if (self.path/'manifest.json').exists() else {}
        self.log('closed', {'manifest_status':manifest.get('status'),
            'hardware_imports':[m for m in sys.modules if m.startswith('piper_control.hardware')]})
def stop(*args): raise SystemExit(143)
signal.signal(signal.SIGTERM,stop)
serve(sys.stdin.buffer,sys.stdout.buffer,bridge_factory=Bridge)
'''


class CollectionProcessTests(unittest.TestCase):
    def config(self, root, backend="ros"):
        return {"record": {"backend": backend, "output_dir": root, "min_free_gb": 0,
                            "python": sys.executable, "ros_setup": None, "queue_size": 16,
                            "startup_timeout_sec": 3., "command_timeout_sec": 2., "shutdown_timeout_sec": 2.},
                "observation": {}}

    def factory(self, root):
        script = Path(root) / "fake_collector.py"
        script.write_text(FAKE_WORKER.replace("ROOT_PATH", repr(str(ROOT))))
        def factory(cfg, directory):
            client = CollectorClient(cfg, directory)
            client._argv = lambda: [sys.executable, str(script)]
            return client
        return factory

    def test_process_owns_snapshots_deduplicates_images_and_closes_before_manifest(self):
        with tempfile.TemporaryDirectory() as root:
            session = RecordingSession(self.config(root), collector_factory=self.factory(root)).start()
            state = np.arange(7, dtype=np.float32)
            frame = {"rgb": np.full((2, 3, 3), 17, dtype=np.uint8),
                     "stamp": time.time(), "monotonic": time.monotonic()}
            session.publish_state({"state": state})
            session.publish_images({"top": frame})
            session.publish_images({"top": frame})
            session.publish_action(state, "HUMAN", {"timestamp": frame["stamp"]})
            session.write_event({"type": "episode_end", "final_observation": {
                "state": state, "stamp": frame["stamp"], "frames": {"top": frame}}})
            state[:] = 99
            frame["rgb"][:] = 0
            session.close()
            audit = [json.loads(line, object_hook=_decode) for line in
                     (session.directory / "collector.audit.jsonl").read_text().splitlines()]
            self.assertEqual([row["kind"] for row in audit], ["ready", "state", "images", "action", "event", "closed"])
            np.testing.assert_array_equal(audit[1]["payload"]["state"], np.arange(7))
            self.assertTrue(np.all(audit[2]["payload"]["top"]["rgb"] == 17))
            self.assertNotIn("frames", audit[4]["payload"]["final_observation"])
            self.assertEqual(audit[-1]["payload"]["manifest_status"], "recording")
            self.assertEqual(audit[-1]["payload"]["hardware_imports"], [])
            self.assertEqual(json.loads((session.directory / "manifest.json").read_text())["status"], "complete")
            self.assertEqual(session.collector.process.returncode, 0)

    def test_sidecar_error_fails_session_instead_of_marking_complete(self):
        with tempfile.TemporaryDirectory() as root:
            session = RecordingSession(self.config(root), collector_factory=self.factory(root)).start()
            session.write_event({"type": "fail"})
            session.collector._writer_thread.join(timeout=3.)
            with self.assertRaises(CollectionError):
                session.check_health()
            with self.assertRaises(CollectionError):
                session.close()
            manifest = json.loads((session.directory / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "failed")
            self.assertIn("failed", manifest["error"])

    def test_aligned_depth_timestamp_change_reaches_collector(self):
        with tempfile.TemporaryDirectory() as root:
            client = CollectorClient(self.config(root), root)
            client._enqueue = mock.Mock()
            frame = {"stamp": 100.0, "depth_stamp": 99.9, "aligned_depth_stamp": 99.8}
            client.publish_images({"top": frame})
            client._enqueue.assert_called_once_with("images", {"top": frame})
            client._enqueue.reset_mock()
            client.publish_images({"top": frame})
            client._enqueue.assert_not_called()
            frame = dict(frame, aligned_depth_stamp=99.81)
            client.publish_images({"top": frame})
            client._enqueue.assert_called_once_with("images", {"top": frame})

    def test_bounded_queue_refuses_to_drop_inputs(self):
        with tempfile.TemporaryDirectory() as root:
            cfg = self.config(root)
            cfg["record"]["queue_size"] = 1
            client = CollectorClient(cfg, root)
            client.process = types.SimpleNamespace(poll=lambda: None)
            client.publish_event({"value": 1})
            with self.assertRaisesRegex(CollectionError, "queue is full"):
                client.publish_event({"value": 2})
            self.assertEqual(client.queue.qsize(), 1)

    def test_worker_rejects_motion_messages_and_closes_collector(self):
        incoming, outgoing = io.BytesIO(), io.BytesIO()
        bridge = mock.Mock()
        write_message(incoming, {"kind": "start", "seq": 0, "config": {}, "directory": "/tmp"})
        write_message(incoming, {"kind": "move_joints", "seq": 1, "payload": [0.] * 7,
                                 "captured_epoch": 10., "captured_monotonic": 1.})
        incoming.seek(0)
        with mock.patch("piper_control.recording.ros_worker.traceback.print_exc"):
            serve(incoming, outgoing, bridge_factory=lambda cfg: bridge)
        outgoing.seek(0)
        self.assertEqual(read_message(outgoing)["kind"], "ready")
        self.assertIn("Unsupported collection input", read_message(outgoing)["error"])
        bridge.close.assert_called_once()
        bridge.publish_action.assert_not_called()

    def test_ros_setup_paths_are_positional_arguments(self):
        with tempfile.TemporaryDirectory() as root:
            setup = Path(root) / "ros $(not-a-command).bash"
            setup.write_text(":\n")
            cfg = self.config(root)
            cfg["record"]["ros_setup"] = str(setup)
            cfg["record"]["python"] = "/path with spaces/python"
            argv = CollectorClient(cfg, root)._argv()
            self.assertNotIn(str(setup), argv[4])
            self.assertEqual(argv[-2:], [str(setup), "/path with spaces/python"])
            self.assertEqual(argv[4], 'source "$1" && exec "$2" -m piper_control.recording.ros_worker')

    def test_none_and_files_never_start_collector(self):
        with tempfile.TemporaryDirectory() as root:
            factory = mock.Mock(side_effect=AssertionError("ROS process must stay absent"))
            none = RecordingSession(self.config(root, "none"), collector_factory=factory).start()
            none.publish_state({})
            none.write_event({"type": "test"})
            none.close()
            self.assertIsNone(none.directory)
            files = RecordingSession(self.config(root, "files"), collector_factory=factory).start()
            files.write_event({"type": "test"})
            files.close()
            factory.assert_not_called()
            self.assertEqual(json.loads((files.directory / "manifest.json").read_text())["status"], "complete")


if __name__ == "__main__":
    unittest.main()
