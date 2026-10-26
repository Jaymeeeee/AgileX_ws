import errno
import struct
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from piper_control.hardware.can import CANForwarder, JOINT_UNIT, TARGET_IDS


def frame(ident, data=None, stamp=None, **kwargs):
    values = dict(arbitration_id=ident, data=bytes(data or [0] * 8),
                  timestamp=time.time() if stamp is None else stamp,
                  is_rx=True, is_extended_id=False, is_error_frame=False,
                  is_remote_frame=False, is_fd=False)
    values.update(kwargs)
    return SimpleNamespace(**values)


class FakeBus:
    def __init__(self): self.sent, self.is_closed = [], False
    def send(self, value, timeout=None): self.sent.append(value)
    def shutdown(self): self.is_closed = True


class ForwarderTests(unittest.TestCase):
    def setUp(self):
        self.bridge = CANForwarder("can1", "can2")
        self.bridge.rx, self.bridge.tx = FakeBus(), FakeBus()

    def test_only_route_ids_and_physical_rx_are_forwarded(self):
        self.bridge.activate()
        for ident in (0x470, 0x471, 0x150, 0x152, 0x153, 0x154, 0x158, 0x191, 0x2A5):
            self.assertFalse(self.bridge.forward(frame(ident)))
        self.assertFalse(self.bridge.forward(frame(0x155, is_rx=False)))
        self.assertFalse(self.bridge.forward(frame(0x155, is_extended_id=True)))
        self.assertEqual(self.bridge.tx.sent, [])

    def test_payload_and_timestamp_are_forwarded_without_age_or_position_checks(self):
        self.bridge.activate()
        mode = frame(0x151, [1, 2, 100, 0, 0, 0, 0, 0], stamp=100.)
        target = frame(0x155, struct.pack(">ii", 1800000, -1800000), stamp=100.)
        gripper = frame(0x159, struct.pack(">iHBB", 200000, 1000, 1, 0), stamp=100.)
        for packet in (mode, target, gripper):
            self.assertTrue(self.bridge.forward(packet))
        self.assertEqual(self.bridge.tx.sent, [mode, target, gripper])
        self.assertIs(self.bridge.tx.sent[0], mode)

    def test_pause_prevents_inflight_old_receive_from_new_handover(self):
        self.bridge.activate()
        generation = self.bridge.generation
        self.bridge.deactivate()
        self.assertFalse(self.bridge.forward(frame(0x155)))
        self.bridge.activate()
        self.assertFalse(self.bridge.forward(frame(0x155), generation))
        self.assertTrue(self.bridge.forward(frame(0x155), self.bridge.generation))
        self.assertEqual(len(self.bridge.tx.sent), 1)

    def test_recorded_action_is_the_last_complete_command_with_source_timestamps(self):
        self.bridge.activate()
        for ident in TARGET_IDS[:3]:
            self.bridge.forward(frame(ident, struct.pack(">ii", 1000, -1000), stamp=100.))
        self.assertIsNone(self.bridge.target())
        self.bridge.forward(frame(0x159, struct.pack(">iHBB", 25000, 1000, 1, 0), stamp=101.))
        target = self.bridge.target()
        np.testing.assert_allclose(target["position"], [1000 * JOINT_UNIT, -1000 * JOINT_UNIT] * 3 + [.025])
        self.assertEqual((target["sequence"], target["stamp"], target["source"]),
                         (1, 100., "forwarded_command"))
        self.bridge.forward(frame(0x155))
        self.assertEqual(self.bridge.target()["sequence"], 1)
        self.bridge.deactivate()
        self.assertIsNone(self.bridge.target())

    def test_send_exception_propagates_without_recording_failed_action(self):
        self.bridge.activate()
        def failed_send(*args, **kwargs): raise OSError("CAN send failed")
        self.bridge.tx.send = failed_send
        with self.assertRaises(OSError): self.bridge.forward(frame(0x155))
        self.assertIsNone(self.bridge.target())

    def test_background_receive_exception_is_reported_to_the_caller(self):
        def failed_recv(timeout): raise OSError("CAN receive failed")
        self.bridge.rx.recv = failed_recv
        self.bridge.activate()
        self.bridge._run()
        self.assertFalse(self.bridge.active)
        with self.assertRaisesRegex(RuntimeError, "CAN receive failed"):
            self.bridge.target()

    def test_pause_waits_for_current_send_then_closes_the_route(self):
        started, release, ended = threading.Event(), threading.Event(), threading.Event()
        def send(value, timeout=None):
            started.set()
            self.assertTrue(release.wait(.5))
            self.bridge.tx.sent.append(value)
        self.bridge.tx.send = send
        self.bridge.activate()
        tx = threading.Thread(target=self.bridge.forward, args=(frame(0x155), self.bridge.generation))
        tx.start()
        self.assertTrue(started.wait(.5))
        closer = threading.Thread(target=lambda: (self.bridge.deactivate(), ended.set()))
        closer.start()
        self.assertFalse(ended.wait(.01))
        release.set()
        tx.join(.5)
        closer.join(.5)
        self.assertTrue(ended.is_set())
        self.assertFalse(self.bridge.active)
        rx, tx = self.bridge.rx, self.bridge.tx
        self.bridge.close()
        self.assertTrue(rx.is_closed and tx.is_closed)

    def test_transient_enobufs_retries_exact_frame_before_counting_success(self):
        for code in (errno.ENOBUFS, errno.EAGAIN):
            with self.subTest(code=code):
                self.setUp()
                self.bridge.activate()
                packet = frame(0x155, struct.pack(">ii", 1000, -2000))
                attempts = []
                error = RuntimeError("python-can wrapped syscall error")
                error.error_code = code

                def send(value, timeout=None):
                    self.assertGreater(timeout, 0)
                    self.assertLessEqual(timeout, .01)
                    attempts.append(value)
                    if len(attempts) == 1:
                        self.assertEqual(self.bridge.parts, {})
                        raise error
                    self.bridge.tx.sent.append(value)

                self.bridge.tx.send = send
                with patch("piper_control.hardware.can.time.sleep"):
                    self.assertTrue(self.bridge.forward(packet))
                self.assertEqual(attempts, [packet, packet])
                self.assertIs(attempts[0], attempts[1])
                self.assertEqual(self.bridge.sent_frames, 1)
                self.assertEqual(self.bridge.send_retries, 1)
                self.assertEqual(self.bridge.tx.sent, [packet])
                self.assertIsNone(self.bridge.target())

    def test_persistent_congestion_is_bounded_and_does_not_complete_action(self):
        self.bridge.activate()
        for ident in TARGET_IDS[:3]:
            self.bridge.forward(frame(ident))
        clock = [100.0]
        attempts = []

        def blocked(value, timeout=None):
            attempts.append(value)
            raise OSError(errno.ENOBUFS, "No buffer space available")

        self.bridge.tx.send = blocked
        def advance(seconds): clock[0] += seconds
        with patch("piper_control.hardware.can.time.monotonic", side_effect=lambda: clock[0]), patch(
            "piper_control.hardware.can.time.sleep", side_effect=advance
        ):
            with self.assertRaisesRegex(RuntimeError, "can2.*congested for 10 ms"):
                self.bridge.forward(frame(0x159))
        self.assertLessEqual(clock[0] - 100.0, .010001)
        self.assertLessEqual(len(attempts), 21)
        self.assertEqual(self.bridge.sent_frames, 3)
        self.assertNotIn(0x159, self.bridge.parts)
        self.assertIsNone(self.bridge.target())

    def test_network_down_is_not_retried(self):
        self.bridge.activate()
        with patch.object(self.bridge.tx, "send", side_effect=OSError(errno.ENETDOWN, "down")) as send:
            with self.assertRaises(OSError): self.bridge.forward(frame(0x155))
        send.assert_called_once()
        self.assertEqual(self.bridge.send_retries, 0)
        self.assertEqual(self.bridge.parts, {})

    def test_close_interrupts_congestion_without_retrying_or_recording(self):
        self.bridge.activate()
        def close_during_send(value, timeout=None):
            self.bridge.closed.set()
            raise OSError(errno.ENOBUFS, "full")
        with patch.object(self.bridge.tx, "send", side_effect=close_during_send) as send:
            self.assertFalse(self.bridge.forward(frame(0x155)))
        send.assert_called_once()
        self.assertEqual(self.bridge.sent_frames, 0)
        self.assertEqual(self.bridge.send_retries, 0)
        self.assertEqual(self.bridge.parts, {})


if __name__ == "__main__":
    unittest.main()
