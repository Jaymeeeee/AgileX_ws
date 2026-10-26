from pathlib import Path
import time
import unittest
from unittest import mock

import numpy as np

from piper_control.config import load_config
from piper_control.control.arbiter import Controller, ControlError
from piper_control.observation import make_observation
from piper_control.hardware.robot import MockRobot


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(
            Path(__file__).resolve().parents[1] / "configs" / "mock.yaml"
        )
        self.robot = MockRobot(self.cfg["robot"])
        self.robot.connect()
        self.controller = Controller(self.cfg, self.robot)
        self.controller.start()

    def tearDown(self):
        self.robot.close()

    def proposal(self, source="BASE"):
        self.controller.expect(1)
        action = self.robot.state.copy()
        action[0] = 2.0
        return dict(
            generation=self.controller.generation,
            seq=1,
            action=action,
            metadata={"source": source},
        )

    def test_stop_during_startup_prevents_initial_motion_target(self):
        for operation in ("enable", "snapshot"):
            with self.subTest(operation=operation):
                self.controller.abort.clear()
                original = getattr(self.robot, operation)

                def interrupted():
                    result = original()
                    self.controller.abort.set()
                    return result

                with mock.patch.object(self.robot, operation, side_effect=interrupted), mock.patch.object(
                    self.robot, "command", wraps=self.robot.command
                ) as command:
                    with self.assertRaisesRegex(ControlError, "Stop requested"):
                        self.controller.start()
                    command.assert_not_called()

    def test_intervention_drops_late_result_and_old_chunk(self):
        self.controller.enter_policy()
        response = self.proposal()
        self.controller.enter_human()
        self.assertFalse(self.controller.offer(response))
        self.controller.enter_policy()
        self.assertFalse(self.controller.offer(response))
        self.assertFalse(self.controller.actions)
        self.assertIsNone(self.controller.pending_seq)

    def test_missing_leader_rejects_takeover_without_invalidating_policy(self):
        for side in self.robot.arms:
            self.robot.config["pairs"][side]["leader"] = None
        self.controller.enter_policy()
        self.controller.offer(self.proposal())
        self.controller.expect(2)
        generation = self.controller.generation
        queued = list(self.controller.actions)
        events = []
        self.controller.event_sink = events.append
        with self.assertRaisesRegex(ControlError, "leader for every selected arm"):
            self.controller.enter_human()
        self.assertEqual(self.controller.state, Controller.POLICY)
        self.assertEqual(self.controller.generation, generation)
        self.assertEqual(self.controller.pending_seq, 2)
        self.assertEqual(len(self.controller.actions), len(queued))
        self.assertIs(self.controller.actions[0], queued[0])
        self.assertEqual(events, [])
        result = self.controller.step(self.robot.snapshot())
        self.assertEqual(result["action"][0], 2.0)

    def test_model_target_is_submitted_without_clipping_or_slew(self):
        self.controller.enter_policy()
        self.controller.offer(self.proposal())
        result = self.controller.step(self.robot.snapshot())
        self.assertEqual(result["predicted_action"][0], 2.0)
        self.assertEqual(result["action"][0], 2.0)
        np.testing.assert_array_equal(result["action"], result["predicted_action"])
        np.testing.assert_allclose(result["action"], self.robot.state)

    def test_invalid_source_is_rejected_before_hardware_send(self):
        self.controller.enter_policy()
        self.controller.offer(self.proposal(source="UNKNOWN"))
        before = self.robot.state.copy()
        with self.assertRaisesRegex(ControlError, "source"):
            self.controller.step(self.robot.snapshot())
        np.testing.assert_array_equal(before, self.robot.state)

    def test_rlt_reference_rows_follow_executed_chunk_and_wait_target(self):
        self.controller.enter_policy()
        response = self.proposal(source="RL")
        response["actions"] = np.stack([response.pop("action")] * 2)
        refs = np.arange(14, dtype=np.float32).reshape(2, 7)
        response["metadata"]["rlt_boundary"] = {"ref_chunk": refs}
        self.controller.offer(response)
        for expected in (refs[0], refs[1], refs[1]):
            result = self.controller.step(self.robot.snapshot())
            np.testing.assert_array_equal(result["metadata"]["rlt_ref_row"], expected)
        self.assertTrue(result["waiting_for_policy"])
        self.controller.pause()
        self.assertIsNone(self.controller.last_rlt_ref_row)

    def test_tracking_difference_does_not_block_new_targets(self):
        self.controller.enter_policy()
        self.controller.offer(self.proposal())
        snapshot = self.robot.snapshot()
        snapshot["state"][0] += 1.0
        result = self.controller.step(snapshot)
        self.assertEqual(result["action"][0], 2.0)

    def test_fault_stops_even_when_event_writer_fails(self):
        def broken(event):
            raise OSError("disk failed")

        self.controller.event_sink = broken
        with self.assertRaisesRegex(OSError, "disk"):
            self.controller.fail("test fault")
        self.assertEqual(self.robot.mode, "STOPPED")
        self.assertEqual(self.controller.state, Controller.FAULT)

    def test_observation_uses_cached_camera_without_retimestamping(self):
        now = time.monotonic()
        frame = {
            "rgb": np.zeros((48, 64, 3), np.uint8),
            "stamp": time.time() - 3,
            "monotonic": now - 3,
        }
        frames = {
            name: frame for name in self.cfg["observation"]["camera_map"].values()
        }
        snapshot = self.robot.snapshot()
        snapshot["monotonic"] = now - 3
        result = make_observation(self.cfg, snapshot, frames)
        self.assertEqual(result["timestamp"], snapshot["stamp"])


if __name__ == "__main__":
    unittest.main()
