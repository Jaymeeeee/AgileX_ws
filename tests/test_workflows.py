"""Workflow integration exercises actual SDK ownership and worker transport with fakes."""

import collections
import json
from pathlib import Path
import signal
import tempfile
import threading
import time
import unittest

import numpy as np

from piper_control.config import load_config
from piper_control.hardware.cameras import MockCameras
from piper_control.hardware.robot import MockRobot
from piper_control.recording.session import RecordingSession
from piper_control.workflows.session import Session

CONFIG = Path(__file__).resolve().parents[1] / "configs" / "mock.yaml"


class _Policy:
    def __init__(self):
        self.capabilities = {}
        self.replies = collections.deque()
        self.closed = False

    def load(self):
        return self

    def submit(self, observation, generation, seq):
        # A large target step should be passed through without software slew.
        predicted = np.full_like(observation["observation.state"], 0.4)
        self.replies.append(
            {
                "kind": "infer",
                "actions": np.stack([predicted] * 6),
                "generation": generation,
                "seq": seq,
                "metadata": {"source": "BASE"},
            }
        )

    def poll(self):
        return self.replies.popleft() if self.replies else None

    def invalidate(self, generation):
        self.replies.clear()

    def close(self):
        self.closed = True


class _Robot(MockRobot):
    def __init__(self, config, trace=None):
        super().__init__(config)
        self.trace = trace if trace is not None else []
        self.sent = []

    def enable(self):
        self.trace.append("robot.enable")
        return super().enable()

    def command(self, action):
        result = super().command(action)
        self.sent.append(result.copy())
        return result

    def enter_human(self):
        super().enter_human()
        self.set_human_action(self.state + 0.0005)

    def close(self):
        self.trace.append("robot.close")
        super().close()


class _Recording(RecordingSession):
    def __init__(self, config, trace=None):
        super().__init__(config)
        self.trace = trace if trace is not None else []
        self.actions = []
        self.events = []
        self.on_action = None

    def start(self):
        self.trace.append("record.start")
        return super().start()

    def publish_action(self, action, source, metadata):
        super().publish_action(action, source, metadata)
        self.actions.append(np.array(action))
        if self.on_action:
            self.on_action()

    def write_event(self, event):
        self.events.append(event)
        return super().write_event(event)

    def close(self, error=None):
        self.trace.append("record.close")
        return super().close(error)


class WorkflowTests(unittest.TestCase):
    def config(self, directory):
        return load_config(
            CONFIG,
            {
                "record": {"output_dir": directory, "min_free_gb": 0},
                "observation": {"width": 8, "height": 6},
            },
        )

    def rows(self, session):
        return [
            json.loads(row)
            for row in (session.recording.directory / "trajectory.jsonl")
            .read_text()
            .splitlines()
        ]

    def manifest(self, session):
        return json.loads((session.recording.directory / "manifest.json").read_text())

    def test_real_worker_keeps_step_cadence_and_stops_at_exact_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Session(self.config(directory), mock=True)
            result = session.run(max_steps=8, max_seconds=4.0, interactive=False)
            steps = [r for r in self.rows(session) if r["record_type"] == "step"]
            self.assertEqual(result["steps"], 8)
            self.assertEqual([r["seq"] for r in steps], list(range(1, 9)))
            self.assertFalse(any(r.get("waiting_for_policy", False) for r in steps))
            self.assertTrue(steps[-1]["truncated"])
            self.assertEqual(self.manifest(session)["status"], "complete")
            self.assertFalse(session.robot.enabled)
            self.assertIsNotNone(session.policy.process.poll())

    def test_follower_only_deployment_rejects_human_and_keeps_current_step(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg = self.config(directory)
            for side in cfg["robot"]["arms"]:
                cfg["robot"]["pairs"][side]["leader"] = None
            recording, policy = _Recording(cfg), _Policy()
            session = Session(cfg, mock=True, policy=policy, recording=recording)
            checked = []

            def attempt_takeover():
                if checked:
                    return
                pending = session.pending
                generation = session.controller.generation
                queued = list(session.controller.actions)
                episode = session.episode_id
                session.command(
                    "h", session.robot.snapshot(), session.cameras.snapshot()
                )
                self.assertIs(session.pending, pending)
                self.assertEqual(session.controller.state, "POLICY")
                self.assertEqual(session.controller.generation, generation)
                self.assertEqual(session.episode_id, episode)
                self.assertEqual(len(session.controller.actions), len(queued))
                self.assertTrue(
                    all(a is b for a, b in zip(session.controller.actions, queued))
                )
                checked.append(True)

            recording.on_action = attempt_takeover
            result = session.run(max_steps=3, max_seconds=2.0, interactive=False)
            self.assertEqual(checked, [True])
            self.assertEqual(result["steps"], 3)
            self.assertEqual(result["episodes"], 1)
            steps = [row for row in self.rows(session) if row["record_type"] == "step"]
            self.assertTrue(all(row["source"] == "BASE" for row in steps))
            rejected = [
                event
                for event in recording.events
                if event["type"] == "command_rejected"
            ]
            self.assertEqual(len(rejected), 1)
            self.assertEqual(rejected[0]["command"], "h")
            self.assertIn("leader", rejected[0]["reason"])
            self.assertEqual(self.manifest(session)["status"], "complete")

    def test_repeated_takeover_and_pause_preserve_actual_actions(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg = self.config(directory)
            robot, recording = _Robot(cfg["robot"]), _Recording(cfg)
            session = Session(
                cfg,
                mock=True,
                robot=robot,
                policy=_Policy(),
                recording=recording,
                command_schedule=[
                    (0.2, "h"),
                    (0.4, "p"),
                    (0.6, "hold"),
                    (0.8, "h"),
                    (1.0, "s"),
                    (1.1, "p"),
                    (1.4, "q"),
                ],
            )
            result = session.run(max_seconds=2.0, interactive=False)
            rows = self.rows(session)
            steps = [r for r in rows if r["record_type"] == "step"]
            self.assertEqual(result["episodes"], 2)
            self.assertEqual({r["source"] for r in steps}, {"BASE", "HUMAN"})
            self.assertEqual(len(recording.actions), len(steps))
            for row, actual in zip(steps, recording.actions):
                np.testing.assert_allclose(row["action"], actual)
                if row["source"] == "BASE":
                    self.assertTrue(
                        any(np.allclose(actual, sent) for sent in robot.sent)
                    )
                self.assertEqual(row["reward"], 0.0)
            endings = [r for r in rows if r.get("type") == "episode_end"]
            self.assertTrue(endings[0]["success"])
            self.assertTrue(endings[1]["truncated"])
            for row in endings:
                for frame in row["final_observation"]["frames"].values():
                    self.assertTrue((recording.directory / frame["rgb_path"]).is_file())

    def test_capture_and_policy_are_ready_before_enable_and_startup_failure_closes(
        self,
    ):
        with tempfile.TemporaryDirectory() as directory:
            cfg, trace = self.config(directory), []
            policy = _Policy()

            def fail_load():
                trace.append("policy.load")
                raise RuntimeError("checkpoint mismatch")

            policy.load = fail_load
            session = Session(
                cfg,
                mock=True,
                robot=_Robot(cfg["robot"], trace),
                policy=policy,
                recording=_Recording(cfg, trace),
            )
            with self.assertRaisesRegex(RuntimeError, "checkpoint mismatch"):
                session.run(max_seconds=1.0, interactive=False)
            self.assertNotIn("robot.enable", trace)
            self.assertLess(trace.index("record.start"), trace.index("policy.load"))
            self.assertLess(trace.index("robot.close"), trace.index("record.close"))
            self.assertTrue(policy.closed)
            self.assertFalse(session.cameras._started)
            self.assertEqual(self.manifest(session)["status"], "failed")

    def test_capture_failure_keeps_the_last_sent_action_without_inventing_next_state(
        self,
    ):
        with tempfile.TemporaryDirectory() as directory:
            cfg = self.config(directory)
            cameras, recording = MockCameras(cfg["observation"]), _Recording(cfg)
            original, failed = cameras.snapshot, [False]

            def snapshot():
                if failed[0]:
                    raise RuntimeError("camera disconnected")
                return original()

            cameras.snapshot = snapshot
            recording.on_action = lambda: failed.__setitem__(0, True)
            session = Session(
                cfg, mock=True, cameras=cameras, policy=_Policy(), recording=recording
            )
            with self.assertRaisesRegex(RuntimeError, "camera disconnected"):
                session.run(max_seconds=1.0, interactive=False)
            steps = [r for r in self.rows(session) if r["record_type"] == "step"]
            self.assertEqual(len(steps), 1)
            np.testing.assert_allclose(steps[0]["action"], recording.actions[0])
            self.assertIsNone(steps[0]["next_observation"])
            self.assertFalse(steps[0]["execution_complete"])
            self.assertFalse(session.robot.enabled)
            self.assertEqual(self.manifest(session)["status"], "failed")

    def test_refill_failure_also_preserves_the_action_sent_this_tick(self):
        class FailingRefill(_Policy):
            def __init__(self):
                super().__init__()
                self.calls = 0

            def submit(self, observation, generation, seq):
                self.calls += 1
                if self.calls > 1:
                    raise RuntimeError("refill queue full")
                super().submit(observation, generation, seq)
                self.replies[-1]["actions"] = self.replies[-1]["actions"][:1]

        with tempfile.TemporaryDirectory() as directory:
            cfg = self.config(directory)
            recording = _Recording(cfg)
            session = Session(
                cfg, mock=True, policy=FailingRefill(), recording=recording
            )
            with self.assertRaisesRegex(RuntimeError, "refill queue full"):
                session.run(max_seconds=1.0, interactive=False)
            steps = [r for r in self.rows(session) if r["record_type"] == "step"]
            self.assertEqual(len(steps), 1)
            np.testing.assert_allclose(steps[0]["action"], recording.actions[0])
            self.assertFalse(session.robot.enabled)

    def test_stop_while_loading_prevents_enable_and_restores_signal_handler(self):
        previous = signal.getsignal(signal.SIGTERM)
        with tempfile.TemporaryDirectory() as directory:
            cfg, trace, policy = self.config(directory), [], _Policy()

            def stopped_load():
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
                return policy

            policy.load = stopped_load
            session = Session(
                cfg, mock=True, robot=_Robot(cfg["robot"], trace), policy=policy
            )
            with self.assertRaisesRegex(RuntimeError, "Stop requested"):
                session.run(max_seconds=1.0, interactive=False)
            self.assertNotIn("robot.enable", trace)
            self.assertFalse(session._stop_thread.is_alive())
        self.assertEqual(signal.getsignal(signal.SIGTERM), previous)

    def test_collection_refreshes_state_after_slow_human_handover(self):
        class SlowRobot(_Robot):
            def enter_human(self):
                time.sleep(0.3)
                return super().enter_human()

        with tempfile.TemporaryDirectory() as directory:
            cfg = self.config(directory)
            cfg["workflow"]["mode"] = "collect"
            session = Session(cfg, mock=True, robot=SlowRobot(cfg["robot"]))
            result = session.run(max_steps=2, max_seconds=2.0, interactive=False)
            self.assertEqual(result["steps"], 2)
            self.assertIsNone(session.policy)
            self.assertTrue(
                all(
                    r["source"] == "HUMAN"
                    for r in self.rows(session)
                    if r["record_type"] == "step"
                )
            )

    def test_quit_does_not_execute_a_queued_new_episode(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Session(
                self.config(directory),
                mock=True,
                policy=_Policy(),
                command_schedule=[(0.2, "q"), (0.2, "p")],
            )
            result = session.run(max_seconds=1.0, interactive=False)
            self.assertEqual(result["episodes"], 1)
            self.assertEqual(
                sum(r.get("type") == "episode_start" for r in self.rows(session)), 1
            )

    def test_paused_update_uses_same_timeout_override_as_policy_client(self):
        for timeout, failed in ((10.0, False), (0.01, True)):
            with self.subTest(
                timeout=timeout
            ), tempfile.TemporaryDirectory() as directory:
                cfg = self.config(directory)
                cfg["workflow"].update(
                    auto_start=False, rlt={"update_timeout_sec": 0.01}
                )
                cfg["policy"]["update_timeout_sec"] = timeout
                policy = _Policy()
                session = Session(
                    cfg, mock=True, policy=policy, command_schedule=[(0.04, "q")]
                )
                session.learning_pending = True
                session.learning_started = time.monotonic() - 0.2
                polls = []

                def delayed_ack():
                    polls.append(1)
                    if len(polls) == 3:
                        return {"kind": "end_episode", "result": {"updates": 1}}
                    return None

                policy.poll = delayed_ack
                if failed:
                    with self.assertRaisesRegex(RuntimeError, "RLT update timed out"):
                        session.run(max_seconds=1.0, interactive=False)
                    self.assertEqual(self.manifest(session)["status"], "failed")
                else:
                    result = session.run(max_seconds=1.0, interactive=False)
                    self.assertEqual(result["steps"], 0)
                    self.assertFalse(session.learning_pending)
                self.assertFalse(session.robot.enabled)

    def test_learning_replans_segments_references_and_paused_episode_end(self):
        class LearningPolicy(_Policy):
            def __init__(self):
                super().__init__()
                self.capabilities = {"episode_id": 7}
                self.boundaries = []
                self.executed = []
                self.endings = []
                self.references = {}
                self.session = None

            def submit(self, observation, generation, seq):
                # Every replan carries distinctive references. They must follow
                # the actions actually consumed, including an interrupted chunk.
                refs = np.stack(
                    [
                        np.full(7, seq * 10 + index, dtype=np.float32)
                        for index in range(2)
                    ]
                )
                self.references[seq] = refs.copy()
                self.replies.append(
                    {
                        "kind": "infer",
                        "generation": generation,
                        "seq": seq,
                        "actions": np.full((2, 7), 0.4, dtype=np.float32),
                        "metadata": {
                            "source": "RL",
                            "rlt_boundary": {
                                "ref_chunk": refs,
                                "z_rl": np.array([seq], dtype=np.float32),
                                "proprio": observation["observation.state"].copy(),
                            },
                        },
                    }
                )

            def invalidate(self, generation):
                self.replies = collections.deque(
                    reply for reply in self.replies if reply.get("kind") != "infer"
                )
                self.replies.append({"kind": "reset"})

            def boundary(self, observation, episode_id, step_id, segment_start=False):
                self.boundaries.append(
                    {
                        "episode_id": episode_id,
                        "step_id": step_id,
                        "segment_start": segment_start,
                        "state": observation["observation.state"].copy(),
                    }
                )
                self.replies.append({"kind": "boundary"})

            def record_step(
                self,
                action,
                source,
                intervention=False,
                reward=0.0,
                next_observation=None,
                ref_row=None,
            ):
                self.executed.append(
                    {
                        "action": np.array(action),
                        "source": source,
                        "ref_row": None if ref_row is None else np.array(ref_row),
                        "next_state": next_observation["observation.state"].copy(),
                    }
                )
                self.replies.append({"kind": "record_step"})

            def end_episode(self, observation, **flags):
                self.endings.append(
                    {
                        "state": self.session.controller.state,
                        "observation": observation,
                        "flags": flags,
                    }
                )
                self.replies.append({"kind": "end_episode", "result": {"updates": 1}})

        with tempfile.TemporaryDirectory() as directory:
            config = self.config(directory)
            config["policy"]["type"] = "pi05_rlt"
            config["workflow"].update(mode="online_rl", learner="pi05_rlt")
            policy = LearningPolicy()
            schedule = [
                (0.55, "h"),
                (0.75, "hold"),
                (0.95, "h"),
                (1.15, "hold"),
                (1.35, "s"),
                (1.45, "q"),
            ]
            session = Session(
                config,
                mock=True,
                robot=_Robot(config["robot"]),
                policy=policy,
                command_schedule=schedule,
            )
            policy.session = session
            result = session.run(max_seconds=2.0, interactive=False)
            rows = self.rows(session)
            steps = [row for row in rows if row["record_type"] == "step"]
            self.assertEqual(result["episodes"], 8)
            self.assertEqual([row["step_id"] for row in steps], list(range(len(steps))))
            self.assertTrue(all(row["episode_id"] == 7 for row in steps))
            self.assertEqual(len(policy.executed), len(steps))
            self.assertTrue(
                any(
                    row["execution_boundary"] and not row["segment_start"]
                    for row in steps
                    if row["source"] == "RL"
                )
            )

            expected_boundaries = [row for row in steps if row["execution_boundary"]]
            self.assertEqual(
                [boundary["step_id"] for boundary in policy.boundaries],
                [row["step_id"] for row in expected_boundaries],
            )
            for boundary, row in zip(policy.boundaries, expected_boundaries):
                self.assertEqual(boundary["segment_start"], row["segment_start"])
                np.testing.assert_allclose(
                    boundary["state"], row["observation"]["state"]
                )
            human = [row for row in steps if row["source"] == "HUMAN"]
            self.assertEqual(len({row["generation"] for row in human}), 2)
            self.assertEqual(sum(row["segment_start"] for row in human), 2)
            for generation in {row["generation"] for row in human}:
                segment = [row for row in human if row["generation"] == generation]
                self.assertTrue(
                    segment[0]["segment_start"] and segment[0]["execution_boundary"]
                )
                self.assertFalse(any(row["segment_start"] for row in segment[1:]))

            for recorded, accepted in zip(steps, policy.executed):
                np.testing.assert_allclose(recorded["action"], accepted["action"])
                np.testing.assert_allclose(
                    recorded["next_observation"]["state"], accepted["next_state"]
                )
                self.assertEqual(recorded["source"], accepted["source"])
                if recorded["source"] == "RL":
                    expected_ref = policy.references[recorded["seq"]][
                        recorded["chunk_index"]
                    ]
                    np.testing.assert_array_equal(accepted["ref_row"], expected_ref)
                    np.testing.assert_array_equal(
                        recorded["metadata"]["rlt_ref_row"], expected_ref
                    )
                else:
                    self.assertIsNone(accepted["ref_row"])

            self.assertEqual(len(policy.endings), 1)
            self.assertEqual(policy.endings[0]["state"], "HOLD")
            self.assertTrue(policy.endings[0]["flags"]["terminal"])
            self.assertFalse(policy.endings[0]["flags"]["truncated"])
            ending = next(row for row in rows if row.get("type") == "episode_end")
            final = ending["final_observation"]
            np.testing.assert_allclose(
                final["state"], policy.endings[0]["observation"]["observation.state"]
            )
            self.assertGreater(final["stamp"], steps[-1]["next_observation"]["stamp"])
            for frame in final["frames"].values():
                self.assertIn("rgb_path", frame)
                self.assertNotIn("rgb", frame)
                self.assertTrue(
                    (session.recording.directory / frame["rgb_path"]).is_file()
                )
            self.assertEqual(self.manifest(session)["status"], "complete")


if __name__ == "__main__":
    unittest.main()
