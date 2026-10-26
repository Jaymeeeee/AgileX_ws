"""Control loop: SDK motion and intervention remain independent of inference."""

import queue
import select
import signal
import sys
import threading
import time

from ..hardware.cameras import CameraRig, MockCameras
from ..control.arbiter import Controller, ControlError
from ..config import component_names
from ..observation import make_observation
from ..policies.client import PolicyClient
from ..recording.session import RecordingSession
from ..hardware.robot import SDKRobot, MockRobot
from .definitions import get_workflow


def _stored_observation(snapshot, frames):
    return {
        "state": snapshot["state"].copy(),
        "stamp": snapshot["stamp"],
        "monotonic": snapshot["monotonic"],
        "frames": frames,
    }


class Session:
    """Compose SDK ownership, policy workers and an optional collection sink."""

    def __init__(
        self,
        config,
        mock=False,
        command_schedule=None,
        robot=None,
        cameras=None,
        policy=None,
        recording=None,
    ):
        self.cfg, self.mock = config, mock
        self.workflow = get_workflow(config["workflow"]["mode"])
        self.robot = robot or (MockRobot if mock else SDKRobot)(config["robot"])
        self.cameras = cameras or (MockCameras if mock else CameraRig)(
            config["observation"]
        )
        self.policy = policy
        self.recording = recording or RecordingSession(config)
        self.controller = Controller(config, self.robot, self.event)
        self.commands = queue.Queue()
        self.schedule = sorted(command_schedule or [])
        self.pending = None
        self.episode_id = 0
        self.step_id = 0
        self.total_steps = 0
        self.episode_active = False
        self.episode_started = None
        self.needs_boundary = True
        self.learning = self.workflow.learns
        self.learning_pending = False
        self.learning_started = None
        self.learning_error = None
        self.seq = 0
        self.quit = False
        self.started = False
        self.last_observation = None
        self._human_action_ready = False
        self._signals = {}
        self._stop_wakeup = threading.Event()
        self._closing = threading.Event()
        self._stop_thread = None
        self._stop_error = None

    def event(self, event):
        self.recording.write_event(event)
        kind = event.get("type")
        if kind == "mode":
            self._human_action_ready = False
            descriptions = {
                Controller.HUMAN: "主臂模式和转发已开启，等待主臂控制数据",
                Controller.POLICY: "模型控制已开启",
                Controller.HOLD: "已暂停并保持姿态",
            }
            state = event["control_state"]
            print("[控制] {}：{}".format(state, descriptions.get(state, state)), flush=True)
        elif kind in ("command_rejected", "fault"):
            print("[控制] {}：{}".format(kind, event.get("reason", "")), flush=True)

    def _log(self, kind, **fields):
        self.event(
            dict(type=kind, timestamp=time.time(), episode_id=self.episode_id, **fields)
        )

    def start(self):
        self.recording.start()
        self.cameras.start()
        if self.workflow.uses_policy:
            if self.learning and self.recording.directory is not None:
                options = self.cfg["workflow"].setdefault("rlt", {})
                options.setdefault(
                    "output_dir", str(self.recording.directory / "training")
                )
            self.policy = self.policy or PolicyClient(self.cfg)
            self.policy.load()
            capabilities = getattr(self.policy, "capabilities", {})
            self.episode_id = int(capabilities.get("episode_id", 0))
        self.robot.connect()
        manual_auto = self.cfg["workflow"].get("auto_start", False) and not self.workflow.uses_policy
        if manual_auto:
            if self.controller.abort.is_set():
                raise ControlError("Stop requested before enabling")
            print("[启动] 将主臂切回 follower 以读取状态，检查驱动并按需恢复电子停止。", flush=True)
            self.robot.prepare_human_start(stop_requested=self.controller.abort.is_set)
            print("[启动] 恢复检查已完成，正在使能并发送 JS 对齐目标。", flush=True)
        # Load process resources before starting SDK control.
        self.controller.start()
        self.started = True
        if manual_auto:
            print("[启动] 使能已确认，已发送一次 move_js()，正在等待运动稳定。", flush=True)
            alignment = self.robot.wait_for_alignment(
                self.controller.target, stop_requested=self.controller.abort.is_set
            )
            if alignment:
                self._log("startup_alignment", **alignment)
                print("[启动] 主从当前姿态差（不要求精确到位）：{}".format(alignment["pair_errors"]), flush=True)
                print("[启动] 相对初始目标的偏差（诊断）：{}".format(alignment["target_errors"]), flush=True)
            print("[启动] JS 运动已稳定，正在自动进入主从控制。", flush=True)
        self._log(
            "session_ready",
            mock=self.mock,
            policy_type=self.cfg["policy"]["type"],
            arms=self.cfg["robot"]["arms"],
            action_names=component_names(self.cfg["robot"]["arms"]),
            policy_capabilities=getattr(self.policy, "capabilities", None),
        )
        return self

    def _invalidate_policy(self):
        if self.policy:
            self.policy.invalidate(self.controller.generation)
        self.needs_boundary = True

    def _finish_step(
        self, snapshot, frames, terminal=False, truncated=False, reward=0.0
    ):
        if self.pending is None:
            return
        step = dict(self.pending)
        next_obs = make_observation(self.cfg, snapshot, frames)
        step["next_observation"] = _stored_observation(snapshot, frames)
        step["duration_sec"] = max(0.0, time.monotonic() - step.pop("_started"))
        before_obs = step.pop("_policy_observation")
        requires_boundary = step.pop("_boundary_required")
        segment_start = step.pop("_segment_start")
        step["execution_boundary"] = requires_boundary
        step["segment_start"] = segment_start
        # Sparse task reward is stored once, in episode_end. Zero here avoids
        # counting it again when replay is reconstructed offline.
        step.update(terminated=bool(terminal), truncated=bool(truncated), reward=0.0)
        self.recording.write_step(step)
        self.pending = None
        if self.learning:
            if requires_boundary:
                # The learner receives the real execution boundary. Its default
                # collector defers costly VLA encoding until the robot is held.
                self.policy.boundary(
                    before_obs,
                    self.episode_id,
                    self.step_id,
                    segment_start=segment_start,
                )
            self.policy.record_step(
                step["action"],
                step["source"],
                intervention=step["intervention"],
                reward=0.0,
                next_observation=next_obs,
                ref_row=step.get("metadata", {}).get("rlt_ref_row"),
            )
        self.step_id += 1
        self.total_steps += 1

    def _begin_episode(self):
        if not self.episode_active:
            self.episode_active = True
            self.episode_started = time.monotonic()
            self.step_id = 0
            self.needs_boundary = True
            self._log("episode_start")

    def command(self, command, snapshot, frames):
        command = command.strip().lower()
        if command in ("p", "policy", "h", "human"):
            destination = (
                Controller.POLICY if command in ("p", "policy") else Controller.HUMAN
            )
            if destination == Controller.HUMAN and not self.robot.supports_human:
                self._log(
                    "command_rejected",
                    command=command,
                    reason="human control requires a leader for every selected arm",
                )
                return
            if self.controller.state == destination:
                return
            if self.learning_pending:
                self._log(
                    "command_rejected",
                    command=command,
                    reason="episode learner update still running",
                )
                return
            if command in ("p", "policy") and not self.workflow.uses_policy:
                self._log(
                    "command_rejected",
                    command=command,
                    reason="this manual workflow has no policy",
                )
                return
            self._finish_step(snapshot, frames)
            if command in ("p", "policy"):
                self.controller.enter_policy()
            else:
                self.controller.enter_human()
            self._invalidate_policy()
            self._begin_episode()
        elif command in ("hold", "pause", " "):
            self._finish_step(snapshot, frames)
            self.controller.pause()
            self._invalidate_policy()
            self._log("episode_pause")
        elif command in ("s", "success", "f", "failure", "t", "truncate"):
            if not self.episode_active:
                return
            success = command in ("s", "success")
            truncated = command in ("t", "truncate")
            reward = (
                self.cfg["workflow"]["success_reward"]
                if success
                else 0.0 if truncated else self.cfg["workflow"]["failure_reward"]
            )
            self.end_episode(
                snapshot, frames, not truncated, truncated, reward, success
            )
        elif command in ("q", "quit"):
            if self.episode_active:
                self.end_episode(snapshot, frames, False, True, 0.0, False)
            self.quit = True
        elif command:
            self._log("command_rejected", command=command, reason="unknown command")

    def end_episode(self, snapshot, frames, terminal, truncated, reward, success):
        final_obs = make_observation(self.cfg, snapshot, frames)
        self._finish_step(
            snapshot, frames, terminal=terminal, truncated=truncated, reward=reward
        )
        self.controller.pause()
        self._invalidate_policy()
        self._log(
            "episode_end",
            terminal=terminal,
            truncated=truncated,
            reward=reward,
            success=success,
            steps=self.step_id,
            reward_in_final_step=False,
            final_observation=_stored_observation(snapshot, frames),
        )
        self.episode_active = False
        if self.learning:
            self.policy.end_episode(
                final_obs,
                terminal=terminal,
                reward=reward,
                success=success,
                truncated=truncated,
            )
            self.learning_pending = True
            self.learning_started = time.monotonic()
        else:
            self.episode_id += 1
        self.step_id = 0

    def _poll(self):
        if not self.policy:
            return
        while True:
            reply = self.policy.poll()
            if reply is None:
                return
            if reply.get("kind", "infer") == "infer":
                self.controller.offer(reply)
            elif reply.get("kind") == "end_episode":
                self._log("learner_result", result=reply.get("result"))
                self.learning_pending = False
                self.episode_id += 1

    def _policy_tick(self, snapshot, frames):
        self._finish_step(snapshot, frames)
        observation = make_observation(self.cfg, snapshot, frames)
        self.last_observation = observation
        step = self.controller.step(snapshot)
        if step is not None:
            metadata = step.get("metadata", {})
            step.update(
                episode_id=self.episode_id,
                step_id=self.step_id,
                timestamp=time.time(),
                action_evidence=(
                    "forwarded_can_target"
                    if step["source"] == "HUMAN"
                    else "sdk_target"
                ),
                observation=_stored_observation(snapshot, frames),
                intervention=step["source"] == "HUMAN",
                reward=0.0,
                terminated=False,
                truncated=False,
                boundary=metadata.get("rlt_boundary"),
                _started=time.monotonic(),
                _policy_observation=observation,
                _segment_start=self.needs_boundary,
                _boundary_required=(self.needs_boundary or "rlt_boundary" in metadata),
            )
            self.needs_boundary = False
            self.pending = step
            self.recording.publish_action(
                step["action"],
                step["source"],
                {
                    "timestamp": step["timestamp"],
                    "episode_id": self.episode_id,
                    "step_id": self.step_id,
                    "generation": self.controller.generation,
                    "predicted_action": step["predicted_action"],
                    "action_evidence": step["action_evidence"],
                    "joint_names": component_names(self.cfg["robot"]["arms"]),
                },
            )
            if step["source"] == "HUMAN" and not self._human_action_ready:
                self._human_action_ready = True
                self._log(
                    "human_action_ready",
                    action_evidence=step["action_evidence"],
                    human_action_seq=snapshot.get("human_action_seq"),
                    human_action_stamp=snapshot.get("human_action_stamp"),
                )
                print("[采集] 已收到主臂动作，人工 action 数据已就绪。", flush=True)
        # Consume this tick's action before deciding whether to refill. A
        # single-action policy can then compute during the interval until the
        # next tick instead of inserting an unavoidable hold every other tick.
        # Retain the executed step first so an enqueue failure cannot erase it.
        if self.controller.state == Controller.POLICY:
            if not self.controller.actions and self.controller.pending_seq is None:
                self.seq += 1
                self.controller.expect(self.seq)
                self.policy.submit(observation, self.controller.generation, self.seq)

    def _input(self):
        # Line-oriented input also supports pipes and scripted demonstrations.
        while not self.quit and not self.controller.abort.is_set():
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                line = sys.stdin.readline()
                if not line:
                    print("[控制] 标准输入已关闭，无法接收键盘命令；程序仍继续运行。", flush=True)
                    return
                if line.strip().lower() in ("stop", "estop"):
                    self.controller.request_stop()
                    return
                self.commands.put(line.strip())
            except (OSError, ValueError) as exc:
                print("[控制] 无法读取终端命令：{}".format(exc), flush=True)
                return

    def _signal_stop(self, signum, frame):
        # A Python signal may interrupt an SDK send while it owns a TX lock.
        # Never re-enter SDK I/O from that handler; a normal thread stops it.
        self.controller.abort.set()
        self._stop_wakeup.set()

    def _relay_stop(self):
        self._stop_wakeup.wait()
        if self.controller.abort.is_set() and not self._closing.is_set():
            try:
                self.controller.request_stop()
            except Exception as exc:
                self._stop_error = str(exc)

    def run(self, max_steps=None, max_seconds=None, interactive=True):
        error = None
        try:
            if threading.current_thread() is threading.main_thread():
                self._stop_thread = threading.Thread(
                    target=self._relay_stop, name="signal-stop", daemon=True
                )
                self._stop_thread.start()
                for signum in (signal.SIGINT, signal.SIGTERM):
                    self._signals[signum] = signal.getsignal(signum)
                    signal.signal(signum, self._signal_stop)
            self.start()
            if interactive:
                commands = []
                if self.workflow.uses_policy:
                    commands.append("p=model")
                if self.robot.supports_human:
                    commands.append("h=human")
                commands.extend(
                    (
                        "hold=pause",
                        "s=success",
                        "f=failure",
                        "t=timeout",
                        "q=quit",
                        "stop=emergency stop",
                    )
                )
                print("Commands + Enter: " + ", ".join(commands), flush=True)
                threading.Thread(
                    target=self._input, name="operator-input", daemon=True
                ).start()
            started = time.monotonic()
            next_policy = started
            next_control = started
            auto = self.cfg["workflow"].get("auto_start", False)
            episode_limit = self.cfg["workflow"]["max_episode_sec"]
            while True:
                now = time.monotonic()
                self.recording.check_health()
                self.cameras.check_health()
                self._poll()
                snapshot, frames = self.robot.snapshot(), self.cameras.snapshot()
                self.recording.publish_state(snapshot)
                self.recording.publish_images(frames)
                self.controller.tick(snapshot)
                if auto:
                    self.command(self.workflow.initial_command, snapshot, frames)
                    auto = False
                    snapshot, frames = self.robot.snapshot(), self.cameras.snapshot()
                while self.schedule and now - started >= self.schedule[0][0]:
                    _, command = self.schedule.pop(0)
                    self.commands.put(command)
                while not self.commands.empty():
                    self.command(self.commands.get_nowait(), snapshot, frames)
                    if self.quit:
                        break
                    snapshot, frames = self.robot.snapshot(), self.cameras.snapshot()
                if (
                    self.episode_active
                    and episode_limit is not None
                    and now - self.episode_started >= episode_limit
                ):
                    self.command("t", snapshot, frames)
                limit = (
                    max_steps is not None
                    and (
                        self.total_steps >= max_steps
                        or (
                            self.pending is not None
                            and self.total_steps + 1 >= max_steps
                            and now >= next_policy
                        )
                    )
                ) or (max_seconds is not None and now - started >= max_seconds)
                if limit and not self.quit:
                    self.command("q", snapshot, frames)
                if self.learning_pending:
                    timeout = self.cfg["policy"].get(
                        "update_timeout_sec",
                        self.cfg["workflow"]
                        .get("rlt", {})
                        .get("update_timeout_sec", 600.0),
                    )
                    if now - self.learning_started > timeout:
                        raise RuntimeError(
                            "RLT update timed out while robot was paused"
                        )
                if self.quit and not self.learning_pending:
                    break
                if now >= next_policy and self.episode_active:
                    self._policy_tick(snapshot, frames)
                    next_policy = now + 1.0 / self.cfg["workflow"]["policy_hz"]
                next_control += 1.0 / self.cfg["robot"]["control_hz"]
                time.sleep(max(0.0, min(next_control - time.monotonic(), 0.05)))
                if next_control < time.monotonic() - 0.1:
                    next_control = time.monotonic()
            self._log("session_end", steps=self.total_steps)
        except BaseException as exc:
            error = "%s: %s" % (type(exc).__name__, exc)
            if self.started:
                try:
                    # Motor stop must not depend on disk, ROS, or camera health.
                    self.controller.fail(error)
                except Exception:
                    pass
            if self.pending is not None:
                try:
                    # Keep evidence of a command that was actually sent even if
                    # no valid successor observation can be acquired.
                    incomplete = {
                        k: v for k, v in self.pending.items() if not k.startswith("_")
                    }
                    incomplete.update(
                        next_observation=None,
                        execution_complete=False,
                        duration_sec=max(
                            0.0, time.monotonic() - self.pending["_started"]
                        ),
                        fault=error,
                        truncated=True,
                    )
                    self.recording.write_step(incomplete)
                    self.pending = None
                except Exception:
                    pass
            raise
        finally:
            self.quit = True
            self._closing.set()
            self._stop_wakeup.set()
            errors = []
            for component in (self.robot, self.policy, self.cameras):
                if component is not None:
                    try:
                        component.close()
                    except Exception as exc:
                        errors.append(str(exc))
            if self._stop_thread is not None:
                self._stop_thread.join(timeout=1.0)
                if self._stop_thread.is_alive():
                    errors.append("Signal stop relay did not exit")
            if self._stop_error:
                errors.append(self._stop_error)
            try:
                self.recording.close(
                    error=error or ("; ".join(errors) if errors else None)
                )
            finally:
                for signum, handler in self._signals.items():
                    signal.signal(signum, handler)
            if not error and errors:
                raise RuntimeError("Shutdown failed: " + "; ".join(errors))
        return {
            "steps": self.total_steps,
            "episodes": self.episode_id,
            "directory": (
                str(self.recording.directory) if self.recording.directory else None
            ),
        }
