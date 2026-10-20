"""One SDK writer per physical arm, with independent CAN receive/watchdog.

    The topology and leader alignment/drag sequence follow kai0's AgileX DAgger
    nodes. This implementation uses the workspace's firmware-aware pyAgxArm.
"""
import fcntl
import os
import threading
import time

import can
import numpy as np

from hardware_bringup.config import check_can, check_identity
from hardware_bringup.decoder import Decoder
from .teleop_core import arm_guard


class TeleopDevice:
    def __init__(self, cfg, side, role):
        if side not in cfg.get('active_arms', ('left', 'right')) or role not in ('leader', 'follower'):
            raise ValueError('Teleop device is outside the selected arm pair(s)')
        self.cfg, self.side, self.role = cfg, side, role
        self.spec = cfg['hardware']['leaders' if role == 'leader' else 'arms'][side]
        self.guard = arm_guard(cfg)
        self.events, self.rx_error = {}, None
        self.last_rx, self.last_send = 0.0, 0.0
        self.phase = 'disconnected'
        self.bus = self.robot = self.gripper = self.file_lock = None
        self.firmware = None
        self.closed = threading.Event()
        self.mutex = threading.RLock()
        self.event_lock = threading.Lock()
        self.threads = []
        self.enabled = False
        self.hold_target = None
        self.fault = None

    def receive(self):
        decoder = Decoder(self.spec, self.cfg['hardware']['arm_settings']['assembly_span_sec'])
        while not self.closed.is_set():
            try:
                frame = self.bus.recv(timeout=0.05)
                if frame is None or not frame.is_rx:
                    continue
                if not -0.1 <= time.time()-frame.timestamp <= self.cfg['feedback_timeout_sec']:
                    raise RuntimeError('CAN receive backlog/clock error')
                if self.role == 'follower' and frame.arbitration_id in (0x150, 0x151, 0x155, 0x156, 0x157, 0x159):
                    raise RuntimeError('External controller on follower CAN: check physical isolation')
                if frame.arbitration_id in (0x4af, 0x2a1, 0x2a5, 0x2a6, 0x2a7, 0x2a8, 0x155, 0x156, 0x157, 0x159):
                    self.last_rx = time.monotonic()
                event = decoder.feed(frame)
                if event:
                    with self.event_lock:
                        self.events[event['kind']] = event
            except Exception as exc:
                if not self.closed.is_set():
                    self.rx_error = str(exc)
                return

    def connect(self, configure=False):
        from pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile
        check_identity(self.spec)
        check_can(self.spec['channel'], self.cfg['hardware']['arm_settings']['bitrate'], 'deploy')
        self.file_lock = open('/tmp/agilex-controller-{}-{}.lock'.format(os.getuid(), self.spec['channel']), 'a')
        fcntl.flock(self.file_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.bus = can.Bus(interface='socketcan', channel=self.spec['channel'], receive_own_messages=False)
        thread = threading.Thread(target=self.receive, daemon=True)
        thread.start(); self.threads.append(thread)
        if not configure:
            self.phase = 'observe'
            return
        expected = self.cfg['expected_firmware'][self.role][self.side]
        profile = resolve_firmware_profile('piper', expected)
        limits = self.cfg['deployment']['control']
        self.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot='piper', firmeware_version=profile, interface='socketcan', channel=self.spec['channel'],
            joint_limits={'joint'+str(i+1): [limits['joint_min'][i], limits['joint_max'][i]] for i in range(6)}))
        self.robot.connect()
        # The reference starts leaders in controlled/follower mode to acquire
        # feedback and align them. This config frame does not enable motors.
        self.robot.set_follower_mode()
        self.teach_frame(False)
        self.phase = 'configured'
        self.wait_actual(8.0, allow_stopped=True)
        deadline = time.monotonic()+3.0
        while self.robot.get_fps() <= 0 and time.monotonic() < deadline:
            self.check(); time.sleep(.02)
        for _ in range(3):
            fw = self.robot.get_firmware(timeout=2.0)
            if fw and fw.get('software_version'):
                break
        else:
            raise RuntimeError('Firmware query failed; no enable/movement sent')
        version = fw['software_version'].strip().rstrip('\x00')
        if version != expected:
            raise RuntimeError('Firmware mismatch: expected {}, received {}'.format(expected, version))
        self.firmware = fw
        self.gripper = self.robot.init_effector('agx_gripper')
        thread = threading.Thread(target=self.watchdog, daemon=True)
        thread.start(); self.threads.append(thread)

    def teach_frame(self, enter):
        # Equivalent to piper_sdk.MotionCtrl_1(grag_teach_ctrl=1/2).
        # pyAgxArm exposes linkage configuration but no public teach-record API.
        self.bus.send(can.Message(arbitration_id=0x150, is_extended_id=False,
                                  data=[0, 0, 1 if enter else 2, 0, 0, 0, 0, 0]), timeout=.1)

    def check(self):
        if self.fault or self.rx_error:
            raise RuntimeError(self.fault or self.rx_error)
        if self.robot and self.robot.has_comm_error():
            raise RuntimeError('SDK communication error: '+str(self.robot.get_comm_error()))

    def actual(self, allow_stopped=False):
        self.check()
        with self.event_lock:
            selected = [self.events.get(k) for k in ('joint_states', 'gripper/actual_stamped', 'status')]
        if not all(selected):
            raise RuntimeError('Waiting for complete arm feedback')
        if any(not -0.1 <= time.time()-e['stamp'] <= self.cfg['feedback_timeout_sec'] for e in selected):
            raise RuntimeError('Arm feedback stale')
        status = selected[2]
        allowed = (0, 1, 11) if allow_stopped else (0, 11)
        if status['err_code'] or status['arm_status'] not in allowed:
            raise RuntimeError('Arm fault: '+str(status))
        if int(selected[1].get('status_code', 0)) & 0x3f:
            raise RuntimeError('Gripper reports a device fault')
        value = np.asarray(selected[0]['position']+selected[1]['position'], dtype=float)
        return self.guard.bounded(np.tile(value, 2))[:7]

    def wait_actual(self, timeout, allow_stopped=False):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.check()
            try:
                return self.actual(allow_stopped=allow_stopped)
            except RuntimeError:
                time.sleep(.02)
        return self.actual(allow_stopped=allow_stopped)

    def _send(self, value):
        self.robot.move_j(value[:6].tolist())
        self.gripper.move_gripper_m(float(value[6]), self.cfg['deployment']['control']['gripper_force'])
        self.last_send = time.monotonic()
        self.check()

    def enable_hold(self):
        with self.mutex:
            actual = self.actual(allow_stopped=True)
            self.robot.set_speed_percent(self.cfg['alignment_sdk_speed_percent'])
            self._send(actual)
            with self.event_lock:
                stopped = self.events.get('status', {}).get('arm_status') == 1
            if stopped:
                # Explicit execute startup only: recover the damped stop left
                # by a previous teaching session, with fresh bounded feedback.
                self.robot.reset()
                self._send(actual)
            self.guard.commit(np.tile(actual, 2))
            self.hold_target = actual.copy()
            self.enabled = True  # include partial enable in cleanup
            self.robot.enable()
            self.phase = 'holding'
        deadline = time.monotonic()+3.0
        while not all(self.robot.get_joints_enable_status_list()):
            if time.monotonic() > deadline:
                raise RuntimeError('Enable feedback timed out')
            current = self.actual()
            if np.any(np.abs(current-actual) > self.guard.initial_error[:7]):
                raise RuntimeError('Arm moved during enable handshake')
            self.send(actual); time.sleep(.03)

    def send(self, target, following=False):
        with self.mutex:
            self.check()
            if not self.enabled or self.phase == 'teaching':
                raise RuntimeError('Arm not in SDK position control')
            actual = self.actual()  # freshness, faults and measured physical bounds
            if following and not self.cfg['limit_follow_speed']:
                target = np.asarray(target, dtype=float)
                if target.shape != (7,) or not np.isfinite(target).all():
                    raise ValueError('Expected finite seven-axis leader target')
                # Explicit fast teleop path: no application slew limit or
                # command-vs-feedback tracking-distance gate. SDK MOVE J uses
                # the separately configured following speed percentage.
                sent = np.clip(target, self.guard.low[:7], self.guard.high[:7])
            else:
                sent = self.guard.prepare(np.tile(target, 2), np.tile(actual, 2))[:7]
            self._send(sent)
            self.guard.commit(np.tile(sent, 2))
            return sent

    def align(self, target, cancelled=lambda: False):
        target = np.asarray(target, dtype=float)
        bounded = self.guard.bounded(np.tile(target, 2))[:7]
        if not np.allclose(target, bounded, atol=1e-8):
            raise ValueError('Alignment target exceeds physical limits')
        self.phase = 'aligning'
        deadline, settled = time.monotonic()+self.cfg['alignment_timeout_sec'], None
        while time.monotonic() < deadline:
            if cancelled() or self.closed.is_set():
                raise RuntimeError('Alignment cancelled')
            sent = self.send(target)
            actual = self.actual()
            reached = (np.max(np.abs(actual[:6]-target[:6])) <= self.cfg['alignment_joint_tolerance_rad']
                       and abs(actual[6]-target[6]) <= self.cfg['alignment_gripper_tolerance_m']
                       and np.allclose(sent, target, atol=1e-5))
            settled = (settled or time.monotonic()) if reached else None
            if settled is not None and time.monotonic()-settled >= self.cfg['alignment_settle_sec']:
                self.phase = 'holding'
                return
            time.sleep(1.0/self.cfg['fps'])
        raise RuntimeError('Leader alignment timed out')

    def enter_teach(self):
        with self.mutex:
            self.check()
            seed = self.actual()
            self.phase = 'switching'
            self.robot.set_leader_mode()
            self.teach_frame(True)
            with self.event_lock:
                # Never interpret old SDK command echoes as human actions.
                self.events.pop('leader_targets', None)
                self.events.pop('gripper/target_stamped', None)
            self.phase = 'teaching'
            return seed

    def leader_target(self, seed, seed_stamp):
        self.check()
        result, stamps = np.asarray(seed).copy(), [seed_stamp, seed_stamp]
        with self.event_lock:
            joint = self.events.get('leader_targets')
            grip = self.events.get('gripper/target_stamped')
        if joint and joint['stamp'] >= seed_stamp:
            result[:6], stamps[0] = joint['position'], joint['stamp']
        if grip and grip['stamp'] >= seed_stamp:
            if grip.get('set_zero'):
                raise RuntimeError('Unexpected leader gripper calibration command')
            result[6], stamps[1] = grip['position'][0], grip['stamp']
        return result, max(stamps), stamps

    def ping(self):
        # Native input is event-like on this firmware; query replies prove CAN
        # liveness without pretending cached target timestamps are fresh.
        if self.bus:
            self.bus.send(can.Message(arbitration_id=0x4af, is_extended_id=False, data=[1]), timeout=.1)

    def watchdog(self):
        next_identity = 0.0
        while not self.closed.wait(.02):
            try:
                self.check()
                if time.monotonic() >= next_identity:
                    check_identity(self.spec)
                    check_can(self.spec['channel'], self.cfg['hardware']['arm_settings']['bitrate'], 'deploy')
                    next_identity = time.monotonic()+1
                if not self.enabled:
                    continue
                if self.phase in ('switching', 'teaching'):
                    if time.monotonic()-self.last_rx > self.cfg['source_timeout_sec']:
                        raise RuntimeError('Leader CAN heartbeat lost')
                else:
                    self.actual()
                    if time.monotonic()-self.last_send > self.cfg['command_timeout_sec']:
                        raise RuntimeError('Command watchdog expired')
            except Exception as exc:
                self.stop(str(exc))
                return

    def stop(self, reason='operator stop'):
        with self.mutex:
            if self.fault:
                return
            try:
                if self.enabled:
                    if self.phase in ('switching', 'teaching'):
                        # Do not snap a manually held leader to an old pose.
                        self.robot.electronic_emergency_stop()
                    else:
                        try:
                            self._send(self.actual())
                        except Exception:
                            self.robot.electronic_emergency_stop()
            finally:
                self.fault, self.phase = reason, 'stopped'

    def close(self):
        self.stop()
        self.closed.set()
        for thread in self.threads:
            if thread is not threading.current_thread():
                thread.join(timeout=1)
        if self.robot:
            self.robot.disconnect()
        if self.bus:
            self.bus.shutdown()
        if self.file_lock:
            self.file_lock.close()
