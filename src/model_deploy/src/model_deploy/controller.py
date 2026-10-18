"""SDK-only actuation with independent watchdog; never imported by dry-run."""
import fcntl
import os
import threading
import time
import can
import numpy as np
from hardware_bringup.config import check_can
from .core import SIDES, ActionGuard


class Controller:
    def __init__(self, cfg, observations, require_images=True):
        self.cfg, self.obs = cfg, observations
        self.require_images = require_images
        self.arms, self.grippers, self.locks, self.buses, self.threads = {}, {}, [], [], []
        self.closed = threading.Event()
        self.mutex = threading.RLock()
        self.active = False
        self.fault = None
        self.external = None
        self.last_send = time.monotonic()
        self.guard = ActionGuard(cfg['control'], 1/cfg['policy']['fps'])
        self.firmware = {}

    def read_firmware(self, robot, side):
        # pyAgxArm get_firmware() discards its response buffer while get_fps()
        # is zero. A new receive/FPS thread is asynchronous: connect() alone
        # does not mean the query parser is ready.
        deadline = time.monotonic()+3.0
        stable_since = None
        while time.monotonic() < deadline:
            self.check_fault()
            feedback = robot.get_joint_angles()
            ready = (robot.get_fps() > 0 and feedback is not None
                     and -0.05 <= time.time()-feedback.timestamp <= 0.25)
            if ready:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic()-stable_since >= 0.2:
                    break
            else:
                stable_since = None
            time.sleep(0.02)
        else:
            raise RuntimeError(side + ': SDK receive not ready (no fresh joint feedback/FPS); '
                               'check power/CAN connection; no enable/motion sent')
        for attempt in range(1, 4):
            self.check_fault()
            fw = robot.get_firmware(timeout=3.0)
            if fw and fw.get('software_version'):
                return fw
            print('{}: firmware response incomplete, attempt {}/3, RX {:.0f} Hz'.format(
                side, attempt, robot.get_fps()), flush=True)
            if attempt < 3:
                time.sleep(0.2)
        raise RuntimeError(side + ': firmware query failed after 3 attempts with SDK receive ready; '
                           'no enable/motion sent')

    def connect(self):
        from pyAgxArm import create_agx_arm_config, AgxArmFactory, resolve_firmware_profile
        for side in SIDES:
            channel = self.cfg['control']['channels'][side]
            check_can(channel, self.cfg['control']['bitrate'], 'deploy')
            lock = open('/tmp/agilex-controller-{}-{}.lock'.format(os.getuid(), channel), 'a')
            self.locks.append(lock)
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError(channel + ': another deploy/reset controller is running; '
                                   'stop it before starting this command') from exc
            bus = can.Bus(interface='socketcan', channel=channel, receive_own_messages=False)
            self.buses.append(bus)
            thread = threading.Thread(target=self.watch_bus, args=(bus, side), daemon=True)
            thread.start()
            self.threads.append(thread)
        # Listen before even querying firmware; do not take over an active leader.
        time.sleep(1.0)
        self.check_fault()
        for side in SIDES:
            channel = self.cfg['control']['channels'][side]
            def create(profile):
                limits = {'joint'+str(i+1): [self.cfg['control']['joint_min'][i],
                                             self.cfg['control']['joint_max'][i]] for i in range(6)}
                return AgxArmFactory.create_arm(create_agx_arm_config(
                    robot='piper', firmeware_version=profile, interface='socketcan', channel=channel,
                    joint_limits=limits))
            robot = create('default')
            self.arms[side] = robot
            robot.connect()
            fw = self.read_firmware(robot, side)
            version = fw['software_version'].strip().rstrip('\x00')
            expected = self.cfg['control']['expected_firmware'][side]
            if expected is not None and version != expected:
                raise RuntimeError(side + ': firmware does not match config')
            profile = resolve_firmware_profile('piper', version)
            self.firmware[side] = dict(fw, sdk_profile=profile)
            if profile != 'default':
                robot.disconnect()
                robot = create(profile)
                self.arms[side] = robot
                robot.connect()
            self.grippers[side] = robot.init_effector('agx_gripper')
            self.check_fault()

    def watch_bus(self, bus, side):
        while not self.closed.is_set():
            try:
                msg = bus.recv(timeout=0.05)
                # SocketCAN marks local TX echo is_rx=False. Remote masters are RX.
                if msg and msg.is_rx and msg.arbitration_id in (0x150, 0x151, 0x155, 0x156, 0x157, 0x159):
                    self.external = side + ': another controller/master is sending commands'
            except Exception as exc:
                if not self.closed.is_set():
                    self.external = side + ': CAN monitor failed: ' + str(exc)
                return

    def check_fault(self):
        if self.fault or self.external:
            raise RuntimeError(self.fault or self.external)
        for side, robot in self.arms.items():
            if robot.has_comm_error():
                raise RuntimeError(side + ': SDK CAN transport error: ' + str(robot.get_comm_error()))

    def _send(self, target):
        for i, side in enumerate(SIDES):
            self.arms[side].move_j(target[i*7:i*7+6].tolist())
            self.grippers[side].move_gripper_m(float(target[i*7+6]), self.cfg['control']['gripper_force'])
        self.check_fault()
        self.last_send = time.monotonic()

    def start(self, on_initial_pose=None):
        self.obs.deployment_hardware()
        actual = self.guard.bounded(self.obs.actual())
        self.check_fault()
        if on_initial_pose is not None:
            on_initial_pose(actual.copy())
        with self.mutex:
            self.active = True  # even partial startup must run stop handling
            self.last_send = time.monotonic()
            for side in SIDES:
                self.arms[side].set_speed_percent(self.cfg['control']['sdk_speed_percent'])
            self._send(actual)  # replace any old target BEFORE enabling
            for side in SIDES:
                self.arms[side].enable()
            self.guard.commit(actual)
        thread = threading.Thread(target=self.watchdog, daemon=True)
        thread.start()
        self.threads.append(thread)
        deadline = time.monotonic()+3
        while not all(all(r.get_joints_enable_status_list()) for r in self.arms.values()):
            self.check_fault()
            if time.monotonic() > deadline:
                raise RuntimeError('Motor enable feedback timed out')
            with self.mutex:
                if not self.active:
                    self.check_fault()
                    raise RuntimeError('Controller stopped during startup')
                current = self.guard.bounded(self.obs.actual())
                if np.any(np.abs(current-actual) > self.guard.initial_error):
                    raise RuntimeError('Pose moved during enable handshake')
                self._send(actual)
            time.sleep(0.03)

    def send(self, target):
        with self.mutex:
            self.check_fault()
            if not self.active:
                raise RuntimeError('Controller inactive')
            actual = self.obs.actual()
            sent = self.guard.prepare(target, actual)
            self._send(sent)
            self.guard.commit(sent)
            return sent

    def watchdog(self):
        while not self.closed.wait(0.02):
            if not self.active:
                return
            try:
                self.check_fault()
                self.obs.actual()
                if self.require_images:
                    self.obs.snapshot()
                self.obs.deployment_hardware()
                if time.monotonic()-self.last_send > self.cfg['control']['command_timeout_sec']:
                    raise RuntimeError('Command watchdog expired')
            except Exception as exc:
                self.stop(str(exc))
                return

    def flange_poses(self):
        """Fresh SDK flange feedback, meters/radians in each arm's base frame."""
        result = []
        for side in SIDES:
            feedback = self.arms[side].get_flange_pose()
            if feedback is None:
                raise RuntimeError(side + ': no SDK flange feedback for vertical lift')
            pose = np.asarray(feedback.msg, dtype=float).copy()
            stamp = float(feedback.timestamp)
            if (pose.shape != (6,) or not np.isfinite(pose).all() or not np.isfinite(stamp)
                    or not -.05 <= time.time()-stamp <= self.cfg['observations']['state_age_sec']):
                raise RuntimeError(side + ': invalid/stale SDK flange feedback for vertical lift')
            # Refuse angles the SDK would silently clamp and change orientation.
            if np.any(np.abs(pose[3:]) > np.array([np.pi, np.pi/2, np.pi])+1e-9):
                raise RuntimeError(side + ': flange Euler angles outside SDK move_l range')
            result.append(pose)
        return np.asarray(result)

    def lift_fk(self, joints):
        return np.asarray([self.arms[s].fk(q.tolist()) for s, q in zip(SIDES, joints)])

    def prepare_joint_lift(self, cancelled=lambda: False):
        """Query limits and plan while inactive, before enabling or moving."""
        from .reset_joint_lift import plan_joint_lift
        if self.active:
            raise RuntimeError('Plan lift before starting controller')
        low = np.tile(np.asarray(self.cfg['control']['joint_min'], dtype=float), (2, 1))
        high = np.tile(np.asarray(self.cfg['control']['joint_max'], dtype=float), (2, 1))
        def poll():
            if cancelled():
                raise InterruptedError('Reset cancelled during lift planning')
            self.check_fault()
        for i, side in enumerate(SIDES):
            for j in range(6):
                poll()
                feedback = self.arms[side].get_joint_angle_vel_limits(j+1)
                if feedback is None:
                    raise RuntimeError(side + ': cannot read device joint limits; no motion sent')
                a, b = feedback.msg.min_angle_limit, feedback.msg.max_angle_limit
                if a is None or b is None or not np.isfinite([a, b]).all() or a >= b:
                    raise RuntimeError(side + ': invalid device joint limits; no motion sent')
                low[i, j], high[i, j] = max(low[i, j], a), min(high[i, j], b)
        joints = self.obs.actual()[[0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]].reshape(2, 6)
        measured = self.flange_poses()
        self.joint_lift_plan = plan_joint_lift([self.arms[s].fk for s in SIDES], joints,
            measured, low, high, self.cfg['reset']['vertical_lift'], poll)
        return {k: v.tolist() for k, v in self.joint_lift_plan.items()}

    def send_lift(self, poses, gripper_width):
        with self.mutex:
            self.check_fault()
            if not self.active:
                raise RuntimeError('Controller inactive')
            self.guard.bounded(self.obs.actual())
            self.flange_poses()  # recheck immediately before sending either arm
            poses = np.asarray(poses, dtype=float)
            if (poses.shape != (2, 6) or not np.isfinite(poses).all()
                    or np.any(np.abs(poses[:, 3:]) > np.array([np.pi, np.pi/2, np.pi])+1e-9)
                    or not self.cfg['control']['gripper_min'] <= gripper_width <= self.cfg['control']['gripper_max']):
                raise ValueError('Invalid Cartesian lift target')
            for i, side in enumerate(SIDES):
                self.arms[side].move_l(poses[i].tolist())
                self.grippers[side].move_gripper_m(float(gripper_width), self.cfg['control']['gripper_force'])
            self.check_fault()
            self.last_send = time.monotonic()

    def finish_lift(self):
        with self.mutex:
            self.check_fault()
            if not self.active:
                raise RuntimeError('Controller inactive')
            actual = self.guard.bounded(self.obs.actual())
            self._send(actual)  # switch back to joint mode at measured pose
            self.guard.commit(actual)
            return actual

    def stop(self, reason='operator stop'):
        with self.mutex:
            if not self.active:
                return
            self.active = False
            self.fault = reason  # latched; no automatic restart
            try:
                actual = self.guard.bounded(self.obs.actual(status=False))
            except Exception:
                actual = None
            for i, side in enumerate(SIDES):
                robot = self.arms.get(side)
                if robot is None:
                    continue
                try:
                    if actual is None:
                        robot.electronic_emergency_stop()
                        print(side + ': stale feedback, requested damped stop; hold unverified', flush=True)
                    else:
                        robot.move_j(actual[i*7:i*7+6].tolist())
                        self.grippers[side].move_gripper_m(float(actual[i*7+6]), self.cfg['control']['gripper_force'])
                        if robot.has_comm_error():
                            raise RuntimeError(str(robot.get_comm_error()))
                        print(side + ': requested hold at measured pose/gripper; no reset/disable', flush=True)
                except Exception as exc:
                    print(side + ': HOLD FAILED: ' + str(exc) + '; use hardware stop', flush=True)
                    try:
                        robot.electronic_emergency_stop()
                    except Exception:
                        pass

    def close(self):
        self.stop()
        self.closed.set()
        for thread in self.threads:
            if thread is not threading.current_thread():
                thread.join(timeout=0.5)
        for robot in self.arms.values():
            try:
                robot.disconnect()
            except Exception as exc:
                print('SDK disconnect failed: ' + str(exc), flush=True)
        for bus in self.buses:
            try:
                bus.shutdown()
            except Exception as exc:
                print('CAN observer close failed: ' + str(exc), flush=True)
        for lock in self.locks:
            lock.close()
