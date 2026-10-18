"""Open grippers, optionally lift vertically, return, and close after arrival.

No ROS or SDK imports: the runner receives a controller/observation interface.
"""
import time
import numpy as np
from .core import ActionGuard
from .reset_lift import validate_lift, lift_before_return


JOINT_INDICES = np.array([0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12])
GRIPPER_INDICES = np.array([6, 13])


def validate_reset_config(cfg):
    r = cfg['reset']
    validate_lift(cfg)
    for key in ('joint_speed_rad_s', 'gripper_speed_m_s', 'phase_timeout_sec',
                'settle_sec', 'joint_tolerance_rad', 'gripper_tolerance_m'):
        if not isinstance(r[key], (int, float)) or not np.isfinite(r[key]) or r[key] <= 0:
            raise ValueError('reset.' + key + ' must be finite and positive')
    if r['phase_timeout_sec'] <= r['settle_sec']:
        raise ValueError('Reset timeout must exceed settle time')
    for key in ('gripper_open_m', 'gripper_closed_m'):
        if not np.isfinite(r[key]) or not cfg['control']['gripper_min'] <= r[key] <= cfg['control']['gripper_max']:
            raise ValueError('reset.' + key + ' outside configured gripper range')
    if r['gripper_open_m'] <= r['gripper_closed_m']:
        raise ValueError('Reset open width must be greater than closed width')
    if not isinstance(r['sdk_speed_percent'], int) or not 1 <= r['sdk_speed_percent'] <= 100:
        raise ValueError('Invalid reset SDK speed percentage')


class ResetSequence:
    def __init__(self, cfg, controller, observations, *, clock=time.monotonic,
                 sleep=time.sleep, cancelled=lambda: False, report=lambda _: None):
        validate_reset_config(cfg)
        self.cfg, self.controller, self.obs = cfg, controller, observations
        self.clock, self.sleep, self.cancelled, self.report = clock, sleep, cancelled, report
        self.period = 1.0 / cfg['policy']['fps']
        r = cfg['reset']
        self.speed = np.tile([r['joint_speed_rad_s']]*6 + [r['gripper_speed_m_s']], 2)
        self.tolerance = np.tile([r['joint_tolerance_rad']]*6 + [r['gripper_tolerance_m']], 2)
        self.guard = ActionGuard(cfg['control'], self.period)

    def run(self, home):
        home = self.guard.bounded(home)
        if self.cancelled():
            raise InterruptedError('Reset cancelled')
        # All targets are validated before any phase is sent.
        home[GRIPPER_INDICES] = self.cfg['reset']['gripper_closed_m']
        hold = self.guard.bounded(self.obs.actual())
        opened = hold.copy()
        opened[GRIPPER_INDICES] = self.cfg['reset']['gripper_open_m']
        home_open = home.copy()
        home_open[GRIPPER_INDICES] = self.cfg['reset']['gripper_open_m']
        command = hold.copy()
        for phase, goal in (('opening_grippers', opened), ('returning_home', home_open), ('closing_grippers', home)):
            if phase == 'returning_home' and self.cfg['reset'].get('vertical_lift') is not None:
                command = lift_before_return(self)
            command = self.phase(phase, goal, command)
        self.report({'state': 'complete', 'phase': 'complete', 'goal': home.tolist(),
                     'actual': self.obs.actual().tolist()})
        return home

    def phase(self, name, goal, command):
        started = self.clock()
        stable_since = None
        self.report({'state': 'running', 'phase': name, 'goal': goal.tolist()})
        while True:
            tick = self.clock()
            if self.cancelled():
                raise InterruptedError('Reset cancelled during ' + name)
            self.controller.check_fault()
            actual = self.guard.bounded(self.obs.actual())
            if tick-started > self.cfg['reset']['phase_timeout_sec']:
                raise RuntimeError('Reset phase timed out: ' + name)
            # Interpolate the known reset trajectory, independently of model outputs.
            # Use the command actually accepted by the controller on the next cycle.
            requested = command + np.clip(goal-command, -self.speed*self.period, self.speed*self.period)
            command = np.asarray(self.controller.send(requested))
            # Check both the sent target and measured position, never just elapsed time.
            settled = (np.all(np.abs(command-goal) <= self.tolerance)
                       and np.all(np.abs(actual-goal) <= self.tolerance))
            if settled:
                if stable_since is None:
                    stable_since = tick
                if tick-stable_since >= self.cfg['reset']['settle_sec']:
                    self.report({'state': 'phase_complete', 'phase': name, 'actual': actual.tolist()})
                    return command
            else:
                stable_since = None
            self.sleep(max(0.0, self.period-(self.clock()-tick)))
