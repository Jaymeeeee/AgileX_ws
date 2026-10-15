"""Decode received CAN frames with pyAgxArm; never instantiate an arm driver.

Unlike SDK get_joint_angles(), emit a pose only after all three fresh
two-axis fragments arrive inside the configured assembly time window.
"""
import copy
import math
from pyAgxArm.utiles.fps import FPSManager
from pyAgxArm.protocols.can_protocol.drivers.piper.default.parser import Parser
from pyAgxArm.protocols.can_protocol.drivers.effector.agx_gripper.default.parser import Parser as GripperParser
from pyAgxArm.api.constants import ROBOT_JOINT_TORQUE_K, ROBOT_JOINT_TORQUE_B, ROBOT_JOINT_TORQUE_C


class Decoder:
    def __init__(self, cfg, max_span):
        parser = Parser
        if cfg['parser'] == 'v188':
            from pyAgxArm.protocols.can_protocol.drivers.piper.versions.v188.parser import Parser as parser
        self.model = cfg.get('model', 'unknown')
        self.torque_coefficients = []
        parser_config = None
        if self.model != 'unknown':
            if self.model not in ('piper','piper_h','piper_l','piper_x'):
                raise ValueError('Unsupported torque model: '+self.model)
            parser_config = dict(joint_torque_k=ROBOT_JOINT_TORQUE_K[self.model],
                                 joint_torque_b=ROBOT_JOINT_TORQUE_B[self.model],
                                 joint_torque_c=ROBOT_JOINT_TORQUE_C[self.model])
            self.torque_coefficients = [k*b*c for k,b,c in zip(*parser_config.values())]
        self.arm = parser(FPSManager(), config=parser_config)
        self.gripper = GripperParser(FPSManager())
        self.max_span = max_span
        self.pending = {}
        self.previous = {}
        self.ids = {}
        for ids, offset in [([0x155, 0x156, 0x157, 0x159], cfg['command_id_offset']),
                            ([0x2a1, 0x2a5, 0x2a6, 0x2a7, 0x2a8], cfg['feedback_id_offset']),
                            ([0x251, 0x252, 0x253, 0x254, 0x255, 0x256], cfg.get('motor_id_offset',0))]:
            self.ids.update({i+offset: i for i in ids})

    def feed(self, frame):
        if frame.is_error_frame:
            raise ValueError('CAN error frame')
        if frame.is_extended_id or frame.is_remote_frame or frame.arbitration_id not in self.ids:
            return None
        if len(frame.data) != 8 or frame.dlc != 8:
            raise ValueError('Malformed CAN packet length')
        if not math.isfinite(frame.timestamp) or frame.timestamp <= 0:
            raise ValueError('Invalid CAN receive timestamp')
        canonical = self.ids[frame.arbitration_id]
        if frame.timestamp <= self.previous.get(canonical, 0):
            raise ValueError('Non-increasing receive timestamp')
        self.previous[canonical] = frame.timestamp
        normalized = copy.copy(frame)
        normalized.arbitration_id = canonical
        parser = self.gripper if canonical in (0x159, 0x2a8) else self.arm
        msg = parser.parse_packet(normalized).msg
        event = dict(stamp=frame.timestamp, can_id=frame.arbitration_id,
                     raw_hex=frame.data.hex(), timestamp_source='socketcan_kernel_host_realtime')
        if 0x251 <= canonical <= 0x256:
            pending = self.pending.setdefault('motor_feedback', {})
            values = dict(current_a=float(msg.current), motor_velocity_rad_s=float(msg.velocity),
                          motor_position_rad=float(msg.position), torque_estimate_nm=float(msg.torque))
            if not all(math.isfinite(v) for v in values.values()):
                raise ValueError('Non-finite motor feedback')
            pending[canonical-0x251] = (values, event)
            if len(pending) != 6:
                return None
            stamps = [pending[i][1]['stamp'] for i in range(6)]
            if max(stamps)-min(stamps) > self.max_span:
                del pending[min(range(6), key=lambda i: stamps[i])]
                return None
            event = dict(kind='motor_feedback', stamp=max(stamps), joint_order=list(range(1,7)),
                         current_a=[pending[i][0]['current_a'] for i in range(6)],
                         motor_velocity_rad_s=[pending[i][0]['motor_velocity_rad_s'] for i in range(6)],
                         motor_position_rad=[pending[i][0]['motor_position_rad'] for i in range(6)],
                         torque_estimate_nm=[pending[i][0]['torque_estimate_nm'] for i in range(6)]
                             if self.torque_coefficients else [],
                         torque_status='current_based_estimate' if self.torque_coefficients else 'unavailable_model_unconfirmed',
                         model=self.model, torque_coefficients_nm_per_a=self.torque_coefficients,
                         torque_method='pyAgxArm: current * k * b * c; not a torque sensor or external-contact torque',
                         component_stamps=stamps, frames=[pending[i][1] for i in range(6)],
                         timestamp_source='socketcan_kernel_host_realtime')
            pending.clear()
        elif canonical in (0x155, 0x156, 0x157, 0x2a5, 0x2a6, 0x2a7):
            start, kind = (0x155, 'leader_targets') if canonical < 0x200 else (0x2a5, 'joint_states')
            part = canonical-start
            pending = self.pending.setdefault(kind, {})
            pending[part] = ([float(getattr(msg, 'joint_'+str(part*2+i))) for i in (1,2)], event)
            if len(pending) != 3:
                return None
            stamps = [pending[i][1]['stamp'] for i in range(3)]
            if max(stamps)-min(stamps) > self.max_span:
                del pending[min(range(3), key=lambda i: stamps[i])]
                return None
            event = dict(kind=kind, stamp=max(stamps), position=sum([pending[i][0] for i in range(3)], []),
                         component_stamps=stamps, frames=[pending[i][1] for i in range(3)],
                         timestamp_source='socketcan_kernel_host_realtime')
            pending.clear()
        elif canonical in (0x159, 0x2a8):
            if canonical == 0x2a8 and (msg.mode != 'width' or frame.data[7] != 0):
                raise ValueError('Gripper angle/unknown mode cannot be published as metres')
            event.update(kind='gripper/target_stamped' if canonical==0x159 else 'gripper/actual_stamped',
                         position=[float(msg.value)], protocol_force=float(msg.force), status_code=int(msg.status_code))
            if canonical==0x159:
                event['set_zero'] = int(msg.set_zero)
        else:
            event.update(kind='status', **{key:int(getattr(msg,key)) for key in
                         ('ctrl_mode', 'arm_status', 'mode_feedback', 'teach_status', 'motion_status', 'err_code')})
        if 'position' in event and not all(math.isfinite(v) for v in event['position']):
            raise ValueError('Non-finite joint/gripper data')
        return event
