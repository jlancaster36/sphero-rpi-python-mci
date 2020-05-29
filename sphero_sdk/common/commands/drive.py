#!/usr/bin/env python3
# This file is automatically generated!
# Source File:        0x16-driving.json
# Device ID:          0x16
# Device Name:        drive
# Timestamp:          05/29/2020 @ 13:40:32.620780 (UTC)

from sphero_sdk.common.enums.drive_enums import CommandsEnum
from sphero_sdk.common.devices import DevicesEnum
from sphero_sdk.common.parameter import Parameter
from sphero_sdk.common.sequence_number_generator import SequenceNumberGenerator


def raw_motors(left_mode, left_speed, right_mode, right_speed, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.raw_motors,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='leftMode',
                data_type='uint8_t',
                index=0,
                value=left_mode,
                size=1
            ),
            Parameter( 
                name='leftSpeed',
                data_type='uint8_t',
                index=1,
                value=left_speed,
                size=1
            ),
            Parameter( 
                name='rightMode',
                data_type='uint8_t',
                index=2,
                value=right_mode,
                size=1
            ),
            Parameter( 
                name='rightSpeed',
                data_type='uint8_t',
                index=3,
                value=right_speed,
                size=1
            ),
        ],
    }


def reset_yaw(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.reset_yaw,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
    }


def drive_with_heading(speed, heading, flags, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.drive_with_heading,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='speed',
                data_type='uint8_t',
                index=0,
                value=speed,
                size=1
            ),
            Parameter( 
                name='heading',
                data_type='uint16_t',
                index=1,
                value=heading,
                size=1
            ),
            Parameter( 
                name='flags',
                data_type='uint8_t',
                index=2,
                value=flags,
                size=1
            ),
        ],
    }


def set_default_control_system_for_type(control_system_type, controller_id, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.set_default_control_system_for_type,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='controlSystemType',
                data_type='uint8_t',
                index=0,
                value=control_system_type,
                size=1
            ),
            Parameter( 
                name='controllerId',
                data_type='uint8_t',
                index=1,
                value=controller_id,
                size=1
            ),
        ],
    }


def set_custom_control_system_timeout(command_timeout, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.set_custom_control_system_timeout,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='commandTimeout',
                data_type='uint16_t',
                index=0,
                value=command_timeout,
                size=1
            ),
        ],
    }


def enable_motor_stall_notify(is_enabled, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.enable_motor_stall_notify,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='isEnabled',
                data_type='bool',
                index=0,
                value=is_enabled,
                size=1
            ),
        ],
    }


def on_motor_stall_notify(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.motor_stall_notify,
        'target': target,
        'timeout': timeout,
        'outputs': [ 
            Parameter( 
                name='motorIndex',
                data_type='uint8_t',
                index=0,
                size=1,
            ),
            Parameter( 
                name='isTriggered',
                data_type='bool',
                index=1,
                size=1,
            ),
        ]
    }


def enable_motor_fault_notify(is_enabled, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.enable_motor_fault_notify,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='isEnabled',
                data_type='bool',
                index=0,
                value=is_enabled,
                size=1
            ),
        ],
    }


def on_motor_fault_notify(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.motor_fault_notify,
        'target': target,
        'timeout': timeout,
        'outputs': [ 
            Parameter( 
                name='isFault',
                data_type='bool',
                index=0,
                size=1,
            ),
        ]
    }


def get_motor_fault_state(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.get_motor_fault_state,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'outputs': [ 
            Parameter( 
                name='isFault',
                data_type='bool',
                index=0,
                size=1,
            ),
        ]
    }


def drive_tank_si_units(left_velocity, right_velocity, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.drive_tank_si_units,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='leftVelocity',
                data_type='float',
                index=0,
                value=left_velocity,
                size=1
            ),
            Parameter( 
                name='rightVelocity',
                data_type='float',
                index=1,
                value=right_velocity,
                size=1
            ),
        ],
    }


def drive_tank_normalized(left_velocity, right_velocity, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.drive_tank_normalized,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='leftVelocity',
                data_type='int8_t',
                index=0,
                value=left_velocity,
                size=1
            ),
            Parameter( 
                name='rightVelocity',
                data_type='int8_t',
                index=1,
                value=right_velocity,
                size=1
            ),
        ],
    }


def drive_rc_si_units(yaw_angular_velocity, linear_velocity, flags, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.drive_rc_si_units,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='yawAngularVelocity',
                data_type='float',
                index=0,
                value=yaw_angular_velocity,
                size=1
            ),
            Parameter( 
                name='linearVelocity',
                data_type='float',
                index=1,
                value=linear_velocity,
                size=1
            ),
            Parameter( 
                name='flags',
                data_type='uint8_t',
                index=2,
                value=flags,
                size=1
            ),
        ],
    }


def drive_rc_normalized(yaw_angular_velocity, linear_velocity, flags, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.drive_rc_normalized,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='yawAngularVelocity',
                data_type='int8_t',
                index=0,
                value=yaw_angular_velocity,
                size=1
            ),
            Parameter( 
                name='linearVelocity',
                data_type='int8_t',
                index=1,
                value=linear_velocity,
                size=1
            ),
            Parameter( 
                name='flags',
                data_type='uint8_t',
                index=2,
                value=flags,
                size=1
            ),
        ],
    }


def drive_with_yaw_si(yaw_angle, linear_velocity, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.drive_with_yaw_si,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='yawAngle',
                data_type='float',
                index=0,
                value=yaw_angle,
                size=1
            ),
            Parameter( 
                name='linearVelocity',
                data_type='float',
                index=1,
                value=linear_velocity,
                size=1
            ),
        ],
    }


def drive_with_yaw_normalized(yaw_angle, linear_velocity, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.drive_with_yaw_normalized,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='yawAngle',
                data_type='int16_t',
                index=0,
                value=yaw_angle,
                size=1
            ),
            Parameter( 
                name='linearVelocity',
                data_type='int8_t',
                index=1,
                value=linear_velocity,
                size=1
            ),
        ],
    }


def drive_to_position_si(yaw_angle, x, y, linear_speed, flags, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.drive_to_position_si,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='yawAngle',
                data_type='float',
                index=0,
                value=yaw_angle,
                size=1
            ),
            Parameter( 
                name='x',
                data_type='float',
                index=1,
                value=x,
                size=1
            ),
            Parameter( 
                name='y',
                data_type='float',
                index=2,
                value=y,
                size=1
            ),
            Parameter( 
                name='linearSpeed',
                data_type='float',
                index=3,
                value=linear_speed,
                size=1
            ),
            Parameter( 
                name='flags',
                data_type='uint8_t',
                index=4,
                value=flags,
                size=1
            ),
        ],
    }


def drive_to_position_normalized(yaw_angle, x, y, linear_speed, flags, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.drive_to_position_normalized,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='yawAngle',
                data_type='int16_t',
                index=0,
                value=yaw_angle,
                size=1
            ),
            Parameter( 
                name='x',
                data_type='float',
                index=1,
                value=x,
                size=1
            ),
            Parameter( 
                name='y',
                data_type='float',
                index=2,
                value=y,
                size=1
            ),
            Parameter( 
                name='linearSpeed',
                data_type='int8_t',
                index=3,
                value=linear_speed,
                size=1
            ),
            Parameter( 
                name='flags',
                data_type='uint8_t',
                index=4,
                value=flags,
                size=1
            ),
        ],
    }


def on_xy_position_drive_result_notify(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.xy_position_drive_result_notify,
        'target': target,
        'timeout': timeout,
        'outputs': [ 
            Parameter( 
                name='success',
                data_type='bool',
                index=0,
                size=1,
            ),
        ]
    }


def set_drive_target_slew_parameters(a, b, c, linear_acceleration, linear_velocity_slew_method, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.set_drive_target_slew_parameters,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='a',
                data_type='float',
                index=0,
                value=a,
                size=1
            ),
            Parameter( 
                name='b',
                data_type='float',
                index=1,
                value=b,
                size=1
            ),
            Parameter( 
                name='c',
                data_type='float',
                index=2,
                value=c,
                size=1
            ),
            Parameter( 
                name='linearAcceleration',
                data_type='float',
                index=3,
                value=linear_acceleration,
                size=1
            ),
            Parameter( 
                name='linearVelocitySlewMethod',
                data_type='uint8_t',
                index=4,
                value=linear_velocity_slew_method,
                size=1
            ),
        ],
    }


def get_drive_target_slew_parameters(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.get_drive_target_slew_parameters,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'outputs': [ 
            Parameter( 
                name='a',
                data_type='float',
                index=0,
                size=1,
            ),
            Parameter( 
                name='b',
                data_type='float',
                index=1,
                size=1,
            ),
            Parameter( 
                name='c',
                data_type='float',
                index=2,
                size=1,
            ),
            Parameter( 
                name='linearAcceleration',
                data_type='float',
                index=3,
                size=1,
            ),
            Parameter( 
                name='linearVelocitySlewMethod',
                data_type='uint8_t',
                index=4,
                size=1,
            ),
        ]
    }


def stop_active_controller_custom_decel(deceleration_rate, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.stop_active_controller_custom_decel,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='decelerationRate',
                data_type='float',
                index=0,
                value=deceleration_rate,
                size=1
            ),
        ],
    }


def on_active_controller_stopped_notify(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.active_controller_stopped_notify,
        'target': target,
        'timeout': timeout,
    }


def restore_default_drive_target_slew_parameters(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.restore_default_drive_target_slew_parameters,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
    }


def get_stop_controller_state(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.get_stop_controller_state,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'outputs': [ 
            Parameter( 
                name='stopped',
                data_type='bool',
                index=0,
                size=1,
            ),
        ]
    }


def stop_active_controller(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.stop_active_controller,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
    }


def restore_default_control_system_timeout(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.restore_default_control_system_timeout,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
    }


def get_active_control_system_id(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.get_active_control_system_id,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'outputs': [ 
            Parameter( 
                name='controllerId',
                data_type='uint8_t',
                index=0,
                size=1,
            ),
        ]
    }


def restore_initial_default_control_systems(target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.restore_initial_default_control_systems,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
    }


def get_default_control_system_for_type(control_system_type, target, timeout): 
    return { 
        'did': DevicesEnum.drive,
        'cid': CommandsEnum.get_default_control_system_for_type,
        'seq': SequenceNumberGenerator.get_sequence_number(),
        'target': target,
        'timeout': timeout,
        'inputs': [ 
            Parameter( 
                name='controlSystemType',
                data_type='uint8_t',
                index=0,
                value=control_system_type,
                size=1
            ),
        ],
        'outputs': [ 
            Parameter( 
                name='controllerId',
                data_type='uint8_t',
                index=0,
                size=1,
            ),
        ]
    }
