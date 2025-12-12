"""
Construct Structure Library for IMetro Demo Robot

This module provides construct structures for encoding and decoding
CCSDS packets for the IMetro demo robot. The structures are based on the XTCE
definitions for the robot's 9-joint configuration.

All structures are available as module-level constants for direct use.
"""

from construct import (
    Struct, Int16ub, Float32b,
    Array, CString
)
from dragoman_fsw_sim.ccsds_header_definitions import CCSDSHeader
from dragoman_fsw_sim.ccsds_secondary_header_definitions import (
    CommandSecondaryHeader,
    TelemetrySecondaryHeader
)

# Telemetry packet structure (IMetroTelemetryPacket)
# CCSDS Header + 9 floats for joint_state
TM_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "sec_header" / TelemetrySecondaryHeader,
    "joint_state" / Array(9, Float32b)
)

TC_ARM_JOINT_STRUCT = Struct(
    "header" / CCSDSHeader,
    "sec_header" / CommandSecondaryHeader,
    "command_id" / Int16ub,
    "arm_joint_values" / Array(6, Float32b)
)

TC_RAIL_JOINT_STRUCT = Struct(
    "header" / CCSDSHeader,
    "sec_header" / CommandSecondaryHeader,
    "command_id" / Int16ub,
    "rail_joint_value" / Float32b
)

TC_LIFT_JOINT_STRUCT = Struct(
    "header" / CCSDSHeader,
    "sec_header" / CommandSecondaryHeader,
    "command_id" / Int16ub,
    "lift_joint_value" / Float32b
)

TC_CANNED_POSE_STRUCT = Struct(
    "header" / CCSDSHeader,
    "sec_header" / CommandSecondaryHeader,
    "command_id" / Int16ub,
    "group_name" / CString("utf8"),
    "group_state" / CString("utf8")
)

# Command ID to struct mapping
COMMAND_STRUCTS = {
    0: TC_CANNED_POSE_STRUCT,
    1: TC_ARM_JOINT_STRUCT,
    2: TC_RAIL_JOINT_STRUCT,
    3: TC_LIFT_JOINT_STRUCT
}
