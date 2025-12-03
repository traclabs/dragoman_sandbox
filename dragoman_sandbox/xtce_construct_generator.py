"""
XTCE Construct Structure Library

This module provides hard-coded construct structures for encoding and decoding
CCSDS packets for the IMetro demo robot. The structures are based on the XTCE
definitions but are now hard-coded for simplicity and performance.

All structures are available as module-level constants for direct use.
"""

from construct import (
    Struct, Int16ub, Float32b,
    Array, BitStruct, BitsInteger, CString
)


# CCSDS Primary Header structure (6 bytes)
CCSDSHeader = BitStruct(
    "version" / BitsInteger(3),
    "type" / BitsInteger(1),
    "secondary_header_flag" / BitsInteger(1),
    "apid" / BitsInteger(11),
    "sequence_flags" / BitsInteger(2),
    "sequence_count" / BitsInteger(14),
    "packet_length" / BitsInteger(16)
)

# Telemetry packet structure (IMetroTelemetryPacket)
# CCSDS Header + 9 floats for joint_state
TM_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "joint_state" / Array(9, Float32b)
)

# Command structures based on XTCE definitions

# arm_joint_state_goal: header + command_id + 6 floats
TC_ARM_JOINT_STRUCT = Struct(
    "header" / CCSDSHeader,
    "command_id" / Int16ub,
    "arm_joint_values" / Array(6, Float32b)
)

# rail_joint_state_goal: header + command_id + 1 float
TC_RAIL_JOINT_STRUCT = Struct(
    "header" / CCSDSHeader,
    "command_id" / Int16ub,
    "rail_joint_value" / Float32b
)

# lift_joint_state_goal: header + command_id + 1 float
TC_LIFT_JOINT_STRUCT = Struct(
    "header" / CCSDSHeader,
    "command_id" / Int16ub,
    "lift_joint_value" / Float32b
)

# send_canned_pose: header + command_id + two null-terminated strings
TC_CANNED_POSE_STRUCT = Struct(
    "header" / CCSDSHeader,
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
