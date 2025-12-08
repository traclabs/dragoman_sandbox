"""
XTCE Construct Structure Library for Curiosity Rover

This module provides hard-coded construct structures for encoding and decoding
CCSDS packets for the Curiosity rover simulator. The structures are based on the
Curiosity rover's joint configuration (24 total joints including arm, mast, wheels, and suspension).

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

# Telemetry packet structure (CuriosityTelemetryPacket)
# CCSDS Header + 24 floats for joint_state (all rover joints)
TM_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "joint_state" / Array(24, Float32b)
)

# Command structures based on XTCE definitions

# arm_joint_state_goal: header + command_id + 5 floats
TC_ARM_JOINT_STRUCT = Struct(
    "header" / CCSDSHeader,
    "command_id" / Int16ub,
    "arm_joint_values" / Array(5, Float32b)
)

# mast_joint_state_goal: header + command_id + 3 floats
TC_MAST_JOINT_STRUCT = Struct(
    "header" / CCSDSHeader,
    "command_id" / Int16ub,
    "mast_joint_values" / Array(3, Float32b)
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
    2: TC_MAST_JOINT_STRUCT
}
