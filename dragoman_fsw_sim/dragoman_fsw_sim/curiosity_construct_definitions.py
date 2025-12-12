"""
Construct Structure Library for Curiosity Rover

This module provides construct structures for encoding and decoding
CCSDS packets for the Curiosity rover simulator. The structures are based on the
Curiosity rover's joint configuration (24 total joints including arm, mast, wheels, and suspension).

All structures are available as module-level constants for direct use.
"""

from construct import (
    Struct, Int16ub, Float32b,
    Array, CString
)
from dragoman_fsw_sim.ccsds_header_definitions import CCSDSHeader

TM_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "joint_state" / Array(24, Float32b)
)

TC_ARM_JOINT_STRUCT = Struct(
    "header" / CCSDSHeader,
    "command_id" / Int16ub,
    "arm_joint_values" / Array(5, Float32b)
)

TC_MAST_JOINT_STRUCT = Struct(
    "header" / CCSDSHeader,
    "command_id" / Int16ub,
    "mast_joint_values" / Array(3, Float32b)
)

TC_CANNED_POSE_STRUCT = Struct(
    "header" / CCSDSHeader,
    "command_id" / Int16ub,
    "group_name" / CString("utf8"),
    "group_state" / CString("utf8")
)

COMMAND_STRUCTS = {
    0: TC_CANNED_POSE_STRUCT,
    1: TC_ARM_JOINT_STRUCT,
    2: TC_MAST_JOINT_STRUCT
}
