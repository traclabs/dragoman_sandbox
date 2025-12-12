"""
XTCE Construct Structure Library

This module provides hard-coded construct structures for encoding and decoding
CCSDS packets for the IMetro demo robot. The structures are based on the XTCE
definitions but are now hard-coded for simplicity and performance.

All structures are available as module-level constants for direct use.
"""

from construct import (
    Struct, Int16ub, Float32b, Float32l,
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

CommandSecondaryHeader = BitStruct(
  "fcn_code" / BitsInteger(8),
  "checksum" / BitsInteger(8)
)

TelemetrySecondaryHeader = BitStruct(
  "sec" / BitsInteger(48),
  "spare" / BitsInteger(32)
)

# Telemetry packet structure (GatewayTelemetryPacket)
# CCSDS Header + 7 floats for joint_state
TM_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "sec_header" / TelemetrySecondaryHeader,
    "joint_state" / Array(7, Float32l)
)

# Command structures based on XTCE definitions

# pose_goal: header + command_id + 7 floats (3 pos, 4 quat)
TC_GOAL_POSE_STRUCT = Struct(
    "header" / CCSDSHeader,
    "sec_header" / CommandSecondaryHeader,
    "ee_pos" / Array(3, Float32l),
    "ee_rot" / Array(4, Float32l)
)


