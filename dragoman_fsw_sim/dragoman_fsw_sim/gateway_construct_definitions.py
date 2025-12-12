"""
Construct Structure Library for Gateway Robot

This module provides construct structures for encoding and decoding
CCSDS packets for the Gateway robot. The structures are based on the XTCE
definitions for the robot's 7-joint configuration.

All structures are available as module-level constants for direct use.
"""

from construct import (
    Struct, Float32l, Array
)
from dragoman_fsw_sim.ccsds_header_definitions import CCSDSHeader
from dragoman_fsw_sim.ccsds_secondary_header_definitions import (
    CommandSecondaryHeader,
    TelemetrySecondaryHeader
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


