"""
XTCE Construct Structure Library for MultiPacket Simulator

This module provides construct structures for encoding and decoding CCSDS packets
for the MultiPacket simulator, which demonstrates multi-packet telemetry with
different APIDs for temperature and voltage readings.

Based on MultiPacket.xtce specification with APIDs 130 (Temperature) and 131 (Voltage).
"""

from construct import Struct, Float32b
from dragoman_fsw_sim.ccsds_header_definitions import CCSDSHeader


TEMPERATURE_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "temperature" / Float32b
)


VOLTAGE_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "voltage" / Float32b
)
