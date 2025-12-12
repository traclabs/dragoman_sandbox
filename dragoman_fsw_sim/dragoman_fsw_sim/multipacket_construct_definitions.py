"""
XTCE Construct Structure Library for MultiPacket Simulator

This module provides construct structures for encoding and decoding CCSDS packets
for the MultiPacket simulator, which demonstrates multi-packet telemetry with
different APIDs for temperature and voltage readings.

Based on MultiPacket.xtce specification with APIDs 130 (Temperature) and 131 (Voltage).
"""

from construct import Struct, Float32b
from dragoman_fsw_sim.ccsds_header_definitions import CCSDSHeader


# Temperature packet structure (TemperaturePacket, APID 130)
# CCSDS Header + single 32-bit float for temperature in degrees Celsius
TEMPERATURE_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "temperature" / Float32b  # Temperature in degrees Celsius
)


# Voltage packet structure (VoltagePacket, APID 131)
# CCSDS Header + single 32-bit float for voltage in Volts
VOLTAGE_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "voltage" / Float32b  # Voltage in Volts
)
