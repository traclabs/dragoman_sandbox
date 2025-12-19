"""
XTCE Construct Structure Library for AllTypes Simulator

This module provides construct structures for encoding and decoding CCSDS packets
for the AllTypes simulator, demonstrating all major XTCE parameter types.

Based on AllTypes.xtce
"""

from construct import (
    Struct, Int8ub, Int16sb, Int16ub, Int32sb, Int32ub, Int64ub,
    Float32b, Float64b, Array, Bytes, CString, Flag
)
from dragoman_fsw_sim.ccsds_header_definitions import CCSDSHeader


TM_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "integer_signed" / Int32sb,
    "float_raw64" / Float64b,
    "enum_alarm" / Int8ub,
    "string_utf8" / CString("utf8"),
    "boolean_flag" / Flag,
    "absolute_time" / Int64ub,
    "relative_time" / Int32ub,
    "binary_blob" / Bytes(16),
    "integer_array" / Array(10, Int16ub),
    "aggregate" / Struct(
        "current_draw" / Float32b,
        "heater_enabled" / Flag,
        "raw_status_flags" / Bytes(4)
    )
)


TC_CONFIGURE_STRUCT = Struct(
    "header" / CCSDSHeader,
    "arg_int16" / Int16sb,
    "arg_float64" / Float64b,
    "arg_string_utf16" / CString("utf8"),
    "arg_boolean" / Flag,
    "arg_array_float3" / Array(3, Float32b),
    "arg_config_struct" / Struct(
        "id" / Int8ub,
        "value" / Float32b,
        "config_data" / Bytes(8)
    )
)
