"""
XTCE Construct Structure Library for AllTypes Simulator

This module provides construct structures for encoding and decoding CCSDS packets
for the AllTypes simulator, demonstrating all major XTCE parameter types.

Based on AllTypes.xtce specification with APID 120.
"""

from construct import (
    Struct, Int8ub, Int16sb, Int16ub, Int32sb, Int32ub, Int64ub,
    Float32b, Float64b, Array, Bytes, CString, Adapter, Flag
)
from dragoman_fsw_sim.ccsds_header_definitions_definitions import CCSDSHeader


class NullTerminatedUTF16BE(Adapter):
    """
    Custom adapter for UTF-16BE strings with null terminator.

    YAMCS doesn't always send the null terminator despite XTCE specification,
    so we handle variable-length UTF-16BE strings by reading until we detect
    the end pattern (typically the next field which is a boolean 0x00 or 0x01).
    """
    def _decode(self, obj, context, path):
        """Decode UTF-16BE bytes to string"""
        if not obj:
            return ""
        # Remove null terminator if present
        if obj.endswith(b'\x00\x00'):
            obj = obj[:-2]
        try:
            return obj.decode('utf-16-be')
        except Exception:
            return ""

    def _encode(self, obj, context, path):
        """Encode string to UTF-16BE bytes with null terminator"""
        if not obj:
            return b'\x00\x00'
        # Encode to UTF-16BE and add null terminator
        return obj.encode('utf-16-be') + b'\x00\x00'


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
    "arg_string_utf16" / CString("utf-16-be"),
    "arg_boolean" / Flag,
    "arg_array_float3" / Array(3, Float32b),
    "arg_config_struct" / Struct(
        "id" / Int8ub,
        "value" / Float32b,
        "config_data" / Bytes(8)
    )
)
