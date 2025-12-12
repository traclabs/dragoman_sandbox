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


# Telemetry packet structure (AllTelemetryPacket)
# CCSDS Header + all parameter types
TM_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "integer_signed" / Int32sb,           # T_IntegerSigned: 32-bit signed
    "float_raw64" / Float64b,             # T_FloatRaw64: 64-bit float
    "enum_alarm" / Int8ub,                # T_EnumeratedAlarm: 8-bit enum
    "string_utf8" / CString("utf8"),      # T_StringUTF8: null-terminated UTF-8
    "boolean_flag" / Flag,                # T_BooleanFlag: 8-bit boolean
    "absolute_time" / Int64ub,            # T_AbsoluteTime: 64-bit unsigned (UNIX epoch)
    "relative_time" / Int32ub,            # T_RelativeTimeRaw: 32-bit unsigned
    "binary_blob" / Bytes(16),            # T_BinaryBlob: 128 bits = 16 bytes
    "integer_array" / Array(10, Int16ub), # T_IntegerArray: 10 x 16-bit unsigned
    "aggregate" / Struct(                 # T_StatusAggregate
        "current_draw" / Float32b,        #   CurrentDraw: 32-bit float
        "heater_enabled" / Flag,          #   HeaterEnabled: 8-bit boolean
        "raw_status_flags" / Bytes(4)     #   RawStatusFlags: 32 bits = 4 bytes
    )
)


# Command packet structure (ConfigureAllTypes)
# CCSDS Header + command arguments
#
# Note: The C_ArgStringUTF16 field uses a simplified approach.
# For production use with actual YAMCS commands, you may need to handle
# the variable-length UTF-16BE string more carefully based on actual packet data.
TC_CONFIGURE_STRUCT = Struct(
    "header" / CCSDSHeader,
    "arg_int16" / Int16sb,                           # C_ArgInt16: signed 16-bit
    "arg_float64" / Float64b,                        # C_ArgFloat64: 64-bit float
    "arg_string_utf16" / CString("utf-16-be"),       # C_ArgStringUTF16: UTF-16BE string (simplified)
    "arg_boolean" / Flag,                            # C_ArgBoolean: 8-bit boolean
    "arg_array_float3" / Array(3, Float32b),         # C_ArgArrayFloat3: 3 x 32-bit float
    "arg_config_struct" / Struct(                    # C_ArgConfigStruct
        "id" / Int8ub,                               #   ID: 8-bit unsigned
        "value" / Float32b,                          #   Value: 32-bit float
        "config_data" / Bytes(8)                     #   ConfigData: 64 bits = 8 bytes
    )
)
