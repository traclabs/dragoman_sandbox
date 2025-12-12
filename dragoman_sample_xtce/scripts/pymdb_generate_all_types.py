#!/usr/bin/env python3

import sys
import os
import yamcs.pymdb as yp
from datetime import datetime

# This function will create the XTCE structure with a representative example
def generate_xtce_all_types(filename):

    # 1. Setup & Boilerplate
    spacecraft = yp.System("AllTypes")

    # Add a base CCSDS header for containers and commands to inherit from
    ccsds_header = yp.ccsds.add_ccsds_header(spacecraft)

    # =========================================================================
    # PARAMETER TYPES (TELEMETRY) - Scalar
    # =========================================================================

    # --- 1. IntegerParameterType (Signed) ---
    integer_param = yp.IntegerParameter(
        system=spacecraft,
        name="T_IntegerSigned",
        signed=True,
        bits=32,
        encoding=yp.uint32_t,
        short_description="Signed Integer, 32-bit, Two's Complement"
    )

    # --- 2. FloatParameterType (64-bit) ---
    float_param = yp.FloatParameter(
        system=spacecraft,
        name="T_FloatRaw64",
        bits=64,
        encoding=yp.float64_t,
        short_description="Raw 64-bit Float, no calibration"
    )

    # --- 3. EnumeratedParameterType (with Alarm) ---
    enum_choices = [
        (0, "STATE_OFF"),
        (1, "STATE_NOMINAL"),
        (2, "STATE_FAULT")
    ]
    enum_param = yp.EnumeratedParameter(
        system=spacecraft,
        name="T_EnumeratedAlarm",
        choices=enum_choices,
        encoding=yp.uint8_t,
        short_description="8-bit Enumerated Type with states"
    )

    # --- 4. StringParameterType ---
    string_param = yp.StringParameter(
        system=spacecraft,
        name="T_StringUTF8",
        min_length=0,
        max_length=64,
        encoding=yp.StringEncoding(charset=yp.Charset.UTF_8),
        short_description="String up to 64 bytes, UTF-8 encoded"
    )

    # --- 5. BooleanParameterType ---
    boolean_param = yp.BooleanParameter(
        system=spacecraft,
        name="T_BooleanFlag",
        encoding=yp.uint8_t,
        short_description="Boolean flag, 1 byte (0/1)"
    )

    # --- 6. AbsoluteTimeParameterType ---
    abs_time_param = yp.AbsoluteTimeParameter(
        system=spacecraft,
        name="T_AbsoluteTime",
        reference=yp.Epoch.UNIX,
        encoding=yp.IntegerTimeEncoding(bits=64),
        short_description="Absolute Time, 64-bit integer (epoch ms)"
    )

    # --- 7. IntegerParameterType for RelativeTime (using IntegerParameter) ---
    rel_time_param = yp.IntegerParameter(
        system=spacecraft,
        name="T_RelativeTimeRaw",
        signed=False,
        bits=32,
        encoding=yp.uint32_t,
        units="seconds",
        short_description="Relative time, 32-bit integer (raw counts)"
    )

    # --- 8. BinaryParameterType (Blob) ---
    binary_param = yp.BinaryParameter(
        system=spacecraft,
        name="T_BinaryBlob",
        min_length=16,
        max_length=16,
        encoding=yp.BinaryEncoding(bits=128),
        short_description="Raw Binary Data/Block, 128 bits"
    )

    # =========================================================================
    # PARAMETER TYPES (TELEMETRY) - Complex
    # =========================================================================

    # --- 9. ArrayParameterType ---
    array_param_instance = yp.ArrayParameter(
        system=spacecraft,
        name="T_IntegerArray",
        data_type=yp.datatypes.IntegerDataType(signed=False, encoding=yp.uint16_t),
        length=10,
        short_description="Array of 10 unsigned 16-bit integers"
    )

    # --- 10. AggregateParameterType (Struct) ---
    aggregate_param_instance = yp.AggregateParameter(
        system=spacecraft,
        name="T_StatusAggregate",
        members=[
            yp.datatypes.FloatMember(name="CurrentDraw", bits=32, encoding=yp.float32_t),
            yp.datatypes.BooleanMember(name="HeaterEnabled", encoding=yp.uint8_t),
            yp.datatypes.BinaryMember(name="RawStatusFlags", min_length=4, max_length=4, encoding=yp.BinaryEncoding(bits=32))
        ],
        short_description="Aggregate/Struct: CurrentDraw (Float), HeaterEnabled (Boolean), and RawStatusFlags (Binary)"
    )


    # =========================================================================
    # TELEMETRY CONTAINER (All Parameters)
    # =========================================================================

    tm_container = yp.Container(
        system=spacecraft,
        name="AllTelemetryPacket",
        base=ccsds_header.tm_container,
        entries=[
            yp.ParameterEntry(parameter=integer_param),
            yp.ParameterEntry(parameter=float_param),
            yp.ParameterEntry(parameter=enum_param),
            yp.ParameterEntry(parameter=string_param),
            yp.ParameterEntry(parameter=boolean_param),
            yp.ParameterEntry(parameter=abs_time_param),
            yp.ParameterEntry(parameter=rel_time_param),
            yp.ParameterEntry(parameter=binary_param),
            yp.ParameterEntry(parameter=array_param_instance),
            yp.ParameterEntry(parameter=aggregate_param_instance),
        ],
        condition=yp.eq(ccsds_header.tm_apid, 120)
    )

    # =========================================================================
    # ARGUMENT TYPES (COMMANDS) - Command Arguments (C_Arg*)
    # =========================================================================

    # --- 1. IntegerArgument (Signed, 16-bit) ---
    arg_int = yp.IntegerArgument(
        name="C_ArgInt16",
        signed=True,
        encoding=yp.int16_t
    )

    # --- 2. FloatArgument (64-bit) ---
    arg_float = yp.FloatArgument(
        name="C_ArgFloat64",
        encoding=yp.float64_t
    )

    # --- 3. StringArgument (UTF-16) ---
    arg_string = yp.StringArgument(
        name="C_ArgStringUTF16",
        max_length=10,
        encoding=yp.StringEncoding()
    )

    # --- 4. BooleanArgument ---
    arg_bool = yp.BooleanArgument(
        name="C_ArgBoolean",
        encoding=yp.uint8_t
    )

    # --- 5. ArrayArgument (3x Float32) ---
    arg_array = yp.commands.ArrayArgument(
        name="C_ArgArrayFloat3",
        data_type=yp.datatypes.FloatDataType(encoding=yp.float32_t),
        length=3
    )

    # --- 6. AggregateArgument (Struct) ---
    arg_aggregate = yp.commands.AggregateArgument(
        name="C_ArgConfigStruct",
        members=[
            yp.datatypes.IntegerMember(name="ID", signed=False, bits=8, encoding=yp.uint8_t),
            yp.datatypes.FloatMember(name="Value", bits=32, encoding=yp.float32_t),
            yp.datatypes.BinaryMember(name="ConfigData", min_length=8, max_length=8, encoding=yp.BinaryEncoding(bits=64))
        ]
    )

    # =========================================================================
    # COMMAND (All Arguments)
    # =========================================================================

    all_args_command = yp.Command(
        system=spacecraft,
        name="ConfigureAllTypes",
        short_description="Command with arguments for all major types",
        base=ccsds_header.tc_command,
        assignments = {
            ccsds_header.tc_apid.name: 200,
        },
        arguments=[
            arg_int,
            arg_float,
            arg_string,
            arg_bool,
            arg_array,
            arg_aggregate
        ],
        entries=[
            yp.ArgumentEntry(arg_int),
            yp.ArgumentEntry(arg_float),
            yp.ArgumentEntry(arg_string),
            yp.ArgumentEntry(arg_bool),
            yp.ArgumentEntry(arg_array),
            yp.ArgumentEntry(arg_aggregate)
        ]
    )


    # Create an XML that conforms to XTCE
    xtce_file = open(filename, 'w')
    xtce_file.write(spacecraft.dumps())


# Mocking the original execution structure as provided by the user
if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: pymdb_generate_all_types.py <output_xtce_file>")
        sys.exit(1)

    filename = sys.argv[1]
    generate_xtce_all_types(filename)
