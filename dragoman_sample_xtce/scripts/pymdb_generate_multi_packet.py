#!/usr/bin/env python3
"""
Generate an XTCE file with multiple packet types.

This script demonstrates that a single XTCE file can define multiple packet layouts.
Each packet type is represented as a separate SequenceContainer that inherits from
a common base container (ccsds_space_packet). This allows you to define all telemetry
formats for a spacecraft in one organized XTCE file.

In this example:
- TemperaturePacket: Contains temperature telemetry
- VoltagePacket: Contains voltage telemetry

Both packets share the same CCSDS header structure but have different payload data.
"""

import sys
import os
import yamcs.pymdb as yp

def generate_xtce_multi_packet(filename):
    """Generate XTCE with multiple packet types"""

    spacecraft = yp.System("MultiPacket")
    ccsds_header = yp.ccsds.add_ccsds_header(spacecraft)

    # =========================================================================
    # PACKET 1: Temperature data (APID 130)
    # =========================================================================

    temp_param = yp.FloatParameter(
        system=spacecraft,
        name="Temperature",
        bits=32,
        encoding=yp.float32_t,
        units="degC"
    )

    yp.Container(
        system=spacecraft,
        name="TemperaturePacket",
        base=ccsds_header.tm_container,
        entries=[yp.ParameterEntry(parameter=temp_param)],
        condition=yp.eq(ccsds_header.tm_apid, 130)
    )

    # =========================================================================
    # PACKET 2: Voltage data (APID 131)
    # =========================================================================

    voltage_param = yp.FloatParameter(
        system=spacecraft,
        name="Voltage",
        bits=32,
        encoding=yp.float32_t,
        units="V"
    )

    yp.Container(
        system=spacecraft,
        name="VoltagePacket",
        base=ccsds_header.tm_container,
        entries=[yp.ParameterEntry(parameter=voltage_param)],
        condition=yp.eq(ccsds_header.tm_apid, 131)
    )

    # =========================================================================
    # Write XTCE file
    # =========================================================================

    with open(filename, 'w') as f:
        f.write(spacecraft.dumps())


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: pymdb_generate_multi_packet.py <output_xtce_file>")
        sys.exit(1)

    filename = sys.argv[1]
    generate_xtce_multi_packet(filename)
