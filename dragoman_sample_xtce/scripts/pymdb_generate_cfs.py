#!/usr/bin/env python3
"""
Generate XTCE for cFS packets with secondary headers.

Command: fcn_code (uint8), checksum (uint8)
Telemetry: Sec (uint8[6]), Spare (uint8[4])

References CCSDSHeader.xtce for packet structure.
"""

import sys
import yamcs.pymdb as yp

def generate_xtce_cfs(filename):
  """
  Generate an XTCE file containing cFS packet definitions with secondary headers.

  Args:
    filename: Output path for the XTCE file
  """
  # Create a system for the cFS packets
  spacecraft = yp.System("Cfs")

  # **********************************************
  # cFS Command Secondary Header
  # **********************************************
  scr_header = yp.AggregateArgument(
    name="scr_header",
    members=[
      yp.IntegerMember(
        name="fcn_code",
        signed=False,
        bits=8,
        encoding=yp.uint8_t,
        initial_value=0
      ),
      yp.IntegerMember(
        name="checksum",
        signed=False,
        bits=8,
        encoding=yp.uint8_t,
        initial_value=0
      )
    ]
  )

  cfs_command = yp.Command(
     system=spacecraft,
     name="cfs_command_packet",
     abstract=True,
     base="/CCSDSHeader/ccsds_space_packet",
     assignments={
       "ccsds_secondary_header": "Present",
     },
     arguments=[
       scr_header
     ],
     entries=[
       yp.ArgumentEntry(scr_header)
     ]
  )

  # *************************************
  # cFS Telemetry Secondary Header
  # *************************************
  secondary_header_parameter = yp.AggregateParameter(
    system=spacecraft,
    name="scr_header",
    members=[
      yp.ArrayMember(
        name="Sec",
        data_type=yp.datatypes.IntegerDataType(encoding=yp.uint8_t, bits=8),
        length=6
      ),
      yp.ArrayMember(
        name="Spare",
        data_type=yp.datatypes.IntegerDataType(encoding=yp.uint8_t, bits=8),
        length=4
      )
    ]
  )

  cfs_telemetry_container = yp.Container(
    system=spacecraft,
    name="cfs_telemetry_packet",
    abstract=True,
    base="/CCSDSHeader/ccsds_space_packet",
    entries=[
      yp.ParameterEntry(parameter=secondary_header_parameter)
    ]
  )

  # Write the XTCE XML to file
  with open(filename, 'w') as xtce_file:
    xtce_file.write(spacecraft.dumps())


# **********************************************
if __name__ == '__main__':
  if len(sys.argv) != 2:
    print("Usage: pymdb_generate_cfs.py <output_xtce_file>")
    print("Example: pymdb_generate_cfs.py cfs.xtce")
    sys.exit(1)

  filename = sys.argv[1]
  generate_xtce_cfs(filename)
