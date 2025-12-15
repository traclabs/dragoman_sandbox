#!/usr/bin/env python3

import sys
import yamcs.pymdb as yp

# ===============================================================================
# Generate XTCE file with just CCSDS header
# ================================================================================
def generate_xtce_ccsds_header(filename):
  """
  Generate an XTCE file containing only the CCSDS header definition.

  Args:
    filename: Output path for the XTCE file
  """
  # Create a system for the CCSDS header
  spacecraft = yp.System("CCSDSHeader")

  # Add CCSDS header - this creates the standard CCSDS packet header
  # with telemetry and telecommand definitions
  ccsds_header = yp.ccsds.add_ccsds_header(spacecraft)

  # Write the XTCE XML to file
  with open(filename, 'w') as xtce_file:
    xtce_file.write(spacecraft.dumps())


# **********************************************
if __name__ == '__main__':
  if len(sys.argv) != 2:
    print("Usage: pymdb_generate_ccsds_header.py <output_xtce_file>")
    print("Example: pymdb_generate_ccsds_header.py ccsds_header.xtce")
    sys.exit(1)

  filename = sys.argv[1]
  generate_xtce_ccsds_header(filename)
