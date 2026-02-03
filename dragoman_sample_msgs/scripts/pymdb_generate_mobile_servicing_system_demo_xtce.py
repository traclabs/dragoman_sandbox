#!/usr/bin/env python3
"""
Generate XTCE for Mobile Servicing System demo.

Telemetry: Joint state (float[31]: 1 MBS joint + 7 Canadarm2 joints + 1 Dextre waist + 6x2 Dextre arms) + (4+1)x2 solar panels
Command: ee_goal_pose (float[7]: xyz position + xyzw quaternion)
"""

import sys
import yamcs.pymdb as yp
import os

from cfs_msg_hdr import (
    add_cfs_command_header,
    add_cfs_telemetry_header
)

# ===============================================================================
# Telemetry: Joint state (float[21]: 6 arm
# Command: arm_joint_goal (float[6]), position (float[3]
# ================================================================================
def generate_xtce_mobile_servicing_system_demo(filename):

  spacecraft = yp.System("MobileServicingSystem")

  # Set CCSDS header
  ccsds_header = yp.ccsds.add_ccsds_header(spacecraft)

  # CMD: 0x1800 TLM: 0x8000
  #define EDORAS_APP_CMD_MID     (CFE_PLATFORM_CMD_MID_BASE + 0x27)
  #define EDORAS_APP_TLM_MID   (CFE_PLATFORM_TLM_MID_BASE + 0x27)
  CMD_MID=(0x1827 - 0x1800)
  TLM_MID= (0x0827 - 0x0800)

  cFS_command = add_cfs_command_header(spacecraft, ccsds_header, name="CfsPacket")

  # ********************************
  # TO Lab Enable command
  # ********************************
  dest_ip = yp.StringArgument(
    name="dest_ip",
    min_length=7,
    max_length=15,
    encoding=yp.StringEncoding()
  )

  to_lab_enable_command = yp.Command(
     system=spacecraft,
     name="TOLabEnablePacket",
     abstract=False,
     base=cFS_command,
     assignments={
       ccsds_header.tc_apid.name: 128,
       "scr_header": {
         "fcn_code": 0x06,
         "checksum": 0
        }
     },
    arguments=[
      dest_ip
    ],
    entries=[
      yp.ArgumentEntry(dest_ip)
    ]

  )

  # ***************************************
  # Generic MobileServicingSystem command
  # ***************************************

  # Mobile Servicing System abstract Command
  mss_command = yp.Command(
     system=spacecraft,
     name="MobileServicingSystemPacket",
     abstract=True,
     base=cFS_command,
     assignments={
       ccsds_header.tc_apid.name: CMD_MID,
       "scr_header": {
         "fcn_code": 0x01,
         "checksum": 0
        }
     },
  )

  #############################################
  # COMMAND
  #############################################

  # ********************************
  # Command to request IC poses
  # ********************************
  group_arg = yp.StringArgument(
    name="group_name",
    min_length=0,
    max_length=30,
    encoding=yp.StringEncoding()
  )

  group_state_arg = yp.StringArgument(
    name="group_canned_pose",
    min_length=0,
    max_length=30,
    encoding=yp.StringEncoding()
  )

  command_set_pose = yp.Command(
    system=spacecraft,
    base=mss_command,
    name="send_canned_pose",
    short_description="Send a canned pose",
    #assignments={command_id.name: 0},
    arguments=[
       group_arg,
       group_state_arg
     ],
     entries=[
       yp.ArgumentEntry(group_arg),
       yp.ArgumentEntry(group_state_arg)
     ]
  )

  #############################################
  # TELEMETRY
  #############################################

  cFS_telemetry_container = add_cfs_telemetry_header(spacecraft, ccsds_header)

  # ***********************************************
  # Telemetry packet containing joint state data
  # ***********************************************
  joint_state_parameter = yp.ArrayParameter(
    system=spacecraft,
    name="joint_state",
    data_type=yp.datatypes.FloatDataType(encoding=yp.float32le_t),
    length=31,
    short_description="full_joint [31]"
  )

  telemetry_container = yp.Container(
    system=spacecraft,
    name="MobileServicingSystemTelemetryPacket",
    base=cFS_telemetry_container,
    entries=[
      yp.ParameterEntry(parameter=joint_state_parameter)
    ],
    condition=yp.eq(ccsds_header.tm_apid, TLM_MID)
  )

  # Create an XML that conforms to XTCE
  xtce_file = open(filename, 'w')
  xtce_file.write(spacecraft.dumps())


# **********************************************
if __name__ == '__main__':
  if len(sys.argv) != 2:
    print("Usage: pymdb_generate_mobile_servicing_system_demo_xtce.py <output_xtce_file>")
    sys.exit(1)

  filename = sys.argv[1]
  generate_xtce_mobile_servicing_system_demo(filename)

