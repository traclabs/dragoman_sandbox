#!/usr/bin/env python3

import sys
import yamcs.pymdb as yp
import os

from dragoman_sample_xtce.cfs_msg_hdr import (
    add_cfs_command_header,
    add_cfs_telemetry_header
)

# ===============================================================================
# Telemetry: Joint state (float[6]: 6 arm
# Command: arm_joint_goal (float[6]), position (float[3]
# ================================================================================
def generate_xtce_gateway_demo(filename):

  spacecraft = yp.System("Gateway")

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
     abstract = False,
     base = cFS_command,
     assignments = {
       ccsds_header.tc_apid.name: 0x1880,
       "scr_header": {
         "fcn_code":  0x06,
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

  # ********************************
  # Generic Gateway command
  # ********************************

  # Gateway abstract Command
  gateway_command = yp.Command(
     system=spacecraft,
     name="GatewayPacket",
     abstract = True,
     base = cFS_command,
     assignments = {
       ccsds_header.tc_apid.name: CMD_MID,
       "scr_header": {
         "fcn_code":  0x01,
         "checksum": 0
        } 
     },
  )

  #############################################
  # COMMAND
  #############################################


  # **************************************************
  # Command to move arm to a 3D pose (3 xyz + 4 xyzw)
  # **************************************************
  ee_pose = yp.commands.ArrayArgument(
        name="ee_pose",
        data_type=yp.datatypes.FloatDataType(encoding=yp.float32le_t),
        length=7
      )

  ee_pose_command = yp.Command(
    system=spacecraft,
    base=gateway_command,
    name="ee_goal_pose",
    short_description="Send an EE 3D pose: [x, y, z, qx, qy, qz, qw]",

    arguments=[
      ee_pose
    ],
    entries=[
      yp.ArgumentEntry(ee_pose)
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
    length=7,
    short_description="big_arm_joint_ [2..8]"
  )

  telemetry_container = yp.Container(
    system=spacecraft,
    name="GatewayTelemetryPacket",
    base=cFS_telemetry_container,
    entries=[
      yp.ParameterEntry(parameter=joint_state_parameter)
    ],
    condition=yp.eq(ccsds_header.tm_apid, TLM_MID)
  )

  # Create an XML that conformst to XTCE
  xtce_file = open(filename, 'w')
  xtce_file.write(spacecraft.dumps())


# **********************************************
if __name__ == '__main__':
  if len(sys.argv) != 2:
    print("Usage: pymdb_generate_gateway_demo_xtce.py <output_xtce_file>")
    sys.exit(1)

  filename = sys.argv[1]
  generate_xtce_gateway_demo(filename)

