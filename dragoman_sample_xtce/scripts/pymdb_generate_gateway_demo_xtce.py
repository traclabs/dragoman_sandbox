#!/usr/bin/env python3
"""
Generate XTCE for Gateway demo.

Telemetry: Joint state (float[7]: 7 arm joints)
Command: ee_goal_pose (float[7]: xyz position + xyzw quaternion)

References CCSDSHeader.xtce and Cfs.xtce for packet structure.
"""

import sys
import yamcs.pymdb as yp
import os

# ===============================================================================
# Telemetry: Joint state (float[6]: 6 arm
# Command: arm_joint_goal (float[6]), position (float[3]
# ================================================================================
def generate_xtce_gateway_demo(filename):

  spacecraft = yp.System("Gateway")

  # CMD: 0x1800 TLM: 0x8000
  #define EDORAS_APP_CMD_MID     (CFE_PLATFORM_CMD_MID_BASE + 0x27)
  #define EDORAS_APP_TLM_MID   (CFE_PLATFORM_TLM_MID_BASE + 0x27)
  CMD_MID=(0x1827 - 0x1800)
  TLM_MID= (0x0827 - 0x0800)

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
     base="/Cfs/cfs_command_packet",
     assignments={
       "ccsds_apid": CMD_MID,
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

  # ********************************
  # Generic Gateway command
  # ********************************

  # Gateway abstract Command - references external cFS command with secondary header
  gateway_command = yp.Command(
     system=spacecraft,
     name="GatewayPacket",
     abstract=True,
     base="/Cfs/cfs_command_packet",
     assignments={
       "ccsds_apid": CMD_MID,
       "scr_header": {
         "fcn_code": 0x01,
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

  # Timestamp parameters (matching ROS 2 builtin_interfaces/Time)
  timestamp_sec_parameter = yp.IntegerParameter(
    system=spacecraft,
    name="timestamp_sec",
    signed=True,
    bits=32,
    encoding=yp.int32le_t,
    short_description="Timestamp seconds (int32)"
  )

  timestamp_nanosec_parameter = yp.IntegerParameter(
    system=spacecraft,
    name="timestamp_nanosec",
    signed=False,
    bits=32,
    encoding=yp.uint32le_t,
    short_description="Timestamp nanoseconds (uint32)"
  )

  # Telemetry container - references external cFS telemetry container with secondary header
  telemetry_container = yp.Container(
    system=spacecraft,
    name="GatewayTelemetryPacket",
    base="/Cfs/cfs_telemetry_packet",
    entries=[
      yp.ParameterEntry(parameter=joint_state_parameter),
      yp.ParameterEntry(parameter=timestamp_sec_parameter),
      yp.ParameterEntry(parameter=timestamp_nanosec_parameter)
    ],
    condition=yp.eq("/CCSDSHeader/ccsds_packet_id/apid", TLM_MID)
  )

  # Create an XML that conforms to XTCE
  xtce_file = open(filename, 'w')
  xtce_file.write(spacecraft.dumps())


# **********************************************
if __name__ == '__main__':
  if len(sys.argv) != 2:
    print("Usage: pymdb_generate_gateway_demo_xtce.py <output_xtce_file>")
    sys.exit(1)

  filename = sys.argv[1]
  generate_xtce_gateway_demo(filename)

