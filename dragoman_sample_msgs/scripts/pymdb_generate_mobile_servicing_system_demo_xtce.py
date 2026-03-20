#!/usr/bin/env python3
"""
Generate XTCE for Mobile Servicing System demo.

Telemetry: Joint state (float[47]) + motion command status (uint8[8])
  - Joint state: all joints for MBS, Canadarm2, Dextre body/arms, SARJ, and BGAs
  - Motion command status: status for mbs, canadarm2, dextre_body, dextre_arm_1, dextre_arm_2, sarj, port_bga, starboard_bga
    Status values: 0=IDLE, 1=IN_PROGRESS, 2=DONE

Command: send_canned_pose (group_name, group_canned_pose)
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

  # ***************************************
  # send_canned_pose Command
  # ***************************************

  # Command arguments
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
    base=cFS_command,
    name="send_canned_pose",
    short_description="Send a canned pose",
    assignments={
      ccsds_header.tc_apid.name: CMD_MID,
      "scr_header": {
        "fcn_code": 0x01,
        "checksum": 0
      }
    },
    arguments=[
       group_arg,
       group_state_arg
     ],
     entries=[
       yp.ArgumentEntry(group_arg),
       yp.ArgumentEntry(group_state_arg)
     ]
  )

  # ********************************
  # Command to grasp battery
  # ********************************
  battery_name_arg = yp.StringArgument(
    name="battery_name",
    min_length=0,
    max_length=30,
    encoding=yp.StringEncoding()
  )

  command_grasp_battery = yp.Command(
    system=spacecraft,
    base=cFS_command,
    name="GraspBattery",
    short_description="Grasp a battery",
    assignments={
      ccsds_header.tc_apid.name: CMD_MID,
      "scr_header": {
        "fcn_code": 0x02,
        "checksum": 0
      }
    },
    arguments=[
       battery_name_arg
     ],
     entries=[
       yp.ArgumentEntry(battery_name_arg)
     ]
  )

  # ********************************
  # Command to release battery
  # ********************************
  command_release_battery = yp.Command(
    system=spacecraft,
    base=cFS_command,
    name="ReleaseBattery",
    short_description="Release a battery",
    assignments={
      ccsds_header.tc_apid.name: CMD_MID,
      "scr_header": {
        "fcn_code": 0x03,
        "checksum": 0
      }
    },
    arguments=[
       battery_name_arg
     ],
     entries=[
       yp.ArgumentEntry(battery_name_arg)
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
    length=49,
    short_description="full_joint [49]"
  )

  # Define reusable motion status choices
  motion_status_choices = [
    (0, "IDLE"),
    (1, "IN_PROGRESS"),
    (2, "DONE")
  ]

  # Motion command status for all subsystems
  motion_command_status = yp.AggregateParameter(
    system=spacecraft,
    name="motion_command_status",
    members=[
      yp.EnumeratedMember(name="mbs", choices=motion_status_choices, encoding=yp.uint8_t),
      yp.EnumeratedMember(name="canadarm2", choices=motion_status_choices, encoding=yp.uint8_t),
      yp.EnumeratedMember(name="dextre_body", choices=motion_status_choices, encoding=yp.uint8_t),
      yp.EnumeratedMember(name="dextre_arm_1", choices=motion_status_choices, encoding=yp.uint8_t),
      yp.EnumeratedMember(name="dextre_arm_2", choices=motion_status_choices, encoding=yp.uint8_t),
      yp.EnumeratedMember(name="sarj", choices=motion_status_choices, encoding=yp.uint8_t),
      yp.EnumeratedMember(name="port_bga", choices=motion_status_choices, encoding=yp.uint8_t),
      yp.EnumeratedMember(name="starboard_bga", choices=motion_status_choices, encoding=yp.uint8_t)
    ],
    short_description="Motion command status"
  )

  telemetry_container = yp.Container(
    system=spacecraft,
    name="MobileServicingSystemTelemetryPacket",
    base=cFS_telemetry_container,
    entries=[
      yp.ParameterEntry(parameter=joint_state_parameter),
      yp.ParameterEntry(parameter=motion_command_status)
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

