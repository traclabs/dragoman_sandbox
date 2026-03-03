#!/usr/bin/env python3
"""
Generate XTCE for Lunar Exploration demo.

Telemetry: Joint state wheels + odometry
Command: twist (float[2]: linear and angular velocity) or camera joint
"""

import sys
import yamcs.pymdb as yp
import os

from cfs_msg_hdr import (
    add_cfs_command_header,
    add_cfs_telemetry_header
)

# ===============================================================================
# Telemetry: Joint state + odom pose (float[17 + 7 = 24]: 17 joints + xyz, qxyzw
# Command: linear velocity + angular velocity
# ================================================================================
def generate_xtce_lunar_exploration_demo(filename):

  spacecraft = yp.System("LunarExploration")

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
  # Generic Lunar Exploration command
  # ***************************************

  # Lunar Exploration abstract Command
  le_command = yp.Command(
     system=spacecraft,
     name="LunarExplorationPacket",
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

  # **************************************************
  # Command to send velocity linear + angular
  # **************************************************

  # Important! Use le_t (little endian) for compatibility with cFS
  twist_rover_arg = yp.commands.ArrayArgument(
        name="twist",
        data_type=yp.datatypes.FloatDataType(encoding=yp.float32le_t),
        length=2
      )

  twist_rover_command = yp.Command(
    system=spacecraft,
    base=le_command,
    name="twist_rover_command",
    short_description="Send a twist [vel_lin, vel_ang]",

    arguments=[
      twist_rover_arg
    ],
    entries=[
      yp.ArgumentEntry(twist_rover_arg)
    ]
  )

  # **************************************************
  # Command to move camera
  # **************************************************

  # Important! Use le_t (little endian) for compatibility with cFS
  camera_joint_arg = yp.commands.ArrayArgument(
        name="camera_joint",
        data_type=yp.datatypes.FloatDataType(encoding=yp.float32le_t),
        length=2
      )

  camera_joint_command = yp.Command(
     system=spacecraft,
     name="CameraJointPacket",
     short_description="Rotate camera [ pan [-3.49, 3.49], tilt [-1.30, 1.30]]",
     #abstract=True,
     base=cFS_command,
     assignments={
       ccsds_header.tc_apid.name: CMD_MID,
       "scr_header": {
         "fcn_code": 0x02,
         "checksum": 0
        }
     },
     arguments=[
        camera_joint_arg
     ],
     entries=[
      yp.ArgumentEntry(camera_joint_arg)
     ]
  )

  # **************************************************
  # Command to navigate to a pose
  # **************************************************

  navigation_pose_arg = yp.commands.AggregateArgument(
        name="pose",
        members=[
          yp.FloatMember(name="x", bits=32, encoding=yp.float32le_t),
          yp.FloatMember(name="y", bits=32, encoding=yp.float32le_t),
          yp.FloatMember(name="theta", bits=32, encoding=yp.float32le_t),
        ]
      )

  navigation_pose_command = yp.Command(
     system=spacecraft,
     name="NavigationPosePacket",
     short_description="Send navigation pose (x, y, theta)",
     base=cFS_command,
     assignments={
       ccsds_header.tc_apid.name: CMD_MID,
       "scr_header": {
         "fcn_code": 0x03,
         "checksum": 0
        }
     },
     arguments=[
        navigation_pose_arg
     ],
     entries=[
      yp.ArgumentEntry(navigation_pose_arg)
     ]
  )


  #############################################
  # TELEMETRY
  #############################################

  cFS_telemetry_container = add_cfs_telemetry_header(spacecraft, ccsds_header)

  # ***********************************************
  # Telemetry packet containing joint state + odom pose
  # ***********************************************
  joint_state_parameter = yp.ArrayParameter(
    system=spacecraft,
    name="joint_state",
    data_type=yp.datatypes.FloatDataType(encoding=yp.float32le_t),
    length=17,
    short_description="Joint state (17 floats)"
  )

  pose_parameter = yp.AggregateParameter(
    system=spacecraft,
    name="pose",
    members=[
      yp.AggregateMember(
        name="position",
        members=[
          yp.FloatMember(name="x", bits=32, encoding=yp.float32le_t),
          yp.FloatMember(name="y", bits=32, encoding=yp.float32le_t),
          yp.FloatMember(name="z", bits=32, encoding=yp.float32le_t),
        ],
      ),
      yp.AggregateMember(
        name="orientation",
        members=[
          yp.FloatMember(name="x", bits=32, encoding=yp.float32le_t),
          yp.FloatMember(name="y", bits=32, encoding=yp.float32le_t),
          yp.FloatMember(name="z", bits=32, encoding=yp.float32le_t),
          yp.FloatMember(name="w", bits=32, encoding=yp.float32le_t),
        ],
      ),
    ],
    short_description="Odometry pose (position + orientation quaternion)"
  )

  # Navigation status enumeration parameter
  navigation_status_parameter = yp.EnumeratedParameter(
    system=spacecraft,
    name="navigation_status",
    choices=[
      (0, "IDLE"),
      (1, "IN_PROGRESS"),
      (2, "DONE")
    ],
    encoding=yp.uint8_t,
    short_description="Navigation command status"
  )

  # Battery aggregate parameter
  battery_parameter = yp.AggregateParameter(
    system=spacecraft,
    name="battery",
    members=[
      yp.FloatMember(name="voltage", bits=32, encoding=yp.float32le_t, units="V", short_description="Battery voltage"),
      yp.FloatMember(name="current", bits=32, encoding=yp.float32le_t, units="A", short_description="Battery current (amps)"),
      yp.FloatMember(name="percentage", bits=32, encoding=yp.float32le_t, units="%", short_description="Battery charge percentage"),
      yp.BooleanMember(name="charging", zero_string_value="Not charging", one_string_value="Charging", encoding=yp.uint8_t, short_description="Battery charging status"),
      yp.FloatMember(name="temperature", bits=32, encoding=yp.float32le_t, units="°C", short_description="Battery temperature in celsius"),
    ],
    short_description="Battery status"
  )

  telemetry_container = yp.Container(
    system=spacecraft,
    name="LunarExplorationTelemetryPacket",
    base=cFS_telemetry_container,
    entries=[
      yp.ParameterEntry(parameter=joint_state_parameter),
      yp.ParameterEntry(parameter=pose_parameter),
      yp.ParameterEntry(parameter=navigation_status_parameter),
      yp.ParameterEntry(parameter=battery_parameter)
    ],
    condition=yp.eq(ccsds_header.tm_apid, TLM_MID)
  )

  # Create an XML that conforms to XTCE
  xtce_file = open(filename, 'w')
  xtce_file.write(spacecraft.dumps())


# **********************************************
if __name__ == '__main__':
  if len(sys.argv) != 2:
    print("Usage: pymdb_generate_lunar_exploration_demo_xtce.py <output_xtce_file>")
    sys.exit(1)

  filename = sys.argv[1]
  generate_xtce_lunar_exploration_demo(filename)
