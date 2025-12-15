#!/usr/bin/env python3
"""
Generate XTCE for IMetro robotic system.

Telemetry: Joint state (float[9]: 6 arm + 1 finger + 1 rail + 1 lift joints)
Commands: arm_joint_goal (float[6]), rail_joint_goal (float), lift_joint_goal (float)

References CCSDSHeader.xtce for packet structure.
"""

import sys
import yamcs.pymdb as yp
import os

def generate_xtce_imetro_demo(filename):

  spacecraft = yp.System("IMetro")


  # ********************************
  # Generic iMetro command
  # ********************************
  command_id = yp.IntegerArgument(
    name="command_id",
    signed = False,
    encoding = yp.uint16_t,
    bits = 16
  )

  # IMetro abstract Command - references external CCSDS command
  imetro_command = yp.Command(
     system=spacecraft,
     name="IMetroPacket",
     abstract = True,
     base = "/CCSDSHeader/ccsds_space_packet",
     assignments = {
       "ccsds_secondary_header": "Not Present",
       "ccsds_apid": 39,
     },
     arguments=[command_id],
     entries=[
       yp.ArgumentEntry(command_id)
     ]
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
    name="group_state",
    min_length=0,
    max_length=30,
    encoding=yp.StringEncoding()
  )


  command_set_pose = yp.Command(
    system=spacecraft,
    base=imetro_command,
    name="send_canned_pose",
    short_description="Send a canned pose",
    assignments={command_id.name: 0},
    arguments=[
       group_arg,
       group_state_arg
     ],
     entries=[
       yp.ArgumentEntry(group_arg),
       yp.ArgumentEntry(group_state_arg)
     ]
  )

  # **************************************************
  # Command to move the arm to a joint state (6DOF)
  # **************************************************
  arm_js = yp.commands.ArrayArgument(
        name="arm_joint_values",
        data_type=yp.datatypes.FloatDataType(encoding=yp.float32_t),
        length=6
      )

  arm_js_command = yp.Command(
    system=spacecraft,
    base=imetro_command,
    name="arm_joint_state_goal",
    short_description="Send an arm js: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]",
    assignments={command_id.name: 1},
    arguments=[
      arm_js
    ],
    entries=[
      yp.ArgumentEntry(arm_js)
    ]
  )


  # **************************************************
  # Command to move the rail joint (1DOF)
  # **************************************************
  rail_js = yp.commands.FloatArgument(
        name="rail_joint_value",
        encoding=yp.float32_t
      )

  rail_js_command = yp.Command(
    system=spacecraft,
    base=imetro_command,
    name="rail_joint_state_goal",
    short_description="Send a rail joint state goal",
    assignments={command_id.name: 2},
    arguments=[
      rail_js
    ],
    entries=[
      yp.ArgumentEntry(rail_js)
    ]
  )


  # **************************************************
  # Command to move the lift joint (1DOF)
  # **************************************************
  lift_js = yp.commands.FloatArgument(
        name="lift_joint_value",
        encoding=yp.float32_t
      )

  lift_js_command = yp.Command(
    system=spacecraft,
    base=imetro_command,
    name="lift_joint_state_goal",
    short_description="Send a lift joint state goal",
    assignments={command_id.name: 3},
    arguments=[
      lift_js
    ],
    entries=[
      yp.ArgumentEntry(lift_js)
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
    data_type=yp.datatypes.FloatDataType(encoding=yp.float32_t),
    length=9,
    short_description="elbow, lift, finger_1, shoulder_lift, shoulder_pan, rail, wrist_1, wrist_2, wrist_3"
  )

  # Telemetry container - references external CCSDS container
  telemetry_container = yp.Container(
    system=spacecraft,
    name="IMetroTelemetryPacket",
    base="/CCSDSHeader/ccsds_space_packet",
    entries=[
      yp.ParameterEntry(parameter=joint_state_parameter)
    ],
    condition=yp.eq("/CCSDSHeader/ccsds_packet_id/apid", 100)
  )


  # Create an XML that conformst to XTCE
  xtce_file = open(filename, 'w')
  xtce_file.write(spacecraft.dumps())


# **********************************************
if __name__ == '__main__':
  if len(sys.argv) != 2:
    print("Usage: pymdb_generate_imetro_demo_xtce.py <output_xtce_file>")
    sys.exit(1)

  filename = sys.argv[1]
  generate_xtce_imetro_demo(filename)

