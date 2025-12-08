#!/usr/bin/env python3

import sys
import yamcs.pymdb as yp
import os

# ===============================================================================
# Telemetry: Joint state (float[9]: 6 arm + 1 finger + 1 rail + 1 lift joints
# Command: arm_joint_goal (float[6]), finger_joint_goal(float),
#          rail_joint_goal (float), lift_joint_goal(float)
# ================================================================================
def generate_xtce_imetro_demo(filename):

  spacecraft = yp.System("IMetro")

  # Set CCSDS header
  ccsds_header = yp.ccsds.add_ccsds_header(spacecraft)


  # ********************************
  # Generic iMetro command
  # ********************************
  command_id = yp.IntegerArgument(
    name="comand_id",
    signed = False,
    encoding = yp.uint16_t,
  )

  # IMetro abstract Command
  imetro_command = yp.Command(
     system=spacecraft,
     name="IMetroPacket",
     abstract = True,
     base = ccsds_header.tc_command,
     assignments = {
       ccsds_header.tc_secondary_header.name: "Not Present",
       ccsds_header.tc_apid.name: 101,
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

  telemetry_container = yp.Container(
    system=spacecraft,
    name="IMetroTelemetryPacket",
    base=ccsds_header.tm_container,
    entries=[
      yp.ParameterEntry(parameter=joint_state_parameter)
    ]
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

