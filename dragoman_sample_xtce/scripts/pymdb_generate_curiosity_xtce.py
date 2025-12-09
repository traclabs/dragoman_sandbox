#!/usr/bin/env python3

import sys
import yamcs.pymdb as yp
import os

# ===============================================================================
# Telemetry: Joint state (float[24]: all rover joints)
# Command: arm_joint_goal (float[5]), mast_joint_goal (float[3])
# ================================================================================
def generate_xtce_curiosity(filename):

  spacecraft = yp.System("Curiosity")

  # Set CCSDS header
  ccsds_header = yp.ccsds.add_ccsds_header(spacecraft)


  # ********************************
  # Generic Curiosity command
  # ********************************
  command_id = yp.IntegerArgument(
    name="command_id",
    signed = False,
    encoding = yp.uint16_t,
  )

  # Curiosity abstract Command
  curiosity_command = yp.Command(
     system=spacecraft,
     name="CuriosityPacket",
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
  # Command to request canned poses
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
    base=curiosity_command,
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
  # Command to move the arm to a joint state (5DOF)
  # **************************************************
  arm_js = yp.commands.ArrayArgument(
        name="arm_joint_values",
        data_type=yp.datatypes.FloatDataType(encoding=yp.float32_t),
        length=5
      )

  arm_js_command = yp.Command(
    system=spacecraft,
    base=curiosity_command,
    name="arm_joint_state_goal",
    short_description="Send an arm js: [arm_01, arm_02, arm_03, arm_04, arm_tools]",
    assignments={command_id.name: 1},
    arguments=[
      arm_js
    ],
    entries=[
      yp.ArgumentEntry(arm_js)
    ]
  )


  # **************************************************
  # Command to move the mast joints (3DOF)
  # **************************************************
  mast_js = yp.commands.ArrayArgument(
        name="mast_joint_values",
        data_type=yp.datatypes.FloatDataType(encoding=yp.float32_t),
        length=3
      )

  mast_js_command = yp.Command(
    system=spacecraft,
    base=curiosity_command,
    name="mast_joint_state_goal",
    short_description="Send a mast js: [mast_p, mast_02, mast_cameras]",
    assignments={command_id.name: 2},
    arguments=[
      mast_js
    ],
    entries=[
      yp.ArgumentEntry(mast_js)
    ]
  )



  #############################################
  # TELEMETRY
  #############################################


  # ***********************************************
  # Telemetry packet containing joint state data
  # ***************################################
  joint_state_parameter = yp.ArrayParameter(
    system=spacecraft,
    name="joint_state",
    data_type=yp.datatypes.FloatDataType(encoding=yp.float32_t),
    length=24,
    short_description="arm_01, arm_02, arm_03, arm_04, arm_tools, back_wheel_L, back_wheel_R, front_wheel_L, front_wheel_R, mast_02, mast_cameras, mast_p, middle_wheel_L, middle_wheel_R, suspension_arm_B2_L, suspension_arm_B2_R, suspension_arm_B_L, suspension_arm_B_R, suspension_arm_F_L, suspension_arm_F_R, suspension_steer_B_L, suspension_steer_B_R, suspension_steer_F_L, suspension_steer_F_R"
  )

  telemetry_container = yp.Container(
    system=spacecraft,
    name="CuriosityTelemetryPacket",
    base=ccsds_header.tm_container,
    entries=[
      yp.ParameterEntry(parameter=joint_state_parameter)
    ],
    condition=yp.eq(ccsds_header.tm_apid, 110)
  )


  # Create an XML that conforms to XTCE
  xtce_file = open(filename, 'w')
  xtce_file.write(spacecraft.dumps())


# **********************************************
if __name__ == '__main__':
  if len(sys.argv) != 2:
    print("Usage: pymdb_generate_curiosity_xtce.py <output_xtce_file>")
    sys.exit(1)

  filename = sys.argv[1]
  generate_xtce_curiosity(filename)
