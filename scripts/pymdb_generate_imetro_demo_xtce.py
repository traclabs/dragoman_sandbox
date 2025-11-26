#!/usr/bin/env python3

import rclpy
import yamcs.pymdb as yp

from ament_index_python.packages import get_package_share_directory
import os

# -----------------------------------------
def generate_xtce_imetro_demo(filename):

  spacecraft = yp.System("Spacecraft")
  
  # Set CCSDS header
  ccsds_header = yp.ccsds.add_ccsds_header(spacecraft)
  
  # Command argument
  command_type = yp.StringArgument(
    name="command_type",
    min_length=1,
    max_length=30,
    encoding=yp.StringEncoding()
  )
  
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


  # Command argument
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
  )
  
  # Command to request IC poses
  command_set_pose = yp.Command(
    system=spacecraft,
    base=imetro_command,  
    name="set_pose",
    arguments=[
       group_arg,
       group_state_arg
     ],
     entries=[
       yp.ArgumentEntry(group_arg),
       yp.ArgumentEntry(group_state_arg)
     ]
  )

  # Command to move the arm to a non-specific pose
  send_arm_pose_command = yp.Command(
    system=spacecraft,
    base=imetro_command,
    name="send_arm_pose",
    short_description="Send joint pose goal",
    #assignments={command_id.name: 4},
    arguments=[
      yp.commands.ArrayArgument(
        name="arm_joint_values",
        data_type=yp.datatypes.FloatDataType(),
        length=6
      )  
    ]
  )

  # Define parameter type
  #joint_state_parameter = yp.FloatParameter(
  #  system=spacecraft,
  #  name="joint_state",
  #  encoding=yp.float32_t
  #)
  
  joint_state_parameter = yp.ArrayParameter(
    system=spacecraft,
    name="joint_state",
    data_type=yp.datatypes.FloatDataType(encoding=yp.float32_t),
    length=6
  )
  
  
  # Set telemetry packet
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

  rclpy.init(args=None)
  filename = os.path.join( get_package_share_directory('dragoman_sandbox'), "xtce", "xtce_imetro_demo.xml")
  generate_xtce_imetro_demo(filename)

