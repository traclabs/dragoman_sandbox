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
       ccsds_header.tc_secondary_header.name: "NotPresent",
       ccsds_header.tc_apid.name: 101,
     },
     arguments=[
       command_id,
     ],
     entries=[
       yp.ArgumentEntry(command_id)
     ]
  )
  
  # Command to move the arm to pose 1, 2 or 3
  pose_1_command = yp.Command(
    system=spacecraft,
    base=imetro_command,  
    name="pose_1",
    assignments={command_id.name: 1}
  )
  

  pose_2_command = yp.Command(
    system=spacecraft,
    base=imetro_command,  
    name="pose_2",
    assignments={command_id.name: 2}
  )


  pose_3_command = yp.Command(
    system=spacecraft,
    base=imetro_command,  
    name="pose_3",
    assignments={command_id.name: 3}
  )

  
  
  # Create an XML that conformst to XTCE
  xtce_file = open(filename, 'w')
  xtce_file.write(spacecraft.dumps())

  # Command to move the arm to a give pose
  send_arm_pose_command = yp.Command(
    system=spacecraft,
    base=imetro_command,
    name="send_arm_pose",
    short_description="Send joit pose goal",
    assignments={command_id.name: 4},
    arguments=[
      yp.ArrayArgument(
        name="arm_joint_values",
        data_type=yp.FloatDataType,
        length=6
      )  
    ]
  )

# **********************************************    
if __name__ == '__main__':

  rclpy.init(args=None)
  filename = os.path.join( get_package_share_directory('dragoman_sandbox'), "xtce", "xtce_imetro_demo.xml")
  generate_xtce_imetro_demo(filename)

