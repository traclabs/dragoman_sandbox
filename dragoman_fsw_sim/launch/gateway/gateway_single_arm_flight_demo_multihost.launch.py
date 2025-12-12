from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

  config = os.path.join(get_package_share_directory('dragoman_fsw_sim'), 
           'config', 'gateway_single_arm', 'flight_bridge_multihost.yaml')

  single_flight_demo_multihost = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('dragoman_fsw_sim'), 
         'launch', 'gateway', 'gateway_single_arm_flight_demo.launch.py')]),
      launch_arguments={'cfs_ip': "10.5.0.3",
                        'robot_ip': "10.5.0.4"}.items()
  )  

  return LaunchDescription(
    [single_flight_demo_multihost]
  )
