from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('clr_deploy'), 'launch'])
    dragoman_dir = get_package_share_directory('dragoman_sandbox')
    
    rviz_config = os.path.join(dragoman_dir, "rviz/clr_sim.rviz")
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'control.launch.py']),
            launch_arguments={
            'use_fake_hardware': 'true',
            'include_mockups_in_description': 'true',
          }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
              "-d",
              rviz_config,
            ],
        ),
    ])
