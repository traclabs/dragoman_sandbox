#!/usr/bin/env python3

import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition


#####################################
def generate_launch_description():

    dragoman_dir = get_package_share_directory("dragoman_sandbox")
    test_data = os.path.join(dragoman_dir, 'data/testdata.ccsds')

    launch_args = [
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("test_data", default_value=test_data),
        DeclareLaunchArgument("tm_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("tm_port", default_value="10015"),
        DeclareLaunchArgument("tc_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("tc_port", default_value="10025"),
        DeclareLaunchArgument("rate", default_value="1"),
    ]
    
    simulator_node = Node(
        package="dragoman_sandbox",
        executable="imetro_demo_simulator.py",
        output="both",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"test_data": LaunchConfiguration("test_data")},
            {"tm_host": LaunchConfiguration("tm_host")},
            {"tm_port": LaunchConfiguration("tm_port")},
            {"tc_host": LaunchConfiguration("tc_host")},
            {"tc_port": LaunchConfiguration("tc_port")},
            {"rate": LaunchConfiguration("rate")},
        ]
    )


    return LaunchDescription(
        launch_args + 
        [simulator_node,
        ]
    )
