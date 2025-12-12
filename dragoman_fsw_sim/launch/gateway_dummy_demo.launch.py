#!/usr/bin/env python3

import os
import xacro
import yaml

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


#####################################
def generate_launch_description():

    # rosfsw: 10.5.0.4, rosgsw: 10.5.0.2 fsw: 10.5.0.3
    # Define launch arguments
    launch_args = [
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("tm_host", default_value="10.5.0.2"), #10.5.0.4
        DeclareLaunchArgument("tm_port", default_value="1235"), #1234 10015
        DeclareLaunchArgument("tc_host", default_value="10.5.0.4"), #10.5.0.3
        DeclareLaunchArgument("tc_port", default_value="1235"), #1235 10025
        DeclareLaunchArgument("rate", default_value="1")
    ]


    simulator_node = Node(
        package="dragoman_fsw_sim",
        executable="gateway_demo_dummy_simulator.py",
        output="both",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"tm_host": LaunchConfiguration("tm_host")},
            {"tm_port": LaunchConfiguration("tm_port")},
            {"tc_host": LaunchConfiguration("tc_host")},
            {"tc_port": LaunchConfiguration("tc_port")},
            {"rate": LaunchConfiguration("rate")},
            #{"robot_description_semantic": ParameterValue(Command(["xacro ", srdf_file]), value_type=str)},
        ],
    )

    return LaunchDescription(
        launch_args
        + [
            simulator_node,
        ]
    )
