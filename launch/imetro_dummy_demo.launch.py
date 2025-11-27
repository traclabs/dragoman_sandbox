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

    launch_args = [
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("tm_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("tm_port", default_value="10015"),
        DeclareLaunchArgument("tc_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("tc_port", default_value="10025"),
        DeclareLaunchArgument("rate", default_value="1"),
    ]
    
    simulator_node = Node(
        package="dragoman_sandbox",
        executable="imetro_demo_dummy_simulator.py",
        output="both",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
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
