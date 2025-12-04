#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


#####################################
def generate_launch_description():

    dragoman_sandbox_dir = get_package_share_directory("dragoman_sandbox")

    # Process SRDF through xacro
    srdf_file = os.path.join(dragoman_sandbox_dir, "config", "curiosity.srdf")
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(Command(["xacro ", srdf_file]), value_type=str)
    }

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
        executable="curiosity_simple_simulator.py",
        output="both",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"tm_host": LaunchConfiguration("tm_host")},
            {"tm_port": LaunchConfiguration("tm_port")},
            {"tc_host": LaunchConfiguration("tc_host")},
            {"tc_port": LaunchConfiguration("tc_port")},
            {"rate": LaunchConfiguration("rate")},
            robot_description_semantic,
        ],
    )

    launch_robot_dir = PathJoinSubstitution([FindPackageShare("dragoman_sandbox"), "launch"])
    robot_launch = IncludeLaunchDescription(
        PathJoinSubstitution([launch_robot_dir, "view_curiosity.launch.py"])
    )

    return LaunchDescription(
        launch_args
        + [
            simulator_node,
            robot_launch,
        ]
    )
