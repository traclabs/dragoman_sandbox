#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable


#####################################
def generate_launch_description():

    dragoman_fsw_sim_dir = get_package_share_directory("dragoman_fsw_sim")

    # Process SRDF through xacro
    srdf_file = os.path.join(dragoman_fsw_sim_dir, "config", "curiosity.srdf")
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
        package="dragoman_fsw_sim",
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

    # Include curiosity_gazebo launch file directly
    curiosity_gazebo_launch_dir = PathJoinSubstitution([FindPackageShare('curiosity_gazebo'), 'launch'])
    curiosity_gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([curiosity_gazebo_launch_dir, 'curiosity_gazebo.launch.py'])
    )

    return LaunchDescription(
        [
            # Set ROS_DOMAIN_ID to 101 for spacecraft system
            SetEnvironmentVariable('ROS_DOMAIN_ID', '101'),
        ]
        + launch_args
        + [
            simulator_node,
            curiosity_gazebo_launch,
        ]
    )
