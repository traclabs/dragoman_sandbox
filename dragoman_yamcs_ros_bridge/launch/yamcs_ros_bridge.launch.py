#!/usr/bin/env python3
"""
Yamcs ROS Bridge Launch File
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Get package directories
    dragoman_yamcs_ros_bridge_dir = get_package_share_directory("dragoman_yamcs_ros_bridge")

    # Default bridge configuration file
    default_bridge_config = os.path.join(
        dragoman_yamcs_ros_bridge_dir,
        "config",
        "yamcs_bridge_params.yaml"
    )

    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulation time"
        ),
        DeclareLaunchArgument(
            "yamcs_url",
            default_value="localhost:8090",
            description="Yamcs server URL"
        ),
        DeclareLaunchArgument(
            "bridge_config",
            default_value=default_bridge_config,
            description="Path to YAMCS bridge configuration file"
        ),
    ]

    # Yamcs-ROS Bridge - subscribes to Yamcs and publishes telemetry messages
    yamcs_ros_bridge = Node(
        package="dragoman_yamcs_ros_bridge",
        executable="yamcs_ros_bridge.py",
        name="yamcs_ros_bridge",
        output="screen",
        arguments=[
            "--config", LaunchConfiguration("bridge_config"),
            "--yamcs-url", LaunchConfiguration("yamcs_url"),
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )


    return LaunchDescription(
        launch_args + [yamcs_ros_bridge]
    )
