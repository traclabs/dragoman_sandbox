#!/usr/bin/env python3
"""
Lunar Exploration Ground Visualization Launch File

Launches RViz, robot_state_publisher, and joint state publisher for ground visualization.
No simulation - only visualization of telemetry from YAMCS.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Lunar Exploration ground visualization."""

    # Get package directories
    dragoman_ground_example_dir = get_package_share_directory("dragoman_ground_example")

    # Bridge configuration file
    lunar_bridge_config = os.path.join(
        dragoman_ground_example_dir,
        "config", "lunar_exploration",
        "yamcs_bridge_params.yaml"
    )

    # RViz configuration file
    rviz_config = os.path.join(dragoman_ground_example_dir, "rviz", "lunar_exploration.rviz")

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
            default_value=lunar_bridge_config,
            description="Path to YAMCS bridge configuration file"
        ),
    ]

    # Robot visualization
    robot = GroupAction([
      IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare("dragoman_ground_example"), "launch", "lunar_exploration", "view_lunar_exploration.launch.py"]),
      launch_arguments={
        "rviz": "False",
        "robot_publisher": "True",
        "joint_publisher": "False",
      }.items()
    ) ])


    # Yamcs-ROS Bridge - subscribes to Yamcs and publishes telemetry messages
    yamcs_ros_bridge = Node(
        package="dragoman_yamcs_ros_bridge",
        executable="yamcs_ros_bridge.py",
        name="yamcs_curiosity_bridge",
        output="screen",
        arguments=[
            "--config", LaunchConfiguration("bridge_config"),
            "--yamcs-url", LaunchConfiguration("yamcs_url"),
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # Joint State Publisher - converts MobileServicingSystemTelemetryPacket to JointState
    joint_state_publisher = Node(
        package="dragoman_ground_example",
        executable="lunar_exploration_joint_state_publisher.py",
        name="lunar_exploration_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_topic": "/yamcs/lunar_exploration",
            }
        ],
    )

    # RViz2 - visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config, "-t", "RViz (GROUND)"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        launch_args
        + [
            robot,
            yamcs_ros_bridge,
            joint_state_publisher,
            rviz_node,
        ]
    )
