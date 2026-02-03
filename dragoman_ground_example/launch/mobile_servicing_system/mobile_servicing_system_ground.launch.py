#!/usr/bin/env python3
"""
Mobile Servicing System Ground Visualization Launch File

Launches RViz, robot_state_publisher, and joint state publisher for ground visualization.
No simulation - only visualization of telemetry from YAMCS.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Mobile Servicing System ground visualization."""

    # Get package directories
    dragoman_ground_example_dir = get_package_share_directory("dragoman_ground_example")

    # Bridge configuration file
    mss_bridge_config = os.path.join(
        dragoman_ground_example_dir,
        "config", "mobile_servicing_system",
        "yamcs_bridge_params.yaml"
    )

    # RViz configuration file
    rviz_config = os.path.join(dragoman_ground_example_dir, "rviz", "mobile_servicing_system.rviz")

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
            default_value=mss_bridge_config,
            description="Path to YAMCS bridge configuration file"
        ),
    ]

    # Robot visualization
    robot = IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare("iss_description"), "launch", "view_mobile_servicing_system.launch.py"]),
      launch_arguments={
        "rviz": "False",
        "robot_publisher": "True",
        "joint_publisher": "False",
      }.items()
    )


    # Yamcs-ROS Bridge - subscribes to Yamcs and publishes telemetry messages
    yamcs_ros_bridge = IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare("dragoman_yamcs_ros_bridge"), "launch", "yamcs_ros_bridge.launch.py"]),
      launch_arguments={
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "yamcs_url": LaunchConfiguration("yamcs_url"),
        "bridge_config": LaunchConfiguration("bridge_config"),
      }.items()
    )

    # Joint State Publisher - converts MobileServicingSystemTelemetryPacket to JointState
    joint_state_publisher = Node(
        package="dragoman_ground_example",
        executable="mobile_servicing_system_joint_state_publisher.py",
        name="mobile_servicing_system_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_topic": "/yamcs/mobile_servicing_system",
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
