#!/usr/bin/env python3
"""
Gateway Ground Visualization Launch File

Launches RViz, robot_state_publisher, and joint state publisher for ground visualization.
No simulation - only visualization of telemetry from YAMCS.
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
    """Generate launch description for Gateway ground visualization."""

    # Get package directories
    dragoman_ground_example_dir = get_package_share_directory("dragoman_ground_example")
    dragoman_yamcs_ros_bridge_dir = get_package_share_directory("dragoman_yamcs_ros_bridge")

    # Default bridge configuration file
    default_bridge_config = os.path.join(
        dragoman_ground_example_dir,
        "config", "gateway",
        "yamcs_bridge_params.yaml"
    )

    # RViz configuration file
    rviz_config = os.path.join(dragoman_ground_example_dir, "rviz", "gateway.rviz")

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

    # Robot State Publisher - publishes TF transforms from URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gateway_description"), "robots", "gateway.urdf.xacro"]),
        ]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }
        ],
    )

    # Yamcs-ROS Bridge - subscribes to Yamcs and publishes telemetry messages
    yamcs_ros_bridge = Node(
        package="dragoman_yamcs_ros_bridge",
        executable="yamcs_ros_bridge.py",
        name="yamcs_gateway_bridge",
        output="screen",
        arguments=[
            "--config", LaunchConfiguration("bridge_config"),
            "--yamcs-url", LaunchConfiguration("yamcs_url"),
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # Joint State Publisher - converts GatewayTelemetryPacket to JointState
    joint_state_publisher = Node(
        package="dragoman_ground_example",
        executable="gateway_joint_state_publisher.py",
        name="gateway_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_topic": "/yamcs/gateway",
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
            robot_state_publisher,
            yamcs_ros_bridge,
            joint_state_publisher,
            rviz_node,
        ]
    )
