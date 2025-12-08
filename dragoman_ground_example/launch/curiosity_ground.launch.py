#!/usr/bin/env python3
"""
Curiosity Ground Visualization Launch File
Launches RViz, robot_state_publisher, and joint state publisher for ground visualization.
No simulation - only visualization of telemetry from Yamcs.
Runs in default ROS_DOMAIN_ID=0 (ground side).
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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Curiosity ground visualization."""

    # Get package directories
    dragoman_ground_example_dir = get_package_share_directory("dragoman_ground_example")
    dragoman_yamcs_ros_bridge_dir = get_package_share_directory("dragoman_yamcs_ros_bridge")

    # Bridge configuration file
    bridge_config = os.path.join(
        dragoman_yamcs_ros_bridge_dir,
        "config",
        "yamcs_bridge_params.yaml"
    )

    # RViz configuration file
    rviz_config = os.path.join(dragoman_ground_example_dir, "rviz", "curiosity.rviz")

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
    ]

    # Robot State Publisher - publishes TF transforms from URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("curiosity_description"), "models", "urdf", "curiosity_mars_rover.xacro"]),
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
        name="yamcs_curiosity_bridge",
        output="screen",
        arguments=[
            "--config", bridge_config,
            "--yamcs-url", LaunchConfiguration("yamcs_url"),
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # Joint State Publisher - converts CuriosityTelemetryPacket to JointState
    joint_state_publisher = Node(
        package="dragoman_ground_example",
        executable="curiosity_joint_state_publisher.py",
        name="curiosity_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_topic": "/yamcs/curiosity",
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
