#!/usr/bin/env python3
"""
Curiosity Ground Visualization Launch File
Launches RViz, robot_state_publisher, and Yamcs-ROS bridge for ground visualization.
No Gazebo simulation - only visualization of telemetry from Yamcs.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Generate launch description for ground visualization."""

    # Get package directories
    curiosity_rover_models_path = get_package_share_directory("curiosity_description")
    dragoman_sandbox_path = get_package_share_directory("dragoman_sandbox")

    # URDF/XACRO file path
    urdf_model_path = os.path.join(
        curiosity_rover_models_path,
        "models",
        "urdf",
        "curiosity_mars_rover.xacro",
    )

    # Process XACRO to get robot description
    doc = xacro.process_file(urdf_model_path)
    robot_description = {"robot_description": doc.toxml()}

    # RViz configuration file
    rviz_config = os.path.join(dragoman_sandbox_path, "rviz", "curiosity.rviz")

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
            "yamcs_instance",
            default_value="curiosity",
            description="Yamcs instance name"
        ),
        DeclareLaunchArgument(
            "yamcs_processor",
            default_value="realtime",
            description="Yamcs processor name"
        ),
    ]

    # Robot State Publisher - publishes TF transforms from URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
    )

    # Yamcs-ROS Bridge - subscribes to Yamcs and publishes joint states
    # Note: The script is located in src/dragoman_sandbox/scripts/curiosity_yamcs_ros_bridge.py
    yamcs_ros_bridge = Node(
        package="dragoman_sandbox",
        executable="curiosity_yamcs_ros_bridge.py",
        name="yamcs_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "yamcs_url": LaunchConfiguration("yamcs_url"),
                "yamcs_instance": LaunchConfiguration("yamcs_instance"),
                "yamcs_processor": LaunchConfiguration("yamcs_processor"),
                "joint_state_parameter": "/Spacecraft/joint_state",
                "publish_rate": 10.0,
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
            rviz_node,
        ]
    )
