#!/usr/bin/env python3
"""
iMetro Ground Visualization Launch File
Launches RViz, robot_state_publisher, and joint state publisher for ground visualization.
No simulation - only visualization of telemetry from Yamcs.
Runs in default ROS_DOMAIN_ID=0 (ground side).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Generate launch description for iMetro ground visualization."""

    # Get package directories
    chonkur_description_path = get_package_share_directory("chonkur_description")
    dragoman_ground_example_path = get_package_share_directory("dragoman_ground_example")
    dragoman_fsw_sim_dir = get_package_share_directory("dragoman_fsw_sim")
    dragoman_yamcs_ros_bridge_dir = get_package_share_directory("dragoman_yamcs_ros_bridge")

    # URDF/XACRO file path for CLR robot
    urdf_model_path = os.path.join(
        chonkur_description_path,
        "urdf",
        "chonkur.urdf.xacro",
    )

    # Process XACRO to get robot description
    doc = xacro.process_file(urdf_model_path)
    robot_description = {"robot_description": doc.toxml()}

    # Bridge configuration file
    bridge_config = os.path.join(
        dragoman_yamcs_ros_bridge_dir,
        "config",
        "yamcs_bridge_params.yaml"
    )

    # RViz configuration file
    rviz_config = os.path.join(dragoman_fsw_sim_dir, "rviz", "clr_sim.rviz")

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

    # Yamcs-ROS Bridge - subscribes to Yamcs and publishes telemetry messages
    yamcs_ros_bridge = Node(
        package="dragoman_yamcs_ros_bridge",
        executable="yamcs_ros_bridge.py",
        name="yamcs_imetro_bridge",
        output="screen",
        arguments=[
            "--config", bridge_config,
            "--yamcs-url", LaunchConfiguration("yamcs_url"),
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # Joint State Publisher - converts IMetroTelemetryPacket to JointState
    joint_state_publisher = Node(
        package="dragoman_ground_example",
        executable="imetro_joint_state_publisher.py",
        name="imetro_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_topic": "/yamcs/imetro",
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
