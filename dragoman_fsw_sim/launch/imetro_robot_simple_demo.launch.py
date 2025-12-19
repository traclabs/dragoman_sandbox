#!/usr/bin/env python3

import os
import xacro
import yaml

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


#####################################
def generate_launch_description():

    # Define launch arguments
    launch_args = [
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("tm_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("tm_port", default_value="10017"),
        DeclareLaunchArgument("tc_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("tc_port", default_value="10027"),
        DeclareLaunchArgument("rate", default_value="1"),
        DeclareLaunchArgument("rviz", default_value="True"),
    ]

    # Include clr_sim.launch.py from clr_deploy package
    clr_deploy_dir = PathJoinSubstitution([FindPackageShare('clr_deploy'), 'launch'])
    clr_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([clr_deploy_dir, 'clr_sim.launch.py'])
        )
    )

    # Process SRDF through xacro
    srdf_file = PathJoinSubstitution([
        FindPackageShare("clr_moveit_config"),
        "srdf",
        "clr.srdf.xacro"
    ])
    simulator_node = Node(
        package="dragoman_fsw_sim",
        executable="imetro_demo_robot_simple_simulator.py",
        output="both",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"tm_host": LaunchConfiguration("tm_host")},
            {"tm_port": LaunchConfiguration("tm_port")},
            {"tc_host": LaunchConfiguration("tc_host")},
            {"tc_port": LaunchConfiguration("tc_port")},
            {"rate": LaunchConfiguration("rate")},
            {"robot_description_semantic": ParameterValue(Command(["xacro ", srdf_file]), value_type=str)},
        ],
    )

    # RViz configuration file
    dragoman_fsw_sim_dir = get_package_share_directory('dragoman_fsw_sim')
    rviz_config = os.path.join(dragoman_fsw_sim_dir, "rviz/clr_sim.rviz")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d", rviz_config, "-t", "RViz (Flight Software Simulation)"],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            # Set ROS_DOMAIN_ID to 100 for spacecraft system
            SetEnvironmentVariable('ROS_DOMAIN_ID', '100'),
        ]
        + launch_args
        + [
            clr_sim_launch,
            simulator_node,
            rviz_node,
        ]
    )
