#!/usr/bin/env python3

import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition

########################################
def evaluate_nodes(context, *args, **kwargs):

    urdf_path = LaunchConfiguration("urdf_file").perform(context)
    urdf_mapping_dict = LaunchConfiguration("urdf_mapping").perform(context)
    urdf_mapping_yaml = yaml.safe_load(urdf_mapping_dict)

    urdf_str = xacro.process_file(urdf_path, mappings=urdf_mapping_yaml).toprettyxml(indent="  ")

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
          {
            "robot_description": urdf_str,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
          }
        ],
    )

    nodes = [rsp]

    return nodes

#######################################
def generate_launch_description():

    dragoman_fsw_sim_dir = get_package_share_directory("dragoman_fsw_sim")
    rviz_config = os.path.join(dragoman_fsw_sim_dir, 'rviz/mujoco_test_robot.rviz')

    robot_dir = get_package_share_directory("mujoco_ros2_simulation")
    urdf_file = os.path.join(robot_dir, 'test_resources/test_robot.urdf')


    launch_args = [
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("rviz", default_value="True"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_config),
        DeclareLaunchArgument(name="urdf_file", default_value=urdf_file),
        DeclareLaunchArgument(name="urdf_mapping", default_value=""),
    ]

    controller_parameters = ParameterFile(
        PathJoinSubstitution([FindPackageShare("mujoco_ros2_simulation"), "config", "controllers.yaml"]),
    )

    nodes_eval = OpaqueFunction(function=evaluate_nodes)

    mujoco_control_node = Node(
        package="mujoco_ros2_simulation",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            controller_parameters,
        ],
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
        ],
        output="both",
    )

    spawn_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_position_controller",
        arguments=[
            "position_controller",
        ],
        output="both",
    )

    # Rviz
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', LaunchConfiguration("rviz_config")],
             parameters=[
              {"use_sim_time": LaunchConfiguration("use_sim_time")},
             ],
             condition=IfCondition(LaunchConfiguration("rviz")),
             output="screen",
    )
    return LaunchDescription(
        launch_args +
        [nodes_eval,
         mujoco_control_node,
         spawn_joint_state_broadcaster,
         spawn_position_controller,
         rviz
        ]
    )
