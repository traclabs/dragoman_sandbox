#!/usr/bin/env python3

import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from chonkur_deploy.launch_helpers import (
    include_launch_file,
    parameter_file,
    spawn_controller,
)

##############################################
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

#####################################
def generate_launch_description():

    dragoman_fsw_sim_dir = get_package_share_directory("dragoman_fsw_sim")
    rviz_config = os.path.join(dragoman_fsw_sim_dir, 'rviz/mujoco_test_clr.rviz')

    robot_dir = get_package_share_directory("mujoco_ros2_control")
    urdf_file = os.path.join(dragoman_fsw_sim_dir, 'urdf/test_clr_xacro.urdf')


    launch_args = [
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("rviz", default_value="True"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_config),
        DeclareLaunchArgument(name="urdf_file", default_value=urdf_file),
        DeclareLaunchArgument(name="urdf_mapping", default_value=""),
        DeclareLaunchArgument(name="tf_prefix", default_value="")
    ]

    tf_prefix = LaunchConfiguration("tf_prefix")

    # Controller parameter
    controller_parameters = ParameterFile(
        PathJoinSubstitution([FindPackageShare("mujoco_ros2_control"), "config", "controllers.yaml"]),
    )

    # Robot publisher
    nodes_eval = OpaqueFunction(function=evaluate_nodes)

    mujoco_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            #controller_parameters,
            parameter_file("clr_deploy", "controllers_common.yaml", True),
            parameter_file("chonkur_deploy", "ur10e_controllers.yaml", True),
            parameter_file("chonkur_deploy", "hande_controllers.yaml", True),
            parameter_file("vention_rail_deploy", "rail_controllers.yaml", True),
            parameter_file("ewellix_liftkit_deploy", "liftkit_controllers.yaml", True),
        ],
        remappings=[("~/robot_description", "/robot_description")],
    )

    # CLR specific joint_state_broadcaster
    spawn_joint_state_broadcaster = spawn_controller("joint_state_broadcaster")
    spawn_rail_position_trajectory_controller = spawn_controller("rail_position_trajectory_controller")
    spawn_joint_trajectory_controller = spawn_controller("joint_trajectory_controller")
    spawn_lift_position_trajectory_controller = spawn_controller("lift_position_trajectory_controller")
    spawn_robotiq_gripper_hande_controller = spawn_controller("robotiq_gripper_hande_controller")

    # Rviz
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', LaunchConfiguration("rviz_config")],
             condition=IfCondition(LaunchConfiguration("rviz")),
             output="screen",
    )

    return LaunchDescription(
        launch_args +
        [nodes_eval,
         mujoco_control_node,
         spawn_joint_state_broadcaster,
         spawn_rail_position_trajectory_controller,
         spawn_lift_position_trajectory_controller,
         spawn_joint_trajectory_controller,
         spawn_robotiq_gripper_hande_controller,
         rviz
        ]
    )
