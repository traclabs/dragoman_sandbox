import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(name="rviz", default_value="True"),
        DeclareLaunchArgument(name="robot_publisher", default_value="True"),
        DeclareLaunchArgument(name="joint_publisher", default_value="True"),                
    ]

    # Urdf
    robot_dir = get_package_share_directory("lunar_pole_exploration_rover_gazebo")
    urdf_string = xacro.process_file(
        os.path.join(robot_dir, "models/lunar_pole_exploration_rover/urdf/lunar_pole_exploration_rover.xacro")
    )
    robot_description = {"robot_description": urdf_string.toxml()}

    # Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration('robot_publisher'))
    )

    # Joint State publisher
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('joint_publisher'))
    )

    # Rviz
    dragoman_ground_dir = get_package_share_directory("dragoman_ground_example")
    rviz_config = os.path.join(dragoman_ground_dir, "rviz/lunar_exploration.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
        robot_description
        ]
        ,
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        launch_args +
        [rsp, jsp, rviz]
    )
