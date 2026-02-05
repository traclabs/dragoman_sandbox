from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    dragoman_fsw_sim_dir = get_package_share_directory("dragoman_fsw_sim")
    rviz_config_file = os.path.join(dragoman_fsw_sim_dir, "rviz", "lunar_exploration_flight_demo.rviz")

    world_models_path = get_package_share_directory("lunar_terrain_gz_worlds") # 'lunar_pole_exploration_rover_gazebo'
    world_terrain = os.path.join(world_models_path, "worlds/dem_moon.sdf") # 'worlds/lunar_pole.world'

    launch_args = [
        DeclareLaunchArgument("rviz", default_value="True"),
        DeclareLaunchArgument("cfs_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("world", default_value=world_terrain),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="550.5"),
        DeclareLaunchArgument("yaw", default_value="3.1416")
    ]

    robot = GroupAction([
      IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("lunar_pole_exploration_rover_gazebo"), "launch", "lunar_pole_exploration_rover_gazebo.launch.py"]),
        launch_arguments={
          "rviz": "False",
          "x": LaunchConfiguration("x"),
          "y": LaunchConfiguration("y"),
          "z": LaunchConfiguration("z"),
          "yaw": LaunchConfiguration("yaw"),
          "world": LaunchConfiguration("world")
        }.items(),
      )
    ])

    odom_node = Node(
        package="lunar_pole_exploration_rover_demo",
        executable="odom_tf_publisher",
        output='screen'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_flight",
        output="screen",
        arguments=["-d", rviz_config_file, "-t", "RViz (Flight Software Simulation)"],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    # ***********************************************
    # Arm communication with cFS and robot control
    # ***********************************************
    robot_comm_node = Node(
        package="dragoman_fsw_sim",
        executable="lunar_exploration_comm_udp_node",
        name="lunar_exploration_comm_udp_node",
        parameters=[
         {'cfs_port': 8080},
         {'robot_port': 8585},
         {'cfs_ip': LaunchConfiguration("cfs_ip")},
         {'robot_ip': LaunchConfiguration("robot_ip")},
    #     {"robot_description": big_arm_xacro},
        ],
        output="screen",
    )

    return LaunchDescription(
      launch_args + 
      [robot, odom_node, robot_comm_node, rviz_node]
    )


