from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    dragoman_fsw_sim_dir = get_package_share_directory("dragoman_fsw_sim")
    rviz_config_file = os.path.join(dragoman_fsw_sim_dir, "rviz", "lunar_exploration_flight_demo.rviz")

    # Lunar Terrain world
    #world_models_path = get_package_share_directory("lunar_terrain_gz_worlds")
    #world_terrain = os.path.join(world_models_path, "worlds/dem_moon.sdf")

    # Regular Lunar Pole Exploration rover world (flatter)
    # world_models_path = get_package_share_directory("lunar_pole_exploration_rover_gazebo")
    # world_terrain = os.path.join(world_models_path, 'worlds/lunar_pole.world')

    # Dragoman specific Lunar Pole Exploration rover world
    world_terrain = os.path.join(dragoman_fsw_sim_dir, 'worlds/lunar_gz.sdf')

    launch_args = [
        DeclareLaunchArgument("rviz", default_value="False"),
        DeclareLaunchArgument("cfs_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("world", default_value=world_terrain),
        # xyz: 0 0 0 for flatter world, 0.0, 0.0, 550.5 for Lunar Terrain world, 35.2, 338.64, -11.50 for Dragoman Lunar Terrain
        DeclareLaunchArgument("x", default_value="53.2"),
        DeclareLaunchArgument("y", default_value="338.64"),
        DeclareLaunchArgument("z", default_value="-11.50"), #550.5 0.0
        DeclareLaunchArgument("yaw", default_value="0.0")
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

    cmd_vel_node = Node(
        package="lunar_pole_exploration_rover_demo",
        executable="move_wheel",
        output='screen'
    )

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

    # ***********************************************
    # Navigation controller
    # ***********************************************
    navigation_controller_node = Node(
        package="dragoman_fsw_sim",
        executable="lunar_exploration_navigation_controller.py",
        name="lunar_navigation_controller",
        output="screen",
    )

    # ***********************************************
    # Move Gazebo camera to desired pose (can't be done in .world file)
    # https://robotics.stackexchange.com/questions/115454/how-to-set-a-default-camera-pose-in-gazebo-harmonic-ubuntu-24-04
    # ***********************************************
    move_camera_action = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/gui/move_to/pose',
                    '--reqtype', 'gz.msgs.GUICamera',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '2000',
                    '--req', 'pose: {position: {x: 79.034355163574219, y: 338.47543334960938, z: -2.2349140644073486}, orientation: {x: 0.25594308972358704, y: 0.085092991590499878, z: -0.9137614369392395, w: 0.30379649996757507}}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription(
      launch_args +
      [robot,
       cmd_vel_node,
       odom_node,
       robot_comm_node,
       navigation_controller_node,
       rviz_node,
       move_camera_action]
    )


