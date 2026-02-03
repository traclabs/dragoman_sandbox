from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dragoman_fsw_sim"), "rviz", "lunar_exploration_flight_demo.rviz"]
    )

    launch_args = [
        DeclareLaunchArgument("rviz", default_value="True"),
        DeclareLaunchArgument("cfs_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1")
    ]


    robot = GroupAction([
      IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("lunar_pole_exploration_rover_gazebo"), "launch", "lunar_pole_exploration_rover_gazebo.launch.py"]),
        launch_arguments={
          "rviz": "False",
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
    #robot_comm_node = Node(
    #    package="dragoman_fsw_sim",
    #    executable="arm_comm_udp_node",
    #    name="arm_comm_udp_node",
    #    parameters=[
    #     {'cfs_port': 8080},
    #     {'robot_port': 8585},
    #     {'cfs_ip': LaunchConfiguration("cfs_ip")},
    #     {'robot_ip': LaunchConfiguration("robot_ip")},
    #     {"robot_description": big_arm_xacro},
    #    ],
    #    output="screen",
    #)

    return LaunchDescription(
      launch_args + 
      [robot, odom_node, rviz_node]
    )


