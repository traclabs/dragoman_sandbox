from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("cfs_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1")
    ]

    # *****************************
    # Gateway body (static)
    # *****************************
    gateway_body_xacro = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gateway_description"), "robots", "gateway_body.urdf.xacro"]),
        ])
    gateway_robot_description = {"robot_description": gateway_body_xacro}

    gateway_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            gateway_robot_description,
            {'frame_prefix': ''}], # gateway/
        namespace="gateway_body"
    )

    # **************************
    #  Big Arm
    # **************************
    big_arm_xacro = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gateway_description"), "robots", "big_arm.urdf.xacro"]),
        ])
    big_arm_robot_description = {"robot_description": big_arm_xacro}

    big_arm_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            big_arm_robot_description,
            {'frame_prefix': ''}], # big_arm/
        namespace="big_arm"
    )
    big_arm_jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="big_arm",
        parameters=[
         {'source_list': ['joint_state_command'] }]
    )

    # *************************
    #  Little Arm
    # **************************
    little_arm_xacro = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gateway_description"), "robots", "little_arm.urdf.xacro"]),
        ])
    little_arm_robot_description = {"robot_description": little_arm_xacro}

    little_arm_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            little_arm_robot_description,
            {'frame_prefix': ''}], # little_arm/
        namespace="little_arm"
    )
    little_arm_jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="little_arm"
    )


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dragoman_fsw_sim"), "rviz", "gateway_big_arm_flight_demo.rviz"]
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
        executable="arm_comm_udp_node",
        name="arm_comm_udp_node",
        parameters=[
         {'cfs_port': 8080},
         {'robot_port': 8585},
         {'cfs_ip': LaunchConfiguration("cfs_ip")},
         {'robot_ip': LaunchConfiguration("robot_ip")},
         {"robot_description": big_arm_xacro},
        ],
        output="screen",
    )

    nodes_to_start = [
        gateway_rsp,
        big_arm_rsp,
        big_arm_jsp,
        little_arm_rsp,
        little_arm_jsp,
        rviz_node,
        robot_comm_node
    ]

    return LaunchDescription(launch_args + nodes_to_start)


