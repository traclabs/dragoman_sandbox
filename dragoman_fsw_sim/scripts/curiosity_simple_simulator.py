#!/usr/bin/env python3

import binascii
import socket
import traceback
from threading import Thread
from time import sleep

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from construct import Int16ub
from dragoman_fsw_sim.curiosity_construct_definitions import TM_PACKET_STRUCT, COMMAND_STRUCTS
from dragoman_fsw_sim.ccsds_header_definitions import CCSDSHeader
from dragoman_fsw_sim.srdf_parser import parse_srdf_group_states
from dragoman_fsw_sim.clr_trajectory_router import send_trajectory

CMD_CANNED_POSE = 0
CMD_ARM_JOINT_GOAL = 1
CMD_MAST_JOINT_GOAL = 2

NUM_JOINTS = 24

def send_tm(simulator):
    """Send telemetry packets at configured rate"""
    tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    simulator.tm_counter = 1
    tm_count = 0

    while True:
        js = simulator.js

        if js is None:
            continue

        if len(js) == NUM_JOINTS:
            packet_length = TM_PACKET_STRUCT.sizeof() - CCSDSHeader.sizeof() - 1

            tm_packet = simulator.tm_packet_struct.build(
                {
                    "header": {
                        "version": 0,
                        "type": 0,  # 0 = Telemetry
                        "secondary_header_flag": 0,
                        "apid": 110,  # APID for telemetry
                        "sequence_flags": 3,  # 3 = Unsegmented
                        "sequence_count": tm_count,
                        "packet_length": packet_length,
                    },
                    "joint_state": list(js),
                }
            )
            tm_socket.sendto(tm_packet, (simulator.TM_SEND_ADDRESS, simulator.TM_SEND_PORT))
            tm_count += 1
            simulator.tm_counter += 1

        sleep(1 / simulator.rate)


def receive_tc(simulator):
    """Receive and process telecommand packets"""
    tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tc_socket.bind((simulator.TC_RECEIVE_ADDRESS, simulator.TC_RECEIVE_PORT))

    while True:
        data, _ = tc_socket.recvfrom(4096)
        parse_tc_data(data, simulator)

        simulator.last_tc = data
        simulator.tc_counter += 1


def parse_tc_data(data, simulator):
    logger = simulator.get_logger()

    try:
        command_id = Int16ub.parse(data[6:8])

        command_struct = COMMAND_STRUCTS.get(command_id)
        if command_struct is None:
            logger.error(f"Unknown command_id: {command_id}")
            return

        parsed_packet = command_struct.parse(data)

        if command_id == CMD_CANNED_POSE:
            parse_canned_pose(parsed_packet, logger, simulator)
        elif command_id == CMD_ARM_JOINT_GOAL:
            parse_arm_joint_state_goal(parsed_packet, logger, simulator.arm_pub)
        elif command_id == CMD_MAST_JOINT_GOAL:
            parse_mast_joint_state_goal(parsed_packet, logger, simulator.mast_pub)
    except Exception as e:
        logger.error(f"Error parsing command packet: {e}")
        logger.error(traceback.format_exc())


def parse_canned_pose(parsed_packet, logger, simulator):
    group_name = parsed_packet.group_name
    group_state = parsed_packet.group_state

    logger.info(f"* Canned pose command: group='{group_name}', state='{group_state}'")

    pose_key = (group_name, group_state)
    if pose_key not in simulator.canned_poses:
        logger.error(f"Unknown canned pose: {pose_key}")
        logger.error(f"Available poses: {list(simulator.canned_poses.keys())}")
        return

    pose_config = simulator.canned_poses[pose_key]

    if group_name == "arm":
        send_trajectory(pose_config, simulator.arm_pub, logger)
    elif group_name == "mast":
        send_trajectory(pose_config, simulator.mast_pub, logger)
    else:
        logger.error(f"Unknown group '{group_name}'")


def parse_arm_joint_state_goal(parsed_packet, logger, pub):
    js_goal = list(parsed_packet.arm_joint_values)
    logger.info(f"* Arm Joint goal: {[f'{v:.3f}' for v in js_goal]}")

    traj = JointTrajectory()
    traj.joint_names = [
        "arm_01_joint",
        "arm_02_joint",
        "arm_03_joint",
        "arm_04_joint",
        "arm_tools_joint",
    ]

    point = JointTrajectoryPoint()
    point.positions = js_goal
    point.time_from_start = Duration(sec=4)

    traj.points.append(point)
    pub.publish(traj)


def parse_mast_joint_state_goal(parsed_packet, logger, pub):
    js_goal = list(parsed_packet.mast_joint_values)
    logger.info(f"* Mast Joint goal: {[f'{v:.3f}' for v in js_goal]}")

    traj = JointTrajectory()
    traj.joint_names = [
        "mast_p_joint",
        "mast_02_joint",
        "mast_cameras_joint",
    ]

    point = JointTrajectoryPoint()
    point.positions = js_goal
    point.time_from_start = Duration(sec=4)

    traj.points.append(point)
    pub.publish(traj)


class Simulator(Node):

    def __init__(self):
        super().__init__("curiosity_simulator")

        self.tm_counter = 0
        self.tc_counter = 0
        self.last_tc = None
        self.js = None

        self.timer = self.create_timer(5, self.timer_cb)

        self.declare_parameter("tm_host", rclpy.Parameter.Type.STRING)
        self.declare_parameter("tm_port", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("rate", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("tc_host", rclpy.Parameter.Type.STRING)
        self.declare_parameter("tc_port", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("robot_description_semantic", rclpy.Parameter.Type.STRING)

        self.TM_SEND_ADDRESS = self.get_parameter("tm_host").value
        self.TM_SEND_PORT = self.get_parameter("tm_port").value
        self.rate = self.get_parameter("rate").value
        self.TC_RECEIVE_ADDRESS = self.get_parameter("tc_host").value
        self.TC_RECEIVE_PORT = self.get_parameter("tc_port").value

        srdf_content = self.get_parameter("robot_description_semantic").value

        self.canned_poses = parse_srdf_group_states(srdf_content)

        self.get_logger().info(f"Loaded {len(self.canned_poses)} canned poses from SRDF:")
        for (group, state), config in self.canned_poses.items():
            self.get_logger().info(
                f"  - {group}/{state}: {len(config['joints'])} joints"
            )

        self.tm_packet_struct = TM_PACKET_STRUCT

        self.get_logger().info(
            "Using hard-coded command structures: {}".format(list(COMMAND_STRUCTS.keys()))
        )

        self.js_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)

        self.arm_pub = self.create_publisher(
            JointTrajectory, "/arm_joint_trajectory_controller/joint_trajectory", 10
        )

        self.mast_pub = self.create_publisher(
            JointTrajectory, "/mast_joint_trajectory_controller/joint_trajectory", 10
        )

    def start(self):
        tm_thread = Thread(target=send_tm, args=(self,), daemon=True)
        tm_thread.start()

        tc_thread = Thread(target=receive_tc, args=(self,), daemon=True)
        tc_thread.start()

        self.get_logger().info(f"* Using playback rate of {self.rate} Hz")
        self.get_logger().info(f"TM host={self.TM_SEND_ADDRESS}, TM port={self.TM_SEND_PORT}")
        self.get_logger().info(f"TC host={self.TC_RECEIVE_ADDRESS}, TC port={self.TC_RECEIVE_PORT}")

    def print_status(self):
        cmdhex = binascii.hexlify(self.last_tc).decode("ascii") if self.last_tc else None
        return f"Sent: {self.tm_counter} packets. Received: {self.tc_counter} commands. Last command: {cmdhex}"

    def timer_cb(self):
        status = self.print_status()
        self.get_logger().info(status)

    def js_cb(self, msg):
        self.js = msg.position


if __name__ == "__main__":
    rclpy.init(args=None)
    simulator = Simulator()
    simulator.start()
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()
