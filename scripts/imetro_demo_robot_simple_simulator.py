#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import binascii
import socket
from threading import Thread
from time import sleep

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from construct import Int16ub
from dragoman_sandbox.xtce_construct_generator import TM_PACKET_STRUCT, COMMAND_STRUCTS

# *************************
# Send telemetry
# *************************
def send_tm(simulator):
    tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    num_joints = 9
    simulator.tm_counter = 1
    tm_count = 0

    while True:
        js = simulator.js

        if js is None:
          continue

        if len(js) == num_joints:
          # Build telemetry packet using construct structures from XTCE
          tm_packet = simulator.tm_packet_struct.build({
              "header": {
                  "version": 0,
                  "type": 0,  # 0 = Telemetry
                  "secondary_header_flag": 0,
                  "apid": 100,  # APID for telemetry
                  "sequence_flags": 3,  # 3 = Unsegmented
                  "sequence_count": tm_count,
                  "packet_length": num_joints * 4 - 1  # 9 floats * 4 bytes - 1
              },
              "joint_state": list(js)
          })

          tm_socket.sendto(tm_packet, (simulator.TM_SEND_ADDRESS, simulator.TM_SEND_PORT))
          tm_count += 1
          simulator.tm_counter += 1

        sleep(1 / simulator.rate)

# *************************
# Receive command
# *************************
def receive_tc(simulator):
    tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tc_socket.bind((simulator.TC_RECEIVE_ADDRESS, simulator.TC_RECEIVE_PORT ))

    while True:
        data, _ = tc_socket.recvfrom(4096)
        parse_tc_data(data, simulator)

        simulator.last_tc = data
        simulator.tc_counter += 1


def parse_tc_data(data, simulator):
    """Parse telecommand data using XTCE-generated construct structures"""
    logger = simulator.get_logger()

    try:
        # Parse command_id (2 bytes after 6-byte CCSDS header)
        command_id = Int16ub.parse(data[6:8])

        # Get the appropriate command structure and parse
        command_struct = COMMAND_STRUCTS.get(command_id)
        if command_struct is None:
            logger.error(f"Unknown command_id: {command_id}")
            return

        # Parse the full packet
        parsed_packet = command_struct.parse(data)

        # Route to appropriate handler
        if command_id == 0:
            parse_canned_pose(parsed_packet, logger)
        elif command_id == 1:
            parse_arm_joint_state_goal(parsed_packet, logger, simulator.arm_pub)
        elif command_id == 2:
            parse_rail_joint_state_goal(parsed_packet, logger, simulator.rail_pub)
        elif command_id == 3:
            parse_lift_joint_state_goal(parsed_packet, logger, simulator.lift_pub)
    except Exception as e:
        logger.error(f"Error parsing command packet: {e}")
        import traceback
        logger.error(traceback.format_exc())


def parse_canned_pose(parsed_packet, logger):
    """Handle canned pose command (not yet implemented)"""
    logger.info("Canned pose command not implemented yet!")


def parse_arm_joint_state_goal(parsed_packet, logger, pub):
    """Parse and execute arm joint state goal command"""
    # Extract arm joint values from parsed packet (field name from XTCE)
    js_goal = list(parsed_packet.arm_joint_values)
    logger.info(f"* Arm Joint goal: {[f'{v:.3f}' for v in js_goal]}")

    # Send arm command
    traj = JointTrajectory()
    traj.joint_names = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    ]

    point = JointTrajectoryPoint()
    point.positions = js_goal
    point.time_from_start = Duration(sec=4)

    traj.points.append(point)
    pub.publish(traj)


def parse_rail_joint_state_goal(parsed_packet, logger, pub):
    """Parse and execute rail joint state goal command"""
    # Extract rail joint value from parsed packet (field name from XTCE)
    js_goal = parsed_packet.rail_joint_value
    logger.info(f"* Rail Joint goal: {js_goal:.3f}")

    # Send rail command
    traj = JointTrajectory()
    traj.joint_names = ["vention_rail_base_to_carriage"]

    point = JointTrajectoryPoint()
    point.positions = [js_goal]
    point.time_from_start = Duration(sec=4)

    traj.points.append(point)
    pub.publish(traj)


def parse_lift_joint_state_goal(parsed_packet, logger, pub):
    """Parse and execute lift joint state goal command"""
    # Extract lift joint value from parsed packet (field name from XTCE)
    js_goal = parsed_packet.lift_joint_value
    logger.info(f"* Lift Joint goal: {js_goal:.3f}")

    # Send lift command
    traj = JointTrajectory()
    traj.joint_names = ["ewellix_lift_lower_to_higher"]

    point = JointTrajectoryPoint()
    point.positions = [js_goal]
    point.time_from_start = Duration(sec=4)

    traj.points.append(point)
    pub.publish(traj)


class Simulator(Node):
    """ROS2 node that simulates robot telemetry and command handling using XTCE definitions"""

    def __init__(self):
        super().__init__('imetro_simulator')

        # Counters and state
        self.tm_counter = 0
        self.tc_counter = 0
        self.last_tc = None
        self.js = None

        # Status timer
        self.timer = self.create_timer(5, self.timer_cb)

        # Declare parameters
        self.declare_parameter("tm_host", rclpy.Parameter.Type.STRING)
        self.declare_parameter("tm_port", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("rate", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("tc_host", rclpy.Parameter.Type.STRING)
        self.declare_parameter("tc_port", rclpy.Parameter.Type.INTEGER)

        # Get parameter values
        self.TM_SEND_ADDRESS = self.get_parameter("tm_host").value
        self.TM_SEND_PORT = self.get_parameter("tm_port").value
        self.rate = self.get_parameter("rate").value
        self.TC_RECEIVE_ADDRESS = self.get_parameter("tc_host").value
        self.TC_RECEIVE_PORT = self.get_parameter("tc_port").value

        # Use hard-coded construct structures from xtce_construct_generator module
        self.tm_packet_struct = TM_PACKET_STRUCT

        # Log available command structures
        self.get_logger().info("Using hard-coded command structures: {}".format(
            list(COMMAND_STRUCTS.keys())))

        # Subscribe to /joint_states
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.js_cb, 10)

        # Send motion commands
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        self.lift_pub = self.create_publisher(
            JointTrajectory, "/lift_position_trajectory_controller/joint_trajectory", 10
        )

        self.rail_pub = self.create_publisher(
            JointTrajectory, "/rail_position_trajectory_controller/joint_trajectory", 10
        )

    def start(self):
        """Start telemetry and telecommand threads"""
        tm_thread = Thread(target=send_tm, args=(self,), daemon=True)
        tm_thread.start()

        tc_thread = Thread(target=receive_tc, args=(self,), daemon=True)
        tc_thread.start()

        self.get_logger().info(f'* Using playback rate of {self.rate} Hz')
        self.get_logger().info(f'TM host={self.TM_SEND_ADDRESS}, TM port={self.TM_SEND_PORT}')
        self.get_logger().info(f'TC host={self.TC_RECEIVE_ADDRESS}, TC port={self.TC_RECEIVE_PORT}')

    def print_status(self):
        """Generate status string for logging"""
        cmdhex = binascii.hexlify(self.last_tc).decode('ascii') if self.last_tc else None
        return f'Sent: {self.tm_counter} packets. Received: {self.tc_counter} commands. Last command: {cmdhex}'

    def timer_cb(self):
        """Periodic status logging callback"""
        status = self.print_status()
        self.get_logger().info(status)

    def js_cb(self, msg):
        """Joint state callback - stores current joint positions"""
        self.js = msg.position


if __name__ == '__main__':
    rclpy.init(args=None)
    simulator = Simulator()
    simulator.start()
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()

