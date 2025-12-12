#!/usr/bin/env python3

import binascii
import socket
import traceback
from threading import Thread
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration

from construct import Int16ub
from dragoman_fsw_sim.gateway_xtce_construct_generator import TM_PACKET_STRUCT, TC_GOAL_POSE_STRUCT
from dragoman_fsw_sim.srdf_parser import parse_srdf_group_states

# Constant
NUM_JOINTS = 7

# *************************
# Send telemetry
# *************************
def send_tm(simulator):
    """Send telemetry packets at configured rate"""
    tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    simulator.tm_counter = 1
    tm_count = 0

    while True:

        # Build telemetry packet using construct structures from XTCE
        js = [0.14, 0.25, 0.37, 0.46, 0.52, 0.61, 0.76]
        tm_packet = simulator.tm_packet_struct.build(
            {
                "header": {
                    "version": 0,
                    "type": 0,  # 0 = Telemetry
                    "secondary_header_flag": 1,
                    "apid": 39, #0x0827 - 0x0800,  # APID for telemetry
                    "sequence_flags": 3,  # 3 = Unsegmented
                    "sequence_count": tm_count,
                    "packet_length": NUM_JOINTS * 4 - 1,  # 9 floats * 4 bytes - 1
                },
                "sec_header": {
                    "sec": 0,
                    "spare": 0,
                },
                "joint_state": js #list(js),
            }
        )
        simulator.get_logger().info(f"Sending telemetry to {simulator.TM_SEND_ADDRESS} port: {simulator.TM_SEND_PORT}")
        tm_socket.sendto(tm_packet, (simulator.TM_SEND_ADDRESS, simulator.TM_SEND_PORT))
        tm_count += 1
        simulator.tm_counter += 1

        sleep(1 / simulator.rate)


# *************************
# Receive command
# *************************
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
    """Parse telecommand data using XTCE-generated construct structures"""
    logger = simulator.get_logger()

    try:
        # Parse command_id (2 bytes after full CCSDS header)
        offset = 6 + 10 # 6 bytes for primary header + 10 bytes (6+4) for secondary header
#        command_id = Int16ub.parse(data[offset:offset+2])

        # Parse the full packet
        parsed_packet = TC_GOAL_POSE_STRUCT.parse(data)
        debug_mid_print(parsed_packet, logger)

        # Route to appropriate handler
        parse_ee_pose_goal(parsed_packet, logger)
    except Exception as e:
        logger.error(f"Error parsing command packet: {e}")
        logger.error(traceback.format_exc())

def debug_mid_print(parsed_packet, logger):
  
  # Verifying that mid and fcn_code are correctly being sent 
  version = parsed_packet.header.version 
  msg_type = parsed_packet.header.type
  sec_flag = parsed_packet.header.secondary_header_flag
  apid = parsed_packet.header.apid      
        
  mid = (version << 13) | (msg_type << 12) | (sec_flag << 11) | apid                
  fcn_code = parsed_packet.sec_header.fcn_code
        
  logger.info(f"MID of received command: {hex(mid)} and fcn code: {fcn_code}")


def parse_ee_pose_goal(parsed_packet, logger):
    """Parse and execute arm joint state goal command"""
    # Extract EE pose (field name from XTCE)
    ee_pos = list(parsed_packet.ee_pos)
    ee_rot = list(parsed_packet.ee_rot)
    logger.info(f"* EE Pos: {[f'{v:.3f}' for v in ee_pos]} and EE Rot: {[f'{v:.3f}' for v in ee_rot]}")


class Simulator(Node):
    """ROS2 node that simulates robot telemetry and command handling using XTCE definitions"""

    def __init__(self):
        super().__init__("gateway_simulator")

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
        #self.declare_parameter("robot_description_semantic", rclpy.Parameter.Type.STRING)

        # Get parameter values
        self.TM_SEND_ADDRESS = self.get_parameter("tm_host").value
        self.TM_SEND_PORT = self.get_parameter("tm_port").value
        self.rate = self.get_parameter("rate").value
        self.TC_RECEIVE_ADDRESS = self.get_parameter("tc_host").value
        self.TC_RECEIVE_PORT = self.get_parameter("tc_port").value

        # Parse SRDF and build canned poses dictionary
        #srdf_content = self.get_parameter("robot_description_semantic").value

        # Parse group states (canned poses)
        #self.canned_poses = parse_srdf_group_states(srdf_content)

        #self.get_logger().info(f"Loaded {len(self.canned_poses)} canned poses from SRDF:")
        #for (group, state), config in self.canned_poses.items():
        #    self.get_logger().info(
        #        f"  - {group}/{state}: {len(config['joints'])} joints"
        #    )

        # Use hard-coded construct structures from gateway_xtce_construct_generator module
        self.tm_packet_struct = TM_PACKET_STRUCT

        # Subscribe to /joint_states
        #self.js_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)


    def start(self):
        """Start telemetry and telecommand threads"""
        tm_thread = Thread(target=send_tm, args=(self,), daemon=True)
        tm_thread.start()

        tc_thread = Thread(target=receive_tc, args=(self,), daemon=True)
        tc_thread.start()

        self.get_logger().info(f"* Using playback rate of {self.rate} Hz")
        self.get_logger().info(f"TM host={self.TM_SEND_ADDRESS}, TM port={self.TM_SEND_PORT}")
        self.get_logger().info(f"TC host={self.TC_RECEIVE_ADDRESS}, TC port={self.TC_RECEIVE_PORT}")

    def print_status(self):
        """Generate status string for logging"""
        cmdhex = binascii.hexlify(self.last_tc).decode("ascii") if self.last_tc else None
        return f"Sent: {self.tm_counter} packets. Received: {self.tc_counter} commands. Last command: {cmdhex}"

    def timer_cb(self):
        """Periodic status logging callback"""
        status = self.print_status()
        self.get_logger().info(status)

    def js_cb(self, msg):
        """Joint state callback - stores current joint positions"""
        self.js = msg.position


if __name__ == "__main__":
    rclpy.init(args=None)
    simulator = Simulator()
    simulator.start()
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()
