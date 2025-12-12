#!/usr/bin/env python3

import binascii
import socket
import traceback
import argparse
from threading import Thread
from time import sleep

from construct import Int16ub
from dragoman_fsw_sim.gateway_construct_definitions import TM_PACKET_STRUCT, TC_GOAL_POSE_STRUCT

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
                    "secondary_header_flag": True,
                    "apid": 39, #0x0827 - 0x0800,  # APID for telemetry
                    "sequence_flags": 3,  # 3 = Unsegmented
                    "sequence_count": tm_count,
                    "packet_length": NUM_JOINTS * 4 - 1,  # 9 floats * 4 bytes - 1
                },
                "sec_header": {
                    "sec": 0,
                    "spare": 0,
                },
                "joint_state": js
            }
        )
        print(f"Sending telemetry to {simulator.TM_SEND_ADDRESS} port: {simulator.TM_SEND_PORT}")
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
        parse_tc_data(data)

        simulator.last_tc = data
        simulator.tc_counter += 1


def parse_tc_data(data):
    """Parse telecommand data using XTCE-generated construct structures"""
    try:
        # Parse the full packet
        parsed_packet = TC_GOAL_POSE_STRUCT.parse(data)
        debug_mid_print(parsed_packet)

        # Route to appropriate handler
        parse_ee_pose_goal(parsed_packet)
    except Exception as e:
        print(f"Error parsing command packet: {e}")
        print(traceback.format_exc())


def debug_mid_print(parsed_packet):
    # Verifying that mid and fcn_code are correctly being sent
    version = parsed_packet.header.version
    msg_type = parsed_packet.header.type
    sec_flag = parsed_packet.header.secondary_header_flag
    apid = parsed_packet.header.apid

    mid = (version << 13) | (msg_type << 12) | (sec_flag << 11) | apid
    fcn_code = parsed_packet.sec_header.fcn_code

    print(f"MID of received command: {hex(mid)} and fcn code: {fcn_code}")


def parse_ee_pose_goal(parsed_packet):
    """Parse and execute arm joint state goal command"""
    # Extract EE pose (field name from XTCE)
    ee_pos = list(parsed_packet.ee_pos)
    ee_rot = list(parsed_packet.ee_rot)
    print(f"* EE Pos: {[f'{v:.3f}' for v in ee_pos]} and EE Rot: {[f'{v:.3f}' for v in ee_rot]}")


class Simulator():
    """Simulator that sends robot telemetry and handles commands using XTCE definitions"""

    def __init__(self, tm_host, tm_port, tc_host, tc_port, rate):
        # Counters and state
        self.tm_counter = 0
        self.tc_counter = 0
        self.tm_thread = None
        self.tc_thread = None
        self.last_tc = None
        self.prev_status = None

        self.TM_SEND_ADDRESS = tm_host
        self.TM_SEND_PORT = tm_port
        self.rate = rate

        self.TC_RECEIVE_ADDRESS = tc_host
        self.TC_RECEIVE_PORT = tc_port

        # Use hard-coded construct structures from gateway_construct_definitions module
        self.tm_packet_struct = TM_PACKET_STRUCT

    def start(self):
        """Start telemetry and telecommand threads"""
        self.tm_thread = Thread(target=send_tm, args=(self,))
        self.tm_thread.daemon = True
        self.tm_thread.start()

        self.tc_thread = Thread(target=receive_tc, args=(self,))
        self.tc_thread.daemon = True
        self.tc_thread.start()

        print(f"* Using playback rate of {self.rate} Hz")
        print(f"TM host={self.TM_SEND_ADDRESS}, TM port={self.TM_SEND_PORT}")
        print(f"TC host={self.TC_RECEIVE_ADDRESS}, TC port={self.TC_RECEIVE_PORT}")

    def print_status(self):
        """Generate status string for logging"""
        cmdhex = None
        if self.last_tc:
            cmdhex = binascii.hexlify(self.last_tc).decode("ascii")
        return f"Sent: {self.tm_counter} packets. Received: {self.tc_counter} commands. Last command: {cmdhex}"

    def timer_cb(self):
        """Periodic status logging callback"""
        prev_status = None
        status = self.print_status()
        if status != prev_status:
            print(status)
            prev_status = status


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Gateway Dummy Simulator - Sends mock telemetry to YAMCS'
    )
    parser.add_argument("--tm-host", type=str, default='127.0.0.1',
                        help='Telemetry destination host (default: 127.0.0.1)')
    parser.add_argument("--tm-port", type=int, default=10015,
                        help='Telemetry destination port (default: 10015)')
    parser.add_argument("--tc-host", type=str, default='127.0.0.1',
                        help='Telecommand receive host (default: 127.0.0.1)')
    parser.add_argument("--tc-port", type=int, default=10025,
                        help='Telecommand receive port (default: 10025)')
    parser.add_argument("--rate", type=int, default=1,
                        help='Telemetry rate in Hz (default: 1)')

    args = parser.parse_args()

    gateway_sim = Simulator(args.tm_host, args.tm_port, args.tc_host, args.tc_port, args.rate)
    gateway_sim.start()

    # Status reporting loop
    try:
        while True:
            sleep(5)
            gateway_sim.timer_cb()
    except KeyboardInterrupt:
        print("\nSimulator stopped")
