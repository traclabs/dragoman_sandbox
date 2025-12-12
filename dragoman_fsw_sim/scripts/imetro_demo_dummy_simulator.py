#!/usr/bin/env python3

import binascii
import socket
import argparse

from threading import Thread
from time import sleep
from construct import Int16ub

from dragoman_fsw_sim.imetro_construct_definitions import TM_PACKET_STRUCT, COMMAND_STRUCTS
from dragoman_fsw_sim.ccsds_header_definitions import CCSDSHeader

def send_tm(simulator):
    tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    simulator.tm_counter = 1
    tm_count = 0
    js_vals = [0.25, 0.36, 0.49, 0.64, 0.81, 1.21, 2.35, 0.22, 0.87]

    while True:
        jsi_vals = [x + 0.001 * float(simulator.tm_counter) for x in js_vals]

        packet = TM_PACKET_STRUCT.build({
            'header': {
                'version': 0,
                'type': 0,  # 0 = Telemetry
                'secondary_header_flag': False,
                'apid': 100,  # APID for IMetro TM
                'sequence_flags': 3,  # 3 = Unsegmented
                'sequence_count': tm_count,
                'packet_length': TM_PACKET_STRUCT.sizeof() - CCSDSHeader.sizeof() - 1
            },
            'joint_state': jsi_vals
        })

        tm_socket.sendto(packet, (simulator.TM_SEND_ADDRESS, simulator.TM_SEND_PORT))
        tm_count = (tm_count + 1) % 16384
        simulator.tm_counter += 1

        sleep(1 / simulator.rate)

def receive_tc(simulator):
    tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tc_socket.bind((simulator.TC_RECEIVE_ADDRESS, simulator.TC_RECEIVE_PORT))

    while True:
        data, _ = tc_socket.recvfrom(4096)
        parse_tc_data(data)

        simulator.last_tc = data
        simulator.tc_counter += 1


def parse_tc_data(data):
    try:
        command_id = Int16ub.parse(data[6:8])

        packet_length = int.from_bytes(data[4:6], 'big')

        print(f"Received command of length: {len(data)}, packet_length: {packet_length} with command_id: {command_id}")

        command_struct = COMMAND_STRUCTS.get(command_id)
        if command_struct is None:
            print(f"Unknown command_id: {command_id}")
            return

        parsed_packet = command_struct.parse(data)

        if command_id == 0:
            parse_canned_pose(parsed_packet)
        elif command_id == 1:
            parse_arm_joint_state_goal(parsed_packet)
        elif command_id == 2:
            parse_rail_joint_state_goal(parsed_packet)
        elif command_id == 3:
            parse_lift_joint_state_goal(parsed_packet)
    except Exception as e:
        print(f"Error parsing command packet: {e}")


def parse_canned_pose(parsed_packet):
    group_name = parsed_packet.group_name
    group_state = parsed_packet.group_state
    print(f"* Canned pose command: group='{group_name}', state='{group_state}'")
    print("Not fully implemented yet!")


def parse_arm_joint_state_goal(parsed_packet):
    js_goal = list(parsed_packet.arm_joint_values)
    js_goal_print = [f"{item:.3f}" for item in js_goal]
    print(f"* Arm Joint goal: {js_goal_print}")


def parse_rail_joint_state_goal(parsed_packet):
    js_goal = parsed_packet.rail_joint_value
    print(f"* Rail Joint goal: {js_goal:.3f}")


def parse_lift_joint_state_goal(parsed_packet):
    js_goal = parsed_packet.lift_joint_value
    print(f"* Lift Joint goal: {js_goal:.3f}")


class Simulator():

    def __init__(self, tm_host, tm_port, tc_host, tc_port, rate):

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

    def start(self):
        self.tm_thread = Thread(target=send_tm, args=(self,))
        self.tm_thread.daemon = True
        self.tm_thread.start()
        self.tc_thread = Thread(target=receive_tc, args=(self,))
        self.tc_thread.daemon = True
        self.tc_thread.start()

        print(f'* Using playback rate of {self.rate} Hz')
        print(f'TM host= {self.TM_SEND_ADDRESS}, TM port= {self.TM_SEND_PORT}')
        print(f'TC host= {self.TC_RECEIVE_ADDRESS}, TC port= {self.TC_RECEIVE_PORT}')

    def print_status(self):
        cmdhex = None
        if self.last_tc:
            cmdhex = binascii.hexlify(self.last_tc).decode('ascii')
        return f'Sent: {self.tm_counter} packets. Received: {self.tc_counter} commands. Last command: {cmdhex}'

    def timer_cb(self):
        prev_status = None
        status = self.print_status()
        if status != prev_status:
            print(status)
            prev_status = status


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description='IMetro Dummy Simulator - Sends mock telemetry to YAMCS'
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

    quickstart_sim = Simulator(args.tm_host, args.tm_port, args.tc_host, args.tc_port, args.rate)
    quickstart_sim.start()

    # Status reporting loop
    try:
        while True:
            sleep(5)
            quickstart_sim.timer_cb()
    except KeyboardInterrupt:
        print("\nSimulator stopped")
