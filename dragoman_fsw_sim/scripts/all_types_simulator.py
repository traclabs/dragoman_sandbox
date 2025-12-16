#!/usr/bin/env python3

"""
AllTypes Telemetry Simulator

A simple simulator that generates mock telemetry data for all parameter types
defined in AllTypes.xtce and sends it to YAMCS via UDP.

This simulator demonstrates all major XTCE parameter types:
- Integer (signed/unsigned)
- Float (32-bit and 64-bit)
- Enumerated
- String (UTF-8)
- Boolean
- Absolute Time
- Relative Time
- Binary Blob
- Array
- Aggregate (struct)

No ROS dependencies - pure Python mocking.
"""

import socket
import time
import random
import math
import argparse
from threading import Thread

from dragoman_fsw_sim.all_types_construct_definitions import (
    TM_PACKET_STRUCT,
    TC_CONFIGURE_STRUCT
)
from dragoman_fsw_sim.ccsds_header_definitions import CCSDSHeader


class AllTypesSimulator:
    """Simulator that generates and sends AllTypes telemetry packets"""

    def __init__(self, tm_host='127.0.0.1', tm_port=10015, tc_host='127.0.0.1', tc_port=10028, rate=1):
        """
        Initialize the simulator

        Args:
            tm_host: Telemetry destination host
            tm_port: Telemetry destination port
            tc_host: Telecommand receive host
            tc_port: Telecommand receive port
            rate: Telemetry rate in Hz
        """
        self.tm_host = tm_host
        self.tm_port = tm_port
        self.tc_host = tc_host
        self.tc_port = tc_port
        self.rate = rate

        self.tm_counter = 0
        self.tc_counter = 0
        self.sequence_count = 0
        self.last_tc = None

        # Start time for relative time calculations
        self.start_time = time.time()

        # State for mock data generation
        self.enum_state = 0  # Cycles through 0, 1, 2
        self.boolean_state = False
        self.string_index = 0
        self.status_messages = [
            "NOMINAL",
            "CALIBRATING",
            "STANDBY",
            "ACTIVE",
            "IDLE"
        ]

        print(f"AllTypes Simulator initialized")
        print(f"  TM Host: {self.tm_host}")
        print(f"  TM Port: {self.tm_port}")
        print(f"  TC Host: {self.tc_host}")
        print(f"  TC Port: {self.tc_port}")
        print(f"  Rate: {self.rate} Hz")

    def generate_mock_data(self):
        """Generate mock values for all telemetry parameters"""
        elapsed = time.time() - self.start_time

        # T_IntegerSigned: Counter that increments (can go negative)
        integer_signed = int((self.tm_counter % 200) - 100)

        # T_FloatRaw64: Sine wave pattern
        float_raw64 = 50.0 + 25.0 * math.sin(elapsed * 0.5)

        # T_EnumeratedAlarm: Cycles through states every 10 packets
        if self.tm_counter % 10 == 0:
            self.enum_state = (self.enum_state + 1) % 3
        enum_alarm = self.enum_state

        # T_StringUTF8: Rotating status messages
        if self.tm_counter % 5 == 0:
            self.string_index = (self.string_index + 1) % len(self.status_messages)
        string_utf8 = self.status_messages[self.string_index]

        # T_BooleanFlag: Toggles every 3 packets
        if self.tm_counter % 3 == 0:
            self.boolean_state = not self.boolean_state
        boolean_flag = self.boolean_state

        # T_AbsoluteTime: Current UNIX timestamp in seconds
        absolute_time = int(time.time())

        # T_RelativeTimeRaw: Seconds since simulator start
        relative_time = int(elapsed)

        # T_BinaryBlob: 16 bytes of pseudo-random data (deterministic pattern)
        random.seed(self.tm_counter)
        binary_blob = bytes([random.randint(0, 255) for _ in range(16)])

        # T_IntegerArray: Array of 10 unsigned 16-bit integers
        integer_array = [(self.tm_counter + i * 100) % 65536 for i in range(10)]

        # T_StatusAggregate: Struct with CurrentDraw, HeaterEnabled, RawStatusFlags
        current_draw = 2.5 + 0.5 * math.sin(elapsed * 0.3)  # 32-bit float
        heater_enabled = (self.tm_counter % 20) < 10  # Boolean
        raw_status_flags = bytes([
            (self.tm_counter >> 24) & 0xFF,
            (self.tm_counter >> 16) & 0xFF,
            (self.tm_counter >> 8) & 0xFF,
            self.tm_counter & 0xFF
        ])

        return {
            'integer_signed': integer_signed,
            'float_raw64': float_raw64,
            'enum_alarm': enum_alarm,
            'string_utf8': string_utf8,
            'boolean_flag': boolean_flag,
            'absolute_time': absolute_time,
            'relative_time': relative_time,
            'binary_blob': binary_blob,
            'integer_array': integer_array,
            'aggregate': {
                'current_draw': current_draw,
                'heater_enabled': heater_enabled,
                'raw_status_flags': raw_status_flags
            }
        }

    def build_telemetry_packet(self):
        """Build complete telemetry packet with CCSDS header and all parameters"""
        data = self.generate_mock_data()

        packet_dict = {
            'header': {
                'version': 0,
                "type": 0,  # 0 = Telemetry
                'secondary_header_flag': False,
                'apid': 0,  # APID
                'sequence_flags': 3, # 3 = Unsegmented
                'sequence_count': self.sequence_count,
                'packet_length': 0  # Placeholder, will be calculated
            },
            **data
        }

        # Build the packet first to determine actual size
        packet = TM_PACKET_STRUCT.build(packet_dict)

        # Calculate actual packet_length field (packet length - CCSDS header size - 1)
        packet_dict['header']['packet_length'] = len(packet) - CCSDSHeader.sizeof() - 1

        # Rebuild with correct packet_length
        packet = TM_PACKET_STRUCT.build(packet_dict)

        return packet

    def send_telemetry(self):
        """
        Telemetry sending thread - generates and sends packets at configured rate
        """
        tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"\nStarting telemetry transmission at {self.rate} Hz...")
        print("Press Ctrl+C to stop\n")

        try:
            while True:
                # Build and send packet
                packet = self.build_telemetry_packet()
                tm_socket.sendto(packet, (self.tm_host, self.tm_port))

                # Update counters
                self.tm_counter += 1
                self.sequence_count = (self.sequence_count + 1) % 16384  # 14-bit counter

                # Sleep for rate control
                time.sleep(1.0 / self.rate)

        except KeyboardInterrupt:
            print("\nStopping telemetry transmission...")
        finally:
            tm_socket.close()

    def receive_telecommand(self):
        """
        Telecommand receiving thread - listens for and processes commands
        """
        tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tc_socket.bind((self.tc_host, self.tc_port))

        print(f"\nListening for telecommands on {self.tc_host}:{self.tc_port}...")

        while True:
            try:
                data, addr = tc_socket.recvfrom(4096)
                self.parse_telecommand(data)
                self.last_tc = data
                self.tc_counter += 1
            except Exception as e:
                print(f"Error receiving telecommand: {e}")

    def parse_telecommand(self, data):
        """
        Parse and process ConfigureAllTypes command
        """
        try:
            # Parse command using construct
            cmd = TC_CONFIGURE_STRUCT.parse(data)

            # Log received command
            print(f"\n{'='*60}")
            print(f"Received ConfigureAllTypes Command:")
            print(f"  C_ArgInt16: {cmd.arg_int16}")
            print(f"  C_ArgFloat64: {cmd.arg_float64:.6f}")
            print(f"  C_ArgStringUTF16: '{cmd.arg_string_utf16}'")
            print(f"  C_ArgBoolean: {cmd.arg_boolean}")
            print(f"  C_ArgArrayFloat3: [{cmd.arg_array_float3[0]:.3f}, {cmd.arg_array_float3[1]:.3f}, {cmd.arg_array_float3[2]:.3f}]")
            print(f"  C_ArgConfigStruct:")
            print(f"    ID: {cmd.arg_config_struct.id}")
            print(f"    Value: {cmd.arg_config_struct.value:.6f}")
            print(f"    ConfigData: {cmd.arg_config_struct.config_data.hex()}")
            print(f"{'='*60}\n")

        except Exception as e:
            print(f"Error parsing telecommand: {e}")
            import traceback
            traceback.print_exc()

    def print_status(self):
        """Print current simulator status"""
        elapsed = time.time() - self.start_time
        tc_hex = self.last_tc.hex() if self.last_tc else "None"
        print(f"TM sent: {self.tm_counter}, TC received: {self.tc_counter}, "
              f"Elapsed: {elapsed:.1f}s, Rate: {self.rate} Hz, Last TC: {tc_hex[:20]}...")

    def start(self):
        """Start the simulator"""
        # Start telemetry thread
        tm_thread = Thread(target=self.send_telemetry, daemon=True)
        tm_thread.start()

        # Start telecommand thread
        tc_thread = Thread(target=self.receive_telecommand, daemon=True)
        tc_thread.start()

        # Status reporting loop
        try:
            while True:
                time.sleep(5)
                self.print_status()
        except KeyboardInterrupt:
            print("\nSimulator stopped")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='AllTypes Telemetry Simulator - Sends mock telemetry to YAMCS'
    )
    parser.add_argument(
        '--tm-host',
        type=str,
        default='127.0.0.1',
        help='Telemetry destination host (default: 127.0.0.1)'
    )
    parser.add_argument(
        '--tm-port',
        type=int,
        default=10015,
        help='Telemetry destination port (default: 10015)'
    )
    parser.add_argument(
        '--tc-host',
        type=str,
        default='127.0.0.1',
        help='Telecommand receive host (default: 127.0.0.1)'
    )
    parser.add_argument(
        '--tc-port',
        type=int,
        default=10028,
        help='Telecommand receive port (default: 10028)'
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=1.0,
        help='Telemetry rate in Hz (default: 1.0)'
    )

    args = parser.parse_args()

    # Create and start simulator
    simulator = AllTypesSimulator(
        tm_host=args.tm_host,
        tm_port=args.tm_port,
        tc_host=args.tc_host,
        tc_port=args.tc_port,
        rate=args.rate
    )
    simulator.start()


if __name__ == '__main__':
    main()
