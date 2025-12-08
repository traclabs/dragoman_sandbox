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
import struct
import time
import random
import math
import argparse
from threading import Thread
from datetime import datetime


class AllTypesSimulator:
    """Simulator that generates and sends AllTypes telemetry packets"""

    def __init__(self, tm_host='127.0.0.1', tm_port=10015, rate=1):
        """
        Initialize the simulator

        Args:
            tm_host: Telemetry destination host
            tm_port: Telemetry destination port
            rate: Telemetry rate in Hz
        """
        self.tm_host = tm_host
        self.tm_port = tm_port
        self.rate = rate

        # Counters
        self.tm_counter = 0
        self.sequence_count = 0

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
        print(f"  Rate: {self.rate} Hz")

    def generate_mock_data(self):
        """
        Generate mock values for all telemetry parameters

        Returns:
            dict: Dictionary containing all parameter values
        """
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

    def build_ccsds_header(self, data_length):
        """
        Build CCSDS Space Packet Primary Header (6 bytes)

        Args:
            data_length: Length of packet data (excluding header)

        Returns:
            bytes: 6-byte CCSDS header
        """
        # Packet ID (2 bytes)
        # - version (3 bits): 0
        # - type (1 bit): 0 (telemetry)
        # - secondary_header_flag (1 bit): 0 (not present)
        # - apid (11 bits): 100
        packet_id = (0 << 13) | (0 << 12) | (0 << 11) | 100

        # Packet Sequence Control (2 bytes)
        # - sequence_flags (2 bits): 3 (unsegmented)
        # - sequence_count (14 bits): incrementing counter
        packet_sequence = (3 << 14) | (self.sequence_count & 0x3FFF)

        # Packet Length (2 bytes)
        # Length of packet data - 1 (as per CCSDS standard)
        packet_length = data_length - 1

        # Pack as big-endian (network byte order)
        header = struct.pack('>HHH', packet_id, packet_sequence, packet_length)

        return header

    def build_telemetry_packet(self):
        """
        Build complete telemetry packet with CCSDS header and all parameters

        Returns:
            bytes: Complete telemetry packet
        """
        # Generate mock data
        data = self.generate_mock_data()

        # Build packet data (all parameters in order)
        packet_data = b''

        # 1. T_IntegerSigned (32-bit signed, big-endian)
        packet_data += struct.pack('>i', data['integer_signed'])

        # 2. T_FloatRaw64 (64-bit float, big-endian)
        packet_data += struct.pack('>d', data['float_raw64'])

        # 3. T_EnumeratedAlarm (8-bit unsigned)
        packet_data += struct.pack('>B', data['enum_alarm'])

        # 4. T_StringUTF8 (null-terminated UTF-8 string)
        string_bytes = data['string_utf8'].encode('utf-8') + b'\x00'
        packet_data += string_bytes

        # 5. T_BooleanFlag (8-bit unsigned: 0 or 1)
        packet_data += struct.pack('>B', 1 if data['boolean_flag'] else 0)

        # 6. T_AbsoluteTime (64-bit unsigned, big-endian)
        packet_data += struct.pack('>Q', data['absolute_time'])

        # 7. T_RelativeTimeRaw (32-bit unsigned, big-endian)
        packet_data += struct.pack('>I', data['relative_time'])

        # 8. T_BinaryBlob (128 bits = 16 bytes)
        packet_data += data['binary_blob']

        # 9. T_IntegerArray (10 x 16-bit unsigned, big-endian)
        for value in data['integer_array']:
            packet_data += struct.pack('>H', value)

        # 10. T_StatusAggregate (struct)
        # - CurrentDraw (32-bit float)
        packet_data += struct.pack('>f', data['aggregate']['current_draw'])
        # - HeaterEnabled (8-bit boolean)
        packet_data += struct.pack('>B', 1 if data['aggregate']['heater_enabled'] else 0)
        # - RawStatusFlags (32 bits = 4 bytes)
        packet_data += data['aggregate']['raw_status_flags']

        # Build CCSDS header
        header = self.build_ccsds_header(len(packet_data))

        # Combine header and data
        packet = header + packet_data

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

    def print_status(self):
        """Print current simulator status"""
        elapsed = time.time() - self.start_time
        print(f"Packets sent: {self.tm_counter}, Elapsed: {elapsed:.1f}s, Rate: {self.rate} Hz")

    def start(self):
        """Start the simulator"""
        # Start telemetry thread
        tm_thread = Thread(target=self.send_telemetry, daemon=True)
        tm_thread.start()

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
        rate=args.rate
    )
    simulator.start()


if __name__ == '__main__':
    main()
