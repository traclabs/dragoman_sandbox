#!/usr/bin/env python3

"""
Lunar Exploration DSN Telemetry Simulator

A simple simulator that generates mock telemetry data from the Deep Space Network (DSN)
and sends it to YAMCS via UDP.

This simulator demonstrates DSN parameters:
- DSN Location (Canberra, Madrid, Goldstone)
- Uplink Power (dBW) - High power transmission from ground station
- Downlink Signal (dBm) - Very weak signal from space
- Downlink SNR (dB) - Signal quality score

No ROS dependencies - pure Python mocking.
"""

import socket
import time
import random
import math
import argparse
from threading import Thread

from construct import (
    Struct, Int8ub, Float32l
)
from dragoman_fsw_sim.ccsds_header_definitions import CCSDSHeader

DSN_TM_PACKET_STRUCT = Struct(
    "header" / CCSDSHeader,
    "dsn_location" / Int8ub,           # 0=Canberra, 1=Madrid, 2=Goldstone
    "uplink_power" / Float32l,         # dBW (decibels relative to 1 Watt)
    "downlink_signal" / Float32l,      # dBm (very weak signal from space)
    "downlink_snr" / Float32l          # dB (signal-to-noise ratio)
)

class DSNSimulator:
    """Simulator that generates and sends DSN telemetry packets"""

    def __init__(self, tm_host='127.0.0.1', tm_port=2238, rate=1):
        """
        Initialize the DSN simulator

        Args:
            tm_host: Telemetry destination host
            tm_port: Telemetry destination port
            rate: Telemetry rate in Hz
        """
        self.tm_host = tm_host
        self.tm_port = tm_port
        self.rate = rate

        self.tm_counter = 0
        self.sequence_count = 0

        # Start time for simulation
        self.start_time = time.time()

        # State for mock data generation
        self.current_dsn_location = 0  # Start with Canberra
        self.dsn_locations = ["Canberra", "Madrid", "Goldstone"]

        print(f"Lunar Exploration DSN Simulator initialized")
        print(f"  TM Host: {self.tm_host}")
        print(f"  TM Port: {self.tm_port}")
        print(f"  Rate: {self.rate} Hz")

    def generate_mock_data(self):
        """Generate mock values for DSN telemetry parameters"""
        elapsed = time.time() - self.start_time

        # DSN Location: Rotate through locations every 30 seconds
        if int(elapsed) % 30 == 0 and self.tm_counter % int(self.rate * 30) == 0:
            self.current_dsn_location = (self.current_dsn_location + 1) % 3
        dsn_location = self.current_dsn_location

        # Uplink Power (dBW): Ground station transmitter power
        # Range: +40 to +53 dBW (High Power)
        # Add some variation to simulate power adjustments
        base_uplink = 46.5  # Middle of range
        uplink_variation = 3.0 * math.sin(elapsed * 0.2)  # ±3 dBW variation
        uplink_power = base_uplink + uplink_variation

        # Downlink Signal (dBm): Very weak signal from spacecraft
        # Range: -120 to -160 dBm (Very Weak)
        # Simulate signal fading and atmospheric effects
        base_downlink = -140.0  # Middle of range
        fade_variation = 10.0 * math.sin(elapsed * 0.1)  # ±10 dBm fading
        atmospheric_noise = random.uniform(-5.0, 5.0)  # Random atmospheric effects
        downlink_signal = base_downlink + fade_variation + atmospheric_noise

        # Downlink SNR (dB): Signal quality
        # Range: 2 to 20+ dB
        # SNR typically improves with stronger downlink signal
        # Calculate SNR based on downlink signal strength
        # Better signal (closer to -120) = higher SNR
        normalized_signal = (downlink_signal + 160.0) / 40.0  # Normalize to 0-1
        base_snr = 2.0 + normalized_signal * 18.0  # Scale to 2-20 dB
        snr_jitter = random.uniform(-2.0, 2.0)  # Add some random variation
        downlink_snr = max(2.0, min(20.0, base_snr + snr_jitter))

        return {
            'dsn_location': dsn_location,
            'uplink_power': uplink_power,
            'downlink_signal': downlink_signal,
            'downlink_snr': downlink_snr
        }

    def build_telemetry_packet(self):
        """Build complete telemetry packet with CCSDS header and DSN parameters"""
        data = self.generate_mock_data()

        # APID 0x28 (40 decimal) for Ground Telemetry Packet
        packet_dict = {
            'header': {
                'version': 0,
                "type": 0,  # 0 = Telemetry
                'secondary_header_flag': False,
                'apid': 0x28,  # Ground Telemetry APID
                'sequence_flags': 3,  # 3 = Unsegmented
                'sequence_count': self.sequence_count,
                'packet_length': 0  # Placeholder, will be calculated
            },
            **data
        }

        # Build the packet first to determine actual size
        packet = DSN_TM_PACKET_STRUCT.build(packet_dict)

        # Calculate actual packet_length field (packet length - CCSDS header size - 1)
        packet_dict['header']['packet_length'] = len(packet) - CCSDSHeader.sizeof() - 1

        # Rebuild with correct packet_length
        packet = DSN_TM_PACKET_STRUCT.build(packet_dict)

        return packet, data

    def send_telemetry(self):
        """
        Telemetry sending thread - generates and sends packets at configured rate
        """
        tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"\nStarting DSN telemetry transmission at {self.rate} Hz...")
        print("Press Ctrl+C to stop\n")

        try:
            while True:
                # Build and send packet
                packet, data = self.build_telemetry_packet()
                tm_socket.sendto(packet, (self.tm_host, self.tm_port))

                # Log telemetry
                if self.tm_counter % 5 == 0:
                    print(f"[{self.tm_counter:5d}] DSN: {self.dsn_locations[data['dsn_location']]:10s} | "
                          f"Uplink: {data['uplink_power']:+6.2f} dBW | "
                          f"Downlink: {data['downlink_signal']:7.2f} dBm | "
                          f"SNR: {data['downlink_snr']:5.2f} dB")

                # Update counters
                self.tm_counter += 1
                self.sequence_count = (self.sequence_count + 1) % 16384  # 14-bit counter

                # Sleep for rate control
                time.sleep(1.0 / self.rate)

        except KeyboardInterrupt:
            print("\nStopping DSN telemetry transmission...")
        finally:
            tm_socket.close()

    def print_status(self):
        """Print current simulator status"""
        elapsed = time.time() - self.start_time
        print(f"\n{'='*70}")
        print(f"DSN Simulator Status:")
        print(f"  TM packets sent: {self.tm_counter}")
        print(f"  Elapsed time: {elapsed:.1f}s")
        print(f"  Rate: {self.rate} Hz")
        print(f"  Current DSN: {self.dsn_locations[self.current_dsn_location]}")
        print(f"{'='*70}")

    def start(self):
        """Start the simulator"""
        # Start telemetry thread
        tm_thread = Thread(target=self.send_telemetry, daemon=True)
        tm_thread.start()

        # Status reporting loop
        try:
            while True:
                time.sleep(30)
                self.print_status()
        except KeyboardInterrupt:
            print("\nDSN Simulator stopped")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Lunar Exploration DSN Telemetry Simulator - Sends mock DSN telemetry to YAMCS'
    )
    parser.add_argument(
        '--tm-host',
        type=str,
        default='localhost',
        help='Telemetry destination host (default: localhost)'
    )
    parser.add_argument(
        '--tm-port',
        type=int,
        default=2238,
        help='Telemetry destination port (default: 2238)'
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=1.0,
        help='Telemetry rate in Hz (default: 1.0)'
    )

    args = parser.parse_args()

    # Create and start simulator
    simulator = DSNSimulator(
        tm_host=args.tm_host,
        tm_port=args.tm_port,
        rate=args.rate
    )
    simulator.start()


if __name__ == '__main__':
    main()
