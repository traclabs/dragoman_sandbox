#!/usr/bin/env python3

"""
MultiPacket Telemetry Simulator

A simulator that generates mock telemetry data for voltage and temperature
parameters defined in MultiPacket.xtce and sends them to YAMCS via UDP.

This simulator demonstrates multi-packet telemetry with different APIDs:
- TemperaturePacket (APID 101): Temperature readings in degrees Celsius
- VoltagePacket (APID 102): Voltage readings in Volts

The simulator alternates between sending temperature and voltage packets,
each with realistic physical patterns and noise characteristics.

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


class MultiPacketSimulator:
    """Simulator that generates and sends Temperature and Voltage telemetry packets"""

    def __init__(self, tm_host='127.0.0.1', tm_port=10015, rate=1,
                 temp_apid=110, voltage_apid=111):
        """
        Initialize the simulator

        Args:
            tm_host: Telemetry destination host
            tm_port: Telemetry destination port
            rate: Telemetry rate in Hz (packets per second, alternating types)
            temp_apid: APID for TemperaturePacket (default: 110)
            voltage_apid: APID for VoltagePacket (default: 111)
        """
        self.tm_host = tm_host
        self.tm_port = tm_port
        self.rate = rate
        self.temp_apid = temp_apid
        self.voltage_apid = voltage_apid

        # Counters
        self.tm_counter = 0
        self.temp_sequence_count = 0
        self.voltage_sequence_count = 0

        # Start time for simulation patterns
        self.start_time = time.time()

        # Temperature simulation state
        self.base_temperature = 22.0  # Room temperature baseline (°C)
        self.temp_cycle_period = 60.0  # Thermal cycle period (seconds)
        self.temp_spike_probability = 0.02  # 2% chance of thermal event per packet

        # Voltage simulation state
        self.nominal_voltage = 28.0  # Nominal bus voltage (V)
        self.voltage_drift_period = 120.0  # Slow drift period (seconds)
        self.voltage_drop_probability = 0.01  # 1% chance of load event per packet

        print(f"MultiPacket Simulator initialized")
        print(f"  TM Host: {self.tm_host}")
        print(f"  TM Port: {self.tm_port}")
        print(f"  Rate: {self.rate} Hz (alternating packet types)")
        print(f"  Temperature APID: {self.temp_apid}")
        print(f"  Voltage APID: {self.voltage_apid}")

    def generate_temperature(self):
        """
        Generate realistic temperature value with thermal patterns

        Returns:
            float: Temperature in degrees Celsius
        """
        elapsed = time.time() - self.start_time

        # Base thermal cycle (slow sinusoidal variation)
        thermal_cycle = 3.0 * math.sin(2 * math.pi * elapsed / self.temp_cycle_period)

        # Sensor noise (small random fluctuations)
        sensor_noise = random.gauss(0, 0.1)

        # Occasional thermal spikes (simulating heat-generating events)
        thermal_spike = 0.0
        if random.random() < self.temp_spike_probability:
            thermal_spike = random.uniform(2.0, 5.0)

        # Combine all components
        temperature = self.base_temperature + thermal_cycle + sensor_noise + thermal_spike

        return temperature

    def generate_voltage(self):
        """
        Generate realistic voltage value with power supply characteristics

        Returns:
            float: Voltage in Volts
        """
        elapsed = time.time() - self.start_time

        # Slow voltage drift (aging, temperature effects)
        voltage_drift = 0.5 * math.sin(2 * math.pi * elapsed / self.voltage_drift_period)

        # Power supply ripple (high-frequency noise)
        ripple = random.gauss(0, 0.05)

        # Occasional voltage drops (simulating load changes)
        voltage_drop = 0.0
        if random.random() < self.voltage_drop_probability:
            voltage_drop = -random.uniform(0.5, 2.0)

        # Combine all components
        voltage = self.nominal_voltage + voltage_drift + ripple + voltage_drop

        return voltage

    def build_ccsds_header(self, apid, sequence_count, data_length):
        """
        Build CCSDS Space Packet Primary Header (6 bytes)

        Args:
            apid: Application Process Identifier
            sequence_count: Packet sequence counter
            data_length: Length of packet data (excluding header)

        Returns:
            bytes: 6-byte CCSDS header
        """
        # Packet ID (2 bytes)
        # - version (3 bits): 0
        # - type (1 bit): 0 (telemetry)
        # - secondary_header_flag (1 bit): 0 (not present)
        # - apid (11 bits): provided as parameter
        packet_id = (0 << 13) | (0 << 12) | (0 << 11) | (apid & 0x7FF)

        # Packet Sequence Control (2 bytes)
        # - sequence_flags (2 bits): 3 (unsegmented)
        # - sequence_count (14 bits): incrementing counter
        packet_sequence = (3 << 14) | (sequence_count & 0x3FFF)

        # Packet Length (2 bytes)
        # Length of packet data - 1 (as per CCSDS standard)
        packet_length = data_length - 1

        # Pack as big-endian (network byte order)
        header = struct.pack('>HHH', packet_id, packet_sequence, packet_length)

        return header

    def build_temperature_packet(self):
        """
        Build complete TemperaturePacket with CCSDS header

        Returns:
            bytes: Complete telemetry packet
        """
        # Generate temperature value
        temperature = self.generate_temperature()

        # Build packet data (single 32-bit float, big-endian)
        packet_data = struct.pack('>f', temperature)

        # Build CCSDS header
        header = self.build_ccsds_header(
            self.temp_apid,
            self.temp_sequence_count,
            len(packet_data)
        )

        # Combine header and data
        packet = header + packet_data

        return packet

    def build_voltage_packet(self):
        """
        Build complete VoltagePacket with CCSDS header

        Returns:
            bytes: Complete telemetry packet
        """
        # Generate voltage value
        voltage = self.generate_voltage()

        # Build packet data (single 32-bit float, big-endian)
        packet_data = struct.pack('>f', voltage)

        # Build CCSDS header
        header = self.build_ccsds_header(
            self.voltage_apid,
            self.voltage_sequence_count,
            len(packet_data)
        )

        # Combine header and data
        packet = header + packet_data

        return packet

    def send_telemetry(self):
        """
        Telemetry sending thread - generates and sends packets at configured rate
        Alternates between TemperaturePacket and VoltagePacket
        """
        tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"\nStarting telemetry transmission at {self.rate} Hz...")
        print("Alternating between TemperaturePacket and VoltagePacket")
        print("Press Ctrl+C to stop\n")

        try:
            while True:
                # Alternate between temperature and voltage packets
                if self.tm_counter % 2 == 0:
                    # Send TemperaturePacket
                    packet = self.build_temperature_packet()
                    tm_socket.sendto(packet, (self.tm_host, self.tm_port))
                    self.temp_sequence_count = (self.temp_sequence_count + 1) % 16384
                else:
                    # Send VoltagePacket
                    packet = self.build_voltage_packet()
                    tm_socket.sendto(packet, (self.tm_host, self.tm_port))
                    self.voltage_sequence_count = (self.voltage_sequence_count + 1) % 16384

                # Update counter
                self.tm_counter += 1

                # Sleep for rate control
                time.sleep(1.0 / self.rate)

        except KeyboardInterrupt:
            print("\nStopping telemetry transmission...")
        finally:
            tm_socket.close()

    def print_status(self):
        """Print current simulator status"""
        elapsed = time.time() - self.start_time
        temp_count = (self.tm_counter + 1) // 2
        voltage_count = self.tm_counter // 2
        print(f"Packets sent: {self.tm_counter} "
              f"(Temp: {temp_count}, Voltage: {voltage_count}), "
              f"Elapsed: {elapsed:.1f}s, Rate: {self.rate} Hz")

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
        description='MultiPacket Telemetry Simulator - Sends temperature and voltage telemetry to YAMCS'
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
        help='Telemetry rate in Hz - alternates between packet types (default: 1.0)'
    )
    parser.add_argument(
        '--temp-apid',
        type=int,
        default=110,
        help='APID for TemperaturePacket (default: 110)'
    )
    parser.add_argument(
        '--voltage-apid',
        type=int,
        default=111,
        help='APID for VoltagePacket (default: 111)'
    )

    args = parser.parse_args()

    # Create and start simulator
    simulator = MultiPacketSimulator(
        tm_host=args.tm_host,
        tm_port=args.tm_port,
        rate=args.rate,
        temp_apid=args.temp_apid,
        voltage_apid=args.voltage_apid
    )
    simulator.start()


if __name__ == '__main__':
    main()
