#!/usr/bin/env python3

"""
MultiPacket Telemetry Simulator

A simulator that generates mock telemetry data for voltage and temperature
parameters defined in MultiPacket.xtce and sends them to YAMCS via UDP.

This simulator demonstrates multi-packet telemetry with different APIDs:
- TemperaturePacket (APID 130): Temperature readings in degrees Celsius
- VoltagePacket (APID 131): Voltage readings in Volts

The simulator alternates between sending temperature and voltage packets,
each with realistic physical patterns and noise characteristics.

No ROS dependencies - pure Python mocking.
"""

import socket
import time
import random
import math
import argparse
from threading import Thread

from dragoman_fsw_sim.multipacket_construct_definitions import (
    TEMPERATURE_PACKET_STRUCT,
    VOLTAGE_PACKET_STRUCT
)
from dragoman_fsw_sim.ccsds_header_definitions import CCSDSHeader


class MultiPacketSimulator:

    def __init__(self, tm_host='127.0.0.1', tm_port=10015, rate=1,
                 temp_apid=130, voltage_apid=131):
        self.tm_host = tm_host
        self.tm_port = tm_port
        self.rate = rate
        self.temp_apid = temp_apid
        self.voltage_apid = voltage_apid

        self.tm_counter = 0
        self.temp_sequence_count = 0
        self.voltage_sequence_count = 0
        self.start_time = time.time()

        self.base_temperature = 22.0
        self.temp_cycle_period = 60.0
        self.temp_spike_probability = 0.02

        self.nominal_voltage = 28.0
        self.voltage_drift_period = 120.0
        self.voltage_drop_probability = 0.01

        print(f"MultiPacket Simulator initialized")
        print(f"  TM Host: {self.tm_host}")
        print(f"  TM Port: {self.tm_port}")
        print(f"  Rate: {self.rate} Hz (alternating packet types)")
        print(f"  Temperature APID: {self.temp_apid}")
        print(f"  Voltage APID: {self.voltage_apid}")

    def generate_temperature(self):
        elapsed = time.time() - self.start_time

        thermal_cycle = 3.0 * math.sin(2 * math.pi * elapsed / self.temp_cycle_period)
        sensor_noise = random.gauss(0, 0.1)

        thermal_spike = 0.0
        if random.random() < self.temp_spike_probability:
            thermal_spike = random.uniform(2.0, 5.0)

        temperature = self.base_temperature + thermal_cycle + sensor_noise + thermal_spike

        return temperature

    def generate_voltage(self):
        elapsed = time.time() - self.start_time

        voltage_drift = 0.5 * math.sin(2 * math.pi * elapsed / self.voltage_drift_period)
        ripple = random.gauss(0, 0.05)

        voltage_drop = 0.0
        if random.random() < self.voltage_drop_probability:
            voltage_drop = -random.uniform(0.5, 2.0)

        voltage = self.nominal_voltage + voltage_drift + ripple + voltage_drop

        return voltage

    def build_temperature_packet(self):
        temperature = self.generate_temperature()

        packet_length = TEMPERATURE_PACKET_STRUCT.sizeof() - CCSDSHeader.sizeof() - 1

        packet = TEMPERATURE_PACKET_STRUCT.build({
            'header': {
                'version': 0,
                'type': False,
                'secondary_header_flag': False,
                'apid': self.temp_apid,
                'sequence_flags': 3,
                'sequence_count': self.temp_sequence_count,
                'packet_length': packet_length
            },
            'temperature': temperature
        })

        return packet

    def build_voltage_packet(self):
        voltage = self.generate_voltage()

        packet_length = VOLTAGE_PACKET_STRUCT.sizeof() - CCSDSHeader.sizeof() - 1

        packet = VOLTAGE_PACKET_STRUCT.build({
            'header': {
                'version': 0,
                'type': False,
                'secondary_header_flag': False,
                'apid': self.voltage_apid,
                'sequence_flags': 3,
                'sequence_count': self.voltage_sequence_count,
                'packet_length': packet_length
            },
            'voltage': voltage
        })

        return packet

    def send_telemetry(self):
        tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"\nStarting telemetry transmission at {self.rate} Hz...")
        print("Alternating between TemperaturePacket and VoltagePacket")
        print("Press Ctrl+C to stop\n")

        try:
            while True:
                if self.tm_counter % 2 == 0:
                    packet = self.build_temperature_packet()
                    tm_socket.sendto(packet, (self.tm_host, self.tm_port))
                    self.temp_sequence_count = (self.temp_sequence_count + 1) % 16384
                else:
                    packet = self.build_voltage_packet()
                    tm_socket.sendto(packet, (self.tm_host, self.tm_port))
                    self.voltage_sequence_count = (self.voltage_sequence_count + 1) % 16384

                self.tm_counter += 1

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
        default=130,
        help='APID for TemperaturePacket (default: 130)'
    )
    parser.add_argument(
        '--voltage-apid',
        type=int,
        default=131,
        help='APID for VoltagePacket (default: 131)'
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
