#!/usr/bin/env python3

import rclpy
from rclpy.node  import Node


import binascii
import io
import socket
import sys
import argparse

from struct import unpack_from
from threading import Thread
from time import sleep


# **********************************************
def send_tm(simulator):
    tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    with io.open(simulator.TEST_DATA, 'rb') as f:
        simulator.tm_counter = 1
        header = bytearray(6)
        while f.readinto(header) == 6:
            (len,) = unpack_from('>H', header, 4)

            packet = bytearray(len + 7)
            f.seek(-6, io.SEEK_CUR)
            f.readinto(packet)

            tm_socket.sendto(packet, (simulator.TM_SEND_ADDRESS, simulator.TM_SEND_PORT))
            simulator.tm_counter += 1

            sleep(1 / simulator.rate)

# **********************************************
def receive_tc(simulator):
    tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tc_socket.bind((simulator.TC_RECEIVE_ADDRESS, simulator.TC_RECEIVE_PORT ))
    while True:
        data, _ = tc_socket.recvfrom(4096)
        simulator.last_tc = data
        simulator.tc_counter += 1

# **********************************************
class Simulator(Node):

    def __init__(self):
        super().__init__('quickstart_simulator')
        
        self.tm_counter = 0
        self.tc_counter = 0
        self.tm_thread = None
        self.tc_thread = None
        self.last_tc = None
        self.prev_status = None

        self.timer = self.create_timer(0.5, self.timer_cb)

        self.declare_parameter("test_data", rclpy.Parameter.Type.STRING)
        self.declare_parameter("tm_host", rclpy.Parameter.Type.STRING) #'127.0.0.1'
        self.declare_parameter("tm_port", rclpy.Parameter.Type.INTEGER) #'10015'
        self.declare_parameter("rate", rclpy.Parameter.Type.INTEGER) #'1 Hz'

        self.declare_parameter("tc_host", rclpy.Parameter.Type.STRING) #'127.0.0.1'
        self.declare_parameter("tc_port", rclpy.Parameter.Type.INTEGER) #'10025'

        self.TEST_DATA = self.get_parameter("test_data").value
        self.TM_SEND_ADDRESS = self.get_parameter("tm_host").value
        self.TM_SEND_PORT = self.get_parameter("tm_port").value
        self.rate = self.get_parameter("rate").value

        self.TC_RECEIVE_ADDRESS = self.get_parameter("tc_host").value
        self.TC_RECEIVE_PORT    = self.get_parameter("tc_port").value

        
    def start(self):
        self.tm_thread = Thread(target=send_tm, args=(self,))
        self.tm_thread.daemon = True
        self.tm_thread.start()
        self.tc_thread = Thread(target=receive_tc, args=(self,))
        self.tc_thread.daemon = True
        self.tc_thread.start()
        
        self.get_logger().info('Using playback rate of {} Hz'.format(self.rate) );
        self.get_logger().info('TM host= {}, TM port= {}'.format(self.TM_SEND_ADDRESS, self.TM_SEND_PORT) );
        self.get_logger().info('TC host= {}, TC port= {}'.format(self.TC_RECEIVE_ADDRESS, self.TC_RECEIVE_PORT) );

    def print_status(self):
        cmdhex = None
        if self.last_tc:
            cmdhex = binascii.hexlify(self.last_tc).decode('ascii')
        return 'Sent: {} packets. Received: {} commands. Last command: {}'.format(
                         self.tm_counter, self.tc_counter, cmdhex)

    def timer_cb(self):
      prev_status = None
      status = self.print_status()
      if status != prev_status:
         self.get_logger().info(status)
         prev_status = status


# **********************************************    
if __name__ == '__main__':

  rclpy.init(args=None)
  quickstart_sim = Simulator()
  quickstart_sim.start()
  rclpy.spin(quickstart_sim)
  quickstart_sim.destroy_node()
  rclpy.shutdown()

