#!/usr/bin/env python3

import rclpy
from rclpy.node  import Node


import binascii
import io
import socket
import sys
import argparse

from struct import unpack_from, pack
from threading import Thread
from time import sleep

# **********************************************
def send_tm(simulator):
    tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    header_length = 6
    tm_data_length = 4*6 # 32 bits of joint_state's float type * array size

    simulator.tm_counter = 1
    header = bytearray(header_length)   
    tm_count = 0xc000    
    
    # Just to update data
    js_vals = [0.25, 0.36, 0.49, 0.64, 0.81, 1.21]
    while True:
    
        tm_msg_length = header_length + tm_data_length
        # 1-3 bits: 000 (packet version number)
        # 4 bit: 0 (telemetry)
        # 5 bit: 0 (secondary header)
        # 6-16 bits: 100 (APID - identifier)
        # 17-18 bit: 11 (sequence flag: 11 = unsegmented data)
        # 19-32 bit: 14 (sequential binary count)
        # 32-48 bit: 	
        # 000|0  |  0|000 0110 0100 | 11 | 00 0010 1110 1000
        header = pack('>HHH', 0x0064, tm_count, tm_data_length - 1)

        jsi_vals = [x + 0.001*float(simulator.tm_counter) for x in js_vals]
        simulator.get_logger().info("Sim jsi: {}, {}".format(jsi_vals[0], jsi_vals[1]))
        js = pack('>ffffff', jsi_vals[0], jsi_vals[1], jsi_vals[2], jsi_vals[3], jsi_vals[4], jsi_vals[5])

        # Debug  
        #packet_hex = binascii.hexlify(packet).decode('ascii')
        #simulator.get_logger().info("Packet: {}".format(packet_hex))

         
        packet = bytearray(tm_msg_length)
        packet[0:header_length] = header
        packet[header_length:tm_msg_length] = js
        
        tm_socket.sendto(packet, (simulator.TM_SEND_ADDRESS, simulator.TM_SEND_PORT))
        tm_count += 1
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
        super().__init__('imetro_simulator')
        
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
        
        self.get_logger().info('* Using playback rate of {} Hz'.format(self.rate) );
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

