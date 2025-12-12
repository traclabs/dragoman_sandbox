#!/usr/bin/env python3

import binascii
import socket
import argparse

from struct import unpack_from, pack
from threading import Thread
from time import sleep

# *************************
# Send telemetry
# *************************
def send_tm(simulator):
    tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    prim_header_length = 6 # Prim header: 6,
    sec_header_length = 10 # Sec header: 6 + 4 spare
    full_header_length = prim_header_length + sec_header_length
    tm_data_length = 4*9 # 4 bytes of joint_state's float type * 9 joints of full arm

    simulator.tm_counter = 1
    prim_header = bytearray(prim_header_length)   
    tm_count = 0xc000    
    
    # Just to update data
    js_vals = [0.25, 0.36, 0.49, 0.64, 0.81, 1.21, 2.35, 0.22, 0.87]
    while True:
    
        tm_msg_length = full_header_length + tm_data_length
        # 1-3 bits: 000 (packet version number)
        # 4 bit: 0 (telemetry)
        # 5 bit: 0 (secondary header)
        # 6-16 bits: 100 (APID - identifier)
        # 17-18 bit: 11 (sequence flag: 11 = unsegmented data)
        # 19-32 bit: 14 (sequential binary count)
        # 32-48 bit:
        # 000|0  |  0|000 0110 0100 | 11 | 00 0010 1110 1000
        prim_header = pack('>HHH', 0x0064, tm_count, tm_data_length - 1)

        jsi_vals = [x + 0.001*float(simulator.tm_counter) for x in js_vals]
        js = pack('>fffffffff', jsi_vals[0], jsi_vals[1], jsi_vals[2], jsi_vals[3], jsi_vals[4], jsi_vals[5], jsi_vals[6], jsi_vals[7], jsi_vals[8])

        # Debug
        #packet_hex = binascii.hexlify(packet).decode('ascii')
        #print("Packet: {}".format(packet_hex))


        packet = bytearray(tm_msg_length)
        packet[0:prim_header_length] = prim_header
        packet[full_header_length:tm_msg_length] = js
        
        tm_socket.sendto(packet, (simulator.TM_SEND_ADDRESS, simulator.TM_SEND_PORT))
        tm_count += 1
        simulator.tm_counter += 1

        sleep(1 / simulator.rate)

# *************************
# Receive command
# *************************
def receive_tc(simulator):
    tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tc_socket.bind((simulator.TC_RECEIVE_ADDRESS, simulator.TC_RECEIVE_PORT ))

    while True:
        data, _ = tc_socket.recvfrom(4096)
        parse_tc_data(data)

        simulator.last_tc = data
        simulator.tc_counter += 1


def parse_tc_data(data, logger):
  logger.info("Received command of length: {}".format(len(data)))

  # Read length of command data
  offset = 4
  tc_length = (unpack_from('>H', data, offset))[0]
  
  # Read command id 
  offset = 8 # prim header: 6, secondary header: 2
  command_id = (unpack_from('>H', data, offset))[0]

  print("Received command of length: {}, tc_length: {} with command_id: {}".format(len(data), tc_length, command_id))

  # From the xtce, canned_pose: command_id= 0, arbitrary joint goal=1
  if command_id == 0:
    parse_canned_pose(data)
  if command_id == 1:
    parse_arm_joint_state_goal(data)
  if command_id == 2:
    parse_rail_joint_state_goal(data)
  if command_id == 3:
    parse_lift_joint_state_goal(data)


def parse_canned_pose(data, logger):
  logger.info("No implemented yet!")
  
def parse_arm_joint_state_goal(data, logger):
  
  header_length = 8
  command_id_length = 2
  float_length = 4

  offset = header_length + command_id_length

  js_goal = [0, 0, 0, 0, 0, 0]
  for i in range (0, 6):
    js_goal[i] = (unpack_from('>f', data, offset))[0]
    offset += float_length

  js_goal_print = [f"{item:.3f}" for item in js_goal]
  print("* Arm Joint goal: {}".format(js_goal_print))

def parse_rail_joint_state_goal(data):

def parse_rail_joint_state_goal(data, logger):
  
  header_length = 8
  command_id_length = 2
  float_length = 4

  offset = header_length + command_id_length

  js_goal = (unpack_from('>f', data, offset))[0]

  print("* Rail Joint goal: {:.3f}".format(js_goal))

def parse_lift_joint_state_goal(data):

def parse_lift_joint_state_goal(data, logger):
  
  header_length = 8
  command_id_length = 2
  float_length = 4

  offset = header_length + command_id_length

  js_goal = (unpack_from('>f', data, offset))[0]

  print("* Lift Joint goal: {:.3f}".format(js_goal))


# **********************************************
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

        print('* Using playback rate of {} Hz'.format(self.rate) );
        print('TM host= {}, TM port= {}'.format(self.TM_SEND_ADDRESS, self.TM_SEND_PORT) );
        print('TC host= {}, TC port= {}'.format(self.TC_RECEIVE_ADDRESS, self.TC_RECEIVE_PORT) );


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
         print(status)
         prev_status = status


# **********************************************
if __name__ == '__main__':

  parser = argparse.ArgumentParser()
  parser.add_argument("--tm-host", type=str, default='127.0.0.1')
  parser.add_argument("--tm-port", type=int, default=10015)
  parser.add_argument("--tc-host", type=str, default='127.0.0.1')
  parser.add_argument("--tc-port", type=int, default=10025)
  parser.add_argument("--rate", type=int, default=1)

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
