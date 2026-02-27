#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

class SerializeMobileServicingSystemManual
{
 public:

 SerializeMobileServicingSystemManual();
 bool initializeComm( const int &_own_port, const int &_other_port,
                      const std::string &_robot_ip,
                      const std::string &_fsw_ip,
                      std::string &_error_msg);
 bool sendMessage( sensor_msgs::msg::JointState* _js, uint8_t motion_status[8] );
 bool receiveMessage(std::string &_group, std::string &_state);

 protected:

  size_t serialize(sensor_msgs::msg::JointState* _js, uint8_t motion_status[8], uint8_t** buf);
  bool deserialize(const uint8_t* buf, const size_t bufSize, size_t start_offset, std::string &_group, std::string &_state);

  bool getString(char _input_string[], int _input_size, int _start_index, char _output_string[], int &_end_index);

  int sockfd_;
  char buffer_[1024];
  struct sockaddr_in own_address_;
  struct sockaddr_in other_address_;


};
