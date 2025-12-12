#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

class SerializeArmManual
{
 public:
 
 SerializeArmManual();
 bool initializeComm( const int &_own_port, const int &_other_port, 
                      const std::string &_robot_ip, 
                      const std::string &_fsw_ip,
                      std::string &_error_msg);
 bool sendMessage( sensor_msgs::msg::JointState* _js );
 bool receiveMessage(geometry_msgs::msg::Pose &_pose);
 
 protected:

  size_t serialize(sensor_msgs::msg::JointState* _js, uint8_t** buf);
  geometry_msgs::msg::Pose deserialize(const uint8_t* buf, const size_t bufSize, size_t start_offset);

  int sockfd_;
  char buffer_[1024];
  struct sockaddr_in own_address_;
  struct sockaddr_in other_address_;
 
 
};
