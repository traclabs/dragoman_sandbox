#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

// Navigation status enum
enum NavigationStatus
{
  NAV_STATUS_IDLE = 0,
  NAV_STATUS_IN_PROGRESS = 1,
  NAV_STATUS_DONE = 2
};

class SerializeLunarExplorationManual
{
 public:

 SerializeLunarExplorationManual();
 bool initializeComm( const int &_own_port, const int &_other_port,
                      const std::string &_robot_ip,
                      const std::string &_fsw_ip,
                      std::string &_error_msg);
 bool sendMessage( sensor_msgs::msg::JointState* _js, geometry_msgs::msg::Pose _pose, uint8_t _nav_status, float _solar_left, float _solar_right, float _solar_rear );
 bool peekCommandCode(uint8_t &_code);
 bool receiveTwistCommand(float &_linear_vel, float &_angular_vel);
 bool receiveCameraCommand(float &_pan, float &_tilt);
 bool receiveNavigationPoseCommand(float &_x, float &_y, float &_theta);

 protected:

  size_t serialize(sensor_msgs::msg::JointState* _js, geometry_msgs::msg::Pose _pose, uint8_t _nav_status, float _solar_left, float _solar_right, float _solar_rear, uint8_t** buf);
  bool deserialize(const uint8_t* buf, const size_t bufSize, size_t start_offset, uint8_t &code, float &_val1, float &_val2);

  int sockfd_;
  char buffer_[1024];
  struct sockaddr_in own_address_;
  struct sockaddr_in other_address_;


};
