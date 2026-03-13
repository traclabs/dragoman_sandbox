/**
 * @file serialize_mobile_servicing_system_manual.cpp
 */
#include <dragoman_fsw_sim/serialize_lunar_exploration_manual.h>
#include <vector>
#include <rclcpp/rclcpp.hpp>
SerializeLunarExplorationManual::SerializeLunarExplorationManual()
{}

bool SerializeLunarExplorationManual::initializeComm( const int &_own_port,
                                         const int &_other_port,
                                         const std::string &_robot_ip,
                                         const std::string &_fsw_ip,
                                         std::string &_error_msg)
{
  // Create socket
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if(sockfd_ < 0)
  {
    _error_msg = "Socket creation failed";
    return false;
  }

  memset(&own_address_, 0, sizeof(own_address_));
  memset(&other_address_, 0, sizeof(other_address_));

  // Fill server information
  own_address_.sin_family = AF_INET;
  own_address_.sin_addr.s_addr = inet_addr(_robot_ip.c_str()); // "127.0.0.1" //INADDR_ANY;
  own_address_.sin_port = htons(_own_port);


  // Fill cFS information
  other_address_.sin_family = AF_INET;
  other_address_.sin_addr.s_addr = inet_addr(_fsw_ip.c_str()); // "127.0.0.1" //INADDR_ANY;
  other_address_.sin_port = htons(_other_port);

  // Bind the socket
  int res = bind(sockfd_, (const struct sockaddr*)&own_address_, sizeof(own_address_));
  if( res < 0 )
  {
    _error_msg = "Bind failed";
    return false;
  }

  return true;
}

/**
 * @brief Serialize joint state from robot and send back to cFS
 */
bool SerializeLunarExplorationManual::sendMessage( sensor_msgs::msg::JointState* _js, geometry_msgs::msg::Pose _pose, uint8_t _nav_status, float _solar_left, float _solar_right, float _solar_rear )
{
  unsigned char* buf     = 0;
  size_t         bufSize = serialize(_js, _pose, _nav_status, _solar_left, _solar_right, _solar_rear, &buf);

  int res = sendto(sockfd_, buf, bufSize, 0, (const struct sockaddr *)&other_address_, sizeof(other_address_));

  // Clean up
  free(buf);

  return (res > 0);
}


bool SerializeLunarExplorationManual::peekCommandCode(uint8_t &_code)
{
  ssize_t buffer_rcvd_size;
  const int MAXLINE = 1024;
  uint8_t buffer_rcvd[MAXLINE];

  // Peek at the buffer without consuming it using MSG_PEEK flag
  buffer_rcvd_size = recvfrom(sockfd_, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT | MSG_PEEK, (struct sockaddr*)NULL, NULL);
  if(buffer_rcvd_size > 0)
  {
    // Extract just the command code (first byte)
    memcpy(&_code, buffer_rcvd, sizeof(uint8_t));
    return true;
  }

  return false;
}

bool SerializeLunarExplorationManual::receiveTwistCommand(float &_linear_vel, float &_angular_vel)
{
  ssize_t buffer_rcvd_size;
  const int MAXLINE = 1024;
  uint8_t buffer_rcvd[MAXLINE];

  // Receive and consume the message
  buffer_rcvd_size = recvfrom(sockfd_, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
  if(buffer_rcvd_size > 0)
  {
    // Deserialize: 1 byte (code) + 2 floats (linear_vel, angular_vel)
    size_t offset = 0;

    // Skip the command code (already peeked)
    offset += sizeof(uint8_t);

    memcpy(&_linear_vel, buffer_rcvd + offset, sizeof(float));
    offset += sizeof(float);

    memcpy(&_angular_vel, buffer_rcvd + offset, sizeof(float));
    offset += sizeof(float);

    return true;
  }

  return false;
}

bool SerializeLunarExplorationManual::receiveCameraCommand(float &_pan, float &_tilt)
{
  ssize_t buffer_rcvd_size;
  const int MAXLINE = 1024;
  uint8_t buffer_rcvd[MAXLINE];

  // Receive and consume the message
  buffer_rcvd_size = recvfrom(sockfd_, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
  if(buffer_rcvd_size > 0)
  {
    // Deserialize: 1 byte (code) + 2 floats (pan, tilt)
    size_t offset = 0;

    // Skip the command code (already peeked)
    offset += sizeof(uint8_t);

    memcpy(&_pan, buffer_rcvd + offset, sizeof(float));
    offset += sizeof(float);

    memcpy(&_tilt, buffer_rcvd + offset, sizeof(float));
    offset += sizeof(float);

    return true;
  }

  return false;
}

bool SerializeLunarExplorationManual::receiveNavigationPoseCommand(float &_x, float &_y, float &_theta)
{
  ssize_t buffer_rcvd_size;
  const int MAXLINE = 1024;
  uint8_t buffer_rcvd[MAXLINE];

  // Receive and consume the message
  buffer_rcvd_size = recvfrom(sockfd_, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
  if(buffer_rcvd_size > 0)
  {
    // Deserialize: 1 byte (code) + 3 floats (x, y, theta)
    size_t offset = 0;

    // Skip the command code (already peeked)
    offset += sizeof(uint8_t);

    memcpy(&_x, buffer_rcvd + offset, sizeof(float));
    offset += sizeof(float);

    memcpy(&_y, buffer_rcvd + offset, sizeof(float));
    offset += sizeof(float);

    memcpy(&_theta, buffer_rcvd + offset, sizeof(float));
    offset += sizeof(float);

    return true;
  }

  return false;
}
/////////////////////////////

/**
 * @function serialize
 * @brief Send data back
 */
size_t SerializeLunarExplorationManual::serialize(sensor_msgs::msg::JointState *_js, geometry_msgs::msg::Pose _pose, uint8_t _nav_status, float _solar_left, float _solar_right, float _solar_rear, uint8_t** _buf)
{
  if (_js->name.size() == 0 || _js->position.size() == 0)
    return 0;

  size_t num_joints = _js->position.size();
  /*
  - front_left_suspension_joint
- front_left_wheel_axle_joint
- front_left_wheel_joint
- front_right_suspension_joint
- front_right_wheel_axle_joint
- front_right_wheel_joint
- left_solar_panel_joint
- mast_camera_joint
- mast_head_pivot_joint
- rear_left_suspension_joint
- rear_left_wheel_axle_joint
- rear_left_wheel_joint
- rear_right_suspension_joint
- rear_right_wheel_axle_joint
- rear_right_wheel_joint
- rear_solar_panel_joint
- right_solar_panel_joint
*/
  // xyz, qxyzw = 17 + 7 = 24
  float x, y, z, qx, qy, qz, qw;
  x = (float)_pose.position.x;
  y = (float)_pose.position.y;
  z = (float)_pose.position.z;
  qx = (float)_pose.orientation.x;
  qy = (float)_pose.orientation.y;
  qz = (float)_pose.orientation.z;
  qw = (float)_pose.orientation.w;

  if(num_joints != 17)
  {
    RCLCPP_ERROR(rclcpp::get_logger("debug_fsw_sim"), "Error in number of joints received. Expecting 17!");
    return 0;
  }

  size_t data_size = num_joints * sizeof(float) + 7*sizeof(float) + sizeof(uint8_t) + 3*sizeof(float) + sizeof(int32_t) + sizeof(uint32_t); // joints + pose + nav_status + solar_panels + sec + nanosec

  *_buf = static_cast<uint8_t *> (malloc(data_size));
  if (*_buf)
  {
    std::vector<float> data(num_joints);
    for(size_t i = 0; i < num_joints; i++)
        data[i] = (float)_js->position[i];

    size_t offset = 0;
    for(size_t i = 0; i < num_joints; i++)
    {
      memcpy(*_buf + offset, &data[i], sizeof(float));
      offset += sizeof(float);
    }

    memcpy(*_buf + offset, &x, sizeof(float));
    offset += sizeof(float);

    memcpy(*_buf + offset, &y, sizeof(float));
    offset += sizeof(float);

    memcpy(*_buf + offset, &z, sizeof(float));
    offset += sizeof(float);

    memcpy(*_buf + offset, &qx, sizeof(float));
    offset += sizeof(float);

    memcpy(*_buf + offset, &qy, sizeof(float));
    offset += sizeof(float);

    memcpy(*_buf + offset, &qz, sizeof(float));
    offset += sizeof(float);

    memcpy(*_buf + offset, &qw, sizeof(float));
    offset += sizeof(float);

    // Navigation status
    memcpy(*_buf + offset, &_nav_status, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    // Solar panel data (left, right, rear)
    memcpy(*_buf + offset, &_solar_left, sizeof(float));
    offset += sizeof(float);

    memcpy(*_buf + offset, &_solar_right, sizeof(float));
    offset += sizeof(float);

    memcpy(*_buf + offset, &_solar_rear, sizeof(float));
    offset += sizeof(float);

    int32_t sec = _js->header.stamp.sec;
    uint32_t nanosec = _js->header.stamp.nanosec;

    memcpy(*_buf + offset, &sec, sizeof(int32_t));
    offset += sizeof(int32_t);
    memcpy(*_buf + offset, &nanosec, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    return data_size;

  } else
    return 0;
}

bool SerializeLunarExplorationManual::deserialize(const uint8_t* _buf, const size_t bufSize, size_t start_offset, uint8_t &_code, float &_val1, float &_val2)
{
  size_t offset = start_offset;

  memcpy(&_code, _buf + offset, sizeof(uint8_t));
  offset += sizeof(uint8_t);

  memcpy(&_val1, _buf + offset, sizeof(float));
  offset += sizeof(float);

  memcpy(&_val2, _buf + offset, sizeof(float));
  offset += sizeof(float);

 return true;
}

