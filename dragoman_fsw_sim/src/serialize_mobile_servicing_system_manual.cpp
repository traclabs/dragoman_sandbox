/**
 * @file serialize_mobile_servicing_system_manual.cpp
 */
#include <dragoman_fsw_sim/serialize_mobile_servicing_system_manual.h>
#include <vector>
#include <rclcpp/rclcpp.hpp>
SerializeMobileServicingSystemManual::SerializeMobileServicingSystemManual()
{}

bool SerializeMobileServicingSystemManual::initializeComm( const int &_own_port,
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
bool SerializeMobileServicingSystemManual::sendMessage( sensor_msgs::msg::JointState* _js, uint8_t motion_status[8] )
{
  unsigned char* buf     = 0;
  size_t         bufSize = serialize(_js, motion_status, &buf);

  int res = sendto(sockfd_, buf, bufSize, 0, (const struct sockaddr *)&other_address_, sizeof(other_address_));

  // Clean up
  free(buf);

  return (res > 0);
}


bool SerializeMobileServicingSystemManual::receiveMessage(std::string &_group, std::string &_state)
{
  ssize_t buffer_rcvd_size;
  const int MAXLINE = 1024;
  uint8_t buffer_rcvd[MAXLINE];

  // Receive............
  buffer_rcvd_size = recvfrom(sockfd_, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
  if(buffer_rcvd_size > 0)
  {
    if(!deserialize(buffer_rcvd, (size_t) buffer_rcvd_size, 0, _group, _state))
      return false;

    return true;
  }

  return false;
}
/////////////////////////////

/**
 * @function serialize
 * @brief Send data back
 */
size_t SerializeMobileServicingSystemManual::serialize(sensor_msgs::msg::JointState *_js, uint8_t motion_status[8], uint8_t** _buf)
{
  if (_js->name.size() == 0 || _js->position.size() == 0)
    return 0;

  size_t num_joints = _js->position.size();

  // boom clpa: 4
  // etvcg: 8
  // canadarm: 7,
  // dextre arm 1: 7, dextre arm 2: 7, dextre body: 1
  // mbs: 1
  // port_bga: 4, port_sraj: 1
  // starboard_bga: 4, starboard_sarj: 1
  // outrigger clpa: 4
  // total: 4 + 8 + 7 + 7 + 7 + 1 + 1 + 4 + 1 + 4 + 1 + 4 = 49
  if(num_joints != 49)
  {
    RCLCPP_ERROR(rclcpp::get_logger("debug_fsw_sim"), "Error in number of joints received. Expecting 47!");
    return 0;
  }

  size_t data_size = num_joints * sizeof(float) + 8 * sizeof(uint8_t) + sizeof(int32_t) + sizeof(uint32_t); // joints + motion_status + sec + nanosec

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

    // Add motion status (8 uint8_t values)
    for(size_t i = 0; i < 8; i++)
    {
      memcpy(*_buf + offset, &motion_status[i], sizeof(uint8_t));
      offset += sizeof(uint8_t);
    }

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

bool SerializeMobileServicingSystemManual::deserialize(const uint8_t* _buf, const size_t bufSize, size_t start_offset, std::string &_group, std::string &_state)
{
  const int char_length = 60;
  char group_state[char_length];

  size_t offset = start_offset;

  memcpy(&group_state, _buf + offset, char_length*sizeof(char)); offset += char_length * sizeof(char);

  int end_group; int end_state;
  char group_array[30]; char state_array[30];

  if(!getString(group_state, char_length, 0, group_array, end_group))
    return false;

  if(!getString(group_state, char_length, 30, state_array, end_state))
    return false;

  _group = std::string(group_array);
  _state = std::string(state_array);

 return true;
}

bool SerializeMobileServicingSystemManual::getString(char _input_string[], int _input_size, int _start_index, char _output_string[], int &_end_index)
{
   // End of group
   _end_index = -1;
   for(int i = _start_index; i < _input_size; ++i)
   {
      if(_input_string[i] == '\0')
      {
         _end_index = i;
         break;
       }
   }

   if(_end_index == -1)
     return false;

   // Fill string
   int index = 0;
   for(int i = _start_index; i <= _end_index; ++i)
   {
      _output_string[index] = _input_string[i];
      index++;
   }

   return true;
}



