/**
 * @file serialize_arm_manual.cpp
 */
#include <dragoman_fsw_sim/serialize_arm_manual.h>
#include <vector>

SerializeArmManual::SerializeArmManual()
{}

bool SerializeArmManual::initializeComm( const int &_own_port,
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

bool SerializeArmManual::sendMessage( sensor_msgs::msg::JointState* _js )
{
    unsigned char* buf     = 0;
    size_t         bufSize = serialize(_js, &buf);

    if(bufSize == 0)
        return false;

    int res = sendto(sockfd_, buf, bufSize, 0, (const struct sockaddr *)&other_address_, sizeof(other_address_));

    // Clean up
    free(buf);

    return (res > 0);
}


bool SerializeArmManual::receiveMessage(geometry_msgs::msg::Pose &_ps)
{
     ssize_t buffer_rcvd_size;
     const int MAXLINE = 1024;
     uint8_t buffer_rcvd[MAXLINE];

     // Receive............
    buffer_rcvd_size = recvfrom(sockfd_, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
    if(buffer_rcvd_size > 0)
    {
      _ps = deserialize(buffer_rcvd, (size_t) buffer_rcvd_size, 0);
      return true;
    }

  return false;
}
/////////////////////////////

size_t SerializeArmManual::serialize(sensor_msgs::msg::JointState *_js, uint8_t** _buf)
{
   if (_js->name.size() == 0 || _js->position.size() == 0)
     return 0;

   size_t num_joints = _js->position.size();

   // Packet structure:
   // - CCSDS Primary Header: 6 bytes
   // - CFS Secondary Header: 10 bytes (Sec[6] + Spare[4])
   // - Payload: joint positions (floats) + timestamp
   size_t ccsds_header_size = 6;
   size_t cfs_secondary_header_size = 10;
   size_t payload_size = num_joints * sizeof(float) + sizeof(int32_t) + sizeof(uint32_t);
   size_t total_size = ccsds_header_size + cfs_secondary_header_size + payload_size;

   *_buf = static_cast<uint8_t *> (malloc(total_size));
   if (*_buf)
   {
      size_t offset = 0;

      // ========================================
      // CCSDS Primary Header (6 bytes)
      // ========================================

      // Header field values
      uint8_t  version                = 0;      // 3 bits: Packet Version Number
      uint8_t  type                   = 0;      // 1 bit:  0 = Telemetry, 1 = Command
      bool     secondary_header_flag  = true;   // 1 bit:  Has secondary header
      uint16_t apid                   = 39;     // 11 bits: Application Process ID (Gateway telemetry)
      uint8_t  sequence_flags         = 3;      // 2 bits:  3 = Unsegmented

      // Sequence counter (static to persist across calls)
      static uint16_t seq_count = 0;
      uint16_t sequence_count = seq_count;
      seq_count = (seq_count + 1) & 0x3FFF;     // 14-bit wrap

      // Packet length = (secondary header + payload) - 1
      uint16_t packet_length = (cfs_secondary_header_size + payload_size) - 1;

      // Bytes 0-1: Packet ID [version:3][type:1][sec_hdr:1][apid:11]
      uint16_t temp = (version << 13) |
                      (type << 12) |
                      (secondary_header_flag << 11) |
                      apid;
      (*_buf)[offset++] = (temp >> 8) & 0xFF;   // High byte
      (*_buf)[offset++] = temp & 0xFF;          // Low byte

      // Bytes 2-3: Packet Sequence [seq_flags:2][seq_count:14]
      temp = (sequence_flags << 14) | sequence_count;
      (*_buf)[offset++] = (temp >> 8) & 0xFF;   // High byte
      (*_buf)[offset++] = temp & 0xFF;          // Low byte

      // Bytes 4-5: Packet Length
      (*_buf)[offset++] = (packet_length >> 8) & 0xFF;  // High byte
      (*_buf)[offset++] = packet_length & 0xFF;         // Low byte

      // ========================================
      // CFS Secondary Header (10 bytes)
      // ========================================
      // Sec[6] - Time or metadata (unused, set to 0)
      for(size_t i = 0; i < 6; i++)
        (*_buf)[offset++] = 0x00;

      // Spare[4] - Padding (unused, set to 0)
      for(size_t i = 0; i < 4; i++)
        (*_buf)[offset++] = 0x00;

      // Payload: Joint positions
      std::vector<float> data(num_joints);
      for(size_t i = 0; i < num_joints; i++)
          data[i] = (float)_js->position[i];

      for(size_t i = 0; i < num_joints; i++)
      {
        memcpy(*_buf + offset, &data[i], sizeof(float));
        offset += sizeof(float);
      }

      // Payload: Timestamp
      int32_t sec = _js->header.stamp.sec;
      uint32_t nanosec = _js->header.stamp.nanosec;

      memcpy(*_buf + offset, &sec, sizeof(int32_t));
      offset += sizeof(int32_t);
      memcpy(*_buf + offset, &nanosec, sizeof(uint32_t));
      offset += sizeof(uint32_t);
      return total_size;

   } else
     return 0;
}

geometry_msgs::msg::Pose SerializeArmManual::deserialize(const uint8_t* _buf, const size_t bufSize, size_t start_offset)
{
 geometry_msgs::msg::Pose pose;

  // Command packet structure from YAMCS:
  // Bytes 0-5:  CCSDS Primary Header (6 bytes)
  // Bytes 6-7:  CFS Command Secondary Header (2 bytes: fcn_code, checksum)
  // Bytes 8-35: Pose data (7 floats × 4 bytes = 28 bytes: x, y, z, qx, qy, qz, qw)
  //             Floats are encoded in little-endian format as specified in XTCE
  const size_t header_size = 8;
  size_t offset = header_size;

  // Verify we have enough data
  if(bufSize < header_size + 7 * sizeof(float)) {
    printf("[DESERIALIZE ERROR] Buffer too small: %zu bytes, need at least %zu\n",
           bufSize, header_size + 7 * sizeof(float));
    return pose;
  }

  float x, y, z, qx, qy, qz, qw;
  memcpy(&x, _buf + offset, sizeof(float)); offset += sizeof(float);
  memcpy(&y, _buf + offset, sizeof(float)); offset += sizeof(float);
  memcpy(&z, _buf + offset, sizeof(float)); offset += sizeof(float);
  memcpy(&qx, _buf + offset, sizeof(float)); offset += sizeof(float);
  memcpy(&qy, _buf + offset, sizeof(float)); offset += sizeof(float);
  memcpy(&qz, _buf + offset, sizeof(float)); offset += sizeof(float);
  memcpy(&qw, _buf + offset, sizeof(float)); offset += sizeof(float);

  pose.position.x = (double)x;
  pose.position.y = (double)y;
  pose.position.z = (double)z;
  pose.orientation.x = (double)qx;
  pose.orientation.y = (double)qy;
  pose.orientation.z = (double)qz;
  pose.orientation.w = (double)qw;

 return pose;
}
