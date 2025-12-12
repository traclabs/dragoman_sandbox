
#include <dragoman_fsw_sim/arm_comm_udp.h>
#include <tf2_kdl/tf2_kdl.hpp>

using std::placeholders::_1;

ArmCommUdp::ArmCommUdp() :
Node("arm_comm_udp")
{
  this->declare_parameter("joint_state", std::string("/big_arm/joint_states"));
  this->declare_parameter("joint_command", std::string("/big_arm/joint_state_command"));
    
  this->declare_parameter("cfs_port", 8080);
  this->declare_parameter("robot_port", 8585);  
  this->declare_parameter("cfs_ip", std::string("127.0.0.1"));
  this->declare_parameter("robot_ip", std::string("127.0.0.1"));    

      
   base_link_ = "big_arm_link_1";
   tip_link_ = "big_arm_link_8";
   robot_description_ = "robot_description";
   eps_ = 1e-5;
   max_time_ = 0.005;   
   solve_type_ = TRAC_IK::Speed;
   
   cmd_freq_ = 30.0;
   cmd_rate_ = 1.0/cmd_freq_; // 30 Hz

}


bool ArmCommUdp::initRobotComm()
{
  std::string js_topic;
  std::string jc_topic;

  this->get_parameter("joint_state", js_topic);  
  this->get_parameter("joint_command", jc_topic);


  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(js_topic, 10, std::bind(&ArmCommUdp::js_cb, this, _1));
  pub_jc_ = this->create_publisher<sensor_msgs::msg::JointState>(jc_topic, 10);
 
  trac_ik_.reset( new TRAC_IK::TRAC_IK(shared_from_this(), base_link_, tip_link_, 
                   robot_description_, max_time_, eps_, solve_type_) );


  return true;
}


bool ArmCommUdp::initUdpComm()
{
  int cfs_port;
  int robot_port;
  std::string cfs_ip;
  std::string robot_ip;
    
  this->get_parameter("cfs_port", cfs_port);  
  this->get_parameter("robot_port", robot_port);
  this->get_parameter("cfs_ip", cfs_ip);  
  this->get_parameter("robot_ip", robot_ip);      

  RCLCPP_INFO(this->get_logger(), "** initUdpComm: cfs port: %d cfs ip: %s robot port: %d robot ip: %s", 
              cfs_port, cfs_ip.c_str(), robot_port, robot_ip.c_str());

   cfs_port_ = cfs_port;
   robot_port_ = robot_port;
   cfs_ip_ = cfs_ip;
   robot_ip_ = robot_ip;
   
   std::string error_msg;
   
   return sm_.initializeComm(robot_port_, cfs_port_, robot_ip_, cfs_ip_, error_msg);
}

bool ArmCommUdp::initRest(const int &_tlm_ms, const int &_cmd_ms)
{
   timer_tlm_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
   timer_cmd_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
      
   timer_tlm_ = this->create_wall_timer(
      std::chrono::milliseconds(_tlm_ms),
      std::bind(&ArmCommUdp::send_telemetry, this), timer_tlm_cb_group_);

   timer_cmd_ = this->create_wall_timer(
      std::chrono::milliseconds(_cmd_ms),
      std::bind(&ArmCommUdp::rcv_command, this), timer_cmd_cb_group_);

   return true;
}

void ArmCommUdp::send_telemetry()
{
   // Grab the latest data
   sensor_msgs::msg::JointState js;
   mux_.lock();
   js = joint_state_;
   mux_.unlock();   
   
   // Send it to cFS
   if(js.name.size() == 0)
     return;

   if(!sm_.sendMessage(&js))
     RCLCPP_ERROR(this->get_logger(), "Error sending message");
}

// Extended example- Translation: [0.442, 4.248, 5.209] - Rotation: in Quaternion [0.851, 0.055, 0.039, 0.521]
// Zero example - - Translation: [0.903, 0.606, 4.755] - Rotation: in Quaternion [0.694, 0.324, 0.272, 0.583]
// Extended 2: - Translation: [-2.622, 1.686, 3.227] - Rotation: in Quaternion [0.860, -0.097, 0.340, 0.369]
/**
 * @function rcv_command
 */
void ArmCommUdp::rcv_command()
{
  geometry_msgs::msg::Pose cmd;

  if(sm_.receiveMessage(cmd))
  {
    RCLCPP_INFO(this->get_logger(), "Received pose command: %f %f %f -- %f %f %f %f", cmd.position.x, cmd.position.y, cmd.position.z, cmd.orientation.x, cmd.orientation.y, cmd.orientation.z, cmd.orientation.w);
    
    // Do IK magic
    double jv = 5.0*M_PI/180.0;
    calculateMotion(cmd, jv);
  }  
}

/**
 * @function calculateMotion
 */
bool ArmCommUdp::calculateMotion(const geometry_msgs::msg::Pose &_pose, const double &_jv)
{
  RCLCPP_INFO(this->get_logger(), "Calculating motion");
  // Fill IK data for problem
  KDL::JntArray qs, qg;
  KDL::Frame Tg;
  KDL::Twist bounds;
  
  Tg = KDL::Frame::Identity();
  tf2::fromMsg(_pose, Tg);
  bounds = KDL::Twist::Zero();
  
  sensor_msgs::msg::JointState js;
  mux_.lock(); js = joint_state_; mux_.unlock();  
  qs.data.resize(js.position.size());
  for(size_t i = 0; i < js.position.size(); ++i)
    qs(i) = js.position[i];

  int res = trac_ik_->CartToJnt(qs, Tg, qg, bounds);
  if(res < 0)
  {
    RCLCPP_INFO(this->get_logger(), "IK result was: %d returning", res);
    return false;
  }  
  // If a solution is found, move the robot
  rclcpp::Rate r(cmd_freq_);

  Eigen::VectorXd qi, dq, dqi;
  dq = (qg.data - qs.data);

  RCLCPP_INFO(this->get_logger(), "Executing motion ");
  size_t counts = ( dq.norm()/_jv ) * cmd_freq_;
  dqi = dq/(double)counts;
    
  sensor_msgs::msg::JointState ji;
  
  ji = js;
  for(size_t i = 0; i < counts; ++i)
  {
     qi = qs.data + dqi*((double)i);
     for(size_t k = 0; k < ji.position.size(); ++k)
       ji.position[k] = qi(k);
        
     pub_jc_->publish(ji);
     r.sleep();
  }
  
  return true;
}


/**
 * @function js_cb
 */
void ArmCommUdp::js_cb(const sensor_msgs::msg::JointState::SharedPtr _msg)
{
  mux_.lock();
 joint_state_ = *_msg;
 mux_.unlock();
}

