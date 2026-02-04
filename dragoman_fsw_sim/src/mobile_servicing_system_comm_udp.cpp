
#include <dragoman_fsw_sim/mobile_servicing_system_comm_udp.h>
#include <tf2_kdl/tf2_kdl.hpp>

using std::placeholders::_1;

MobileServicingSystemCommUdp::MobileServicingSystemCommUdp() :
Node("arm_comm_udp")
{
  this->declare_parameter("joint_state", std::string("/joint_states"));

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

/**
 * @function initDefaults
 */
bool MobileServicingSystemCommUdp::initDefaults()
{
   canadarm_joints_ = {"joint_canadarm2_1", "joint_canadarm2_2", "joint_canadarm2_3", "joint_canadarm2_4", "joint_canadarm2_5", "joint_canadarm2_6", "joint_canadarm2_7"};
   mbs_joints_ = {"joint_mbs"};
   
   duration_ = 10;
   
   // Fill map
   group_states_["canadarm"]["default"] = {0.0, 0.628, -0.187, 0.0, 0.0, 0.0, 0.0};
   group_states_["canadarm"]["forward"] = {0.11, 1.0, -1.2, 0.0, 0.94, 0.0, -0.42};   
   
   group_states_["mbs"]["default"] = {0.0};
   group_states_["mbs"]["starboard"] = {-11.0};
   group_states_["mbs"]["port"] = {10.0};      
   return true;
}


bool MobileServicingSystemCommUdp::initRobotComm()
{
  std::string js_topic;
  std::string jc_topic;

  this->get_parameter("joint_state", js_topic);

  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(js_topic, 10, std::bind(&MobileServicingSystemCommUdp::js_cb, this, _1));

  pub_canadarm_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/canadarm2_joint_trajectory_controller/joint_trajectory", 10);
  pub_mbs_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/mobile_base_system_joint_trajectory_controller/joint_trajectory", 10);

  initDefaults();
  
  return true;
}


bool MobileServicingSystemCommUdp::initUdpComm()
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

bool MobileServicingSystemCommUdp::initRest(const int &_tlm_ms, const int &_cmd_ms)
{
   timer_tlm_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
   timer_cmd_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

   timer_tlm_ = this->create_wall_timer(
      std::chrono::milliseconds(_tlm_ms),
      std::bind(&MobileServicingSystemCommUdp::send_telemetry, this), timer_tlm_cb_group_);

   timer_cmd_ = this->create_wall_timer(
      std::chrono::milliseconds(_cmd_ms),
      std::bind(&MobileServicingSystemCommUdp::rcv_command, this), timer_cmd_cb_group_);

   return true;
}

void MobileServicingSystemCommUdp::send_telemetry()
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


/**
 * @function rcv_command
 */
void MobileServicingSystemCommUdp::rcv_command()
{
  std::string group, state;

  if(sm_.receiveMessage(group, state))
  {
    RCLCPP_INFO(this->get_logger(), "Received command group: %s and state: %s", group.c_str(), state.c_str());

    // Send service call
    if(group == "canadarm")
    {    
      auto traj = trajectory_msgs::msg::JointTrajectory();
      traj.joint_names = canadarm_joints_;
        
      auto point1 = trajectory_msgs::msg::JointTrajectoryPoint();
      
      if(group_states_.find(group) == group_states_.end())
      {
        RCLCPP_ERROR(this->get_logger(), "Group %s not stored", group.c_str());
        return;
      }
      if(group_states_[group].find(state) == group_states_[group].end())
      {
        RCLCPP_ERROR(this->get_logger(), "State %s for group %s not stored", state.c_str(), group.c_str());
        return;
      }
      point1.positions = group_states_[group][state];
      point1.time_from_start = rclcpp::Duration(duration_, 0);

      traj.points.push_back(point1);
      pub_canadarm_->publish(traj);
      
    } else if(group == "mbs")
    {
      auto traj = trajectory_msgs::msg::JointTrajectory();
      traj.joint_names = mbs_joints_;
        
      auto point1 = trajectory_msgs::msg::JointTrajectoryPoint();
      
      if(group_states_.find(group) == group_states_.end())
      {
        RCLCPP_ERROR(this->get_logger(), "Group %s not stored", group.c_str());
        return;
      }
      if(group_states_[group].find(state) == group_states_[group].end())
      {
        RCLCPP_ERROR(this->get_logger(), "State %s for group %s not stored", state.c_str(), group.c_str());
        return;
      }
      point1.positions = group_states_[group][state];
      point1.time_from_start = rclcpp::Duration(duration_, 0);

      traj.points.push_back(point1);
      pub_mbs_->publish(traj);
      
    
    }
    
    
    // Do IK magic
    //double jv = 5.0*M_PI/180.0;
    //calculateMotion(cmd, jv);
  }
}


/**
 * @function js_cb
 */
void MobileServicingSystemCommUdp::js_cb(const sensor_msgs::msg::JointState::SharedPtr _msg)
{
  mux_.lock();
 joint_state_ = *_msg;
 mux_.unlock();
}

