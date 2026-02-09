
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
   dextre_body_joints_ = {"joint_dextre_body"};
   dextre_arm_1_joints_ = {"joint_dextre_arm_1_shoulder_roll", "joint_dextre_arm_1_shoulder_yaw", "joint_dextre_arm_1_shoulder_pitch", "joint_dextre_arm_1_elbow_pitch", "joint_dextre_arm_1_wrist_pitch_yaw", "joint_dextre_arm_1_wrist_roll"};
   dextre_arm_2_joints_ = {"joint_dextre_arm_2_shoulder_roll", "joint_dextre_arm_2_shoulder_yaw", "joint_dextre_arm_2_shoulder_pitch", "joint_dextre_arm_2_elbow_pitch", "joint_dextre_arm_2_wrist_pitch_yaw", "joint_dextre_arm_2_wrist_roll"};
   starboard_bga_joints_ = {"joint_starboard_bga_1", "joint_starboard_bga_2", "joint_starboard_bga_3", "joint_starboard_bga_4"};
   duration_ = 5;

   // Fill map
   group_states_["canadarm"]["default"] = {0.0, 0.628, -0.187, 0.0, 0.0, 0.0, 0.0};
   group_states_["canadarm"]["forward"] = {0.11, 1.0, -1.2, 0.0, 0.94, 0.0, -0.42};

   group_states_["mbs"]["default"] = {0.0};
   group_states_["mbs"]["starboard"] = {-11.0};
   group_states_["mbs"]["port"] = {10.0};

   // Battery approach sequence
   group_states_["starboard_bga"]["battery_approach_1"] = {1.51, 0.0, 0.0, 0.0};
   group_states_["mbs"]["battery_approach_2"] = {-11.9};
   group_states_["dextre_arm_2"]["battery_approach_3"] = {0.18523718752476404, -0.5241987958083291, -1.682263170735807, -1.9513613753267156, 1.0311423725630433, 2.3745585005169585};
   group_states_["canadarm"]["battery_approach_4"] = {-0.7979664, 0.8997542399999996, -0.7465880981813328, -0.26361884120485235, 0.051124460930129345, -0.2960438164053687, 0.38809889416538945};
   group_states_["canadarm"]["battery_approach_5"] = {-0.605002074884394, 0.21107525936356275, -0.7465880981813328, -0.26361884120485235, 0.051124460930129345, -0.2960438164053687, 0.38809889416538945};
   group_states_["dextre_arm_2"]["battery_approach_6"] = {0.1852359229366735, -0.5242074747970021, -1.1729191164615993, -1.6061683715275838, 0.17661993791396646, 2.3745498761609554};

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
  pub_dextre_body_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/dextre_body_joint_trajectory_controller/joint_trajectory", 10);
  pub_dextre_arm_1_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/dextre_arm_1_joint_trajectory_controller/joint_trajectory", 10);
  pub_dextre_arm_2_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/dextre_arm_2_joint_trajectory_controller/joint_trajectory", 10);
  pub_starboard_bga_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/starboard_bga_joint_trajectory_controller/joint_trajectory", 10);

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

    auto traj = trajectory_msgs::msg::JointTrajectory();

    // Select the correct joint list and publisher based on the group
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub;
    if(group == "canadarm")
    {
      traj.joint_names = canadarm_joints_;
      pub = pub_canadarm_;
    }
    else if (group == "mbs")
    {
      traj.joint_names = mbs_joints_;
      pub = pub_mbs_;
    }
    else if (group == "dextre_body")
    {
      traj.joint_names = dextre_body_joints_;
      pub = pub_dextre_body_;
    }
    else if (group == "dextre_arm_1")
    {
      traj.joint_names = dextre_arm_1_joints_;
      pub = pub_dextre_arm_1_;
    }
    else if (group == "dextre_arm_2")
    {
      traj.joint_names = dextre_arm_2_joints_;
      pub = pub_dextre_arm_2_;
    }
    else if (group == "starboard_bga")
    {
      traj.joint_names = starboard_bga_joints_;
      pub = pub_starboard_bga_;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown group %s", group.c_str());
      return;
    }

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
    pub->publish(traj);

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
