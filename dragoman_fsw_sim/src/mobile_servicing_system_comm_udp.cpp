
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

   // Initialize motion status to IDLE for all subsystems
   motion_status_.mbs = MotionStatus::IDLE;
   motion_status_.canadarm2 = MotionStatus::IDLE;
   motion_status_.dextre_body = MotionStatus::IDLE;
   motion_status_.dextre_arm_1 = MotionStatus::IDLE;
   motion_status_.dextre_arm_2 = MotionStatus::IDLE;
   motion_status_.sarj = MotionStatus::IDLE;
   motion_status_.port_bga = MotionStatus::IDLE;
   motion_status_.starboard_bga = MotionStatus::IDLE;

}

/**
 * @function initDefaults
 */
bool MobileServicingSystemCommUdp::initDefaults()
{
   canadarm_joints_ = {"joint_canadarm2_1", "joint_canadarm2_2", "joint_canadarm2_3", "joint_canadarm2_4", "joint_canadarm2_5", "joint_canadarm2_6", "joint_canadarm2_7"};
   mbs_joints_ = {"joint_mbs"};
   dextre_body_joints_ = {"joint_dextre_body"};
   dextre_arm_1_joints_ = {"joint_dextre_arm_1_shoulder_roll", "joint_dextre_arm_1_shoulder_yaw", "joint_dextre_arm_1_shoulder_pitch", "joint_dextre_arm_1_elbow_pitch", "joint_dextre_arm_1_wrist_pitch", "joint_dextre_arm_1_wrist_yaw", "joint_dextre_arm_1_wrist_roll"};
   dextre_arm_2_joints_ = {"joint_dextre_arm_2_shoulder_roll", "joint_dextre_arm_2_shoulder_yaw", "joint_dextre_arm_2_shoulder_pitch", "joint_dextre_arm_2_elbow_pitch", "joint_dextre_arm_2_wrist_pitch", "joint_dextre_arm_2_wrist_yaw", "joint_dextre_arm_2_wrist_roll"};
   sarj_joints_ = {"joint_starboard_sarj", "joint_port_sarj"};
   port_bga_joints_ = {"joint_port_bga_1", "joint_port_bga_2", "joint_port_bga_3", "joint_port_bga_4"};
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
   group_states_["dextre_arm_2"]["battery_approach_3"] = {0.18523718752476404, -0.5241987958083291, -1.682263170735807, -1.9513613753267156, 1.0311423725630433, 0.0, 2.3745585005169585};
   group_states_["canadarm"]["battery_approach_4"] = {-0.7979664, 0.8997542399999996, -0.7465880981813328, -0.26361884120485235, 0.051124460930129345, -0.2960438164053687, 0.38809889416538945};
   group_states_["canadarm"]["battery_approach_5"] = {-0.605002074884394, 0.21107525936356275, -0.7465880981813328, -0.26361884120485235, 0.051124460930129345, -0.2960438164053687, 0.38809889416538945};
   group_states_["dextre_arm_2"]["battery_approach_6"] = {0.1852359229366735, -0.5242074747970021, -1.1729191164615993, -1.6061683715275838, 0.17661993791396646, 0.0, 2.3745498761609554};

   // Canadarm2 poses used in bt.xml
   group_states_["canadarm"]["go_to_rack_1"] = {-0.28, 1.0, -1.36, -0.214, 1.551, 0.168, -0.3924};
   group_states_["canadarm"]["go_to_rack_2"] = {-0.57, 0.86, -1.18, -0.28, 1.66, 0.38, -1.52};
   group_states_["canadarm"]["go_to_rack_3"] = {-0.22, 0.24, -1.22, -0.30, 1.56, 0.12, -0.91};
   group_states_["canadarm"]["go_to_pallet_1"] = {-0.197, 0.213, -1.177, -0.724, 1.922, 0.117, -0.886};
   group_states_["canadarm"]["go_to_pallet_2"] = {-0.692, -0.024, -1.883, 0.202, 2.223, 0.098, -0.651};
   group_states_["canadarm"]["go_to_pallet_3"] = {-0.121, -0.807, -2.555, 0.174, 2.439, 0.099, -1.552};
   group_states_["canadarm"]["go_to_pallet_4"] = {-0.836, -0.977, -2.304, 0.162, 2.665, -0.478, -1.813};
   group_states_["canadarm"]["go_to_pallet_5"] = {-1.137, -1.064, -2.306, 0.436, 2.608, -0.736, -1.911};

   // Dextre Arm 1 poses used in bt.xml (7 joints: shoulder_roll, shoulder_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_yaw, wrist_roll)
   group_states_["dextre_arm_1"]["zero"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   group_states_["dextre_arm_1"]["go_to_rack_1"] = {-0.675, -0.092, -0.589, 1.656, 2.752, -0.420, -0.109};
   group_states_["dextre_arm_1"]["go_to_rack_2"] = {-0.317, 0.001, 0.578, 1.363, 1.771, -0.182, 0.187};
   group_states_["dextre_arm_1"]["place_on_pallet_1"] = {-0.456, -0.0187, 1.150, 0.551, 1.151, -1.798, -0.715};
   group_states_["dextre_arm_1"]["place_on_pallet_2"] = {0.0696, 0.103, 1.047, 0.711, 1.213, -1.260, -0.806};
   group_states_["dextre_arm_1"]["leave_on_pallet_1"] = {-0.309, 0.042, 0.898, 0.918, 1.261, -1.644, -0.938};
   group_states_["dextre_arm_1"]["go_to_spare_1"] = {-0.321, 0.101, 1.197, 1.541, 0.977, -1.671, -1.579};
   group_states_["dextre_arm_1"]["go_to_spare_2"] = {0.1103, -0.141, 1.227, 1.409, 0.988, -1.177, -1.499};
   group_states_["dextre_arm_1"]["carry_spare_1"] = {-0.321, 0.101, 1.197, 1.541, 0.977, -1.671, -1.579};
   group_states_["dextre_arm_1"]["carry_spare_2"] = {-0.036, 0.171, 1.154, 1.670, 0.904, -0.0215, -1.357};

   // Initialize done timestamps to zero
   done_timestamps_.mbs = rclcpp::Time(0);
   done_timestamps_.canadarm2 = rclcpp::Time(0);
   done_timestamps_.dextre_body = rclcpp::Time(0);
   done_timestamps_.dextre_arm_1 = rclcpp::Time(0);
   done_timestamps_.dextre_arm_2 = rclcpp::Time(0);
   done_timestamps_.sarj = rclcpp::Time(0);
   done_timestamps_.port_bga = rclcpp::Time(0);
   done_timestamps_.starboard_bga = rclcpp::Time(0);

   return true;
}


bool MobileServicingSystemCommUdp::initRobotComm()
{
  std::string js_topic;

  this->get_parameter("joint_state", js_topic);

  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(js_topic, 10, std::bind(&MobileServicingSystemCommUdp::js_cb, this, _1));

  // Create action clients for each controller
  action_client_canadarm_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/canadarm2_joint_trajectory_controller/follow_joint_trajectory");
  action_client_mbs_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/mobile_base_system_joint_trajectory_controller/follow_joint_trajectory");
  action_client_dextre_body_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/dextre_body_joint_trajectory_controller/follow_joint_trajectory");
  action_client_dextre_arm_1_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/dextre_arm_1_joint_trajectory_controller/follow_joint_trajectory");
  action_client_dextre_arm_2_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/dextre_arm_2_joint_trajectory_controller/follow_joint_trajectory");
  action_client_sarj_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/sarj_joint_trajectory_controller/follow_joint_trajectory");
  action_client_port_bga_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/port_bga_joint_trajectory_controller/follow_joint_trajectory");
  action_client_starboard_bga_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/starboard_bga_joint_trajectory_controller/follow_joint_trajectory");

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

   // Lambda to check and reset DONE status to IDLE after 5 seconds
   auto check_and_reset_done_status = [this](MotionStatus& status, rclcpp::Time& timestamp) {
     if (status == MotionStatus::DONE && timestamp.nanoseconds() > 0) {
       auto current_time = this->get_clock()->now();
       double elapsed_time = (current_time - timestamp).seconds();
       if (elapsed_time >= 5.0) {
         status = MotionStatus::IDLE;
         timestamp = rclcpp::Time(0);
       }
     }
   };

   // Check if any subsystem has been in DONE state for 5 seconds, then switch to IDLE
   check_and_reset_done_status(motion_status_.mbs, done_timestamps_.mbs);
   check_and_reset_done_status(motion_status_.canadarm2, done_timestamps_.canadarm2);
   check_and_reset_done_status(motion_status_.dextre_body, done_timestamps_.dextre_body);
   check_and_reset_done_status(motion_status_.dextre_arm_1, done_timestamps_.dextre_arm_1);
   check_and_reset_done_status(motion_status_.dextre_arm_2, done_timestamps_.dextre_arm_2);
   check_and_reset_done_status(motion_status_.sarj, done_timestamps_.sarj);
   check_and_reset_done_status(motion_status_.port_bga, done_timestamps_.port_bga);
   check_and_reset_done_status(motion_status_.starboard_bga, done_timestamps_.starboard_bga);

   // Convert MotionStatus struct to uint8_t array for transmission
   // Order: mbs, canadarm2, dextre_body, dextre_arm_1, dextre_arm_2, sarj, port_bga, starboard_bga
   uint8_t status_bytes[8] = {
     static_cast<uint8_t>(motion_status_.mbs),
     static_cast<uint8_t>(motion_status_.canadarm2),
     static_cast<uint8_t>(motion_status_.dextre_body),
     static_cast<uint8_t>(motion_status_.dextre_arm_1),
     static_cast<uint8_t>(motion_status_.dextre_arm_2),
     static_cast<uint8_t>(motion_status_.sarj),
     static_cast<uint8_t>(motion_status_.port_bga),
     static_cast<uint8_t>(motion_status_.starboard_bga)
   };

   if(!sm_.sendMessage(&js, status_bytes))
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

    // Check if this is a battery command
    if(state == "grasp" || state == "release")
    {
      handle_battery_command(group, state);
      return;
    }

    // Map group name to action client and motion status pointer
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client;
    std::vector<std::string> joint_names;
    MotionStatus* status_ptr = nullptr;
    rclcpp::Time* timestamp_ptr = nullptr;

    if(group == "canadarm")
    {
      action_client = action_client_canadarm_;
      joint_names = canadarm_joints_;
      status_ptr = &motion_status_.canadarm2;
      timestamp_ptr = &done_timestamps_.canadarm2;
    }
    else if (group == "mbs")
    {
      action_client = action_client_mbs_;
      joint_names = mbs_joints_;
      status_ptr = &motion_status_.mbs;
      timestamp_ptr = &done_timestamps_.mbs;
    }
    else if (group == "dextre_body")
    {
      action_client = action_client_dextre_body_;
      joint_names = dextre_body_joints_;
      status_ptr = &motion_status_.dextre_body;
      timestamp_ptr = &done_timestamps_.dextre_body;
    }
    else if (group == "dextre_arm_1")
    {
      action_client = action_client_dextre_arm_1_;
      joint_names = dextre_arm_1_joints_;
      status_ptr = &motion_status_.dextre_arm_1;
      timestamp_ptr = &done_timestamps_.dextre_arm_1;
    }
    else if (group == "dextre_arm_2")
    {
      action_client = action_client_dextre_arm_2_;
      joint_names = dextre_arm_2_joints_;
      status_ptr = &motion_status_.dextre_arm_2;
      timestamp_ptr = &done_timestamps_.dextre_arm_2;
    }
    else if (group == "sarj")
    {
      action_client = action_client_sarj_;
      joint_names = sarj_joints_;
      status_ptr = &motion_status_.sarj;
      timestamp_ptr = &done_timestamps_.sarj;
    }
    else if (group == "port_bga")
    {
      action_client = action_client_port_bga_;
      joint_names = port_bga_joints_;
      status_ptr = &motion_status_.port_bga;
      timestamp_ptr = &done_timestamps_.port_bga;
    }
    else if (group == "starboard_bga")
    {
      action_client = action_client_starboard_bga_;
      joint_names = starboard_bga_joints_;
      status_ptr = &motion_status_.starboard_bga;
      timestamp_ptr = &done_timestamps_.starboard_bga;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown group %s", group.c_str());
      return;
    }

    // Validate group and state exist
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

    // Check if action server is ready
    if (!action_client->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server for %s not available", group.c_str());
      return;
    }

    // Create goal message
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names;

    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions = group_states_[group][state];
    point.time_from_start = rclcpp::Duration(duration_, 0);
    goal_msg.trajectory.points.push_back(point);

    // Send goal with async callbacks to avoid blocking
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    // Goal response callback - called when server accepts/rejects goal
    send_goal_options.goal_response_callback =
      [this, status_ptr](std::shared_ptr<GoalHandleFJT> goal_handle)
      {
        this->goal_response_callback(status_ptr, goal_handle);
      };

    // Result callback - called when action completes
    send_goal_options.result_callback =
      [this, status_ptr, timestamp_ptr](const GoalHandleFJT::WrappedResult & result)
      {
        this->result_callback(status_ptr, timestamp_ptr, result);
      };

    // Send goal asynchronously
    action_client->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(this->get_logger(), "Goal sent for group %s", group.c_str());
  }
}

/**
 * @function goal_response_callback
 */
void MobileServicingSystemCommUdp::goal_response_callback(MotionStatus* status_ptr, std::shared_ptr<GoalHandleFJT> goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    *status_ptr = MotionStatus::IDLE;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    *status_ptr = MotionStatus::IN_PROGRESS;
  }
}

/**
 * @function result_callback
 */
void MobileServicingSystemCommUdp::result_callback(MotionStatus* status_ptr, rclcpp::Time* timestamp_ptr, const GoalHandleFJT::WrappedResult & result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      *status_ptr = MotionStatus::DONE;
      *timestamp_ptr = this->get_clock()->now();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      *status_ptr = MotionStatus::IDLE;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Goal was canceled");
      *status_ptr = MotionStatus::IDLE;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      *status_ptr = MotionStatus::IDLE;
      break;
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

/**
 * @function handle_battery_command
 * @brief Handle battery grasp/release commands by calling gz service
 */
void MobileServicingSystemCommUdp::handle_battery_command(const std::string& battery_name, const std::string& command)
{
  RCLCPP_INFO(this->get_logger(), "Handling battery command: %s for battery: %s", command.c_str(), battery_name.c_str());

  // Determine the attach/detach command
  std::string gz_command;
  if(command == "grasp")
  {
    gz_command = "attach";
  }
  else if(command == "release")
  {
    gz_command = "detach";
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Unknown battery command: %s", command.c_str());
    return;
  }

  // Build gz service command similar to the example provided
  // Simplified version - you may need to adjust child_link_name based on your robot setup
  std::string child_link = "base_link";  // Default link name, may need to be adjusted
  std::string cmd_line = "gz service -s /payload/attach_detach"
                        " --reqtype gz.custom_msgs.AttachDetachRequest"
                        " --reptype gz.custom_msgs.AttachDetachResponse"
                        " --timeout 3000"
                        " --req 'child_model_name: \"" + battery_name + "\","
                        " child_link_name: \"" + child_link + "\","
                        " command: \"" + gz_command + "\"'";

  RCLCPP_INFO(this->get_logger(), "Executing: %s", cmd_line.c_str());

  int return_code = std::system(cmd_line.c_str());

  if(return_code == 0)
  {
    RCLCPP_INFO(this->get_logger(), "Battery command executed successfully");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Battery command failed with return code: %d", return_code);
  }
}
