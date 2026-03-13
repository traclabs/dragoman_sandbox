
#include <dragoman_fsw_sim/lunar_exploration_comm_udp.h>
#include <tf2_kdl/tf2_kdl.hpp>

using std::placeholders::_1;

LunarExplorationCommUdp::LunarExplorationCommUdp() :
Node("lunar_exploration_comm_udp")
{
  this->declare_parameter("joint_state", std::string("/joint_states"));

  this->declare_parameter("cfs_port", 8080);
  this->declare_parameter("robot_port", 8585);
  this->declare_parameter("cfs_ip", std::string("127.0.0.1"));
  this->declare_parameter("robot_ip", std::string("127.0.0.1"));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  continuous_twist_mode_ = false;
  nav_status_ = NAV_STATUS_IDLE;  // Initialize to IDLE

  // Initialize solar panel data
  solar_left_ = 0.0f;
  solar_right_ = 0.0f;
  solar_rear_ = 0.0f;

}

/**
 * @function initDefaults
 */
bool LunarExplorationCommUdp::initDefaults()
{
   camera_joints_ = {"mast_head_pivot_joint", "mast_camera_joint"};
   duration_ = 5.0;

   return true;
}


bool LunarExplorationCommUdp::initRobotComm()
{
  std::string js_topic;
  std::string jc_topic;

  this->get_parameter("joint_state", js_topic);

  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(js_topic, 10, std::bind(&LunarExplorationCommUdp::js_cb, this, _1));

  sub_nav_status_ = this->create_subscription<std_msgs::msg::UInt8>("/nav_status", 10, std::bind(&LunarExplorationCommUdp::nav_status_cb, this, _1));

  // Subscribe to solar panel topics
  sub_solar_left_ = this->create_subscription<std_msgs::msg::Float32>("/model/lunar_pole_exploration_rover/left_solar_panel/solar_panel_output", 10, std::bind(&LunarExplorationCommUdp::solar_left_cb, this, _1));
  sub_solar_right_ = this->create_subscription<std_msgs::msg::Float32>("/model/lunar_pole_exploration_rover/right_solar_panel/solar_panel_output", 10, std::bind(&LunarExplorationCommUdp::solar_right_cb, this, _1));
  sub_solar_rear_ = this->create_subscription<std_msgs::msg::Float32>("/model/lunar_pole_exploration_rover/rear_solar_panel/solar_panel_output", 10, std::bind(&LunarExplorationCommUdp::solar_rear_cb, this, _1));

  pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  pub_camera_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/mast_camera_joint_trajectory_controller/joint_trajectory", 10);
  pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

  initDefaults();

  return true;
}


bool LunarExplorationCommUdp::initUdpComm()
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

bool LunarExplorationCommUdp::initRest(const int &_tlm_ms, const int &_cmd_ms)
{
   timer_tlm_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
   timer_cmd_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

   timer_tlm_ = this->create_wall_timer(
      std::chrono::milliseconds(_tlm_ms),
      std::bind(&LunarExplorationCommUdp::send_telemetry, this), timer_tlm_cb_group_);

   timer_cmd_ = this->create_wall_timer(
      std::chrono::milliseconds(_cmd_ms),
      std::bind(&LunarExplorationCommUdp::rcv_command, this), timer_cmd_cb_group_);

   return true;
}

void LunarExplorationCommUdp::send_telemetry()
{
   // Grab the latest data
   sensor_msgs::msg::JointState js;
   mux_.lock();
   js = joint_state_;
   mux_.unlock();

   //
   geometry_msgs::msg::Pose pose;
   pose.orientation.w = 1.0;
   if(!getTransform("odom", "base_footprint", pose))
     RCLCPP_WARN(this->get_logger(), "Not getting robot pose w.r.t. odom! Returning identity");

   // Send it to cFS
   if(js.name.size() == 0)
     return;

   if(!sm_.sendMessage(&js, pose, nav_status_, solar_left_, solar_right_, solar_rear_))
     RCLCPP_ERROR(this->get_logger(), "Error sending message");
}


/**
 * @function rcv_command
 */
void LunarExplorationCommUdp::rcv_command()
{
  float linear_vel, angular_vel;
  float pan, tilt;
  float x, y, theta;

  // Peek at the command code to determine message type without consuming it
  // Code 1 (twist): linear velocity, angular velocity
  // Code 2 (camera): pan, tilt
  // Code 3 (navigation pose): x, y, theta
  uint8_t code = 0;
  if(sm_.peekCommandCode(code))
  {
    // Based on command code, receive the appropriate data format
    switch(code)
    {
      case 1:
        // Twist command
        if(sm_.receiveTwistCommand(linear_vel, angular_vel))
        {
          RCLCPP_INFO(this->get_logger(), "** Twist command: linear=%f, angular=%f", linear_vel, angular_vel);
          // Publish motion
          twist_ = geometry_msgs::msg::Twist();
          twist_.linear.x = linear_vel;
          twist_.angular.z = angular_vel;
          continuous_twist_mode_ = true;  // Enable continuous twist publishing
          // Set navigation status based on whether robot is moving
          if (linear_vel == 0.0 && angular_vel == 0.0)
            nav_status_ = NAV_STATUS_IDLE;
          else
            nav_status_ = NAV_STATUS_IN_PROGRESS;
        }
        break;

      case 2:
        // Camera command
        if(sm_.receiveCameraCommand(pan, tilt))
        {
          RCLCPP_INFO(this->get_logger(), "** Camera command: pan=%f, tilt=%f", pan, tilt);
          auto traj = trajectory_msgs::msg::JointTrajectory();
          traj.joint_names = camera_joints_;

          auto point1 = trajectory_msgs::msg::JointTrajectoryPoint();

          point1.positions = {pan, tilt};
          point1.time_from_start = rclcpp::Duration(duration_, 0);

          traj.points.push_back(point1);
          pub_camera_->publish(traj);
        }
        break;

      case 3:
        // Navigation pose command
        if(sm_.receiveNavigationPoseCommand(x, y, theta))
        {
          RCLCPP_INFO(this->get_logger(), "** Navigation pose command: x=%f, y=%f, theta=%f", x, y, theta);

          // Create PoseStamped message for navigation controller
          auto goal_pose = geometry_msgs::msg::PoseStamped();
          goal_pose.header.stamp = this->now();
          goal_pose.header.frame_id = "odom";

          // Set position
          goal_pose.pose.position.x = x;
          goal_pose.pose.position.y = y;
          goal_pose.pose.position.z = 0.0;

          // Convert theta (yaw) to quaternion
          // Using simple 2D rotation: qw = cos(theta/2), qz = sin(theta/2)
          double half_theta = theta / 2.0;
          goal_pose.pose.orientation.x = 0.0;
          goal_pose.pose.orientation.y = 0.0;
          goal_pose.pose.orientation.z = sin(half_theta);
          goal_pose.pose.orientation.w = cos(half_theta);

          // Publish to navigation controller
          pub_goal_pose_->publish(goal_pose);
          // Stop continuous twist mode - let navigation controller handle cmd_vel
          continuous_twist_mode_ = false;
          // Set navigation status to IN_PROGRESS when new goal is received
          nav_status_ = NAV_STATUS_IN_PROGRESS;
        }
        break;

      default:
        RCLCPP_WARN(this->get_logger(), "** Received unknown command code: %d", code);
        break;
    }
  }

  // Publish twist if we're in twist mode
  if (continuous_twist_mode_)
  {
    pub_cmd_vel_->publish(twist_);
  }
}


/**
 * @function js_cb
 */
void LunarExplorationCommUdp::js_cb(const sensor_msgs::msg::JointState::SharedPtr _msg)
{
  mux_.lock();
 joint_state_ = *_msg;
 mux_.unlock();
}

/**
 * @function nav_status_cb
 */
void LunarExplorationCommUdp::nav_status_cb(const std_msgs::msg::UInt8::SharedPtr _msg)
{
  nav_status_ = _msg->data;
  RCLCPP_DEBUG(this->get_logger(), "Nav status updated to: %d", nav_status_);
}

/**
 * @function solar_left_cb
 */
void LunarExplorationCommUdp::solar_left_cb(const std_msgs::msg::Float32::SharedPtr _msg)
{
  solar_left_ = _msg->data;
  RCLCPP_DEBUG(this->get_logger(), "Left solar panel: %.2f W", solar_left_);
}

/**
 * @function solar_right_cb
 */
void LunarExplorationCommUdp::solar_right_cb(const std_msgs::msg::Float32::SharedPtr _msg)
{
  solar_right_ = _msg->data;
  RCLCPP_DEBUG(this->get_logger(), "Right solar panel: %.2f W", solar_right_);
}

/**
 * @function solar_rear_cb
 */
void LunarExplorationCommUdp::solar_rear_cb(const std_msgs::msg::Float32::SharedPtr _msg)
{
  solar_rear_ = _msg->data;
  RCLCPP_DEBUG(this->get_logger(), "Rear solar panel: %.2f W", solar_rear_);
}

/**
 * @function getTransform
 */
bool  LunarExplorationCommUdp::getTransform(const std::string &_source,
                                            const std::string &_target,
                                            geometry_msgs::msg::Pose &_pose)
{
    geometry_msgs::msg::TransformStamped tfs;
    try
    {
      tfs = tf_buffer_->lookupTransform(_source, _target, rclcpp::Time(0),
                                        rclcpp::Duration(1, 0));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "No transform from " << _source << " to " << _target
                                                       << ".  Error: " << ex.what());
      return false;
    }

    _pose.position.x  = tfs.transform.translation.x;
    _pose.position.y  = tfs.transform.translation.y;
    _pose.position.z  = tfs.transform.translation.z;
    _pose.orientation = tfs.transform.rotation;

  return true;
}

