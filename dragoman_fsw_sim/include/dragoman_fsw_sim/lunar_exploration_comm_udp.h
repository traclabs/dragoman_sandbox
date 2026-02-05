/**
 * @file arm_comm_udp.h
 */
#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <dragoman_fsw_sim/serialize_lunar_exploration_manual.h>
#include <trac_ik/trac_ik.hpp>

class LunarExplorationCommUdp : public rclcpp::Node {

public:

  LunarExplorationCommUdp();
  bool initRobotComm();
  bool initUdpComm();
  bool initRest(const int &_tlm_ms, const int &_cmd_ms);
  
protected:

  void send_telemetry();
  void rcv_command();
  
  void js_cb(const sensor_msgs::msg::JointState::SharedPtr _msg);
  bool initDefaults();
  bool getTransform(const std::string &_source, 
                    const std::string &_target, 
                    geometry_msgs::msg::Pose &_pose);
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_camera_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    
  rclcpp::TimerBase::SharedPtr timer_tlm_;
  rclcpp::TimerBase::SharedPtr timer_cmd_; 
  rclcpp::CallbackGroup::SharedPtr timer_tlm_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cmd_cb_group_;
    
  std::mutex mux_;
  
  SerializeLunarExplorationManual sm_;
  int cfs_port_;
  int robot_port_;
  std::string cfs_ip_;
  std::string robot_ip_;
    
  sensor_msgs::msg::JointState joint_state_;
  std::shared_ptr<TRAC_IK::TRAC_IK> trac_ik_;
  std::shared_ptr<rclcpp::Node> trac_ik_sub_node_;
  double cmd_freq_;
  double cmd_rate_;

  std::string base_link_;
  std::string tip_link_;
  std::string robot_description_;
  double max_time_; 
  double eps_;
  TRAC_IK::SolveType solve_type_;
  
  // Joints
  std::vector<std::string> camera_joints_;
  int duration_;

  // TF
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  
  geometry_msgs::msg::Twist twist_;

};
