/**
 * @file arm_comm_udp.h
 */
#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <dragoman_fsw_sim/serialize_mobile_servicing_system_manual.h>
#include <trac_ik/trac_ik.hpp>

class MobileServicingSystemCommUdp : public rclcpp::Node {

public:

  MobileServicingSystemCommUdp();
  bool initRobotComm();
  bool initUdpComm();
  bool initRest(const int &_tlm_ms, const int &_cmd_ms);

protected:

  void send_telemetry();
  void rcv_command();

  void js_cb(const sensor_msgs::msg::JointState::SharedPtr _msg);
  bool initDefaults();

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_canadarm_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_mbs_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_dextre_body_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_dextre_arm_1_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_dextre_arm_2_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_starboard_bga_;

  rclcpp::TimerBase::SharedPtr timer_tlm_;
  rclcpp::TimerBase::SharedPtr timer_cmd_;
  rclcpp::CallbackGroup::SharedPtr timer_tlm_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cmd_cb_group_;

  std::mutex mux_;

  SerializeMobileServicingSystemManual sm_;
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
  std::vector<std::string> canadarm_joints_, mbs_joints_;
  std::vector<std::string> dextre_joints_;
  std::vector<std::string> dextre_body_joints_, dextre_arm_1_joints_, dextre_arm_2_joints_;
  std::vector<std::string> starboard_bga_joints_;
  std::map<std::string, std::map<std::string, std::vector<double> > > group_states_;
  int duration_;

};
