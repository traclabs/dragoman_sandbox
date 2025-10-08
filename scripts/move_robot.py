#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from std_msgs.msg import String, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty

class MoveRobot(Node):

    def __init__(self):
        super().__init__('arm_node')
        self.arm_publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.rail_publisher_ = self.create_publisher(JointTrajectory, '/rail_position_trajectory_controller/joint_trajectory', 10)
        self.lift_publisher_ = self.create_publisher(JointTrajectory, '/lift_position_trajectory_controller/joint_trajectory', 10)
                        
        self.up_arm_srv = self.create_service(Empty, 'up_arm', self.up_arm_callback)
        self.stow_arm_srv = self.create_service(Empty, 'stow_arm', self.stow_arm_callback)
        self.test_arm_srv = self.create_service(Empty, 'test_arm', self.test_arm_callback)
        self.rail_end_srv = self.create_service(Empty, 'rail_end', self.rail_end_callback)
        self.rail_zero_srv = self.create_service(Empty, 'rail_zero', self.rail_zero_callback)
        self.rail_mid_srv = self.create_service(Empty, 'rail_mid', self.rail_mid_callback)
        self.lift_high_srv = self.create_service(Empty, 'lift_high', self.lift_high_callback)
        self.lift_mid_srv = self.create_service(Empty, 'lift_mid', self.lift_mid_callback)
        self.lift_zero_srv = self.create_service(Empty, 'lift_zero', self.lift_zero_callback)                

    def up_arm_callback(self, request, response):
        traj = JointTrajectory()        
        traj.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [1.57, 0.0, 0.0, -1.57, 0.0, 0.0]
        point1.time_from_start = Duration(sec=5)

        traj.points.append(point1)
        self.arm_publisher_.publish(traj)


        return response

    def stow_arm_callback(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [-2.65, -2.11, 2.44, 0.0, 1.08, 3.26]
        point1.time_from_start = Duration(sec=5)

        traj.points.append(point1)
        self.arm_publisher_.publish(traj)

        return response

    def test_arm_callback(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [1.62, -1.54, 1.4, -1.2, -1.6, -0.11]
        point1.time_from_start = Duration(sec=5)

        traj.points.append(point1)
        self.arm_publisher_.publish(traj)

        return response

    def rail_end_callback(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ["vention_rail_base_to_carriage"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [1.5]
        point1.time_from_start = Duration(sec=10)

        traj.points.append(point1)
        self.rail_publisher_.publish(traj)

        return response

    def rail_zero_callback(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ["vention_rail_base_to_carriage"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [0.05]
        point1.time_from_start = Duration(sec=10)

        traj.points.append(point1)
        self.rail_publisher_.publish(traj)

        return response

    def rail_mid_callback(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ["vention_rail_base_to_carriage"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [0.75]
        point1.time_from_start = Duration(sec=10)

        traj.points.append(point1)
        self.rail_publisher_.publish(traj)

        return response

    def lift_high_callback(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ["ewellix_lift_lower_to_higher"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [0.695]
        point1.time_from_start = Duration(sec=10)

        traj.points.append(point1)
        self.lift_publisher_.publish(traj)

        return response

    def lift_mid_callback(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ["ewellix_lift_lower_to_higher"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [0.35]
        point1.time_from_start = Duration(sec=10)

        traj.points.append(point1)
        self.lift_publisher_.publish(traj)

        return response

    def lift_zero_callback(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ["ewellix_lift_lower_to_higher"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [0.02]
        point1.time_from_start = Duration(sec=10)

        traj.points.append(point1)
        self.lift_publisher_.publish(traj)

        return response



def main(args=None):
    rclpy.init(args=args)

    robot_node = MoveRobot()

    rclpy.spin(robot_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
