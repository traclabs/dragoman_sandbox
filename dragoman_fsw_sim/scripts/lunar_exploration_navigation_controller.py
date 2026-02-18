#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class LunarNavigationController(Node):
    """
    Simple turn-go-turn navigation controller that:
    1. Turns to face the goal
    2. Drives straight to the goal
    3. Turns to match the goal orientation

    Assumes PoseStamped is x,y,theta in a 3D pose message format.
    """

    def __init__(self):
        super().__init__('lunar_navigation_controller')

        # Parameters
        self.declare_parameter('angular_velocity', 0.5)  # rad/s
        self.declare_parameter('linear_velocity', 0.6)   # m/s
        self.declare_parameter('position_tolerance', 0.20)  # meters
        self.declare_parameter('angle_tolerance', 0.05)  # radians (~3 degrees)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')

        self.angular_vel = self.get_parameter('angular_velocity').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.ang_tol = self.get_parameter('angle_tolerance').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value

        # TF2 listener for current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        # State variables
        self.goal_pose = None
        self.state = 'IDLE'  # States: IDLE, TURN_TO_GOAL, DRIVE_TO_GOAL, TURN_TO_HEADING

        self.get_logger().info('Lunar Navigation Controller initialized')

    def goal_callback(self, msg):
        """Receive new goal pose"""
        self.goal_pose = msg
        self.state = 'TURN_TO_GOAL'
        self.get_logger().info(f'New goal received: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')

    def get_current_pose(self):
        """Get current robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            return transform
        except TransformException as ex:
            self.get_logger().warn(f'Could not get transform: {ex}')
            return None

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        # Assuming q is geometry_msgs/Quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        """Main control loop running at fixed rate"""
        if self.state == 'IDLE' or self.goal_pose is None:
            return

        # Get current pose
        current_transform = self.get_current_pose()
        if current_transform is None:
            return

        # Extract current position and orientation
        current_x = current_transform.transform.translation.x
        current_y = current_transform.transform.translation.y
        current_yaw = self.quaternion_to_yaw(current_transform.transform.rotation)

        # Extract goal position and orientation
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        goal_yaw = self.quaternion_to_yaw(self.goal_pose.pose.orientation)

        # Calculate differences
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)

        # Create twist message
        cmd = Twist()

        # State machine for turn-go-turn behavior
        if self.state == 'TURN_TO_GOAL':
            # Turn to face the goal
            angle_diff = self.normalize_angle(angle_to_goal - current_yaw)

            if abs(angle_diff) < self.ang_tol:
                # Done turning, start driving
                self.state = 'DRIVE_TO_GOAL'
                self.get_logger().info('Aligned to goal, starting drive')
            else:
                # Continue turning
                cmd.angular.z = self.angular_vel if angle_diff > 0 else -self.angular_vel

        elif self.state == 'DRIVE_TO_GOAL':
            # Drive straight to goal
            if distance < self.pos_tol:
                # Reached goal position, now turn to final heading
                self.state = 'TURN_TO_HEADING'
                self.get_logger().info('Reached goal position, adjusting heading')
            else:
                # Keep driving forward
                cmd.linear.x = self.linear_vel

                # Minor heading correction while driving
                angle_diff = self.normalize_angle(angle_to_goal - current_yaw)
                if abs(angle_diff) > self.ang_tol:
                    # If we've drifted too much, slow down and correct
                    cmd.angular.z = 0.3 * self.angular_vel * (1.0 if angle_diff > 0 else -1.0)

        elif self.state == 'TURN_TO_HEADING':
            # Turn to match goal orientation
            angle_diff = self.normalize_angle(goal_yaw - current_yaw)

            if abs(angle_diff) < self.ang_tol:
                # Goal reached!
                self.state = 'IDLE'
                self.get_logger().info('Goal reached!')
                self.goal_pose = None
            else:
                # Continue turning to final heading
                cmd.angular.z = self.angular_vel if angle_diff > 0 else -self.angular_vel

        # Publish velocity command
        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    controller = LunarNavigationController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
