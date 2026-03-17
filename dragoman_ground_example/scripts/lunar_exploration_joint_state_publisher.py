#!/usr/bin/env python3
"""
Lunar Exploration Joint State Publisher

Subscribes to Lunar Exploration telemetry messages and converts them to ROS2 JointState messages
for the Lunar Exploration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dragoman_sample_msgs.msg import LunarExplorationTelemetryPacket

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class LunarExplorationJointStatePublisher(Node):
    """
    Joint state publisher for Lunar Exploration demo.

    Subscribes to LunarExplorationTelemetryPacket messages and converts them to ROS2 JointState messages + TF
    """

    # Joint names in the order they appear in the telemetry packet
    # These names must match the joint names in the URDF
    # 17 JOINTS + xyz/qxyzw
    JOINT_NAMES = [
    "front_left_suspension_joint",
    "front_left_wheel_axle_joint",
    "front_left_wheel_joint",
    "front_right_suspension_joint",
    "front_right_wheel_axle_joint",
    "front_right_wheel_joint",
    "left_solar_panel_joint",
    "mast_camera_joint",
    "mast_head_pivot_joint",
    "rear_left_suspension_joint",
    "rear_left_wheel_axle_joint",
    "rear_left_wheel_joint",
    "rear_right_suspension_joint",
    "rear_right_wheel_axle_joint",
    "rear_right_wheel_joint",
    "rear_solar_panel_joint",
    "right_solar_panel_joint"
    ]

    def __init__(self):
        super().__init__('lunar_exploration_joint_state_publisher')

        # Declare parameters
        self.declare_parameter('input_topic', '/yamcs/lunar_exploration_space')

        # Get parameters
        input_topic = self.get_parameter('input_topic').value

        # Create ROS2 publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribe to LunarExploration telemetry topic
        self.subscription = self.create_subscription(
            LunarExplorationTelemetryPacket,
            input_topic,
            self.telemetry_callback,
            10
        )

        # Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().debug(f'LunarExploration Joint State Publisher initialized')
        self.get_logger().debug(f'Subscribing to: {input_topic}')
        self.get_logger().debug(f'Publishing to: /joint_states')

    def telemetry_callback(self, msg):
        """Convert LunarExplorationTelemetryPacket to JointState message and publish."""
        joint_state_msg = JointState()
        # Set timestamp
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = ''

        # Set joint positions and names from telemetry packet
        joint_state_msg.position = list(msg.joint_state)
        joint_state_msg.name = self.JOINT_NAMES


        # Publish the joint state message
        self.publisher.publish(joint_state_msg)
        self.get_logger().debug(f'Published {len(joint_state_msg.name)} joint states')

        # Publish transform between odom and base_footprint
        tfx = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        tfx.header.stamp = self.get_clock().now().to_msg()
        tfx.header.frame_id = 'odom'
        tfx.child_frame_id = 'base_footprint'

        tfx.transform.translation.x = float(msg.pose.position.x)
        tfx.transform.translation.y = float(msg.pose.position.y)
        tfx.transform.translation.z = float(msg.pose.position.z)

        tfx.transform.rotation.x = float(msg.pose.orientation.x)
        tfx.transform.rotation.y = float(msg.pose.orientation.y)
        tfx.transform.rotation.z = float(msg.pose.orientation.z)
        tfx.transform.rotation.w = float(msg.pose.orientation.w)

        # Send the transformation
        self.tf_broadcaster.sendTransform(tfx)


def main(args=None):
    rclpy.init(args=args)
    node = LunarExplorationJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
