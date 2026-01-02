#!/usr/bin/env python3
"""
Curiosity Joint State Publisher

Subscribes to Curiosity telemetry messages and converts them to ROS2 JointState messages
for the Curiosity rover.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dragoman_sample_msgs.msg import CuriosityTelemetryPacket


class CuriosityJointStatePublisher(Node):
    """
    Joint state publisher for Curiosity rover.

    Subscribes to CuriosityTelemetryPacket messages and converts them to ROS2 JointState messages.
    """

    # Joint names in the order they appear in the telemetry packet
    # These names must match the joint names in the URDF
    JOINT_NAMES = [
        "arm_01_joint", "arm_02_joint", "arm_03_joint", "arm_04_joint", "arm_tools_joint",
        "back_wheel_L_joint", "back_wheel_R_joint", "front_wheel_L_joint", "front_wheel_R_joint",
        "mast_02_joint", "mast_cameras_joint", "mast_p_joint",
        "middle_wheel_L_joint", "middle_wheel_R_joint",
        "suspension_arm_B2_L_joint", "suspension_arm_B2_R_joint",
        "suspension_arm_B_L_joint", "suspension_arm_B_R_joint",
        "suspension_arm_F_L_joint", "suspension_arm_F_R_joint",
        "suspension_steer_B_L_joint", "suspension_steer_B_R_joint",
        "suspension_steer_F_L_joint", "suspension_steer_F_R_joint"
    ]

    def __init__(self):
        super().__init__('curiosity_joint_state_publisher')

        # Declare parameters
        self.declare_parameter('input_topic', '/yamcs/curiosity')

        # Get parameters
        input_topic = self.get_parameter('input_topic').value

        # Create ROS2 publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribe to Curiosity telemetry topic
        self.subscription = self.create_subscription(
            CuriosityTelemetryPacket,
            input_topic,
            self.telemetry_callback,
            10
        )

        self.get_logger().info(f'Curiosity Joint State Publisher initialized')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to: /joint_states')

    def telemetry_callback(self, msg):
        """Convert CuriosityTelemetryPacket to JointState message and publish."""
        joint_state_msg = JointState()

        # Set timestamp
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = ''

        # Set joint positions and names from telemetry packet
        joint_state_msg.position = list(msg.joint_state)
        joint_state_msg.name = self.JOINT_NAMES

        # Publish the message
        self.publisher.publish(joint_state_msg)
        self.get_logger().debug(f'Published {len(joint_state_msg.name)} joint states')


def main(args=None):
    rclpy.init(args=args)
    node = CuriosityJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
