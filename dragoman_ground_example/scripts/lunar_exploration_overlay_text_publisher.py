#!/usr/bin/env python3
"""
Lunar Exploration Overlay Text Publisher

Subscribes to Lunar Exploration telemetry messages and publishes navigation status
as RViz overlay text
"""

import rclpy
from rclpy.node import Node
from dragoman_sample_msgs.msg import LunarExplorationTelemetryPacket
from rviz_2d_overlay_msgs.msg import OverlayText

class LunarExplorationOverlayTextPublisher(Node):
    """
    Overlay text publisher for Lunar Exploration demo.

    Subscribes to LunarExplorationTelemetryPacket messages and publishes
    navigation status as RViz overlay text.
    """

    def __init__(self):
        super().__init__('lunar_exploration_overlay_text_publisher')

        # Declare parameters
        self.declare_parameter('input_topic', '/yamcs/lunar_exploration_space')
        self.declare_parameter('output_topic', '/overlay_text/navigation_status')
        self.declare_parameter('width', 400)
        self.declare_parameter('height', 60)
        self.declare_parameter('left', 10)
        self.declare_parameter('top', 10)
        self.declare_parameter('text_size', 14)
        self.declare_parameter('font', 'Roboto Mono')

        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.left = self.get_parameter('left').value
        self.top = self.get_parameter('top').value
        self.text_size = self.get_parameter('text_size').value
        self.font = self.get_parameter('font').value

        # Create ROS2 publisher
        self.publisher = self.create_publisher(OverlayText, output_topic, 10)

        # Subscribe to LunarExploration telemetry topic
        self.subscription = self.create_subscription(
            LunarExplorationTelemetryPacket,
            input_topic,
            self.telemetry_callback,
            10
        )

        self.get_logger().debug('LunarExploration Overlay Text Publisher initialized')
        self.get_logger().debug(f'Subscribing to: {input_topic}')
        self.get_logger().debug(f'Publishing to: {output_topic}')

    def telemetry_callback(self, msg):
        """Convert navigation status to overlay text message and publish."""
        overlay_msg = OverlayText()

        # Set action to ADD
        overlay_msg.action = OverlayText.ADD

        # Set dimensions and position
        overlay_msg.width = int(self.width)
        overlay_msg.height = int(self.height)
        overlay_msg.horizontal_distance = int(self.left)
        overlay_msg.vertical_distance = int(self.top)
        overlay_msg.horizontal_alignment = OverlayText.LEFT
        overlay_msg.vertical_alignment = OverlayText.TOP

        # Set background color (transparent)
        overlay_msg.bg_color.r = 0.0
        overlay_msg.bg_color.g = 0.0
        overlay_msg.bg_color.b = 0.0
        overlay_msg.bg_color.a = 0.0

        # Set text properties
        overlay_msg.line_width = int(2)
        overlay_msg.text_size = float(self.text_size)
        overlay_msg.font = self.font

        # Get navigation status (it's already a string)
        status_label = msg.navigation_status

        # Set text color based on status
        if status_label == "IDLE":
            overlay_msg.fg_color.r = 0.7
            overlay_msg.fg_color.g = 0.7
            overlay_msg.fg_color.b = 0.7
            overlay_msg.fg_color.a = 0.7
        elif status_label == "IN_PROGRESS":
            overlay_msg.fg_color.r = 1.0
            overlay_msg.fg_color.g = 0.8
            overlay_msg.fg_color.b = 0.0
            overlay_msg.fg_color.a = 0.7
        elif status_label == "DONE":
            overlay_msg.fg_color.r = 0.0
            overlay_msg.fg_color.g = 1.0
            overlay_msg.fg_color.b = 0.0
            overlay_msg.fg_color.a = 0.7
        else:  # UNKNOWN
            overlay_msg.fg_color.r = 1.0
            overlay_msg.fg_color.g = 0.0
            overlay_msg.fg_color.b = 0.0
            overlay_msg.fg_color.a = 0.7

        # Set text with prepended label
        overlay_msg.text = f"Navigation: {status_label}"

        # Publish the overlay text message
        self.publisher.publish(overlay_msg)
        self.get_logger().debug(f'Published navigation status: {status_label}')


def main(args=None):
    rclpy.init(args=args)
    node = LunarExplorationOverlayTextPublisher()

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
