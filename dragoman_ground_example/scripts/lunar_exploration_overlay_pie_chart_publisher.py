#!/usr/bin/env python3
"""
Lunar Exploration Overlay Pie Chart Publisher

Subscribes to Lunar Exploration telemetry messages and publishes battery power
as RViz overlay pie chart
"""

import rclpy
from rclpy.node import Node
from dragoman_sample_msgs.msg import LunarExplorationTelemetryPacket
from std_msgs.msg import Float32

class LunarExplorationOverlayPieChartPublisher(Node):
    """
    Overlay pie chart publisher for Lunar Exploration demo.

    Subscribes to LunarExplorationTelemetryPacket messages and publishes
    battery power as RViz overlay pie chart.
    """

    def __init__(self):
        super().__init__('lunar_exploration_overlay_pie_chart_publisher')

        # Declare parameters
        self.declare_parameter('input_topic', '/yamcs/lunar_exploration')
        self.declare_parameter('output_topic', '/overlay_pie_chart/battery_power')

        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Create ROS2 publisher
        self.publisher = self.create_publisher(Float32, output_topic, 10)

        # Subscribe to LunarExploration telemetry topic
        self.subscription = self.create_subscription(
            LunarExplorationTelemetryPacket,
            input_topic,
            self.telemetry_callback,
            10
        )

        self.get_logger().debug('LunarExploration Overlay Pie Chart Publisher initialized')
        self.get_logger().debug(f'Subscribing to: {input_topic}')
        self.get_logger().debug(f'Publishing to: {output_topic}')

    def telemetry_callback(self, msg):
        """Convert battery percentage to Float32 message and publish."""
        battery_msg = Float32()
        battery_msg.data = float(msg.battery.percentage)

        # Publish the Float32 message
        self.publisher.publish(battery_msg)
        self.get_logger().debug(f'Published battery percentage: {battery_msg.data:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    node = LunarExplorationOverlayPieChartPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
