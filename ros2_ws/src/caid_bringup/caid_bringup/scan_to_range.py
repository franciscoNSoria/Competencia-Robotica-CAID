import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


class ScanToRange(Node):

    def __init__(self):
        super().__init__('scan_to_range')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('range_topic', 'range')
        self.declare_parameter('fov', 0.2618)
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 4.0)
        scan_topic = self.get_parameter('scan_topic').value
        range_topic = self.get_parameter('range_topic').value
        self.fov = self.get_parameter('fov').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.pub = self.create_publisher(Range, range_topic, 10)
        self.create_subscription(
            LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan):
        valid = [r for r in scan.ranges
                 if math.isfinite(r) and self.min_range <= r <= self.max_range]
        msg = Range()
        msg.header = scan.header
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = min(valid) if valid else math.inf
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanToRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
