#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class SimpleDriver(Node):
    def __init__(self):
        super().__init__('simple_driver')
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.1, self.drive)

    def drive(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 1.0
        msg.drive.steering_angle = 0.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

