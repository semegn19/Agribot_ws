#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AgriGuard(Node):
    def __init__(self):
        super().__init__('agri_guard')
        self.teleop_sub = self.create_subscription(Twist, '/cmd_vel_input', self.teleop_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.real_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_nearby = False
        self.get_logger().info('AgriGuard started: mediating teleop commands.')

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        center = len(ranges)//2
        spread = 40
        min_dist = 10.0
        for i in range(center-spread, center+spread):
            if i >= 0 and i < len(ranges):
                val = ranges[i]
                if val is not None and val > 0.0 and val < min_dist:
                    min_dist = val
        self.obstacle_nearby = (min_dist < 0.6)

    def teleop_callback(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.angular.z = msg.angular.z
        if self.obstacle_nearby and msg.linear.x > 0:
            self.get_logger().warn('DANGER! Blocking forward command to save the crops!')
            out.linear.x = 0.0
            out.angular.z = 0.5
        self.real_cmd_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = AgriGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

