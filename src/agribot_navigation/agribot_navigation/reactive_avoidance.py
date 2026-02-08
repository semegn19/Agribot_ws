#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ReactiveAvoid(Node):
    def __init__(self):
        super().__init__('reactive_avoid')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('ReactiveAvoid started: will stop/turn on nearby obstacles.')
        self.front_angle_width = 20  
        self.threshold = 0.6        

    def scan_cb(self, msg: LaserScan):
        # compute indices for +/- front_angle_width
        import math
        ang_inc = msg.angle_increment
        ang_min = msg.angle_min
        # convert degrees to radians
        half = math.radians(self.front_angle_width)
        center_idx = int((0.0 - ang_min) / ang_inc)
        low_idx = max(0, int(( -half - ang_min) / ang_inc))
        high_idx = min(len(msg.ranges)-1, int(( half - ang_min) / ang_inc))

        nearest = float('inf')
        for i in range(low_idx, high_idx+1):
            r = msg.ranges[i]
            if r == float('inf') or r == float('nan'):
                continue
            if r > 0.0 and r < nearest:
                nearest = r

        twist = Twist()
        if nearest < self.threshold:
            # obstacle too close, stop and turn away
            self.get_logger().warn(f'Obstacle at {nearest:.2f} m -> evasive turn')
            twist.linear.x = 0.0
            twist.angular.z = 0.6  # turn speed
        else:
            # otherwise, do nothing (let teleop or nav control)
            return

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

