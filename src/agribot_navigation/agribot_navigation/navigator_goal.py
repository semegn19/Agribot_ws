#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

def main():
    # Initialize ROS 2
    rclpy.init()
    
    # Create the Nav2 Simple Commander
    nav = BasicNavigator()

    # 1. Set a destination
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 7.0  
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.w = 1.0

    # 2. Start Navigation
    print('Sending AgriBot to goal...')
    nav.goToPose(goal_pose)

    # 3. Wait until it arrives
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f'Distance remaining: {feedback.distance_remaining:.2f} m')

    # 4. Check result
    result = nav.getResult()
    print(f'Navigation Result: {result}')

    # Cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()
