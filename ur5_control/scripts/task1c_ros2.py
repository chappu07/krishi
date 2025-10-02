#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from moveit2 import MoveGroupInterface
import time

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('task1c_node')

    # MoveGroupInterface for ROS2 Humble MoveIt2
    group = MoveGroupInterface(node, "manipulator", "base_link")

    # Waypoints
    waypoints = []

    # P1
    p1 = Pose()
    p1.position.x = -0.214
    p1.position.y = -0.532
    p1.position.z = 0.557
    p1.orientation.x = 0.707
    p1.orientation.y = 0.028
    p1.orientation.z = 0.034
    p1.orientation.w = 0.707
    waypoints.append(p1)

    # P2
    p2 = Pose()
    p2.position.x = -0.159
    p2.position.y = 0.501
    p2.position.z = 0.415
    p2.orientation.x = 0.029
    p2.orientation.y = 0.997
    p2.orientation.z = 0.045
    p2.orientation.w = 0.033
    waypoints.append(p2)

    # P3
    p3 = Pose()
    p3.position.x = -0.806
    p3.position.y = 0.010
    p3.position.z = 0.182
    p3.orientation.x = -0.684
    p3.orientation.y = 0.726
    p3.orientation.z = 0.05
    p3.orientation.w = 0.008
    waypoints.append(p3)

    # Move through waypoints
    for i, pose in enumerate(waypoints):
        node.get_logger().info(f"Moving to P{i+1}")
        result = group.move_to_pose(pose, end_effector_link="ee_link", wait=True)
        if result:
            node.get_logger().info(f"Reached P{i+1}")
        else:
            node.get_logger().warn(f"Failed to reach P{i+1}")
        time.sleep(1)  # Wait 1 second at each waypoint

    node.get_logger().info("Task 1C Completed!")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
