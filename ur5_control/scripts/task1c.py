#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Pose
from moveit2 import MoveGroupInterface

class UR5WaypointMover(Node):
    def __init__(self):
        super().__init__('ur5_waypoint_mover')

        # Create MoveGroupInterface for 'manipulator'
        self.move_group = MoveGroupInterface(node=self, group_name="manipulator")

        # Define waypoints (w.r.t base_link)
        self.waypoints = []

        # P1
        p1 = Pose()
        p1.position.x = -0.214
        p1.position.y = -0.532
        p1.position.z = 0.557
        p1.orientation.x = 0.707
        p1.orientation.y = 0.028
        p1.orientation.z = 0.034
        p1.orientation.w = 0.707
        self.waypoints.append(p1)

        # P2
        p2 = Pose()
        p2.position.x = -0.159
        p2.position.y = 0.501
        p2.position.z = 0.415
        p2.orientation.x = 0.029
        p2.orientation.y = 0.997
        p2.orientation.z = 0.045
        p2.orientation.w = 0.033
        self.waypoints.append(p2)

        # P3
        p3 = Pose()
        p3.position.x = -0.806
        p3.position.y = 0.010
        p3.position.z = 0.182
        p3.orientation.x = -0.684
        p3.orientation.y = 0.726
        p3.orientation.z = 0.05
        p3.orientation.w = 0.008
        self.waypoints.append(p3)

        self.get_logger().info("Starting waypoint execution...")
        self.move_through_waypoints()

    def move_through_waypoints(self):
        for idx, pose in enumerate(self.waypoints):
            self.get_logger().info(f"Moving to P{idx+1}...")
            result = self.move_group.move_to_pose(pose, wait=True)

            if result:
                self.get_logger().info(f"Reached P{idx+1}, waiting 1 second...")
                time.sleep(1)
            else:
                self.get_logger().error(f"Failed to reach P{idx+1}!")

def main(args=None):
    rclpy.init(args=args)
    node = UR5WaypointMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

