#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from webots_ros2_driver.srv import SetRobotPose

class CollisionResetNode(Node):
    def __init__(self):
        super().__init__("collision_reset_node")
        
        # Collision detection threshold (in meters)
        self.collision_threshold = 0.5  

        # Subscribe to the laser scan topic from Webots
        self.subscription = self.create_subscription(
            LaserScan,
            "/Spot/scan",
            self.scan_callback,
            10
        )

        # Create a client to reset the robot position
        self.reset_client = self.create_client(SetRobotPose, "/Spot/set_pose")

        # Wait for the service to be available
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for reset service...")

        # Initial position for reset (x, y, z, roll, pitch, yaw)
        self.initial_pose = SetRobotPose.Request()
        self.initial_pose.x = 0.0
        self.initial_pose.y = 0.0
        self.initial_pose.z = 0.5
        self.initial_pose.roll = 0.0
        self.initial_pose.pitch = 0.0
        self.initial_pose.yaw = 0.0

    def scan_callback(self, msg):
        # Detect collision if any distance is below the threshold
        if any(distance < self.collision_threshold for distance in msg.ranges):
            self.get_logger().warn("Collision detected! Resetting Spot...")
            self.reset_robot()

    def reset_robot(self):
        future = self.reset_client.call_async(self.initial_pose)
        future.add_done_callback(self.reset_done)

    def reset_done(self, future):
        if future.result() is not None:
            self.get_logger().info("Spot successfully reset to the initial position.")
        else:
            self.get_logger().error("Failed to reset Spot.")

def main(args=None):
    rclpy.init(args=args)
    node = CollisionResetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
