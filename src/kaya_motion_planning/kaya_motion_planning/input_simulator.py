#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from kaya_msgs.msg import CubeInfo


class InputSimulator(Node):
    """
    Simulates the robot's current position and subscribes to CubeInfo
    to publish detected cube positions as planning goals.
    """

    def __init__(self):
        super().__init__('input_simulator')

        # Publisher for robot's simulated odometry
        self.robot_position_publisher = self.create_publisher(PoseStamped, 'robot_position', 10)

        # Publisher for goal positions received from vision
        self.goal_position_publisher = self.create_publisher(PoseStamped, 'goal_position', 10)

        # Subscribe to cube_info topic
        self.create_subscription(
            CubeInfo,
            '/cube_info',
            self.cube_info_callback,
            10
        )

        # Timer to periodically publish robot pose
        self.create_timer(1.0, self.publish_robot_pose)

        self.get_logger().info("InputSimulator initialized: publishing robot_position and listening to /cube_info")

    def publish_robot_pose(self):
        # Create a PoseStamped representing the robot at the origin
        robot_pose = PoseStamped()
        robot_pose.header.stamp = self.get_clock().now().to_msg()
        robot_pose.header.frame_id = 'robot_base_frame'  # Or 'map' depending on your setup

        # Default position is (0, 0, 0), orientation defaults to identity
        self.robot_position_publisher.publish(robot_pose)
        self.get_logger().debug(f"Published robot_position: {robot_pose.pose.position}")

    def cube_info_callback(self, msg: CubeInfo):
        # Convert CubeInfo.point into PoseStamped for goal
        goal_pose = PoseStamped()
        goal_pose.header.stamp = msg.header.stamp
        goal_pose.header.frame_id = msg.header.frame_id  # Typically 'robot_base_frame' or 'map'
        goal_pose.pose.position = msg.point

        self.goal_position_publisher.publish(goal_pose)
        self.get_logger().info(
            f"Received CubeInfo → goal_position: x={msg.point.x:.3f}, "
            f"y={msg.point.y:.3f}, z={msg.point.z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = InputSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
