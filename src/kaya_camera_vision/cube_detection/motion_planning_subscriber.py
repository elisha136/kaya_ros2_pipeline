#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from kaya_msgs.msg import CubeInfo
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import PointStamped, do_transform_point

class MotionPlanningSubscriber(Node):
    """ROS 2 node that transforms cube position, computes distance, and publishes both.

    Subscribes to /cube/position (or user-defined topic), transforms the position
    to the robot base frame (using tf2), calculates distance to the origin,
    and republishes both position and distance.

    Additionally, publishes CubeInfo messages for state machine transitions.

    Attributes:
        tf_buffer (Buffer): Buffer for storing transforms.
        tf_listener (TransformListener): Listener to fill the buffer.
    """

    def __init__(self):
        """Initializes the MotionPlanningSubscriber node."""
        super().__init__('motion_planning_subscriber')

       # Parameters
        self.declare_parameter('input_topic', '/cube/positions')
        self.declare_parameter('source_frame', 'camera_link')
        self.declare_parameter('target_frame', 'robot_base_frame')

        self.input_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        # TF2 setup
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber to PoseArray of detected cube(s)
        self.subscription = self.create_subscription(
            PoseArray,
            self.input_topic,
            self.poses_callback,
            10
        )

        # Publisher for CubeInfo
        self.publisher_cube_info = self.create_publisher(
            CubeInfo,
            '/cube_info',
            10
        )

        self.get_logger().info(
            f"MotionPlanningSubscriber listening on '{self.input_topic}' "
            f"(camera frame='{self.source_frame}'), will publish CubeInfo->'/cube_info'."
        )

    def poses_callback(self, msg: PoseArray):
        # No detections: skip
        if not msg.poses:
            return

        # Extract the first pose's position
        stamped = PointStamped()
        stamped.header = msg.header
        stamped.header.frame_id = self.source_frame
        stamped.point = msg.poses[0].position

        try:
            # Transform to target frame
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )
            transformed = do_transform_point(stamped, t).point
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Build and publish CubeInfo
        info = CubeInfo()
        info.header.stamp    = self.get_clock().now().to_msg()
        info.header.frame_id = self.target_frame
        info.point           = transformed
        info.detected        = True

        self.publisher_cube_info.publish(info)
        self.get_logger().info(
            f"Published CubeInfo: x={transformed.x:.3f}, y={transformed.y:.3f}, z={transformed.z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
