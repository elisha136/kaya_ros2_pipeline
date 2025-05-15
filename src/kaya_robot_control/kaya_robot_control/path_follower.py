#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

class PathFollower(Node):
    """Converts a Path into a sequence of Twist commands."""
    def __init__(self):
        super().__init__('path_follower')
        self.sub = self.create_subscription(Path, 'planned_path', self.on_path, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("PathFollower ready, listening to /planned_path")

        def on_path(self, path: Path):
            if not path.poses:
                return
            target = path.poses[0].pose.position
    
            cmd = Twist()             # ← instantiate!
            K   = 0.5  # Proportional gain
            cmd.linear.x  = K * target.x
            cmd.linear.y  = K * target.y
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            self.get_logger().info(f"PathFollower → cmd_vel: x={cmd.linear.x:.2f}, y={cmd.linear.y:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
