import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        # Initialize the state
        self.state = 'IDLE'
        
        # Publisher to announce state changes
        self.publisher_ = self.create_publisher(String, 'state_info', 10)
        
        # Subscriber to receive state trigger events
        self.subscription = self.create_subscription(String,
                                                     'state_trigger',
                                                     self.trigger_callback,
                                                     10)
        self.get_logger().info(f"Initialized with state: {self.state}")

    def trigger_callback(self, msg):
        trigger = msg.data.strip().lower()
        self.get_logger().info(f"Received trigger: {trigger}")

        # Simple state transitions based on the current state and trigger
        if self.state == 'IDLE':
            if trigger == 'start':
                self.state = 'ACTIVE'
        elif self.state == 'ACTIVE':
            if trigger == 'pause':
                self.state = 'PAUSED'
            elif trigger == 'stop':
                self.state = 'IDLE'
        elif self.state == 'PAUSED':
            if trigger == 'resume':
                self.state = 'ACTIVE'
            elif trigger == 'stop':
                self.state = 'IDLE'

        # Publish the updated state
        state_msg = String()
        state_msg.data = self.state
        self.publisher_.publish(state_msg)
        self.get_logger().info(f"Transitioned to state: {self.state}")

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
