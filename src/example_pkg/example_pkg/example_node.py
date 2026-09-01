import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        
        # Create a subscriber to the 'string_topic'
        self.subscription = self.create_subscription(
            String,
            'string_topic',
            self.listener_callback,
            10
        )

        # Create a publisher to the 'output_string_topic'
        self.publisher = self.create_publisher(
            String,
            'output_string_topic',
            10
        )

        # Initialize the last received message with a default value
        self.last_received_string = "No message received yet!"

        # Create a timer that triggers every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info('Node has started and is listening for strings on "string_topic" and publishing to "output_string_topic".')

    def listener_callback(self, msg):
        # Update the last received string whenever a new message is received
        self.last_received_string = msg.data

    def timer_callback(self):
        # Publish the last received string every 2 seconds
        msg = String()
        msg.data = self.last_received_string
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
