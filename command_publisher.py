# api/command_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher_node')
        self.publisher_ = self.create_publisher(String, '/target_waypoints', 10)

    def publish_command(self, command_list):
        msg = String()
        msg.data = ','.join(command_list)
        self.publisher_.publish(msg)
        self.get_logger().info(f"📤 퍼블리시됨: {msg.data}")