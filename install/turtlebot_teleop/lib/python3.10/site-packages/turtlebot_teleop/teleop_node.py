import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.service = self.create_service(Empty, 'stop_robot', self.stop_robot_callback)
        self.cmd_vel = Twist()
        self.get_logger().info("Teleop Node Initialized")

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.get_logger().info(f'Received command: {msg.linear.x}, {msg.angular.z}')

    def stop_robot_callback(self, request, response):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.publisher.publish(self.cmd_vel)
        self.get_logger().info("Robot stopped")
        return response

    def publish_velocity(self, linear, angular):
        self.cmd_vel.linear.x = linear
        self.cmd_vel.angular.z = angular
        self.publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
