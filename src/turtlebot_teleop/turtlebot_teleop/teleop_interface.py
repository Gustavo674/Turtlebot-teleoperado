import curses
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class TeleopInterface(Node):
    def __init__(self):
        super().__init__('teleop_interface')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.velocity = Twist()
        self.key_mapping = {
            'w': (1.0, 0.0),
            's': (-1.0, 0.0),
            'a': (0.0, 1.0),
            'd': (0.0, -1.0),
            'q': (0.0, 0.0)
        }
        self.stdscr = curses.initscr()
        curses.cbreak()
        self.stdscr.keypad(True)
        self.stdscr.nodelay(True)

    def publish_velocity(self):
        key = self.stdscr.getch()
        if key == curses.ERR:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
        else:
            key = chr(key)
            if key in self.key_mapping:
                vels = self.key_mapping[key]
                self.velocity.linear.x = vels[0]
                self.velocity.angular.z = vels[1]
                if key == 'q':
                    rclpy.shutdown()
        self.publisher.publish(self.velocity)
        self.stdscr.addstr(0, 0, f'Linear Velocity: {self.velocity.linear.x}, Angular Velocity: {self.velocity.angular.z}')

    def run(self):
        rclpy.spin(self)
        curses.endwin()

def main(args=None):
    rclpy.init(args=args)
    teleop_interface = TeleopInterface()
    teleop_interface.run()

if __name__ == '__main__':
    main()
