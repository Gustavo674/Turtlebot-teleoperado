# teleop_interface_gui.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tkinter as tk
from tkinter import ttk
from PIL import Image as PILImage, ImageTk
import time

class TeleopInterfaceGUI(Node):
    def __init__(self):
        super().__init__('teleop_interface_gui')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_subscriber = self.create_subscription(Image, 'video_frames', self.image_callback, 10)
        self.br = CvBridge()
        self.window = tk.Tk()
        self.window.title("TurtleBot Teleop Interface")
        self.image_label = ttk.Label(self.window)
        self.image_label.grid(row=0, column=0, columnspan=4)
        self.latency_label = ttk.Label(self.window, text="Latency: Calculating...")
        self.latency_label.grid(row=1, column=0, columnspan=4)

        # Botões de comando
        self.create_button("Forward", self.move_forward, 2, 1)
        self.create_button("Left", self.turn_left, 3, 0)
        self.create_button("Stop", self.stop, 3, 1)
        self.create_button("Right", self.turn_right, 3, 2)
        self.create_button("Backward", self.move_backward, 4, 1)

        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.window.after(0, self.update_ros)

    def create_button(self, text, command, row, column):
        button = ttk.Button(self.window, text=text, command=command)
        button.grid(row=row, column=column, padx=10, pady=10)

    def move_forward(self):
        self.publish_velocity(0.2, 0.0)

    def move_backward(self):
        self.publish_velocity(-0.2, 0.0)

    def turn_left(self):
        self.publish_velocity(0.0, 0.5)

    def turn_right(self):
        self.publish_velocity(0.0, -0.5)

    def stop(self):
        self.publish_velocity(0.0, 0.0)

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher_.publish(twist)

    def image_callback(self, data):
        current_time = self.get_clock().now().to_msg()
        frame_time = data.header.stamp

        # Calculate latency
        current_sec = current_time.sec
        current_nanosec = current_time.nanosec
        frame_sec = frame_time.sec
        frame_nanosec = frame_time.nanosec
        latency = (current_sec - frame_sec) + (current_nanosec - frame_nanosec) / 1e9

        self.latency_label.config(text=f"Latency: {latency:.6f} seconds")

        # Convert the ROS Image message to OpenCV format
        frame = self.br.imgmsg_to_cv2(data)

        # Convert the OpenCV image to PIL format
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_pil = PILImage.fromarray(frame)
        imgtk = ImageTk.PhotoImage(image=img_pil)

        # Update the image label with the new image
        self.image_label.imgtk = imgtk
        self.image_label.configure(image=imgtk)

    def update_ros(self):
        rclpy.spin_once(self)
        self.window.after(100, self.update_ros)

    def on_closing(self):
        self.stop()
        self.destroy_node()
        rclpy.shutdown()
        self.window.destroy()

def main(args=None):
    rclpy.init(args=args)
    teleop_interface_gui = TeleopInterfaceGUI()
    teleop_interface_gui.window.mainloop()

if __name__ == '__main__':
    main()
