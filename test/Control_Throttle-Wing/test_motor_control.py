#!/usr/bin/env python3
import threading
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DroneGUI(Node):
    def __init__(self):
        super().__init__('drone_throttle_gui')
        self.pub_throttle = self.create_publisher(Float32, '/uav1/set_throttle', 10)

        # Tkinter Window Setup
        self.root = tk.Tk()
        self.root.title("UAV Wind Tunnel Controller")
        self.root.geometry("350x250")

        # Throttle Slider Label
        self.label = ttk.Label(self.root, text="Throttle: 0.05", font=("Helvetica", 14))
        self.label.pack(pady=10)

        # Throttle Slider (0.00 to 1.00)
        self.slider = ttk.Scale(
            self.root,
            from_=0.0,
            to=1.0,
            orient='horizontal',
            command=self.on_slider_move
        )
        self.slider.set(0.05)
        self.slider.pack(fill='x', padx=20, pady=10)

        # Emergency Cutoff Button
        self.cut_btn = tk.Button(
            self.root,
            text="EMERGENCY CUTOFF (0%)",
            bg="red",
            fg="white",
            font=("Helvetica", 12, "bold"),
            command=self.emergency_cutoff
        )
        self.cut_btn.pack(pady=20)

    def on_slider_move(self, val):
        throttle_val = float(val)
        self.label.config(text=f"Throttle: {throttle_val:.2f}")
        msg = Float32()
        msg.data = throttle_val
        self.pub_throttle.publish(msg)

    def emergency_cutoff(self):
        self.slider.set(0.0)
        self.on_slider_move(0.0)

    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    gui_node = DroneGUI()

    # Spin ROS 2 in a background thread so Tkinter GUI stays responsive
    ros_thread = threading.Thread(target=rclpy.spin, args=(gui_node,), daemon=True)
    ros_thread.start()

    gui_node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()