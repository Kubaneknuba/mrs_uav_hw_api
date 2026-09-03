#!/usr/bin/env python3
import sys
import threading
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QSlider, QSpinBox, QDoubleSpinBox, QPushButton, QFrame
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from mavros_msgs.srv import CommandLong


class ROSNode(Node):
    def __init__(self):
        super().__init__('wind_tunnel_pyside_gui')
        self.pub_throttle = self.create_publisher(Float32, '/uav1/set_throttle', 10)
        self.client_servo = self.create_client(CommandLong, '/uav1/mavros/cmd/command')

    def send_throttle(self, value: float):
        msg = Float32()
        msg.data = float(value)
        self.pub_throttle.publish(msg)

    def send_servo(self, channel: int, pwm_value: int):
        if self.client_servo.service_is_ready():
            req = CommandLong.Request()
            req.command = 183  # MAV_CMD_DO_SET_SERVO
            req.param1 = float(channel)  # 1.0 = MAIN 1, 2.0 = MAIN 2, etc.
            req.param2 = float(pwm_value)
            self.client_servo.call_async(req)


class MainWindow(QMainWindow):
    def __init__(self, ros_node: ROSNode):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("PX4 Wind Tunnel Controller")
        self.setMinimumWidth(480)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setSpacing(15)

        # ---------------- 1. Throttle Control Group ----------------
        throttle_group = QGroupBox("Motor Throttle (AUX Outputs)")
        throttle_layout = QHBoxLayout(throttle_group)

        self.throttle_slider = QSlider(Qt.Horizontal)
        self.throttle_slider.setRange(0, 100)  # Maps 0% to 100%
        self.throttle_slider.setValue(5)       # Default 0.05

        self.throttle_spin = QDoubleSpinBox()
        self.throttle_spin.setRange(0.00, 1.00)
        self.throttle_spin.setSingleStep(0.01)
        self.throttle_spin.setDecimals(2)
        self.throttle_spin.setValue(0.05)
        self.throttle_spin.setFixedWidth(80)

        # Sync Throttle Slider <-> SpinBox
        self.throttle_slider.valueChanged.connect(self._on_throttle_slider_changed)
        self.throttle_spin.valueChanged.connect(self._on_throttle_spin_changed)

        throttle_layout.addWidget(QLabel("Throttle:"))
        throttle_layout.addWidget(self.throttle_slider)
        throttle_layout.addWidget(self.throttle_spin)
        main_layout.addWidget(throttle_group)

        # ---------------- 2. Wing Servos Group (MAIN 1-4) ----------------
        servo_group = QGroupBox("Wing Servos PWM (MAIN 1 to 4)")
        servo_layout = QVBoxLayout(servo_group)
        servo_layout.setSpacing(10)

        self.servo_sliders = []
        self.servo_spins = []

        for i in range(1, 5):
            row_layout = QHBoxLayout()

            label = QLabel(f"MAIN {i}:")
            label.setFixedWidth(55)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(1000, 2000)
            slider.setValue(1500)

            spin = QSpinBox()
            spin.setRange(1000, 2000)
            spin.setSingleStep(1)
            spin.setValue(1500)
            spin.setSuffix(" µs")
            spin.setFixedWidth(90)

            # Bind callbacks using lambda captures
            slider.valueChanged.connect(
                lambda val, idx=i-1: self._on_servo_slider_changed(idx, val)
            )
            spin.valueChanged.connect(
                lambda val, idx=i-1: self._on_servo_spin_changed(idx, val)
            )

            row_layout.addWidget(label)
            row_layout.addWidget(slider)
            row_layout.addWidget(spin)
            servo_layout.addLayout(row_layout)

            self.servo_sliders.append(slider)
            self.servo_spins.append(spin)

        main_layout.addWidget(servo_group)

        # ---------------- 3. Emergency Cutoff Button ----------------
        self.cutoff_btn = QPushButton("EMERGENCY STOP (0% Throttle)")
        self.cutoff_btn.setStyleSheet(
            "QPushButton { background-color: #d32f2f; color: white; font-weight: bold; padding: 10px; font-size: 14px; }"
            "QPushButton:hover { background-color: #b71c1c; }"
        )
        self.cutoff_btn.clicked.connect(self.emergency_stop)
        main_layout.addWidget(self.cutoff_btn)

    # --- Throttle Sync Handlers ---
    def _on_throttle_slider_changed(self, value: int):
        float_val = value / 100.0
        self.throttle_spin.blockSignals(True)
        self.throttle_spin.setValue(float_val)
        self.throttle_spin.blockSignals(False)
        self.ros_node.send_throttle(float_val)

    def _on_throttle_spin_changed(self, value: float):
        int_val = int(round(value * 100))
        self.throttle_slider.blockSignals(True)
        self.throttle_slider.setValue(int_val)
        self.throttle_slider.blockSignals(False)
        self.ros_node.send_throttle(value)

    # --- Servo Sync Handlers ---
    def _on_servo_slider_changed(self, index: int, value: int):
        self.servo_spins[index].blockSignals(True)
        self.servo_spins[index].setValue(value)
        self.servo_spins[index].blockSignals(False)
        self.ros_node.send_servo(channel=index + 1, pwm_value=value)

    def _on_servo_spin_changed(self, index: int, value: int):
        self.servo_sliders[index].blockSignals(True)
        self.servo_sliders[index].setValue(value)
        self.servo_sliders[index].blockSignals(False)
        self.ros_node.send_servo(channel=index + 1, pwm_value=value)

    # --- Emergency Cutoff ---
    def emergency_stop(self):
        self.throttle_spin.setValue(0.00)


def main():
    # Install dependency check: PySide6
    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        print("PySide6 not installed. Install via: pip install PySide6")
        sys.exit(1)

    rclpy.init()
    ros_node = ROSNode()

    # Run ROS 2 executor in a daemon thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Start PySide6 Qt Application
    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    exit_code = app.exec()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()