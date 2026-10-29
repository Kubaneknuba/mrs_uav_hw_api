#!/usr/bin/env python3
import sys
import threading
from PySide6.QtCore import Qt, QRectF, QObject, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QSlider, QDoubleSpinBox, QPushButton
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong


class CenteredFillSlider(QSlider):
    def __init__(self, orientation=Qt.Horizontal, parent=None):
        super().__init__(orientation, parent)
        self._inactive = QColor("#111111")
        self._active = QColor("#ff9800")
        self._handle_fill = QColor("#f5f5f5")
        self._handle_border = QColor("#888888")
        self._handle_radius = 7.0
        self._groove_height = 8.0
        self._side_padding = 10.0
        self._dragging = False
        self.setMinimumHeight(28)
        self.setPageStep(1)
        self.setSingleStep(1)

    def _slider_span(self):
        x_min = self._side_padding + self._handle_radius
        x_max = self.width() - self._side_padding - self._handle_radius
        return x_min, x_max

    def _value_to_x(self, value: int) -> float:
        x_min, x_max = self._slider_span()
        slider_span = self.maximum() - self.minimum()
        if slider_span <= 0 or x_max <= x_min:
            return x_min
        ratio = (value - self.minimum()) / slider_span
        ratio = max(0.0, min(1.0, ratio))
        return x_min + ratio * (x_max - x_min)

    def _x_to_value(self, x: float) -> int:
        x_min, x_max = self._slider_span()
        slider_span = self.maximum() - self.minimum()
        if slider_span <= 0 or x_max <= x_min:
            return self.minimum()
        clamped_x = max(x_min, min(x_max, x))
        ratio = (clamped_x - x_min) / (x_max - x_min)
        return int(round(self.minimum() + ratio * slider_span))

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.orientation() == Qt.Horizontal:
            self._dragging = True
            self.setSliderDown(True)
            self.setValue(self._x_to_value(float(event.position().x())))
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._dragging and self.orientation() == Qt.Horizontal:
            self.setValue(self._x_to_value(float(event.position().x())))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self._dragging:
            self._dragging = False
            self.setSliderDown(False)
            self.setValue(self._x_to_value(float(event.position().x())))
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def paintEvent(self, event):
        if self.orientation() != Qt.Horizontal:
            super().paintEvent(event)
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        handle_radius = self._handle_radius
        groove_height = self._groove_height

        x_min, x_max = self._slider_span()
        y_center = self.height() / 2.0

        if x_max <= x_min:
            return

        groove = QRectF(x_min, y_center - groove_height / 2.0, x_max - x_min, groove_height)
        painter.setPen(Qt.NoPen)
        painter.setBrush(self._inactive)
        painter.drawRoundedRect(groove, groove_height / 2.0, groove_height / 2.0)

        x_zero = self._value_to_x(0)
        x_value = self._value_to_x(self.value())

        if abs(x_value - x_zero) > 0.001:
            active_left = min(x_zero, x_value)
            active_right = max(x_zero, x_value)
            active_rect = QRectF(active_left, y_center - groove_height / 2.0, active_right - active_left, groove_height)
            painter.setBrush(self._active)
            painter.drawRoundedRect(active_rect, groove_height / 2.0, groove_height / 2.0)

        painter.setBrush(self._handle_fill)
        painter.setPen(QPen(self._handle_border, 1))
        painter.drawEllipse(QRectF(x_value - handle_radius, y_center - handle_radius, 2 * handle_radius, 2 * handle_radius))


class FilledSlider(QSlider):
    def __init__(self, orientation=Qt.Horizontal, parent=None):
        super().__init__(orientation, parent)
        self._inactive = QColor("#111111")
        self._active = QColor("#ff9800")
        self._handle_fill = QColor("#f5f5f5")
        self._handle_border = QColor("#888888")
        self._handle_radius = 7.0
        self._groove_height = 8.0
        self._side_padding = 10.0
        self._dragging = False
        self.setMinimumHeight(28)
        self.setPageStep(1)
        self.setSingleStep(1)

    def _slider_span(self):
        x_min = self._side_padding + self._handle_radius
        x_max = self.width() - self._side_padding - self._handle_radius
        return x_min, x_max

    def _value_to_x(self, value: int) -> float:
        x_min, x_max = self._slider_span()
        slider_span = self.maximum() - self.minimum()
        if slider_span <= 0 or x_max <= x_min:
            return x_min
        ratio = (value - self.minimum()) / slider_span
        ratio = max(0.0, min(1.0, ratio))
        return x_min + ratio * (x_max - x_min)

    def _x_to_value(self, x: float) -> int:
        x_min, x_max = self._slider_span()
        slider_span = self.maximum() - self.minimum()
        if slider_span <= 0 or x_max <= x_min:
            return self.minimum()
        clamped_x = max(x_min, min(x_max, x))
        ratio = (clamped_x - x_min) / (x_max - x_min)
        return int(round(self.minimum() + ratio * slider_span))

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.orientation() == Qt.Horizontal:
            self._dragging = True
            self.setSliderDown(True)
            self.setValue(self._x_to_value(float(event.position().x())))
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._dragging and self.orientation() == Qt.Horizontal:
            self.setValue(self._x_to_value(float(event.position().x())))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self._dragging:
            self._dragging = False
            self.setSliderDown(False)
            self.setValue(self._x_to_value(float(event.position().x())))
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def paintEvent(self, event):
        if self.orientation() != Qt.Horizontal:
            super().paintEvent(event)
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        handle_radius = self._handle_radius
        groove_height = self._groove_height

        x_min, x_max = self._slider_span()
        y_center = self.height() / 2.0

        if x_max <= x_min:
            return

        groove = QRectF(x_min, y_center - groove_height / 2.0, x_max - x_min, groove_height)
        painter.setPen(Qt.NoPen)
        painter.setBrush(self._inactive)
        painter.drawRoundedRect(groove, groove_height / 2.0, groove_height / 2.0)

        x_value = self._value_to_x(self.value())

        if x_value > x_min:
            active_rect = QRectF(x_min, y_center - groove_height / 2.0, x_value - x_min, groove_height)
            painter.setBrush(self._active)
            painter.drawRoundedRect(active_rect, groove_height / 2.0, groove_height / 2.0)

        painter.setBrush(self._handle_fill)
        painter.setPen(QPen(self._handle_border, 1))
        painter.drawEllipse(QRectF(x_value - handle_radius, y_center - handle_radius, 2 * handle_radius, 2 * handle_radius))


class ArmStateBridge(QObject):
    arm_state_changed = Signal(bool)


class ROSNode(Node):
    def __init__(self, arm_state_bridge: ArmStateBridge):
        super().__init__('wind_tunnel_pyside_gui')
        self.arm_state_bridge = arm_state_bridge
        self.pub_throttle = self.create_publisher(Float32, '/uav1/set_throttle', 10)
        self.pub_servos = self.create_publisher(Float32MultiArray, '/uav1/set_actuator_cmd', 10)
        self.client_arm_disarm = self.create_client(SetBool, '/uav1/arming')
        self.client_servo = self.create_client(CommandLong, '/uav1/mavros/cmd/command')
        self.armed_state = self.create_subscription(Bool, '/uav1/arm_disarm_cmd', self._on_arm_disarm, 10)

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

    def send_servo_array(self, normalized_values):
        msg = Float32MultiArray()
        msg.data = [float(0) for _ in range(6)]  # Initialize with 6 zeros
        for i, value in enumerate(normalized_values):
            if value < -1.0 or value > 1.0:
                self.get_logger().warn(f"Servo value {value} is out of range [-1.0, 1.0]. Clamping.")
                value = max(-1.0, min(1.0, value))
            msg.data[i] = float(value)
            
        print(f"Sending servo array: {msg.data}")
        self.pub_servos.publish(msg)

    def publish_arm_disarm(self, arm: bool):
        """Arm/disarm publish hook."""
        msg = SetBool.Request()
        msg.data = bool(arm)
        self.client_arm_disarm.call_async(msg)
        state = "ARM" if arm else "DISARM"
        self.get_logger().info(f"Published {state} command")

    def _on_arm_disarm(self, msg: Bool):
        self.arm_state_bridge.arm_state_changed.emit(bool(msg.data))

class MainWindow(QMainWindow):
    def __init__(self, ros_node: ROSNode, arm_state_bridge: ArmStateBridge):
        super().__init__()
        self.ros_node = ros_node
        self.arm_state_bridge = arm_state_bridge

        self.servo_values = [0.0, 0.0, 0.0, 0.0]  # Normalized values for MAIN 1-4

        self.setWindowTitle("PX4 Wind Tunnel Controller")
        self.setMinimumWidth(560)
        self._apply_modern_style()

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(18, 18, 18, 18)
        main_layout.setSpacing(16)

        # ---------------- 1. Throttle Control Group ----------------
        throttle_group = QGroupBox("Motor Throttle (AUX Outputs)")
        throttle_layout = QHBoxLayout(throttle_group)

        self.throttle_slider = FilledSlider(Qt.Horizontal)
        self.throttle_slider.setRange(0, 100)  # Maps 0% to 100%
        self.throttle_slider.setValue(5)       # Default 0.05

        self.throttle_spin = QDoubleSpinBox()
        self.throttle_spin.setRange(0.00, 1.00)
        self.throttle_spin.setSingleStep(0.01)
        self.throttle_spin.setDecimals(2)
        self.throttle_spin.setValue(0.05)
        self.throttle_spin.setFixedWidth(90)

        # Sync Throttle Slider <-> SpinBox
        self.throttle_slider.valueChanged.connect(self._on_throttle_slider_changed)
        self.throttle_spin.valueChanged.connect(self._on_throttle_spin_changed)

        throttle_layout.addWidget(QLabel("Throttle:"))
        throttle_layout.addWidget(self.throttle_slider)
        throttle_layout.addWidget(self.throttle_spin)
        main_layout.addWidget(throttle_group)

        # ---------------- 2. Wing Servos Group (MAIN 1-4) ----------------
        servo_group = QGroupBox("Wing Servos (normalized, MAIN 1 to 4)")
        servo_layout = QVBoxLayout(servo_group)
        servo_layout.setSpacing(18)

        self.servo_sliders = []
        self.servo_spins = []

        for i in range(1, 5):
            row_layout = QHBoxLayout()

            label = QLabel(f"MAIN {i}:")
            label.setFixedWidth(55)

            slider = CenteredFillSlider(Qt.Horizontal)
            slider.setRange(-100, 100)
            slider.setValue(0)

            spin = QDoubleSpinBox()
            spin.setRange(-1.00, 1.00)
            spin.setSingleStep(0.01)
            spin.setDecimals(2)
            spin.setValue(0.00)
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

        # ---------------- 3. Arm/Disarm + Emergency Buttons ----------------
        button_row = QHBoxLayout()
        button_row.setSpacing(12)

        self.arm_btn = QPushButton("ARM")
        self.arm_btn.setObjectName("armButton")
        self.arm_btn.setCheckable(True)
        self.arm_btn.toggled.connect(self._on_arm_toggled)
        button_row.addWidget(self.arm_btn)

        self.cutoff_btn = QPushButton("EMERGENCY STOP (0% Throttle)")
        self.cutoff_btn.setObjectName("emergencyButton")
        self.cutoff_btn.clicked.connect(self.emergency_stop)
        button_row.addWidget(self.cutoff_btn)
        main_layout.addLayout(button_row)

        self.arm_state_bridge.arm_state_changed.connect(self._on_arm_state_received)

    def _apply_modern_style(self):
        self.setStyleSheet(
            """
            QMainWindow {
                background-color: #0b1220;
            }
            QGroupBox {
                background-color: #111827;
                color: #f8fafc;
                border: 1px solid #334155;
                border-radius: 10px;
                margin-top: 14px;
                padding: 14px 12px 12px 12px;
                font-weight: 600;
                font-size: 13px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
            }
            QLabel {
                color: #cbd5e1;
                font-size: 13px;
            }
            QDoubleSpinBox {
                background-color: #0f172a;
                color: #f8fafc;
                border: 1px solid #475569;
                border-radius: 6px;
                padding: 4px 6px;
                selection-background-color: #f59e0b;
            }
            QDoubleSpinBox:focus {
                border: 1px solid #f59e0b;
            }
            QPushButton#emergencyButton {
                background-color: #dc2626;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 11px;
                font-size: 14px;
                font-weight: 700;
            }
            QPushButton#emergencyButton:hover {
                background-color: #b91c1c;
            }
            QPushButton#emergencyButton:pressed {
                background-color: #991b1b;
            }
            QPushButton#armButton {
                background-color: #059669;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 11px;
                font-size: 14px;
                font-weight: 700;
                min-width: 120px;
            }
            QPushButton#armButton:hover {
                background-color: #047857;
            }
            QPushButton#armButton:pressed {
                background-color: #065f46;
            }
            QPushButton#armButton:checked {
                background-color: #f59e0b;
            }
            QPushButton#armButton:checked:hover {
                background-color: #d97706;
            }
            """
        )

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

    def _on_arm_toggled(self, checked: bool):
        self.arm_btn.setText("DISARM" if checked else "ARM")
        self.ros_node.publish_arm_disarm(checked)

    def _on_arm_state_received(self, is_armed: bool):
        self.arm_btn.blockSignals(True)
        self.arm_btn.setChecked(is_armed)
        self.arm_btn.setText("DISARM" if is_armed else "ARM")
        self.arm_btn.blockSignals(False)

    # --- Servo Sync Handlers ---
    def _on_servo_slider_changed(self, index: int, value: int):
        normalized = value / 100.0
        self.servo_spins[index].blockSignals(True)
        self.servo_spins[index].setValue(normalized)
        self.servo_spins[index].blockSignals(False)
        self.servo_values[index] = normalized
        self.ros_node.send_servo_array(self.servo_values)

    def _on_servo_spin_changed(self, index: int, value: float):
        slider_value = int(round(value * 100))
        self.servo_sliders[index].blockSignals(True)
        self.servo_sliders[index].setValue(slider_value)
        self.servo_sliders[index].blockSignals(False)
        self.servo_values[index] = value
        self.ros_node.send_servo_array(self.servo_values)

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

    app = QApplication(sys.argv)
    arm_state_bridge = ArmStateBridge()
    ros_node = ROSNode(arm_state_bridge)
    window = MainWindow(ros_node, arm_state_bridge)

    # Run ROS 2 executor in a daemon thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Start PySide6 Qt Application
    window.show()

    exit_code = app.exec()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()