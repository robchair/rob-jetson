#!/usr/bin/env python3
import sys
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QPushButton, QGridLayout, QVBoxLayout,
    QHBoxLayout, QLabel, QStackedWidget
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

DEFAULT_LINEAR  = 0.50
DEFAULT_ANGULAR = 1.00


class ROSNode(Node):
    def __init__(self):
        super().__init__("wheelchair_ui")
        self.cmd_pub   = self.create_publisher(Twist, "/cmd_vel_keyboard", 10)
        self.voice_pub = self.create_publisher(Bool,  "/voice_enabled",    10)

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)
        print(f"[cmd_vel] lin={linear:.2f}  ang={angular:.2f}")

    def publish_twist_z(self, z):
        msg = Twist()
        msg.linear.z = float(z)
        self.cmd_pub.publish(msg)
        print(f"[cmd_vel] z={z:.2f}")

    def publish_voice(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.voice_pub.publish(msg)
        print(f"[voice] {'ENABLED' if enabled else 'DISABLED'}")


class ManualPage(QWidget):
    def __init__(self, ros_node: ROSNode):
        super().__init__()
        self.ros     = ros_node
        self.linear  = DEFAULT_LINEAR
        self.angular = DEFAULT_ANGULAR
        self._build()

    def _ctrl_btn(self, label, size=100, font_size=26, stop=False):
        btn = QPushButton(label)
        btn.setFixedSize(size, size)
        btn.setFont(QFont("Segoe UI", font_size))
        if stop:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #1f1010; color: #ef4444;
                    border-radius: 14px; border: 2px solid #7f1d1d;
                }
                QPushButton:pressed { background-color: #ef4444; color: white; }
            """)
        else:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #1a1d27; color: #f0f0f0;
                    border-radius: 14px; border: 2px solid #2a2d3a;
                }
                QPushButton:pressed {
                    background-color: #3b82f6; border-color: #3b82f6; color: white;
                }
            """)
        return btn

    def _speed_btn(self, label):
        btn = QPushButton(label)
        btn.setFixedSize(64, 48)
        btn.setFont(QFont("Segoe UI", 13))
        btn.setStyleSheet("""
            QPushButton {
                background-color: #1a1d27; color: #9ca3af;
                border-radius: 8px; border: 1px solid #2a2d3a;
            }
            QPushButton:pressed { background-color: #2a2d3a; color: white; }
        """)
        return btn

    def _update_speed_display(self):
        self.speed_display.setText(
            f"speed  {self.linear:.2f}    turn  {self.angular:.2f}"
        )

    def _adjust_both(self, factor):
        self.linear  = round(self.linear  * factor, 2)
        self.angular = round(self.angular * factor, 2)
        self._update_speed_display()

    def _adjust_linear(self, factor):
        self.linear = round(self.linear * factor, 2)
        self._update_speed_display()

    def _adjust_angular(self, factor):
        self.angular = round(self.angular * factor, 2)
        self._update_speed_display()

    def _build(self):
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(20)

        main_row = QHBoxLayout()
        main_row.setSpacing(32)

        # 3x3 grid
        GRID = {
            (0,0):"u",(0,1):"i",(0,2):"o",
            (1,0):"j",(1,1):"k",(1,2):"l",
            (2,0):"m",(2,1):",",(2,2):".",
        }
        LABELS = {
            "u":"↖","i":"↑","o":"↗",
            "j":"←","k":"■","l":"→",
            "m":"↙",",":"↓",".":"↘",
        }
        BINDINGS = {
            "u":( DEFAULT_LINEAR,  DEFAULT_ANGULAR),
            "i":( DEFAULT_LINEAR,  0.0),
            "o":( DEFAULT_LINEAR, -DEFAULT_ANGULAR),
            "j":( 0.0,             DEFAULT_ANGULAR),
            "k":( 0.0,             0.0),
            "l":( 0.0,            -DEFAULT_ANGULAR),
            "m":(-DEFAULT_LINEAR, -DEFAULT_ANGULAR),
            ",":(-DEFAULT_LINEAR,  0.0),
            ".":(-DEFAULT_LINEAR,  DEFAULT_ANGULAR),
        }

        grid_widget = QWidget()
        grid = QGridLayout(grid_widget)
        grid.setSpacing(12)
        for (row, col), key in GRID.items():
            btn = self._ctrl_btn(LABELS[key], stop=(key == "k"))
            lin, ang = BINDINGS[key]
            btn.pressed.connect(lambda l=lin, a=ang: self.ros.publish_twist(l, a))
            btn.released.connect(lambda: self.ros.publish_twist(0.0, 0.0))
            grid.addWidget(btn, row, col)

        main_row.addWidget(grid_widget, alignment=Qt.AlignCenter)

        # right column
        right_col = QVBoxLayout()
        right_col.setSpacing(12)
        right_col.setAlignment(Qt.AlignCenter)

        z_label = QLabel("Z axis")
        z_label.setStyleSheet("color: #6b7280; font-size: 12px;")
        z_label.setAlignment(Qt.AlignCenter)

        btn_t = self._ctrl_btn("▲", size=72, font_size=20)
        btn_t.pressed.connect(lambda: self.ros.publish_twist_z(self.linear))
        btn_t.released.connect(lambda: self.ros.publish_twist_z(0.0))

        btn_b = self._ctrl_btn("▼", size=72, font_size=20)
        btn_b.pressed.connect(lambda: self.ros.publish_twist_z(-self.linear))
        btn_b.released.connect(lambda: self.ros.publish_twist_z(0.0))

        self.speed_display = QLabel(f"speed  {self.linear:.2f}    turn  {self.angular:.2f}")
        self.speed_display.setStyleSheet("color: #9ca3af; font-size: 13px; font-family: monospace;")
        self.speed_display.setAlignment(Qt.AlignCenter)

        def speed_row(label, plus_cb, minus_cb):
            row = QHBoxLayout()
            row.setSpacing(6)
            lbl = QLabel(label)
            lbl.setStyleSheet("color: #6b7280; font-size: 12px; min-width: 80px;")
            plus  = self._speed_btn("+")
            minus = self._speed_btn("−")
            plus.clicked.connect(plus_cb)
            minus.clicked.connect(minus_cb)
            row.addWidget(lbl)
            row.addWidget(plus)
            row.addWidget(minus)
            return row

        right_col.addWidget(z_label)
        right_col.addWidget(btn_t, alignment=Qt.AlignCenter)
        right_col.addWidget(btn_b, alignment=Qt.AlignCenter)
        right_col.addSpacing(16)
        right_col.addWidget(self.speed_display)
        right_col.addSpacing(8)
        right_col.addLayout(speed_row("q/z  both",    lambda: self._adjust_both(1.1),    lambda: self._adjust_both(0.9)))
        right_col.addLayout(speed_row("w/x  linear",  lambda: self._adjust_linear(1.1),  lambda: self._adjust_linear(0.9)))
        right_col.addLayout(speed_row("e/c  angular", lambda: self._adjust_angular(1.1), lambda: self._adjust_angular(0.9)))

        main_row.addLayout(right_col)
        layout.addLayout(main_row)

        hint = QLabel("Hold to move  ·  ■ to stop  ·  Release to stop")
        hint.setStyleSheet("color: #4b5563; font-size: 13px;")
        hint.setAlignment(Qt.AlignCenter)
        layout.addWidget(hint)


class VoicePage(QWidget):
    def __init__(self):
        super().__init__()
        self._build()

    def _build(self):
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(20)

        mic = QLabel("🎙️")
        mic.setFont(QFont("Segoe UI", 64))
        mic.setAlignment(Qt.AlignCenter)
        mic.setStyleSheet("""
            QLabel {
                background-color: #1a1d27;
                border: 3px solid #3b82f6;
                border-radius: 75px;
                min-width: 150px; max-width: 150px;
                min-height: 150px; max-height: 150px;
            }
        """)

        status = QLabel("Listening...")
        status.setStyleSheet("color: #3b82f6; font-size: 16px;")
        status.setAlignment(Qt.AlignCenter)

        layout.addWidget(mic,    alignment=Qt.AlignCenter)
        layout.addWidget(status, alignment=Qt.AlignCenter)


class WheelchairUI(QMainWindow):
    def __init__(self, ros_node: ROSNode):
        super().__init__()
        self.ros  = ros_node
        self.mode = "manual"
        self.setWindowTitle("ROB Control Panel")
        self.setStyleSheet("background-color: #0f1117; color: #f0f0f0;")
        self.resize(800, 600)
        self._build_ui()

    def _tab_btn(self, text, active=False):
        btn = QPushButton(text)
        btn.setFixedSize(160, 52)
        btn.setFont(QFont("Segoe UI", 14, QFont.Bold))
        self._style_tab(btn, active)
        return btn

    def _style_tab(self, btn, active):
        if active:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #3b82f6;
                    color: white; border-radius: 10px; border: none;
                }
            """)
        else:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #1a1d27; color: #6b7280;
                    border-radius: 10px; border: 1px solid #2a2d3a;
                }
                QPushButton:hover { background-color: #2a2d3a; color: #d1d5db; }
            """)

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(40, 40, 40, 40)
        layout.setSpacing(24)

        # status bar
        status_bar = QHBoxLayout()
        self.status_label = QLabel("ROS Connected")
        self.status_label.setStyleSheet("color: #22c55e; font-size: 14px;")
        title = QLabel("ROB Control Panel")
        title.setStyleSheet("color: #4b5563; font-size: 13px;")
        title.setAlignment(Qt.AlignCenter)
        self.mode_label = QLabel("Manual Control")
        self.mode_label.setStyleSheet("color: #6b7280; font-size: 14px;")
        self.mode_label.setAlignment(Qt.AlignRight)
        status_bar.addWidget(self.status_label)
        status_bar.addWidget(title)
        status_bar.addWidget(self.mode_label)
        layout.addLayout(status_bar)

        # tabs
        tabs = QHBoxLayout()
        self.btn_manual = self._tab_btn("Manual", active=True)
        self.btn_voice  = self._tab_btn("Voice",  active=False)
        self.btn_manual.clicked.connect(lambda: self._set_mode("manual"))
        self.btn_voice.clicked.connect(lambda: self._set_mode("voice"))
        tabs.addStretch()
        tabs.addWidget(self.btn_manual)
        tabs.addSpacing(12)
        tabs.addWidget(self.btn_voice)
        tabs.addStretch()
        layout.addLayout(tabs)

        # stacked pages
        self.stack = QStackedWidget()
        self.manual_page = ManualPage(self.ros)
        self.voice_page  = VoicePage()
        self.stack.addWidget(self.manual_page)  # index 0
        self.stack.addWidget(self.voice_page)   # index 1
        self.stack.setCurrentIndex(0)
        layout.addWidget(self.stack)

    def _set_mode(self, mode):
        self.mode = mode
        self._style_tab(self.btn_manual, mode == "manual")
        self._style_tab(self.btn_voice,  mode == "voice")
        self.mode_label.setText("Manual Control" if mode == "manual" else "Voice Control")
        self.ros.publish_voice(mode == "voice")
        self.ros.publish_twist(0.0, 0.0)
        self.stack.setCurrentIndex(0 if mode == "manual" else 1)


def main():
    rclpy.init()
    ros_node = ROSNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    ui  = WheelchairUI(ros_node)
    ui.show()

    exit_code = app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()