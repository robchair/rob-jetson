import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Make /home/rob/rob importable so "import roboVoice" works
sys.path.insert(0, os.path.expanduser("~/rob"))
sys.path.insert(0, "/home/rob/rob")

from roboVoice.main_realtime_asr import run_realtime


class VoiceCmdNode(Node):
    def __init__(self):
        super().__init__("voice_cmd_node")

        self.declare_parameter("linear_speed", 0.30)
        self.declare_parameter("angular_speed", 0.80)
        self.declare_parameter("publish_rate_hz", 10.0)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.pub = self.create_publisher(Twist, "/cmd_vel_voice", 10)

        self._lock = threading.Lock()
        self._last_twist = Twist()
        self._have_cmd = False

        # publish last command continuously (so twist_mux never times out)
        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info("VoiceCmdNode started. Listening for voice commands...")

    def _on_timer(self):
        with self._lock:
            if not self._have_cmd:
                return
            self.pub.publish(self._last_twist)

    def publish_cmd(self, cmd: str | None):
        if cmd is None:
            return

        cmd = cmd.strip().lower()
        msg = Twist()

        if cmd == "forward":
            msg.linear.x = self.linear_speed
        elif cmd in ("back", "backward"):
            msg.linear.x = -self.linear_speed
        elif cmd == "left":
            msg.angular.z = self.angular_speed
        elif cmd == "right":
            msg.angular.z = -self.angular_speed
        elif cmd == "stop":
            pass
        elif cmd == "manual":
            pass
        else:
            self.get_logger().warn(f"Unknown voice cmd {cmd!r}")
            return

        with self._lock:
            self._last_twist = msg
            self._have_cmd = True

        self.get_logger().info(f"[voice] cmd={cmd} -> lin={msg.linear.x:.2f} ang={msg.angular.z:.2f}")


def main():
    rclpy.init()
    node = VoiceCmdNode()

    def worker():
        # If run_realtime ever returns, we WANT to see it in logs
        node.get_logger().info("[voice] starting ASR worker thread")
        try:
            run_realtime(on_command=node.publish_cmd)
        except Exception as e:
            node.get_logger().error(f"[voice] ASR thread crashed: {e}")
            raise
        finally:
            node.get_logger().warn("[voice] ASR worker thread exited (run_realtime returned)")

    t = threading.Thread(target=worker, daemon=True)
    t.start()

    try:
        rclpy.spin(node)  # keeps process alive
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

