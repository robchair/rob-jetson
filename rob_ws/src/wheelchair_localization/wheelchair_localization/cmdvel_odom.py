#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    # z-only quaternion
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class CmdVelOdom(Node):
    """
    Temporary odom source:
      subscribes to /cmd_vel_safe and integrates pose. Since no IMU or encoder integration yet, this is not best but temporary setup for SLAM bringup.
      publishes:
        - /odom (nav_msgs/Odometry)
        - TF: odom -> base_link
    """

    def __init__(self):
        super().__init__("cmdvel_odom")

        self.declare_parameter("cmd_topic", "/cmd_vel_safe")    # When IMU or encoders are integrated, we should make a new topic.
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("timeout_sec", 0.5)

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.timeout = float(self.get_parameter("timeout_sec").value)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.cmd_sub = self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.wz = 0.0
        self.last_cmd_time = time.time()
        self.last_time = time.time()

        self.timer = self.create_timer(0.02, self.step)  # 50 Hz
        self.get_logger().info(f"CmdVelOdom integrating {self.cmd_topic} -> TF {self.odom_frame}->{self.base_frame}")

    def on_cmd(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.wz = float(msg.angular.z)
        self.last_cmd_time = time.time()

    def step(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0:
            return

        # If commands went stale, treat as stopped
        if (now - self.last_cmd_time) > self.timeout:
            vx = 0.0
            wz = 0.0
        else:
            vx = self.vx
            wz = self.wz

        # integrate in odom frame
        self.yaw += wz * dt
        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt

        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        # publish odom msg
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = CmdVelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
