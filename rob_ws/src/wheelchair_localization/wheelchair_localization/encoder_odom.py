#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class EncoderOdom(Node):
    """
    Subscribes to /wheel_encoder_ticks and publishes:
      - /odom
      - TF: odom -> base_link

    Expected encoder topic format:
      Int64MultiArray.data = [left_count, right_count, arduino_time_ms]

    Notes:
    - This is encoder-based differential-drive odom.
    - For now, wheel and tick parameters are user-supplied and can be calibrated later.
    """

    def __init__(self):
        super().__init__("encoder_odom")

        self.declare_parameter("encoder_topic", "/wheel_encoder_ticks")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("publish_tf", False)

        self.declare_parameter("wheel_radius_m", 0.305)#change to new radius
        self.declare_parameter("wheel_base_m", 0.515)#change to new base 
        self.declare_parameter("ticks_per_rev", 1200.0)

        # Optional sign correction at ROS level
        self.declare_parameter("left_sign", 1.0)
        self.declare_parameter("right_sign", 1.0)

        self.encoder_topic = self.get_parameter("encoder_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)
        self.ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)

        self.left_sign = float(self.get_parameter("left_sign").value)
        self.right_sign = float(self.get_parameter("right_sign").value)

        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius_m) / self.ticks_per_rev

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Int64MultiArray,
            self.encoder_topic,
            self.on_encoder,
            50,
        )

        self.prev_left = None
        self.prev_right = None
        self.prev_t_ms = None
        self.prev_ros_time = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_vx = 0.0
        self.last_wz = 0.0

        self.get_logger().info(
            f"EncoderOdom listening on {self.encoder_topic}, publishing {self.odom_topic}"
        )

    def on_encoder(self, msg: Int64MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warn("Encoder message missing fields")
            return

        left = int(msg.data[0])
        right = int(msg.data[1])
        t_ms = int(msg.data[2])

        # Apply software sign convention
        left = int(self.left_sign * left)
        right = int(self.right_sign * right)

        #now_ros = time.time()
        now_ros = self.get_clock().now().nanoseconds * 1e-9      # use ROS /clock instead of OS clock

        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            self.prev_t_ms = t_ms
            self.prev_ros_time = now_ros
            return

        d_left_ticks = left - self.prev_left
        d_right_ticks = right - self.prev_right

        # Prefer Arduino delta time when valid
        dt = self.compute_dt_seconds(t_ms, now_ros)
        if dt is None or dt <= 0.0:
            self.prev_left = left
            self.prev_right = right
            self.prev_t_ms = t_ms
            self.prev_ros_time = now_ros
            return

        d_left = d_left_ticks * self.meters_per_tick
        d_right = d_right_ticks * self.meters_per_tick

        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.wheel_base_m

        # Midpoint integration is better than simple Euler here
        yaw_mid = self.yaw + 0.5 * d_theta
        self.x += d_center * math.cos(yaw_mid)
        self.y += d_center * math.sin(yaw_mid)
        self.yaw += d_theta
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))    #Normalize yaw after update

        self.last_vx = d_center / dt
        self.last_wz = d_theta / dt

        self.publish_odom()

        self.prev_left = left
        self.prev_right = right
        self.prev_t_ms = t_ms
        self.prev_ros_time = now_ros

    def compute_dt_seconds(self, current_t_ms: int, current_ros_time: float):
        """
        Use Arduino time if available.
        Handle millis() rollover.
        Fallback to ROS wall time if needed.
        """
        if self.prev_t_ms is None:
            return None

        if current_t_ms >= self.prev_t_ms:
            dt_ms = current_t_ms - self.prev_t_ms
        else:
            # millis() rollover for uint32
            dt_ms = (2**32 - self.prev_t_ms) + current_t_ms

        dt = dt_ms / 1000.0

        # Fallback if Arduino dt looks unreasonable
        if dt <= 0.0 or dt > 1.0:
            if self.prev_ros_time is None:
                return None
            dt = current_ros_time - self.prev_ros_time

        return dt

    def publish_odom(self):
        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.last_vx
        odom.twist.twist.angular.z = self.last_wz

        # Conservative placeholder covariances for early bringup
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.1

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            #self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = EncoderOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()