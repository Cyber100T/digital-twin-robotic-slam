#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, Imu, Image, CameraInfo
from nav_msgs.msg import Odometry


def shift_stamp(stamp, offset_sec_float):
    total_nsec = int(stamp.sec * 1_000_000_000 + stamp.nanosec)
    offset_nsec = int(offset_sec_float * 1_000_000_000)
    new_total = total_nsec + offset_nsec

    if new_total < 0:
        new_total = 0

    stamp.sec = new_total // 1_000_000_000
    stamp.nanosec = new_total % 1_000_000_000
    return stamp


class TimeShiftRepublisher(Node):
    def __init__(self):
        super().__init__("time_shift_republisher")

        self.offset = self.declare_parameter("offset_sec", 0.0).value
        self.input_topic = self.declare_parameter("input_topic", "/livox/lidar").value
        self.output_topic = self.declare_parameter("output_topic", "/livox/lidar_fixed").value
        self.msg_type_name = self.declare_parameter("msg_type", "PointCloud2").value

        msg_type_map = {
            "PointCloud2": PointCloud2,
            "Imu": Imu,
            "Image": Image,
            "CameraInfo": CameraInfo,
            "Odometry": Odometry,
        }

        if self.msg_type_name not in msg_type_map:
            raise ValueError(f"Unsupported msg_type: {self.msg_type_name}")

        self.msg_type = msg_type_map[self.msg_type_name]

        self.pub = self.create_publisher(self.msg_type, self.output_topic, 10)
        self.sub = self.create_subscription(
            self.msg_type,
            self.input_topic,
            self.callback,
            10
        )

        self.get_logger().info(
            f"Republishing {self.input_topic} -> {self.output_topic}, "
            f"type={self.msg_type_name}, offset={self.offset} sec"
        )

    def callback(self, msg):
        if hasattr(msg, "header"):
            msg.header.stamp = shift_stamp(msg.header.stamp, self.offset)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TimeShiftRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
