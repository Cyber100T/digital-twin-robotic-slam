#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


class OdomTimeShift(Node):
    def __init__(self):
        super().__init__('odom_time_shift')

        self.declare_parameter('offset_sec', -24005.0)

        self.offset_sec = float(self.get_parameter('offset_sec').value)

        self.sub = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            Odometry,
            '/utlidar/robot_odom_synced',
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Republishing /utlidar/robot_odom -> /utlidar/robot_odom_synced with offset {self.offset_sec} sec'
        )

    def callback(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        old_sec = int(msg.header.stamp.sec)
        old_nsec = int(msg.header.stamp.nanosec)

        total_ns = old_sec * 1_000_000_000 + old_nsec + int(self.offset_sec * 1_000_000_000)

        if total_ns < 0:
            self.get_logger().warn('Shifted timestamp is negative, dropping message.')
            return

        out.header.stamp.sec = total_ns // 1_000_000_000
        out.header.stamp.nanosec = total_ns % 1_000_000_000

        self.pub.publish(out)


def main():
    rclpy.init()
    node = OdomTimeShift()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
