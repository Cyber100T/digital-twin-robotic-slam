#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_stamp(ns):
    sec = ns // 1_000_000_000
    nanosec = ns % 1_000_000_000
    return int(sec), int(nanosec)


class OdomAutoSyncTF(Node):
    def __init__(self):
        super().__init__('odom_auto_sync_tf')

        self.ref_stamp_ns = None
        self.odom_first_stamp_ns = None
        self.offset_ns = None

        self.br = TransformBroadcaster(self)

        self.ref_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.ref_callback,
            qos_profile_sensor_data
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/utlidar/robot_odom_synced',
            qos_profile_sensor_data
        )

        self.get_logger().info(
            'Auto-sync odom using /livox/lidar as time reference.'
        )

    def ref_callback(self, msg):
        if self.ref_stamp_ns is None:
            self.ref_stamp_ns = stamp_to_ns(msg.header.stamp)
            self.get_logger().info(
                f'Reference LiDAR stamp set: {self.ref_stamp_ns / 1e9:.6f}'
            )

    def odom_callback(self, msg):
        if self.ref_stamp_ns is None:
            return

        if self.offset_ns is None:
            self.odom_first_stamp_ns = stamp_to_ns(msg.header.stamp)
            self.offset_ns = self.ref_stamp_ns - self.odom_first_stamp_ns

            self.get_logger().warn(
                f'Computed odom time offset: {self.offset_ns / 1e9:.6f} sec'
            )

        old_ns = stamp_to_ns(msg.header.stamp)
        new_ns = old_ns + self.offset_ns

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        sec, nanosec = ns_to_stamp(new_ns)
        out.header.stamp.sec = sec
        out.header.stamp.nanosec = nanosec

        # 保证 frame 名干净
        out.header.frame_id = out.header.frame_id.lstrip('/')
        out.child_frame_id = out.child_frame_id.lstrip('/')

        self.odom_pub.publish(out)

        # 同时发布 odom -> base_link TF
        t = TransformStamped()
        t.header.stamp = out.header.stamp
        t.header.frame_id = out.header.frame_id
        t.child_frame_id = out.child_frame_id

        t.transform.translation.x = out.pose.pose.position.x
        t.transform.translation.y = out.pose.pose.position.y
        t.transform.translation.z = out.pose.pose.position.z
        t.transform.rotation = out.pose.pose.orientation

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomAutoSyncTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
