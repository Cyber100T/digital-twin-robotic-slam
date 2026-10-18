#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuScaleNode(Node):
    def __init__(self):
        super().__init__('imu_scale_node')

        self.declare_parameter('input_topic', '/livox/imu')
        self.declare_parameter('output_topic', '/imu_scaled')
        self.declare_parameter('scale', 9.81)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.scale = float(self.get_parameter('scale').value)

        self.sub = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            200
        )

        self.pub = self.create_publisher(
            Imu,
            output_topic,
            200
        )

        self.get_logger().info(
            f'IMU scaling node started: {input_topic} -> {output_topic}, scale={self.scale}'
        )

    def imu_callback(self, msg: Imu):
        out = Imu()

        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration.x = msg.linear_acceleration.x * self.scale
        out.linear_acceleration.y = msg.linear_acceleration.y * self.scale
        out.linear_acceleration.z = msg.linear_acceleration.z * self.scale
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuScaleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
