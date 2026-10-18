#!/usr/bin/env python3

import os
import struct
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class LaserMapSaver(Node):
    def __init__(self):
        super().__init__('laser_map_saver')

        self.declare_parameter('topic', '/Laser_map')
        self.declare_parameter('output', '/home/jerry/fastlio_ws/output_pcd/laser_map.pcd')

        topic = self.get_parameter('topic').value
        self.output = self.get_parameter('output').value

        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self.callback,
            10
        )

        self.saved = False
        self.get_logger().info(f'Listening on {topic}, will save to {self.output}')

    def callback(self, msg: PointCloud2):
        if self.saved:
            return

        points = self.pointcloud2_to_xyz(msg)
        if len(points) == 0:
            self.get_logger().warn('Received empty point cloud, skipping.')
            return

        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        self.write_ascii_pcd(self.output, points)

        self.get_logger().info(f'Saved {len(points)} points to {self.output}')
        self.saved = True

        # 保存一次就退出
        rclpy.shutdown()

    def pointcloud2_to_xyz(self, cloud: PointCloud2) -> List[Tuple[float, float, float]]:
        field_names = [f.name for f in cloud.fields]
        if not all(name in field_names for name in ['x', 'y', 'z']):
            self.get_logger().error('PointCloud2 does not contain x/y/z fields.')
            return []

        field_offsets = {f.name: f.offset for f in cloud.fields}
        point_step = cloud.point_step
        data = cloud.data

        points = []
        for i in range(cloud.width * cloud.height):
            base = i * point_step
            x = struct.unpack_from('f', data, base + field_offsets['x'])[0]
            y = struct.unpack_from('f', data, base + field_offsets['y'])[0]
            z = struct.unpack_from('f', data, base + field_offsets['z'])[0]

            if not any(map(lambda v: v != v or abs(v) > 1e6, [x, y, z])):  # 简单过滤 nan/异常
                points.append((x, y, z))
        return points

    def write_ascii_pcd(self, filename: str, points: List[Tuple[float, float, float]]):
        with open(filename, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z\n')
            f.write('SIZE 4 4 4\n')
            f.write('TYPE F F F\n')
            f.write('COUNT 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            for x, y, z in points:
                f.write(f'{x} {y} {z}\n')


def main(args=None):
    rclpy.init(args=args)
    node = LaserMapSaver()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
