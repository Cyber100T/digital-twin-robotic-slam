#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2

class RGBResizeRepublisher(Node):
    def __init__(self):
        super().__init__('rgb_resize_republisher')

        

        self.bridge = CvBridge()
        self.target_width = 848
        self.target_height = 480
        self.last_camera_info = None

        self.sub_img = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.img_callback,
            qos_profile_sensor_data
        )

        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.info_callback,
            qos_profile_sensor_data
        )

        self.pub_img = self.create_publisher(
            Image,
            '/camera/color/image_resized',
            qos_profile_sensor_data
        )

        self.pub_info = self.create_publisher(
            CameraInfo,
            '/camera/color/camera_info_resized',
            qos_profile_sensor_data
        )

    def info_callback(self, msg: CameraInfo):
        self.last_camera_info = msg

    def img_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        resized = cv2.resize(
            cv_img,
            (self.target_width, self.target_height),
            interpolation=cv2.INTER_LINEAR
        )

        out_img = self.bridge.cv2_to_imgmsg(resized, encoding=msg.encoding)
        out_img.header = msg.header
        self.pub_img.publish(out_img)

        if self.last_camera_info is not None:
            info = CameraInfo()
            info.header = msg.header
            info.height = self.target_height
            info.width = self.target_width
            info.distortion_model = self.last_camera_info.distortion_model
            info.d = list(self.last_camera_info.d)

            sx = self.target_width / float(self.last_camera_info.width)
            sy = self.target_height / float(self.last_camera_info.height)

            k = list(self.last_camera_info.k)
            k[0] *= sx
            k[2] *= sx
            k[4] *= sy
            k[5] *= sy
            info.k = k

            p = list(self.last_camera_info.p)
            p[0] *= sx
            p[2] *= sx
            p[5] *= sy
            p[6] *= sy
            info.p = p

            info.r = list(self.last_camera_info.r)
            info.binning_x = self.last_camera_info.binning_x
            info.binning_y = self.last_camera_info.binning_y
            info.roi = self.last_camera_info.roi

            self.pub_info.publish(info)

def main():
    rclpy.init()
    node = RGBResizeRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
