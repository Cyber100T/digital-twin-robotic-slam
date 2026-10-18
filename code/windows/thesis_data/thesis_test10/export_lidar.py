import os
import csv
import json
import numpy as np
from PIL import Image

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


bag_path = "."

TOPIC_LIDAR = "/livox/lidar"
TOPIC_IMU = "/livox/imu"
TOPIC_ODOM = "/utlidar/robot_odom"
TOPIC_TF = "/tf"
TOPIC_TF_STATIC = "/tf_static"

TOPIC_COLOR_IMAGE = "/camera/color/image_raw"
TOPIC_COLOR_INFO = "/camera/color/camera_info"

TOPIC_DEPTH_IMAGE = "/camera/depth/image_rect_raw"
TOPIC_DEPTH_INFO = "/camera/depth/camera_info"

max_lidar_frames = None
max_color_frames = None
max_depth_frames = None


def open_bag(path):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)
    return reader


def get_topics(reader):
    topics = reader.get_all_topics_and_types()
    return {t.name: t.type for t in topics}


def ensure_dirs():
    os.makedirs("lidar_frames", exist_ok=True)
    os.makedirs("camera_color", exist_ok=True)
    os.makedirs("camera_depth", exist_ok=True)


def pointcloud_msg_to_npy(msg):
    field_names = [f.name for f in msg.fields]

    if "intensity" in field_names:
        points = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True
        )
        pts = np.array([[p[0], p[1], p[2], p[3]] for p in points], dtype=np.float32)
    else:
        points = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )
        xyz = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        intensity = np.zeros((xyz.shape[0], 1), dtype=np.float32)
        pts = np.hstack([xyz, intensity])

    return pts


def image_msg_to_pil(msg):
    encoding = msg.encoding.lower()
    h, w = msg.height, msg.width
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding == "rgb8":
        arr = data.reshape((h, w, 3))
        return Image.fromarray(arr, "RGB")

    elif encoding == "bgr8":
        arr = data.reshape((h, w, 3))[:, :, ::-1]
        return Image.fromarray(arr, "RGB")

    elif encoding == "rgba8":
        arr = data.reshape((h, w, 4))
        return Image.fromarray(arr, "RGBA")

    elif encoding == "bgra8":
        arr = data.reshape((h, w, 4))[:, :, [2, 1, 0, 3]]
        return Image.fromarray(arr, "RGBA")

    elif encoding in ("mono8", "8uc1"):
        arr = data.reshape((h, w))
        return Image.fromarray(arr, "L")

    elif encoding == "16uc1":
        arr = data.reshape((h, w))
        return Image.fromarray(arr)  # 保存为16位PNG

    else:
        raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")


def save_depth_npy(msg, out_path):
    encoding = msg.encoding.lower()
    h, w = msg.height, msg.width

    if encoding == "16uc1":
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
        np.save(out_path, arr)

    elif encoding == "32fc1":
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
        np.save(out_path, arr)

    else:
        raise RuntimeError(f"Unsupported depth encoding: {msg.encoding}")


def tf_msg_to_dict(msg, bag_timestamp):
    out = []
    for t in msg.transforms:
        out.append({
            "bag_timestamp_ns": int(bag_timestamp),
            "header_stamp_sec": int(t.header.stamp.sec),
            "header_stamp_nanosec": int(t.header.stamp.nanosec),
            "header_frame_id": t.header.frame_id,
            "child_frame_id": t.child_frame_id,
            "translation": {
                "x": t.transform.translation.x,
                "y": t.transform.translation.y,
                "z": t.transform.translation.z,
            },
            "rotation": {
                "x": t.transform.rotation.x,
                "y": t.transform.rotation.y,
                "z": t.transform.rotation.z,
                "w": t.transform.rotation.w,
            }
        })
    return out


def camera_info_to_row(msg, bag_timestamp):
    return [
        int(bag_timestamp),
        int(msg.header.stamp.sec),
        int(msg.header.stamp.nanosec),
        msg.header.frame_id,
        msg.height,
        msg.width,
        msg.distortion_model,
        *list(msg.d),
        *list(msg.k),
        *list(msg.r),
        *list(msg.p)
    ]


def main():
    ensure_dirs()

    reader = open_bag(bag_path)
    topic_types = get_topics(reader)

    required_topics = [
        TOPIC_LIDAR, TOPIC_IMU, TOPIC_ODOM, TOPIC_TF, TOPIC_TF_STATIC,
        TOPIC_COLOR_IMAGE, TOPIC_COLOR_INFO,
        TOPIC_DEPTH_IMAGE, TOPIC_DEPTH_INFO
    ]

    print("Available topics:")
    for k, v in topic_types.items():
        print(f"  {k}: {v}")

    msg_type_map = {}
    for t in required_topics:
        if t in topic_types:
            msg_type_map[t] = get_message(topic_types[t])
        else:
            print(f"[WARN] Topic not found: {t}")

    lidar_idx = 0
    color_idx = 0
    depth_idx = 0

    imu_rows = []
    odom_rows = []
    tf_records = []
    tf_static_records = []
    lidar_timestamps = []
    color_timestamps = []
    depth_timestamps = []
    color_info_rows = []
    depth_info_rows = []

    print("Exporting data...")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic not in msg_type_map:
            continue

        msg = deserialize_message(data, msg_type_map[topic])

        if topic == TOPIC_LIDAR:
            pts = pointcloud_msg_to_npy(msg)
            np.save(f"lidar_frames/frame_{lidar_idx:06d}.npy", pts)
            lidar_timestamps.append([
                lidar_idx,
                int(timestamp),
                int(msg.header.stamp.sec),
                int(msg.header.stamp.nanosec),
                msg.header.frame_id
            ])
            lidar_idx += 1

            if lidar_idx % 100 == 0:
                print(f"Lidar: {lidar_idx} frames exported")

            if (max_lidar_frames is not None) and (lidar_idx >= max_lidar_frames):
                break

        elif topic == TOPIC_IMU:
            imu_rows.append([
                int(timestamp),
                int(msg.header.stamp.sec),
                int(msg.header.stamp.nanosec),
                msg.header.frame_id,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ])

        elif topic == TOPIC_ODOM:
            odom_rows.append([
                int(timestamp),
                int(msg.header.stamp.sec),
                int(msg.header.stamp.nanosec),
                msg.header.frame_id,
                msg.child_frame_id,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ])

        elif topic == TOPIC_TF:
            tf_records.extend(tf_msg_to_dict(msg, timestamp))

        elif topic == TOPIC_TF_STATIC:
            tf_static_records.extend(tf_msg_to_dict(msg, timestamp))

        elif topic == TOPIC_COLOR_IMAGE:
            img = image_msg_to_pil(msg)
            img.save(f"camera_color/frame_{color_idx:06d}.png")
            color_timestamps.append([
                color_idx,
                int(timestamp),
                int(msg.header.stamp.sec),
                int(msg.header.stamp.nanosec),
                msg.header.frame_id,
                msg.height,
                msg.width,
                msg.encoding
            ])
            color_idx += 1

            if color_idx % 100 == 0:
                print(f"Color image: {color_idx} frames exported")

            if (max_color_frames is not None) and (color_idx >= max_color_frames):
                pass

        elif topic == TOPIC_DEPTH_IMAGE:
            save_depth_npy(msg, f"camera_depth/frame_{depth_idx:06d}.npy")
            depth_timestamps.append([
                depth_idx,
                int(timestamp),
                int(msg.header.stamp.sec),
                int(msg.header.stamp.nanosec),
                msg.header.frame_id,
                msg.height,
                msg.width,
                msg.encoding
            ])
            depth_idx += 1

            if depth_idx % 100 == 0:
                print(f"Depth image: {depth_idx} frames exported")

            if (max_depth_frames is not None) and (depth_idx >= max_depth_frames):
                pass

        elif topic == TOPIC_COLOR_INFO:
            color_info_rows.append(camera_info_to_row(msg, timestamp))

        elif topic == TOPIC_DEPTH_INFO:
            depth_info_rows.append(camera_info_to_row(msg, timestamp))

    np.savetxt(
        "lidar_frames/timestamps.csv",
        np.array(lidar_timestamps, dtype=object),
        fmt="%s",
        delimiter=",",
        header="frame_id,bag_timestamp_ns,header_stamp_sec,header_stamp_nanosec,frame_id_msg",
        comments=""
    )

    with open("imu.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "bag_timestamp_ns", "header_stamp_sec", "header_stamp_nanosec", "frame_id",
            "gyro_x", "gyro_y", "gyro_z",
            "acc_x", "acc_y", "acc_z",
            "ori_x", "ori_y", "ori_z", "ori_w"
        ])
        writer.writerows(imu_rows)

    with open("odom.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "bag_timestamp_ns", "header_stamp_sec", "header_stamp_nanosec",
            "frame_id", "child_frame_id",
            "pos_x", "pos_y", "pos_z",
            "quat_x", "quat_y", "quat_z", "quat_w",
            "lin_x", "lin_y", "lin_z",
            "ang_x", "ang_y", "ang_z"
        ])
        writer.writerows(odom_rows)

    with open("tf.json", "w") as f:
        json.dump(tf_records, f, indent=2)

    with open("tf_static.json", "w") as f:
        json.dump(tf_static_records, f, indent=2)

    with open("camera_color/timestamps.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "frame_id", "bag_timestamp_ns",
            "header_stamp_sec", "header_stamp_nanosec",
            "frame_id_msg", "height", "width", "encoding"
        ])
        writer.writerows(color_timestamps)

    with open("camera_depth/timestamps.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "frame_id", "bag_timestamp_ns",
            "header_stamp_sec", "header_stamp_nanosec",
            "frame_id_msg", "height", "width", "encoding"
        ])
        writer.writerows(depth_timestamps)

    with open("camera_color/camera_info.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(color_info_rows)

    with open("camera_depth/camera_info.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(depth_info_rows)

    print("Finished.")
    print(f"Lidar frames: {lidar_idx}")
    print(f"Color image frames: {color_idx}")
    print(f"Depth image frames: {depth_idx}")
    print(f"IMU rows: {len(imu_rows)}")
    print(f"Odom rows: {len(odom_rows)}")
    print(f"TF records: {len(tf_records)}")
    print(f"TF static records: {len(tf_static_records)}")

if __name__ == "__main__":
    main()
