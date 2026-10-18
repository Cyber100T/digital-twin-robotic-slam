#!/bin/bash

# ===== 配置区域 =====
BAG_PATH="/mnt/hgfs/Ubuntu_share/test1/thesis_new21"
RATE="0.5"

# 👉 输出路径（Windows能看到）
SAVE_DIR="/mnt/hgfs/Ubuntu_share/test1/output"
mkdir -p $SAVE_DIR

# ===== IMU FIX =====
gnome-terminal -- bash -c "
echo '=== IMU FIX ===';
source /opt/ros/humble/setup.bash;
source ~/fastlio_ws/install/setup.bash;
ros2 run imu_scale_fix imu_scale_node;
exec bash"

# ===== FAST-LIO =====
gnome-terminal -- bash -c "
echo '=== FAST-LIO ===';
source /opt/ros/humble/setup.bash;
source ~/fastlio_ws/install/setup.bash;
cd ~/fastlio_ws;
ros2 launch fast_lio mapping.launch.py;
exec bash"

sleep 2

# ===== BAG PLAY =====
gnome-terminal -- bash -c "
echo '=== BAG PLAY ===';
source /opt/ros/humble/setup.bash;
cd \"$BAG_PATH\";
ros2 bag play . --rate $RATE;
exec bash"

sleep 3

# ===== 自动导出 FAST-LIO pose =====
gnome-terminal -- bash -c "
echo '=== EXPORT FAST-LIO TRAJECTORY ===';
source /opt/ros/humble/setup.bash;

python3 - <<EOF
import rclpy
from nav_msgs.msg import Odometry
import numpy as np

poses = []

def callback(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation

    # quaternion → rotation matrix
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    R = np.array([
        [1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
        [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw],
        [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy]
    ])

    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = [p.x, p.y, p.z]

    poses.append(T)

def main():
    rclpy.init()
    node = rclpy.create_node('pose_listener')

    node.create_subscription(
        Odometry,
        '/Odometry',   # ⚠️ FAST-LIO topic
        callback,
        100
    )

    print("Recording poses... Ctrl+C to save")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    poses_np = np.array(poses)
    np.save("$SAVE_DIR/trajectory_fastlio.npy", poses_np)

    print("Saved:", poses_np.shape)

    node.destroy_node()
    rclpy.shutdown()

main()
EOF

exec bash"

# ===== MAP SAVER =====
gnome-terminal -- bash -c "
echo '=== MAP SAVER ===';
source /opt/ros/humble/setup.bash;
source ~/fastlio_ws/install/setup.bash;

echo '';
echo '👉 When ready:';
echo '1. Pause bag';
echo '2. Run: ros2 run map_saver save_laser_map';
echo '';

exec bash"
