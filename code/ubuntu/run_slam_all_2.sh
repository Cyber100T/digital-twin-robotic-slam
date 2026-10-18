#!/bin/bash

# ===== 配置区域 =====
BAG_PATH="/mnt/hgfs/Ubuntu_share/test1/thesis_new61"
#BAG_PATH="~/Desktop/thesisdata/thesis_new61"
RATE="0.5"

# 👉 输出路径：在当前 bag 文件夹下面创建 output

SAVE_DIR="$BAG_PATH/output"
mkdir -p "$SAVE_DIR"

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

# ===== BAG PLAY + AUTO MAP SAVE =====
gnome-terminal -- bash -c "
echo '=== BAG PLAY + AUTO MAP SAVE ===';
source /opt/ros/humble/setup.bash;
source ~/fastlio_ws/install/setup.bash;

cd \"$BAG_PATH\";

echo '';
echo 'Playing bag with rate: $RATE';
echo '';

ros2 bag play . --rate $RATE;

echo '';
echo '=== BAG PLAY FINISHED OR STOPPED ===';
echo 'Waiting 5 seconds for FAST-LIO to process remaining messages...';
sleep 5;

echo '';
echo '=== AUTO SAVING FAST-LIO MAP ===';
mkdir -p \"$SAVE_DIR\";
cd \"$SAVE_DIR\";

echo 'Saving map to:';
pwd;

ros2 run map_saver save_laser_map;

echo '';
cp /home/jerry/fastlio_ws/output_pcd/laser_map.pcd "$SAVE_DIR/"

echo 'Map saver command finished.';
echo 'Please check: $SAVE_DIR';

exec bash"
sleep 3
