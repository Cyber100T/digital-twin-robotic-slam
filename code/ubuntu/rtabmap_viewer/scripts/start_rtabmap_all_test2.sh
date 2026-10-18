#!/bin/bash

echo "=== Starting RTAB-Map Pipeline ==="

BAG_PATH="$HOME/Desktop/thesis_new15"
BAG_RATE="0.1"
RVIZ_CONFIG="$HOME/rtabmap_viewer/config/rtabmap_livox_view.rviz"
ODOM_SHIFT_NODE="$HOME/rtabmap_tools/odom_time_shift.py"
ODOM_OFFSET="-24003.0"

pkill -f "rtabmap_slam rtabmap" || true
pkill -f "rtabmap_launch" || true
pkill -f "rgbd_odometry" || true
pkill -f "rtabmap_viz" || true
pkill -f "rviz2" || true
pkill -f "ros2 bag play" || true
pkill -f "static_transform_publisher" || true
pkill -f "odom_time_shift.py" || true

sleep 2

if [ ! -d "$BAG_PATH" ]; then
  echo "ERROR: Bag path not found: $BAG_PATH"
  exit 1
fi

echo "Step 1: Odom time shift"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
python3 $ODOM_SHIFT_NODE --ros-args -p use_sim_time:=true -p offset_sec:=$ODOM_OFFSET;
exec bash" &

sleep 2
echo "Step X: Odom synced to TF"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
python3 $HOME/rtabmap_tools/odom_to_tf.py --ros-args -p use_sim_time:=true;
exec bash" &

sleep 1

echo "Step 2: RTAB-Map"
gnome-terminal -- bash -c '
source /opt/ros/humble/setup.bash;
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  rtabmap_viz:=false \
  rviz:=false \
  visual_odometry:=false \
  icp_odometry:=false \
  frame_id:=base_link \
  odom_frame_id:=odom \
  odom_topic:=/utlidar/robot_odom_synced \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/color/camera_info \
  subscribe_rgb:=true \
  subscribe_depth:=true \
  subscribe_scan:=false \
  subscribe_scan_cloud:=true \
  scan_cloud_topic:=/livox/lidar \
  approx_sync:=true \
  approx_sync_max_interval:=0.2 \
  topic_queue_size:=200 \
  sync_queue_size:=200 \
  qos_scan:=2 \
  rtabmap_args:="--delete_db_on_start \
                 --Grid/CellSize 0.05 \
                 --Cloud/VoxelSize 0.05 \
                 --tf_delay 0.1 \
                 --tf_tolerance 0.2 \
                 --Rtabmap/DetectionRate 2.0 \
                 --RGBD/LinearUpdate 0.02 \
                 --RGBD/AngularUpdate 0.02 \
                 --RGBD/OptimizeFromGraphEnd false \
                 --Rtabmap/TimeThr 0"
exec bash' &

sleep 3

echo "Step 3: RViz"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
rviz2 -d $RVIZ_CONFIG --ros-args -p use_sim_time:=true;
exec bash" &

sleep 2

echo "Step 4: Play bag"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
ros2 bag play $BAG_PATH --clock --rate $BAG_RATE \
  --remap /tf:=/tf_bag_unused /tf_static:=/tf_static_bag_unused
exec bash" &

# 关键：等 /clock 开始发布后，再启动静态TF
sleep 5

echo "Step 5: Static TF - LiDAR"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
ros2 run tf2_ros static_transform_publisher \
0 0 0 0 0 0 base_link livox_frame \
--ros-args -p use_sim_time:=true;
exec bash" &

sleep 1

echo "Step 6: Static TF - RGB optical"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
ros2 run tf2_ros static_transform_publisher \
0.10 0 0.30 -1.571 0 -1.571 base_link camera_color_optical_frame \
--ros-args -p use_sim_time:=true;
exec bash" &

sleep 1

echo "Step 7: Static TF - Depth optical"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
ros2 run tf2_ros static_transform_publisher \
0.10 0 0.30 -1.571 0 -1.571 base_link camera_depth_optical_frame \
--ros-args -p use_sim_time:=true;
exec bash" &

echo "=== All processes launched ==="
