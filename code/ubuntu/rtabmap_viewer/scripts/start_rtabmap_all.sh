#!/bin/bash

echo "=== Starting RTAB-Map Pipeline ==="

BAG_PATH="~/Desktop/thesis_new11"
BAG_RATE="0.1"

RVIZ_CONFIG="$HOME/rtabmap_viewer/config/rtabmap_livox_view.rviz"

# topics
SCAN_TOPIC="/livox/lidar"
RGB_TOPIC="/camera/color/image_raw"
ODOM_TOPIC="/odom"

# frames
BASE_FRAME="base_link"
LIDAR_FRAME="livox_frame"
CAMERA_FRAME="camera_link"

#echo "Step 1: Static TF (LiDAR)"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 $BASE_FRAME $LIDAR_FRAME;
exec bash" &

sleep 1

echo "Step 2: Static TF (Camera)"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
ros2 run tf2_ros static_transform_publisher 0.10 0 0.30 0 0 0 $BASE_FRAME $CAMERA_FRAME;
exec bash" &

sleep 1

echo "Step 3: RTAB-Map"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  rtabmap_viz:=false \
  rviz:=false \
  frame_id:=base_link \
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
  topic_queue_size:=50 \
  sync_queue_size:=50 \
  qos_scan:=2 \
  rtabmap_args:="--delete_db_on_start \
                 --Grid/CellSize 0.05 \
                 --Cloud/VoxelSize 0.05 \
                 --tf_delay 0.1 \
                 --tf_tolerance 0.2
                 --Rtabmap/DetectionRate 5.0 \
                 --RGBD/LinearUpdate 0.02 \
                 --RGBD/AngularUpdate 0.02 \
                 --RGBD/OptimizeFromGraphEnd false \
                 --Rtabmap/TimeThr 0"
exec bash" &

sleep 3

echo "Step 4: RViz"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
rviz2 -d $RVIZ_CONFIG;
exec bash" &

sleep 2

echo "Step 5: Play bag"
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
ros2 bag play $BAG_PATH --clock --rate $BAG_RATE;
exec bash" &

echo "=== All processes launched ==="
