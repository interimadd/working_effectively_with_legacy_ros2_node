#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/humble/setup.bash
source install/setup.bash

# rosbag play (loop)
ros2 bag play rosbag/sample-rosbag --loop \
  --remap /point_cloud:=/input/pointcloud &
BAG_PID=$!

# crop box filter node
ros2 run pointcloud_crop_filter pointcloud_crop_filter_node --ros-args \
  -p input_pointcloud_frame:=d435_depth_optical_frame \
  -p crop_box_frame:=d435_depth_optical_frame \
  -p min_x:=-0.3 -p min_y:=-0.3 -p min_z:=0.0 \
  -p max_x:=0.3 -p max_y:=0.3 -p max_z:=1.5 \
  -p keep_outside:=false \
  --remap input:=/input/pointcloud \
  --remap output:=/output/pointcloud &
NODE_PID=$!

# rviz2
rviz2 -d crop_box_filter_node/config/demo.rviz &
RVIZ_PID=$!

# Ctrl+C で全プロセスを終了
cleanup() {
  kill $BAG_PID $NODE_PID $RVIZ_PID 2>/dev/null
  wait $BAG_PID $NODE_PID $RVIZ_PID 2>/dev/null
}
trap cleanup EXIT INT TERM

wait $RVIZ_PID
