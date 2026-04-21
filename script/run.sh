#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/.."

source install/setup.bash
ros2 run pointcloud_crop_filter pointcloud_crop_filter_node --ros-args \
  -p input_pointcloud_frame:=base_link \
  -p crop_box_frame:=base_link \
  -p min_x:=-50.0 -p min_y:=-50.0 -p min_z:=-50.0 \
  -p max_x:=50.0  -p max_y:=50.0  -p max_z:=50.0 \
  -p keep_outside:=false \
  --remap input:=/your/pointcloud \
  --remap output:=/filtered/pointcloud
