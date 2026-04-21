#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/.."

source /opt/ros/humble/setup.bash
colcon build --packages-select pointcloud_crop_filter
