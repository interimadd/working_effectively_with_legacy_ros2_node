#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/.."

source /opt/ros/humble/setup.bash

# 1. カバレッジ計測用のフラグを付けてビルド
colcon build --symlink-install \
  --cmake-args -DBUILD_TESTING=ON \
    -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage -O0 -g' \
    -DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage -O0 -g' \
  --packages-select pointcloud_crop_filter

# 2. テストを実行してカバレッジデータ (.gcda) を生成
colcon test --packages-select pointcloud_crop_filter --event-handlers console_direct+

# 3. lcov でカバレッジ情報を収集し、パッケージのソースのみを抽出 (テストコード自体は除外)
lcov --capture --directory build/pointcloud_crop_filter --output-file coverage.info
lcov --extract coverage.info "*/crop_box_filter_node/*" --output-file coverage.info
lcov --remove  coverage.info "*/test/*"                 --output-file coverage.info

# 4. サマリーを表示
lcov --summary coverage.info

# 5. HTMLレポートを生成してブラウザで確認
genhtml coverage.info --output-directory coverage_html
xdg-open coverage_html/index.html
