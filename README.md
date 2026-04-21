# Working effectively with legacy ROS2 Node

このパッケージは **レガシーコードのリファクタリング用サンプル** として作成されたROS2 Nodeのパッケージです。Nodeは、[crop_box_filter_node/src/pointcloud_crop_filter_node.cpp](crop_box_filter_node/src/pointcloud_crop_filter_node.cpp)に実装されており、リファクタリングの練習のために意図的に以下の問題を含んだ実装になっています。

- **Node とフィルタロジックの密結合**: フィルタ処理が `pointcloud_callback()` に直接記述されており、ノードを起動せずに単体テストできない
- **単体テストや結合テストがない**: 自動テストができず、rviz2で表示することでしか動作確認ができない
- **TF 変換ロジックの内包**: 座標変換もノードクラスのメンバとして管理されており、分離・差し替えが困難
- **低レベルなデータアクセス**: `memcpy` によるバイトバッファ直接操作がコールバック内に展開されている
- **パラメータのグルーピング不足**: CropBoxを構成する `min_x_` / `max_x_` / ... / `keep_outside_` が独立したメンバ変数としてノードクラスに平置きされており、関連パラメータの凝集度が低く、受け渡しや差し替えもしづらい

Nodeの機能としては、ROS2の点群データ (`sensor_msgs/PointCloud2`) に対して、バウンディングボックス内の点群をフィルタリングする機能を提供しています。

> [!NOTE]
> レポジトリ名は[レガシーコード改善ガイド](https://www.shoeisha.co.jp/book/detail/9784798116839)の原題 "Working Effectively with Legacy Code" のもじりです。


## 動作確認方法

### ビルド

```bash
source /opt/ros/humble/setup.bash
cd ~/practice/working_effectively_with_legacy_ros2_node
colcon build --packages-select pointcloud_crop_filter
```

### 起動

```bash
source install/setup.bash
ros2 run pointcloud_crop_filter pointcloud_crop_filter_node --ros-args \
  -p input_pointcloud_frame:=base_link \
  -p crop_box_frame:=base_link \
  --remap input:=/your/pointcloud \
  --remap output:=/filtered/pointcloud
```

### rviz2 での表示

サンプルの rosbag を再生しながらノードと rviz2 を起動して、フィルタ結果を可視化できます。

```bash
# ビルド後に実行
./debug_with_rviz2.sh
```

rviz2 上には以下が表示されます:

- **Input PointCloud** — 入力点群全体 (Z軸カラー、半透明)
- **Output PointCloud (Filtered)** — クロップボックスでフィルタされた点群 (緑)
- **Crop Box** — フィルタ領域の境界 (赤ポリゴン)

![rviz2 visualization result](rosbag/rviz2-visualization-result.gif)


## テストの実行

[crop_box_filter_node/test/test_pointcloud_crop_filter_node.cpp](crop_box_filter_node/test/test_pointcloud_crop_filter_node.cpp) のテストは `colcon test` で実行できます。

```bash
source /opt/ros/humble/setup.bash
cd ~/practice/working_effectively_with_legacy_ros2_node
colcon build --packages-select pointcloud_crop_filter
colcon test --packages-select pointcloud_crop_filter --event-handlers console_direct+
colcon test-result --verbose
```


## テストカバレッジの計測

[lcov](https://github.com/linux-test-project/lcov) を用いて、テストのラインカバレッジを計測できます。事前に `lcov` をインストールしてください。

```bash
sudo apt install lcov
```

### 計測手順

1. カバレッジ計測用のフラグを付けてビルド

    ```bash
    source /opt/ros/humble/setup.bash
    cd ~/practice/working_effectively_with_legacy_ros2_node
    colcon build --symlink-install \
      --cmake-args -DBUILD_TESTING=ON \
        -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage -O0 -g' \
        -DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage -O0 -g' \
      --packages-select pointcloud_crop_filter
    ```

2. テストを実行してカバレッジデータ (`.gcda`) を生成

    ```bash
    colcon test --packages-select pointcloud_crop_filter --event-handlers console_direct+
    ```

3. `lcov` でカバレッジ情報を収集し、パッケージのソースのみを抽出 (テストコード自体は除外)

    ```bash
    lcov --capture --directory build/pointcloud_crop_filter --output-file coverage.info
    lcov --extract coverage.info "*/crop_box_filter_node/*" --output-file coverage.info
    lcov --remove  coverage.info "*/test/*"                 --output-file coverage.info
    ```

4. サマリーを表示

    ```bash
    lcov --summary coverage.info
    ```

5. HTMLレポートを生成してブラウザで確認

    ```bash
    genhtml coverage.info --output-directory coverage_html
    xdg-open coverage_html/index.html
    ```

> [!TIP]
> カバレッジフラグなしでビルドし直したい場合は `rm -rf build/ install/ log/` でクリーンビルドしてください。同じビルドディレクトリに `.gcno` / `.gcda` が残っていると計測結果がずれます。


## Nodeの詳細

| 項目 | 値 |
|---|---|
| ノード名 | `pointcloud_crop_filter` |
| Subscribe | `input` (`sensor_msgs/PointCloud2`) |
| Publish | `output` (`sensor_msgs/PointCloud2`) |
| Publish | `~/crop_box_polygon` (`geometry_msgs/PolygonStamped`) |

### パラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
|---|---|---|---|
| `input_pointcloud_frame` | string | `"base_link"` | 入力点群のTFフレーム |
| `crop_box_frame` | string | `"base_link"` | クロップボックスを定義するTFフレーム |
| `max_queue_size` | int | `5` | サブスクリプションのキューサイズ |
| `min_x` | double | `-50.0` | ボックスのX最小値 [m] |
| `min_y` | double | `-50.0` | ボックスのY最小値 [m] |
| `min_z` | double | `-50.0` | ボックスのZ最小値 [m] |
| `max_x` | double | `50.0` | ボックスのX最大値 [m] |
| `max_y` | double | `50.0` | ボックスのY最大値 [m] |
| `max_z` | double | `50.0` | ボックスのZ最大値 [m] |
| `keep_outside` | bool | `false` | `true` の場合、ボックス **外側** の点群を保持する |

### 処理の流れ

1. `input` トピックから `PointCloud2` メッセージを受信
2. `input_pointcloud_frame` と `crop_box_frame` が異なる場合、TFを用いて座標変換を適用
3. 各点の座標を `memcpy` でバイトバッファから直接読み取り、ボックス範囲内かを判定
4. 条件に合致する点のみ `memcpy` で出力バッファにコピー
5. `output` トピックにフィルタ済み点群を publish
6. `~/crop_box_polygon` トピックにクロップボックスの可視化用ポリゴンを publish
