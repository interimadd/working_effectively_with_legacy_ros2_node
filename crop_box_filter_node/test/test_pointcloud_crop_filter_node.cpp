#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "pointcloud_crop_filter/pointcloud_crop_filter_node.hpp"

using geometry_msgs::msg::PolygonStamped;
using pointcloud_crop_filter::PointCloudCropFilterNode;
using sensor_msgs::msg::PointCloud2;

using Point = std::array<float, 3>;

struct FilterNodeOptions
{
  std::string input_pointcloud_frame = "base_link";
  std::string crop_box_frame = "base_link";
  double min_x = -5.0;
  double min_y = -5.0;
  double min_z = -5.0;
  double max_x = 5.0;
  double max_y = 5.0;
  double max_z = 5.0;
  bool keep_outside = false;
};

class PointCloudCropFilterIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);

    pointcloud_publisher_ = test_node_->create_publisher<PointCloud2>(
      "/input", rclcpp::SensorDataQoS().keep_last(5));

    output_subscription_ = test_node_->create_subscription<PointCloud2>(
      "/output", rclcpp::SensorDataQoS().keep_last(5),
      [this](const PointCloud2::SharedPtr message) {received_pointcloud_ = message;});

    polygon_subscription_ = test_node_->create_subscription<PolygonStamped>(
      "/pointcloud_crop_filter/crop_box_polygon", rclcpp::QoS(10),
      [this](const PolygonStamped::SharedPtr message) {received_polygon_ = message;});
  }

  void TearDown() override
  {
    executor_.reset();
    polygon_subscription_.reset();
    output_subscription_.reset();
    pointcloud_publisher_.reset();
    static_tf_broadcaster_.reset();
    test_node_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void initialize_filter_node(const FilterNodeOptions & options)
  {
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override(
      "input_pointcloud_frame", options.input_pointcloud_frame);
    node_options.append_parameter_override("crop_box_frame", options.crop_box_frame);
    node_options.append_parameter_override("min_x", options.min_x);
    node_options.append_parameter_override("min_y", options.min_y);
    node_options.append_parameter_override("min_z", options.min_z);
    node_options.append_parameter_override("max_x", options.max_x);
    node_options.append_parameter_override("max_y", options.max_y);
    node_options.append_parameter_override("max_z", options.max_z);
    node_options.append_parameter_override("keep_outside", options.keep_outside);

    node_ = std::make_shared<PointCloudCropFilterNode>(node_options);
    executor_->add_node(node_);

    wait_until_pub_sub_matched();
  }

  /// @brief Publish a static transform: a point in source_frame appears translated by
  ///        (x, y, z) when expressed in target_frame.
  void publish_static_transform(
    const std::string & target_frame, const std::string & source_frame, double x, double y,
    double z)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = test_node_->get_clock()->now();
    transform.header.frame_id = target_frame;
    transform.child_frame_id = source_frame;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    transform.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(transform);
    spin_for(std::chrono::milliseconds(100));
  }

  void publish_pointcloud(const PointCloud2 & pointcloud)
  {
    pointcloud_publisher_->publish(pointcloud);
  }

  PointCloud2::SharedPtr receive_published_pointcloud(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    auto start = std::chrono::steady_clock::now();
    while (!received_pointcloud_) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return nullptr;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return received_pointcloud_;
  }

  PolygonStamped::SharedPtr receive_published_polygon(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    auto start = std::chrono::steady_clock::now();
    while (!received_polygon_) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return nullptr;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return received_polygon_;
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::shared_ptr<PointCloudCropFilterNode> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Subscription<PointCloud2>::SharedPtr output_subscription_;
  rclcpp::Subscription<PolygonStamped>::SharedPtr polygon_subscription_;

  PointCloud2::SharedPtr received_pointcloud_;
  PolygonStamped::SharedPtr received_polygon_;

private:
  // The node's pub/sub use best-effort QoS without durability, so a message published
  // before discovery completes is silently lost. Wait until both directions are matched.
  void wait_until_pub_sub_matched(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    auto start = std::chrono::steady_clock::now();
    while (pointcloud_publisher_->get_subscription_count() == 0 ||
      output_subscription_->get_publisher_count() == 0 ||
      polygon_subscription_->get_publisher_count() == 0)
    {
      ASSERT_LT(std::chrono::steady_clock::now() - start, timeout)
        << "Timed out waiting for pub/sub matching with the filter node.";
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

PointCloud2 make_pointcloud(const std::vector<Point> & points, const std::string & frame_id)
{
  PointCloud2 pointcloud;
  pointcloud.header.frame_id = frame_id;
  pointcloud.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  pointcloud.height = 1;

  sensor_msgs::PointCloud2Modifier modifier(pointcloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");
  for (const auto & point : points) {
    *iter_x = point[0];
    *iter_y = point[1];
    *iter_z = point[2];
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return pointcloud;
}

std::vector<Point> extract_points(const PointCloud2 & pointcloud)
{
  std::vector<Point> points;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    points.push_back({*iter_x, *iter_y, *iter_z});
  }
  return points;
}

std::vector<Point> to_points(const PolygonStamped & polygon)
{
  std::vector<Point> points;
  for (const auto & point : polygon.polygon.points) {
    points.push_back({point.x, point.y, point.z});
  }
  return points;
}

bool is_same(const std::vector<Point> & actual, const std::vector<Point> & expected)
{
  if (actual.size() != expected.size()) {
    return false;
  }
  for (size_t i = 0; i < actual.size(); ++i) {
    if (actual[i] != expected[i]) {
      return false;
    }
  }
  return true;
}

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

/// When input and crop box share the same frame, no TF is required and points are
/// filtered against the box as-is.
TEST_F(PointCloudCropFilterIntegrationTest, SameFramePointsInsideCropBoxAreKept)
{
  // Arrange
  initialize_filter_node(FilterNodeOptions{});

  const Point inside_point = {1.0f, 2.0f, 3.0f};
  const Point outside_point = {6.0f, 0.0f, 0.0f};
  const std::vector<Point> input_points = {inside_point, outside_point};
  const std::vector<Point> expected_points = {inside_point};

  // Act
  publish_pointcloud(make_pointcloud(input_points, "base_link"));
  auto result = receive_published_pointcloud();

  // Assert
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->header.frame_id, "base_link");
  EXPECT_TRUE(is_same(extract_points(*result), expected_points));
}

/// With keep_outside=true the filter is inverted: points outside the box are kept and
/// points inside are dropped.
TEST_F(PointCloudCropFilterIntegrationTest, KeepOutsideRetainsOnlyPointsOutsideCropBox)
{
  // Arrange
  FilterNodeOptions options;
  options.keep_outside = true;
  initialize_filter_node(options);

  const Point inside_point = {1.0f, 2.0f, 3.0f};
  const Point outside_in_x = {6.0f, 0.0f, 0.0f};
  const Point outside_in_y = {0.0f, -7.0f, 0.0f};
  const std::vector<Point> input_points = {inside_point, outside_in_x, outside_in_y};
  const std::vector<Point> expected_points = {outside_in_x, outside_in_y};

  // Act
  publish_pointcloud(make_pointcloud(input_points, "base_link"));
  auto result = receive_published_pointcloud();

  // Assert
  ASSERT_NE(result, nullptr);
  EXPECT_TRUE(is_same(extract_points(*result), expected_points));
}

/// - input_pointcloud_frame (lidar_top) != crop_box_frame (base_link), and the static TF
///   base_link <- lidar_top is available before node construction, so every point goes
///   through the preprocess transform branch.
/// - keep_outside=false with points inside the box, so the copy-to-output branch runs.
/// - one NaN point exercises the non-finite skip branch and its warning.
///
///   base_link x:  -5          0          5      10
///               ---+----------+----------+------+---
///                  [ ==== crop box ==== ]
///   lidar_top x: -15        -10         -5       0
///                          inside              outside
///
///   Static TF: p_base_link = p_lidar_top + (10, 0, 0)
///   Output points keep their original lidar_top coordinates.
TEST_F(PointCloudCropFilterIntegrationTest, TransformedPointsInsideCropBoxAreKept)
{
  // Arrange
  publish_static_transform("base_link", "lidar_top", 10.0, 0.0, 0.0);
  FilterNodeOptions options;
  options.input_pointcloud_frame = "lidar_top";
  options.crop_box_frame = "base_link";
  initialize_filter_node(options);

  const Point inside_origin = {-10.0f, 0.0f, 0.0f};   // -> (0, 0, 0) in base_link
  const Point inside_offset = {-8.0f, 1.0f, 2.0f};    // -> (2, 1, 2) in base_link
  const Point outside_point = {0.0f, 0.0f, 0.0f};     // -> (10, 0, 0) in base_link
  const Point non_finite_point = {kNaN, 0.0f, 0.0f};  // skipped as non-finite
  const std::vector<Point> input_points = {
    inside_origin, inside_offset, outside_point, non_finite_point};
  const std::vector<Point> expected_points = {inside_origin, inside_offset};

  // Act
  publish_pointcloud(make_pointcloud(input_points, "lidar_top"));
  auto result = receive_published_pointcloud();

  // Assert
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->header.frame_id, "lidar_top");
  EXPECT_TRUE(is_same(extract_points(*result), expected_points));
}

/// Each callback also publishes the crop box outline as a PolygonStamped in the crop box
/// frame: the bottom rectangle first, then vertical edges interleaved with the top corners.
TEST_F(PointCloudCropFilterIntegrationTest, CropBoxPolygonOutlinesConfiguredBox)
{
  // Arrange
  publish_static_transform("crop_box_frame", "base_link", 0.0, 0.0, 0.0);
  FilterNodeOptions options;
  options.input_pointcloud_frame = "base_link";
  options.crop_box_frame = "crop_box_frame";
  initialize_filter_node(options);

  const std::vector<Point> expected_polygon_points = {
    {5.0f, 5.0f, -5.0f}, {-5.0f, 5.0f, -5.0f}, {-5.0f, -5.0f, -5.0f}, {5.0f, -5.0f, -5.0f},
    {5.0f, 5.0f, -5.0f}, {5.0f, 5.0f, 5.0f}, {-5.0f, 5.0f, 5.0f}, {-5.0f, 5.0f, -5.0f},
    {-5.0f, 5.0f, 5.0f}, {-5.0f, -5.0f, 5.0f}, {-5.0f, -5.0f, -5.0f}, {-5.0f, -5.0f, 5.0f},
    {5.0f, -5.0f, 5.0f}, {5.0f, -5.0f, -5.0f}, {5.0f, -5.0f, 5.0f}, {5.0f, 5.0f, 5.0f}};

  // Act
  publish_pointcloud(make_pointcloud({}, "base_link"));
  auto result = receive_published_polygon();

  // Assert
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->header.frame_id, "crop_box_frame");
  EXPECT_TRUE(is_same(to_points(*result), expected_polygon_points));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
