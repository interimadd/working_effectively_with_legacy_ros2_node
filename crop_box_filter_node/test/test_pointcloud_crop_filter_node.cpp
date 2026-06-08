#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "pointcloud_crop_filter/pointcloud_crop_filter_node.hpp"

using pointcloud_crop_filter::PointCloudCropFilterNode;
using sensor_msgs::msg::PointCloud2;

using Point = std::array<float, 3>;

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
  }

  void TearDown() override
  {
    executor_.reset();
    output_subscription_.reset();
    pointcloud_publisher_.reset();
    static_tf_broadcaster_.reset();
    test_node_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void initialize_filter_node()
  {
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("input_pointcloud_frame", std::string("lidar_top"));
    node_options.append_parameter_override("crop_box_frame", std::string("base_link"));
    node_options.append_parameter_override("min_x", -5.0);
    node_options.append_parameter_override("min_y", -5.0);
    node_options.append_parameter_override("min_z", -5.0);
    node_options.append_parameter_override("max_x", 5.0);
    node_options.append_parameter_override("max_y", 5.0);
    node_options.append_parameter_override("max_z", 5.0);
    node_options.append_parameter_override("keep_outside", false);

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

  PointCloud2::SharedPtr received_pointcloud_;

private:
  // The node's pub/sub use best-effort QoS without durability, so a message published
  // before discovery completes is silently lost. Wait until both directions are matched.
  void wait_until_pub_sub_matched(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    auto start = std::chrono::steady_clock::now();
    while (pointcloud_publisher_->get_subscription_count() == 0 ||
      output_subscription_->get_publisher_count() == 0)
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

/// Integration check of the node's wiring, including the TF lookup it performs at
/// construction. The filtering logic itself is covered by the unit tests in
/// test_pointcloud_crop_filter.cpp; here we verify the node-side glue the unit tests
/// cannot reach -- that the node looks the input->crop_box transform up from TF and feeds
/// it to the filter.
///
///   base_link x:  -5     0     5     10
///                  [== crop box ==]
///   lidar_top x:  -15   -10    -5     0
///
/// With the static TF base_link <- lidar_top = (+10, 0, 0), an input point at x=-10 in
/// lidar_top maps to the box centre (x=0) in base_link and is kept. Without the looked-up
/// transform (i.e. the identity fallback) the same point lies outside the box and would be
/// dropped, so a kept point is proof the node obtained and applied the transform. The
/// output keeps the point's original (untransformed) lidar_top coordinates.
TEST_F(PointCloudCropFilterIntegrationTest, LooksUpTransformFromTfAndAppliesItBeforeFiltering)
{
  // Arrange
  publish_static_transform("base_link", "lidar_top", 10.0, 0.0, 0.0);
  initialize_filter_node();

  const Point inside_after_transform = {-10.0f, 0.0f, 0.0f};  // -> (0, 0, 0) in base_link

  // Act
  publish_pointcloud(make_pointcloud({inside_after_transform}, "lidar_top"));
  auto result = receive_published_pointcloud();

  // Assert
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->header.frame_id, "lidar_top");
  const std::vector<Point> expected = {inside_after_transform};
  EXPECT_EQ(extract_points(*result), expected);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
