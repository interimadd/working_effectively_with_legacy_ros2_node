#include <gtest/gtest.h>

#include "pointcloud_crop_filter/pointcloud_crop_filter_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::PolygonStamped;

namespace
{

PointCloud2 make_xyz_point_cloud(
  const std::vector<std::array<float, 3>> & points,
  const std::string & frame_id)
{
  PointCloud2 pc;
  pc.header.frame_id = frame_id;
  pc.header.stamp.sec = 1;
  pc.header.stamp.nanosec = 0;
  pc.height = 1;
  sensor_msgs::PointCloud2Modifier modifier(pc);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  if (!points.empty()) {
    sensor_msgs::PointCloud2Iterator<float> it_x(pc, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(pc, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(pc, "z");
    for (const auto & p : points) {
      *it_x = p[0];
      *it_y = p[1];
      *it_z = p[2];
      ++it_x;
      ++it_y;
      ++it_z;
    }
  }
  pc.is_dense = true;
  return pc;
}

std::vector<std::array<float, 3>> extract_xyz(const PointCloud2 & pc)
{
  std::vector<std::array<float, 3>> points;
  if (pc.width == 0 || pc.data.empty()) {
    return points;
  }
  sensor_msgs::PointCloud2ConstIterator<float> it_x(pc, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(pc, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(pc, "z");
  for (size_t i = 0; i < pc.width * pc.height; ++i, ++it_x, ++it_y, ++it_z) {
    points.push_back({*it_x, *it_y, *it_z});
  }
  return points;
}

struct BoxSize
{
  double min_x;
  double min_y;
  double min_z;
  double max_x;
  double max_y;
  double max_z;
};

struct CropBoxFilterOptions
{
  std::string input_frame;
  std::string crop_box_frame;
  BoxSize box_size;
  bool keep_outside;
};

rclcpp::NodeOptions make_options(const CropBoxFilterOptions & opts)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"input_pointcloud_frame", opts.input_frame},
      {"crop_box_frame", opts.crop_box_frame},
      {"min_x", opts.box_size.min_x},
      {"min_y", opts.box_size.min_y},
      {"min_z", opts.box_size.min_z},
      {"max_x", opts.box_size.max_x},
      {"max_y", opts.box_size.max_y},
      {"max_z", opts.box_size.max_z},
      {"keep_outside", opts.keep_outside},
    });
  return options;
}

struct Translation
{
  double x;
  double y;
  double z;
};

struct StaticTransform
{
  std::string parent_frame;
  std::string child_frame;
  Translation translation;
};

class CropBoxIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    helper_node_ = std::make_shared<rclcpp::Node>("test_helper");
    static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(*helper_node_);
  }

  void TearDown() override
  {
    filter_node_.reset();
    out_sub_.reset();
    poly_sub_.reset();
    static_tf_broadcaster_.reset();
    helper_node_.reset();
    rclcpp::shutdown();
  }

  geometry_msgs::msg::TransformStamped make_transform(const StaticTransform & tf)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = helper_node_->now();
    t.header.frame_id = tf.parent_frame;
    t.child_frame_id = tf.child_frame;
    t.transform.translation.x = tf.translation.x;
    t.transform.translation.y = tf.translation.y;
    t.transform.translation.z = tf.translation.z;
    t.transform.rotation.w = 1.0;
    return t;
  }

  void publish_transform(const geometry_msgs::msg::TransformStamped & t)
  {
    static_tf_broadcaster_->sendTransform(t);
  }

  struct Result
  {
    PointCloud2::SharedPtr pointcloud;
    PolygonStamped::SharedPtr polygon;
  };

  void create_crop_box_filter_node(const rclcpp::NodeOptions & options)
  {
    filter_node_ =
      std::make_shared<pointcloud_crop_filter::PointCloudCropFilterNode>(options);
  }

  void subscribe_published_topics()
  {
    out_sub_ = helper_node_->create_subscription<PointCloud2>(
      "/output", rclcpp::SensorDataQoS().keep_last(5),
      [this](PointCloud2::SharedPtr msg) {result_.pointcloud = msg;});
    poly_sub_ = helper_node_->create_subscription<PolygonStamped>(
      "/pointcloud_crop_filter/crop_box_polygon", 10,
      [this](PolygonStamped::SharedPtr msg) {result_.polygon = msg;});
  }

  void publish_input_pointcloud(
    const PointCloud2 & input_msg,
    std::chrono::milliseconds timeout = 3000ms)
  {
    auto in_pub = helper_node_->create_publisher<PointCloud2>(
      "/input", rclcpp::SensorDataQoS().keep_last(5));

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(helper_node_);
    exec.add_node(filter_node_);

    // Wait for pub/sub discovery.
    auto disc_deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < disc_deadline &&
      (in_pub->get_subscription_count() == 0 ||
      out_sub_->get_publisher_count() == 0 ||
      poly_sub_->get_publisher_count() == 0))
    {
      exec.spin_some();
      std::this_thread::sleep_for(20ms);
    }

    in_pub->publish(input_msg);

    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline &&
      (!result_.pointcloud || !result_.polygon))
    {
      exec.spin_some();
      std::this_thread::sleep_for(10ms);
    }

    exec.remove_node(filter_node_);
    exec.remove_node(helper_node_);

    EXPECT_NE(result_.pointcloud, nullptr) << "pointcloud was not received";
    EXPECT_NE(result_.polygon, nullptr) << "crop_box_polygon was not received";
  }

  Result run_crop_box_filter_node(
    const rclcpp::NodeOptions & options,
    const PointCloud2 & input_msg,
    std::chrono::milliseconds timeout = 3000ms)
  {
    create_crop_box_filter_node(options);
    subscribe_published_topics();
    publish_input_pointcloud(input_msg, timeout);
    return result_;
  }

  std::shared_ptr<rclcpp::Node> helper_node_;
  std::shared_ptr<pointcloud_crop_filter::PointCloudCropFilterNode> filter_node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Subscription<PointCloud2>::SharedPtr out_sub_;
  rclcpp::Subscription<PolygonStamped>::SharedPtr poly_sub_;
  Result result_;
};

TEST_F(CropBoxIntegrationTest, EmptyInputProducesEmptyOutput)
{
  // Arrange
  auto options = make_options({
    .input_frame = "sensor_frame",
    .crop_box_frame = "sensor_frame",
    .box_size = {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0},
    .keep_outside = false,
  });

  const std::vector<std::array<float, 3>> input_points = {};
  const std::vector<std::array<float, 3>> expected_points = {};
  auto input_msg = make_xyz_point_cloud(input_points, "sensor_frame");

  // Act
  auto result = run_crop_box_filter_node(options, input_msg);

  // Assert
  EXPECT_EQ(extract_xyz(*result.pointcloud), expected_points);
  EXPECT_EQ(result.pointcloud->height, 1u);
  EXPECT_EQ(result.pointcloud->header.frame_id, "sensor_frame");
  EXPECT_EQ(result.pointcloud->point_step, input_msg.point_step);
  EXPECT_EQ(result.pointcloud->fields.size(), input_msg.fields.size());
}

TEST_F(CropBoxIntegrationTest, FiltersOutsidePoints)
{
  // Arrange
  auto options = make_options({
    .input_frame = "sensor_frame",
    .crop_box_frame = "sensor_frame",
    .box_size = {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0},
    .keep_outside = false,
  });

  const std::vector<std::array<float, 3>> input_points = {
    {0.0f, 0.0f, 0.0f},   // inside -> keep
    {5.0f, 0.0f, 0.0f},   // outside -> drop
    {0.5f, 0.5f, 0.5f},   // inside -> keep
    {-2.0f, 0.0f, 0.0f},  // outside -> drop
  };
  const std::vector<std::array<float, 3>> expected_points = {
    {0.0f, 0.0f, 0.0f},
    {0.5f, 0.5f, 0.5f}
  };
  auto input_msg = make_xyz_point_cloud(input_points, "sensor_frame");

  // Act
  auto result = run_crop_box_filter_node(options, input_msg);

  // Assert
  EXPECT_EQ(extract_xyz(*result.pointcloud), expected_points);
}

TEST_F(CropBoxIntegrationTest, FiltersInsidePoints)
{
  // Arrange
  auto options = make_options({
    .input_frame = "sensor_frame",
    .crop_box_frame = "sensor_frame",
    .box_size = {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0},
    .keep_outside = true,
  });

  const std::vector<std::array<float, 3>> input_points = {
    {0.0f, 0.0f, 0.0f},   // inside -> drop
    {5.0f, 0.0f, 0.0f},   // outside -> keep
    {0.5f, 0.5f, 0.5f},   // inside -> drop
    {-2.0f, 0.0f, 0.0f},  // outside -> keep
  };
  const std::vector<std::array<float, 3>> expected_points = {
    {5.0f, 0.0f, 0.0f},
    {-2.0f, 0.0f, 0.0f},
  };
  auto input_msg = make_xyz_point_cloud(input_points, "sensor_frame");

  // Act
  auto result = run_crop_box_filter_node(options, input_msg);

  // Assert
  EXPECT_EQ(extract_xyz(*result.pointcloud), expected_points);
}

TEST_F(CropBoxIntegrationTest, TransformAndKeepInsidePoints)
{
  // Arrange
  auto transform = make_transform({
    .parent_frame = "crop_box_frame",
    .child_frame = "sensor_frame",
    .translation = {10.0, 0.0, 0.0},
  });

  auto options = make_options({
    .input_frame = "sensor_frame",
    .crop_box_frame = "crop_box_frame",
    .box_size = {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0},
    .keep_outside = false,
  });

  const std::vector<std::array<float, 3>> input_points = {
    {-10.0f, 0.0f, 0.0f},     // -> (0, 0, 0) in crop_box_frame       -> inside
    {-10.0f, 0.5f, -0.5f},    // -> (0, 0.5, -0.5) in crop_box_frame  -> inside
    {-10.5f, 0.0f, 0.0f},     // -> (-0.5, 0, 0) in crop_box_frame    -> inside
    {0.0f, 0.0f, 0.0f},       // -> (10, 0, 0) in crop_box_frame      -> outside
    {-10.0f, 5.0f, 0.0f},     // -> (0, 5, 0) in crop_box_frame       -> outside
  };
  auto input_msg = make_xyz_point_cloud(input_points, "sensor_frame");

  const std::vector<std::array<float, 3>> expected_points = {
    {-10.0f, 0.0f, 0.0f},
    {-10.0f, 0.5f, -0.5f},
    {-10.5f, 0.0f, 0.0f},
  };

  // Act
  publish_transform(transform);
  auto result = run_crop_box_filter_node(options, input_msg);

  // Assert
  EXPECT_EQ(extract_xyz(*result.pointcloud), expected_points);
}

TEST_F(CropBoxIntegrationTest, CropBoxPolygonIsPublishedInCropBoxFrame)
{
  // Arrange
  auto transform = make_transform({
    .parent_frame = "crop_box_frame",
    .child_frame = "sensor_frame",
    .translation = {0.0, 0.0, 0.0},
  });

  auto options = make_options({
    .input_frame = "sensor_frame",
    .crop_box_frame = "crop_box_frame",
    .box_size = {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0},
    .keep_outside = false,
  });

  const std::vector<std::array<float, 3>> input_points = {};
  auto input_msg = make_xyz_point_cloud(input_points, "sensor_frame");

  // Act
  publish_transform(transform);
  auto result = run_crop_box_filter_node(options, input_msg);

  // Assert
  EXPECT_EQ(result.polygon->header.frame_id, "crop_box_frame");
  EXPECT_FALSE(result.polygon->polygon.points.empty());
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
