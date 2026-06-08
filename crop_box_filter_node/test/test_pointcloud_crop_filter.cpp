#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "pointcloud_crop_filter/pointcloud_crop_filter.hpp"

namespace
{
using pointcloud_crop_filter::PointCloudCropFilter;
using pointcloud_crop_filter::PointCloudCropFilterConfig;
using sensor_msgs::msg::PointCloud2;

using Point = std::array<float, 3>;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// Default config: a symmetric box [-5, 5] in every axis with an identity input->crop_box
// transform. The transform frames are set to distinct names so frame propagation can be
// asserted (header.frame_id -> polygon frame, child_frame_id -> filtered output frame).
PointCloudCropFilterConfig make_default_config()
{
  PointCloudCropFilterConfig config;
  config.min_x = -5.0f;
  config.max_x = 5.0f;
  config.min_y = -5.0f;
  config.max_y = 5.0f;
  config.min_z = -5.0f;
  config.max_z = 5.0f;
  config.keep_outside = false;
  config.transform_input_to_crop_box.header.frame_id = "crop_box";
  config.transform_input_to_crop_box.child_frame_id = "input";
  config.transform_input_to_crop_box.transform.rotation.w = 1.0;  // identity rotation
  return config;
}

PointCloud2 make_pointcloud(const std::vector<Point> & points)
{
  PointCloud2 pointcloud;
  pointcloud.header.frame_id = "input";
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
}  // namespace

TEST(PointCloudCropFilterTest, EmptyPointCloudYieldsEmptyOutput)
{
  // Arrange
  PointCloudCropFilter filter(make_default_config());

  // Act
  const auto result = filter.filter(make_pointcloud({}));

  // Assert
  EXPECT_TRUE(extract_points(result.output).empty());
  EXPECT_EQ(result.non_finite_count, 0u);
}

TEST(PointCloudCropFilterTest, KeepsPointsInsideAndDropsPointsOutside)
{
  // Arrange
  PointCloudCropFilter filter(make_default_config());
  const Point inside = {1.0f, 2.0f, 3.0f};
  const Point outside = {6.0f, 0.0f, 0.0f};
  const std::vector<Point> input = {inside, outside};
  const std::vector<Point> expected = {inside};

  // Act
  const auto result = filter.filter(make_pointcloud(input));

  // Assert
  EXPECT_EQ(extract_points(result.output), expected);
  EXPECT_EQ(result.non_finite_count, 0u);
}

TEST(PointCloudCropFilterTest, DropsPointsOutsideOnEachAxis)
{
  // Arrange
  PointCloudCropFilter filter(make_default_config());
  const Point inside = {0.0f, 0.0f, 0.0f};
  const std::vector<Point> input = {
    inside,
    {6.0f, 0.0f, 0.0f}, {-6.0f, 0.0f, 0.0f},
    {0.0f, 6.0f, 0.0f}, {0.0f, -6.0f, 0.0f},
    {0.0f, 0.0f, 6.0f}, {0.0f, 0.0f, -6.0f}};
  const std::vector<Point> expected = {inside};

  // Act
  const auto result = filter.filter(make_pointcloud(input));

  // Assert
  EXPECT_EQ(extract_points(result.output), expected);
}

TEST(PointCloudCropFilterTest, ExcludesPointsExactlyOnBoundary)
{
  // Arrange
  PointCloudCropFilter filter(make_default_config());
  const Point on_max_boundary = {5.0f, 0.0f, 0.0f};
  const Point on_min_boundary = {-5.0f, 0.0f, 0.0f};
  const Point just_inside = {4.999f, 0.0f, 0.0f};
  const std::vector<Point> input = {on_max_boundary, on_min_boundary, just_inside};
  const std::vector<Point> expected = {just_inside};

  // Act
  const auto result = filter.filter(make_pointcloud(input));

  // Assert
  EXPECT_EQ(extract_points(result.output), expected);
}

TEST(PointCloudCropFilterTest, KeepOutsideRetainsOnlyPointsOutsideBox)
{
  // Arrange
  auto config = make_default_config();
  config.keep_outside = true;
  PointCloudCropFilter filter(config);

  const Point inside = {1.0f, 2.0f, 3.0f};
  const Point outside = {6.0f, 0.0f, 0.0f};
  const std::vector<Point> input = {inside, outside};
  const std::vector<Point> expected = {outside};

  // Act
  const auto result = filter.filter(make_pointcloud(input));

  // Assert
  EXPECT_EQ(extract_points(result.output), expected);
}

TEST(PointCloudCropFilterTest, SkipsAndCountsNonFinitePoints)
{
  // Arrange
  PointCloudCropFilter filter(make_default_config());
  const Point inside = {1.0f, 1.0f, 1.0f};
  const std::vector<Point> input = {
    inside,
    {kNaN, 0.0f, 0.0f},
    {0.0f, kNaN, 0.0f},
    {0.0f, 0.0f, kNaN},
    {kInf, 0.0f, 0.0f}};
  const std::vector<Point> expected = {inside};

  // Act
  const auto result = filter.filter(make_pointcloud(input));

  // Assert
  EXPECT_EQ(extract_points(result.output), expected);
  EXPECT_EQ(result.non_finite_count, 4u);
}

TEST(PointCloudCropFilterTest, AppliesTransformBeforeBoundsCheck)
{
  // Arrange
  auto config = make_default_config();
  config.transform_input_to_crop_box.transform.translation.x = 10.0;  // crop_box = input + 10
  PointCloudCropFilter filter(config);

  // In input frame these are far outside [-5, 5], but after the +10 shift the first maps to
  // x=0 (inside) and the second to x=10 (outside).
  const Point inside_after_transform = {-10.0f, 0.0f, 0.0f};
  const Point outside_after_transform = {0.0f, 0.0f, 0.0f};
  const std::vector<Point> input = {inside_after_transform, outside_after_transform};
  // Output keeps the original (untransformed) coordinates of the kept point.
  const std::vector<Point> expected = {inside_after_transform};

  // Act
  const auto result = filter.filter(make_pointcloud(input));

  // Assert
  EXPECT_EQ(extract_points(result.output), expected);
}

TEST(PointCloudCropFilterTest, OutputUsesCropBoxChildFrameAndPreservesMetadata)
{
  // Arrange
  PointCloudCropFilter filter(make_default_config());
  const auto input = make_pointcloud({{0.0f, 0.0f, 0.0f}});

  // Act
  const auto result = filter.filter(input);

  // Assert
  EXPECT_EQ(result.output.header.frame_id, "input");  // transform's child_frame_id
  EXPECT_EQ(result.output.header.stamp, input.header.stamp);
  EXPECT_EQ(result.output.point_step, input.point_step);
  EXPECT_EQ(result.output.fields.size(), input.fields.size());
  EXPECT_EQ(result.output.height, 1u);
  EXPECT_EQ(result.output.width, 1u);
}

TEST(PointCloudCropFilterTest, CreateCropBoxPolygonSpansConfiguredExtents)
{
  // Arrange
  auto config = make_default_config();
  config.min_x = -1.0f;
  config.max_x = 2.0f;
  config.min_y = -3.0f;
  config.max_y = 4.0f;
  config.min_z = -5.0f;
  config.max_z = 6.0f;
  PointCloudCropFilter filter(config);

  // Act
  const auto polygon = filter.create_crop_box_polygon(rclcpp::Time(0, 0, RCL_ROS_TIME));

  // Assert
  EXPECT_EQ(polygon.header.frame_id, "crop_box");  // transform's header.frame_id

  ASSERT_FALSE(polygon.polygon.points.empty());
  float min_x = polygon.polygon.points[0].x;
  float max_x = polygon.polygon.points[0].x;
  float min_y = polygon.polygon.points[0].y;
  float max_y = polygon.polygon.points[0].y;
  float min_z = polygon.polygon.points[0].z;
  float max_z = polygon.polygon.points[0].z;
  for (const auto & point : polygon.polygon.points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
  }
  EXPECT_FLOAT_EQ(min_x, config.min_x);
  EXPECT_FLOAT_EQ(max_x, config.max_x);
  EXPECT_FLOAT_EQ(min_y, config.min_y);
  EXPECT_FLOAT_EQ(max_y, config.max_y);
  EXPECT_FLOAT_EQ(min_z, config.min_z);
  EXPECT_FLOAT_EQ(max_z, config.max_z);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
