#ifndef POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_HPP_
#define POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_HPP_

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Core>

#include <cstddef>

namespace pointcloud_crop_filter
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;

struct PointCloudCropFilterConfig
{
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float min_z;
  float max_z;
  bool keep_outside;
  geometry_msgs::msg::TransformStamped transform_input_to_crop_box;
};

struct PointCloudCropFilterResult
{
  PointCloud2 output;
  size_t non_finite_count;
};

class PointCloudCropFilter
{
public:
  explicit PointCloudCropFilter(const PointCloudCropFilterConfig & config);
  PointCloudCropFilter() = default;

  PointCloudCropFilterResult filter(const PointCloud2 & msg) const;
  geometry_msgs::msg::PolygonStamped create_crop_box_polygon(const rclcpp::Time & stamp) const;

private:
  PointCloudCropFilterConfig config_{};
  Eigen::Matrix4f transform_input_to_crop_box_{Eigen::Matrix4f::Identity()};
};

}  // namespace pointcloud_crop_filter

#endif  // POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_HPP_
