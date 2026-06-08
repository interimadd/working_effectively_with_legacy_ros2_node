#ifndef POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_HPP_
#define POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_HPP_

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <string>

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
  // Frame the filtered output is published in (the input pointcloud's original frame).
  std::string input_frame;
  // Frame the crop box polygon is published in.
  std::string crop_box_frame;
  // Whether to apply the preprocess transform (input_orig_frame -> crop_box_frame).
  bool need_preprocess_transform;
  Eigen::Matrix4f eigen_transform_preprocess;
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
};

}  // namespace pointcloud_crop_filter

#endif  // POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_HPP_
