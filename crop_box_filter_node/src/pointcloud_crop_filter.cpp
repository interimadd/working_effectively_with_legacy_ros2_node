#include "pointcloud_crop_filter/pointcloud_crop_filter.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>

namespace pointcloud_crop_filter
{

PointCloudCropFilter::PointCloudCropFilter(const PointCloudCropFilterConfig & config)
: config_(config)
{
}

PointCloudCropFilterResult PointCloudCropFilter::filter(const PointCloud2 & msg) const
{
  auto output = PointCloud2();

  // Filter pointcloud using index-based memcpy access
  int x_offset = msg.fields[pcl::getFieldIndex(msg, "x")].offset;
  int y_offset = msg.fields[pcl::getFieldIndex(msg, "y")].offset;
  int z_offset = msg.fields[pcl::getFieldIndex(msg, "z")].offset;

  output.data.resize(msg.data.size());
  size_t output_size = 0;
  size_t non_finite_count = 0;

  for (size_t global_offset = 0; global_offset + msg.point_step <= msg.data.size();
       global_offset += msg.point_step) {
    Eigen::Vector4f point;

    std::memcpy(&point[0], &msg.data[global_offset + x_offset], sizeof(float));
    std::memcpy(&point[1], &msg.data[global_offset + y_offset], sizeof(float));
    std::memcpy(&point[2], &msg.data[global_offset + z_offset], sizeof(float));
    point[3] = 1;

    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      ++non_finite_count;
      continue;
    }

    Eigen::Vector4f point_preprocessed = point;

    if (config_.need_preprocess_transform) {
      point_preprocessed = config_.eigen_transform_preprocess * point;
    }

    bool point_is_inside =
      point_preprocessed[2] > config_.min_z && point_preprocessed[2] < config_.max_z &&
      point_preprocessed[1] > config_.min_y && point_preprocessed[1] < config_.max_y &&
      point_preprocessed[0] > config_.min_x && point_preprocessed[0] < config_.max_x;

    if ((!config_.keep_outside && point_is_inside) || (config_.keep_outside && !point_is_inside)) {
      std::memcpy(&output.data[output_size], &msg.data[global_offset], msg.point_step);
      output_size += msg.point_step;
    }
  }

  output.data.resize(output_size);
  output.header.frame_id = config_.input_frame;
  output.header.stamp = msg.header.stamp;
  output.height = 1;
  output.fields = msg.fields;
  output.is_bigendian = msg.is_bigendian;
  output.point_step = msg.point_step;
  output.is_dense = msg.is_dense;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);

  PointCloudCropFilterResult result;
  result.output = std::move(output);
  result.non_finite_count = non_finite_count;
  return result;
}

geometry_msgs::msg::PolygonStamped PointCloudCropFilter::create_crop_box_polygon(
  const rclcpp::Time & stamp) const
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  const double x1 = config_.max_x;
  const double x2 = config_.min_x;
  const double x3 = config_.min_x;
  const double x4 = config_.max_x;

  const double y1 = config_.max_y;
  const double y2 = config_.max_y;
  const double y3 = config_.min_y;
  const double y4 = config_.min_y;

  const double z1 = config_.min_z;
  const double z2 = config_.max_z;

  geometry_msgs::msg::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = config_.crop_box_frame;
  polygon_msg.header.stamp = stamp;
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  return polygon_msg;
}

}  // namespace pointcloud_crop_filter
