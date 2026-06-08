#ifndef POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_NODE_HPP_
#define POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_NODE_HPP_

#include "pointcloud_crop_filter/pointcloud_crop_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace pointcloud_crop_filter
{

using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

class PointCloudCropFilterNode : public rclcpp::Node
{
public:
  explicit PointCloudCropFilterNode(const rclcpp::NodeOptions & options);

private:
  void pointcloud_callback(const PointCloud2ConstPtr msg);

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool lookup_transform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform);

  // Core logic
  PointCloudCropFilter filter_;

  // Pub / Sub
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr crop_box_polygon_pub_;

  size_t max_queue_size_{5};
};

}  // namespace pointcloud_crop_filter

#endif  // POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_NODE_HPP_
