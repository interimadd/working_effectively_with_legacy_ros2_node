#ifndef POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_NODE_HPP_
#define POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/common.h>

#include <memory>
#include <string>

namespace pointcloud_crop_filter
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

class PointCloudCropFilterNode : public rclcpp::Node
{
public:
  explicit PointCloudCropFilterNode(const rclcpp::NodeOptions & options);

private:
  // Core filtering - tightly coupled with Node
  void pointcloud_callback(const PointCloud2ConstPtr msg);
  void publish_crop_box_polygon();

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string crop_box_frame_;
  std::string tf_input_orig_frame_;

  bool need_preprocess_transform_{false};
  Eigen::Matrix4f eigen_transform_preprocess_{Eigen::Matrix4f::Identity()};

  bool lookup_transform(
    const std::string & target_frame, const std::string & source_frame,
    Eigen::Matrix4f & transform);

  // Crop box parameters
  struct CropBoxParam
  {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    bool keep_outside{false};
  } param_;

  // Pub / Sub
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr crop_box_polygon_pub_;

  size_t max_queue_size_{5};
};

}  // namespace pointcloud_crop_filter

#endif  // POINTCLOUD_CROP_FILTER__POINTCLOUD_CROP_FILTER_NODE_HPP_
