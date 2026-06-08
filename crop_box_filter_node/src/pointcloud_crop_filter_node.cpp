#include "pointcloud_crop_filter/pointcloud_crop_filter_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <string>
#include <utility>

namespace pointcloud_crop_filter
{

PointCloudCropFilterNode::PointCloudCropFilterNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_crop_filter", options)
{
  // TF setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Get transform frame parameters
  const auto tf_input_orig_frame =
    static_cast<std::string>(declare_parameter("input_pointcloud_frame", "base_link"));
  const auto crop_box_frame =
    static_cast<std::string>(declare_parameter("crop_box_frame", "base_link"));
  max_queue_size_ = static_cast<size_t>(declare_parameter("max_queue_size", 5));

  // Transform from the input pointcloud frame to the crop box frame.
  // Default to identity; when the frames differ, look the transform up from TF.
  geometry_msgs::msg::TransformStamped transform_input_to_crop_box;
  transform_input_to_crop_box.header.frame_id = crop_box_frame;
  transform_input_to_crop_box.child_frame_id = tf_input_orig_frame;
  transform_input_to_crop_box.transform.rotation.w = 1.0;  // identity rotation
  if (tf_input_orig_frame != crop_box_frame) {
    if (!lookup_transform(crop_box_frame, tf_input_orig_frame, transform_input_to_crop_box)) {
      RCLCPP_ERROR(
        this->get_logger(), "Cannot get transform from %s to %s. Please check your TF tree.",
        tf_input_orig_frame.c_str(), crop_box_frame.c_str());
    }
  }

  // Crop box parameters
  PointCloudCropFilterConfig config;
  config.min_x = static_cast<float>(declare_parameter<double>("min_x"));
  config.min_y = static_cast<float>(declare_parameter<double>("min_y"));
  config.min_z = static_cast<float>(declare_parameter<double>("min_z"));
  config.max_x = static_cast<float>(declare_parameter<double>("max_x"));
  config.max_y = static_cast<float>(declare_parameter<double>("max_y"));
  config.max_z = static_cast<float>(declare_parameter<double>("max_z"));
  config.keep_outside = declare_parameter<bool>("keep_outside");
  config.transform_input_to_crop_box = transform_input_to_crop_box;

  filter_ = PointCloudCropFilter(config);

  RCLCPP_INFO(
    get_logger(),
    "CropBox min=(%.1f, %.1f, %.1f) max=(%.1f, %.1f, %.1f) keep_outside=%s",
    config.min_x, config.min_y, config.min_z,
    config.max_x, config.max_y, config.max_z,
    config.keep_outside ? "true" : "false");

  // Output publisher
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_output_ = this->create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_), pub_options);
  }

  // Crop box polygon publisher
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    crop_box_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "~/crop_box_polygon", 10, pub_options);
  }

  // Input subscriber
  {
    sub_input_ = this->create_subscription<PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_),
      std::bind(&PointCloudCropFilterNode::pointcloud_callback, this, std::placeholders::_1));
  }

  RCLCPP_DEBUG(this->get_logger(), "[Filter Constructor] successfully created.");
}

bool PointCloudCropFilterNode::lookup_transform(
  const std::string & target_frame, const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return false;
  }
}

void PointCloudCropFilterNode::pointcloud_callback(const PointCloud2ConstPtr msg)
{
  auto result = filter_.filter(*msg);

  if (result.non_finite_count > 0) {
    RCLCPP_WARN(
      this->get_logger(), "Skipped %zu non-finite points in the input pointcloud.",
      result.non_finite_count);
  }

  crop_box_polygon_pub_->publish(filter_.create_crop_box_polygon(get_clock()->now()));
  pub_output_->publish(std::move(result.output));
}

}  // namespace pointcloud_crop_filter

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_crop_filter::PointCloudCropFilterNode)
