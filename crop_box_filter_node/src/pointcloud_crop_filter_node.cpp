#include "pointcloud_crop_filter/pointcloud_crop_filter_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cstring>
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
  tf_input_orig_frame_ =
    static_cast<std::string>(declare_parameter("input_pointcloud_frame", "base_link"));
  crop_box_frame_ =
    static_cast<std::string>(declare_parameter("crop_box_frame", "base_link"));
  max_queue_size_ = static_cast<size_t>(declare_parameter("max_queue_size", 5));

  // Preprocess transform: input_orig_frame -> crop_box_frame
  if (tf_input_orig_frame_ == crop_box_frame_) {
    need_preprocess_transform_ = false;
    eigen_transform_preprocess_ = Eigen::Matrix4f::Identity();
  } else {
    need_preprocess_transform_ =
      lookup_transform(crop_box_frame_, tf_input_orig_frame_, eigen_transform_preprocess_);
    if (!need_preprocess_transform_) {
      RCLCPP_ERROR(
        this->get_logger(), "Cannot get transform from %s to %s. Please check your TF tree.",
        tf_input_orig_frame_.c_str(), crop_box_frame_.c_str());
    }
  }

  // Crop box parameters
  {
    auto & p = param_;
    p.min_x = static_cast<float>(declare_parameter<double>("min_x"));
    p.min_y = static_cast<float>(declare_parameter<double>("min_y"));
    p.min_z = static_cast<float>(declare_parameter<double>("min_z"));
    p.max_x = static_cast<float>(declare_parameter<double>("max_x"));
    p.max_y = static_cast<float>(declare_parameter<double>("max_y"));
    p.max_z = static_cast<float>(declare_parameter<double>("max_z"));
    p.keep_outside = declare_parameter<bool>("keep_outside");
  }

  RCLCPP_INFO(
    get_logger(),
    "CropBox min=(%.1f, %.1f, %.1f) max=(%.1f, %.1f, %.1f) keep_outside=%s",
    param_.min_x, param_.min_y, param_.min_z,
    param_.max_x, param_.max_y, param_.max_z,
    param_.keep_outside ? "true" : "false");

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
  Eigen::Matrix4f & transform)
{
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
    Eigen::Isometry3d eigen_tf = tf2::transformToEigen(tf_stamped);
    transform = eigen_tf.matrix().cast<float>();
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return false;
  }
}

void PointCloudCropFilterNode::pointcloud_callback(const PointCloud2ConstPtr msg)
{
  auto output = PointCloud2();

  // Filter pointcloud using index-based memcpy access
  int x_offset = msg->fields[pcl::getFieldIndex(*msg, "x")].offset;
  int y_offset = msg->fields[pcl::getFieldIndex(*msg, "y")].offset;
  int z_offset = msg->fields[pcl::getFieldIndex(*msg, "z")].offset;

  output.data.resize(msg->data.size());
  size_t output_size = 0;

  for (size_t global_offset = 0; global_offset + msg->point_step <= msg->data.size();
       global_offset += msg->point_step) {
    Eigen::Vector4f point;

    std::memcpy(&point[0], &msg->data[global_offset + x_offset], sizeof(float));
    std::memcpy(&point[1], &msg->data[global_offset + y_offset], sizeof(float));
    std::memcpy(&point[2], &msg->data[global_offset + z_offset], sizeof(float));
    point[3] = 1;

    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      continue;
    }

    Eigen::Vector4f point_preprocessed = point;

    if (need_preprocess_transform_) {
      point_preprocessed = eigen_transform_preprocess_ * point;
    }

    bool point_is_inside =
      point_preprocessed[2] > param_.min_z && point_preprocessed[2] < param_.max_z &&
      point_preprocessed[1] > param_.min_y && point_preprocessed[1] < param_.max_y &&
      point_preprocessed[0] > param_.min_x && point_preprocessed[0] < param_.max_x;

    if ((!param_.keep_outside && point_is_inside) || (param_.keep_outside && !point_is_inside)) {
      std::memcpy(&output.data[output_size], &msg->data[global_offset], msg->point_step);
      output_size += msg->point_step;
    }
  }

  output.data.resize(output_size);
  output.header.frame_id = tf_input_orig_frame_;
  output.header.stamp = msg->header.stamp;
  output.height = 1;
  output.fields = msg->fields;
  output.is_bigendian = msg->is_bigendian;
  output.point_step = msg->point_step;
  output.is_dense = msg->is_dense;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);

  publish_crop_box_polygon();
  pub_output_->publish(std::move(output));
}

void PointCloudCropFilterNode::publish_crop_box_polygon()
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  const double x1 = param_.max_x;
  const double x2 = param_.min_x;
  const double x3 = param_.min_x;
  const double x4 = param_.max_x;

  const double y1 = param_.max_y;
  const double y2 = param_.max_y;
  const double y3 = param_.min_y;
  const double y4 = param_.min_y;

  const double z1 = param_.min_z;
  const double z2 = param_.max_z;

  geometry_msgs::msg::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = crop_box_frame_;
  polygon_msg.header.stamp = get_clock()->now();
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

  crop_box_polygon_pub_->publish(polygon_msg);
}

}  // namespace pointcloud_crop_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_crop_filter::PointCloudCropFilterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
