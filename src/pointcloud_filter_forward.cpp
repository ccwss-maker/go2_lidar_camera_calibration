#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class PointcloudFilterForward : public rclcpp::Node
{
public:
  PointcloudFilterForward()
  : Node("pointcloud_filter_forward")
  {
    camera_topic_ = declare_parameter<std::string>(
      "camera_cloud_topic", "/camera/camera/depth/color/points");
    lidar_topic_ = declare_parameter<std::string>("lidar_cloud_topic", "/utlidar/cloud_base");
    filtered_camera_topic_ = declare_parameter<std::string>(
      "filtered_camera_cloud_topic", "/icp/filtered_camera_points");
    filtered_lidar_topic_ = declare_parameter<std::string>(
      "filtered_lidar_cloud_topic", "/icp/filtered_lidar_points");
    camera_frame_fallback_ = declare_parameter<std::string>("camera_frame", "camera_depth_optical_frame");
    lidar_frame_fallback_ = declare_parameter<std::string>("lidar_frame", "base_link");

    camera_bounds_ = Bounds{
      declare_parameter<double>("camera_x_min", -2.0),
      declare_parameter<double>("camera_x_max", 2.0),
      declare_parameter<double>("camera_y_min", -0.5),
      declare_parameter<double>("camera_y_max", 0.5),
      declare_parameter<double>("camera_z_min", 0.0),
      declare_parameter<double>("camera_z_max", 3.0)};
    lidar_bounds_ = Bounds{
      declare_parameter<double>("lidar_x_min", 0.0),
      declare_parameter<double>("lidar_x_max", 3.0),
      declare_parameter<double>("lidar_y_min", -2.0),
      declare_parameter<double>("lidar_y_max", 2.0),
      declare_parameter<double>("lidar_z_min", 0.0),
      declare_parameter<double>("lidar_z_max", 2.0)};

    camera_voxel_leaf_size_ = declare_parameter<double>("camera_voxel_leaf_size", 0.03);
    lidar_voxel_leaf_size_ = declare_parameter<double>("lidar_voxel_leaf_size", 0.0);
    camera_accum_frames_ = static_cast<int>(
      std::max<int64_t>(0, declare_parameter<int64_t>("camera_accum_frames", 0)));
    lidar_accum_frames_ = static_cast<int>(
      std::max<int64_t>(0, declare_parameter<int64_t>("lidar_accum_frames", 20)));

    auto output_qos = rclcpp::SensorDataQoS();
    camera_pub_ = create_publisher<PointCloud2>(filtered_camera_topic_, output_qos);
    lidar_pub_ = create_publisher<PointCloud2>(filtered_lidar_topic_, output_qos);

    auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    auto lidar_qos = rclcpp::SensorDataQoS();
    camera_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    lidar_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions camera_options;
    camera_options.callback_group = camera_callback_group_;
    rclcpp::SubscriptionOptions lidar_options;
    lidar_options.callback_group = lidar_callback_group_;

    camera_sub_ = create_subscription<PointCloud2>(
      camera_topic_, camera_qos,
      std::bind(&PointcloudFilterForward::onCameraCloud, this, std::placeholders::_1),
      camera_options);
    lidar_sub_ = create_subscription<PointCloud2>(
      lidar_topic_, lidar_qos,
      std::bind(&PointcloudFilterForward::onLidarCloud, this, std::placeholders::_1),
      lidar_options);

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&PointcloudFilterForward::onSetParameters, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Pointcloud filter forward started\n"
      " - camera input: %s\n"
      " - lidar input: %s\n"
      " - camera output: %s\n"
      " - lidar output: %s\n"
      " - camera accum frames: %d\n"
      " - lidar accum frames: %d",
      camera_topic_.c_str(), lidar_topic_.c_str(),
      filtered_camera_topic_.c_str(), filtered_lidar_topic_.c_str(),
      camera_accum_frames_, lidar_accum_frames_);
  }

private:
  struct Bounds
  {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;

    bool isWideOpen() const
    {
      return x_min <= -999.0 && x_max >= 999.0 &&
        y_min <= -999.0 && y_max >= 999.0 &&
        z_min <= -999.0 && z_max >= 999.0;
    }
  };

  void onCameraCloud(const PointCloud2::SharedPtr msg)
  {
    ++camera_input_count_;

    Bounds bounds;
    double voxel_leaf_size = 0.0;
    int accum_frames = 0;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      bounds = camera_bounds_;
      voxel_leaf_size = camera_voxel_leaf_size_;
      accum_frames = camera_accum_frames_;
    }

    if (accum_frames <= 0 && voxel_leaf_size <= 0.0 && bounds.isWideOpen()) {
      publishInputCloud(*msg, camera_frame_fallback_, camera_pub_);
      ++camera_publish_count_;
      return;
    }

    const auto cloud = filterCloud(*msg, bounds, voxel_leaf_size);
    if (!cloud || cloud->empty()) {
      warnThrottled("camera filtered cloud is empty; publishing empty cloud");
      publishCloud(CloudT(), msg->header, camera_frame_fallback_, camera_pub_);
      ++camera_publish_count_;
      return;
    }

    if (accum_frames <= 0) {
      publishCloud(*cloud, msg->header, camera_frame_fallback_, camera_pub_);
      ++camera_publish_count_;
      return;
    }

    CloudT::Ptr merged;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      camera_buffer_.push_back(cloud);
      while (static_cast<int>(camera_buffer_.size()) > accum_frames) {
        camera_buffer_.pop_front();
      }
      if (static_cast<int>(camera_buffer_.size()) < accum_frames) {
        return;
      }
      merged = mergeBuffer(camera_buffer_);
    }

    publishCloud(*merged, msg->header, camera_frame_fallback_, camera_pub_);
    ++camera_publish_count_;
  }

  void onLidarCloud(const PointCloud2::SharedPtr msg)
  {
    ++lidar_input_count_;

    Bounds bounds;
    double voxel_leaf_size = 0.0;
    int accum_frames = 0;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      bounds = lidar_bounds_;
      voxel_leaf_size = lidar_voxel_leaf_size_;
      accum_frames = lidar_accum_frames_;
    }

    const auto cloud = filterCloud(*msg, bounds, voxel_leaf_size);
    if (!cloud || cloud->empty()) {
      warnThrottled("lidar filtered cloud is empty; publishing empty cloud");
      publishCloud(CloudT(), msg->header, lidar_frame_fallback_, lidar_pub_);
      ++lidar_publish_count_;
      return;
    }

    if (accum_frames <= 0) {
      publishCloud(*cloud, msg->header, lidar_frame_fallback_, lidar_pub_);
      ++lidar_publish_count_;
      return;
    }

    CloudT::Ptr merged;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      lidar_buffer_.push_back(cloud);
      while (static_cast<int>(lidar_buffer_.size()) > accum_frames) {
        lidar_buffer_.pop_front();
      }
      if (static_cast<int>(lidar_buffer_.size()) < accum_frames) {
        return;
      }
      merged = mergeBuffer(lidar_buffer_);
    }

    publishCloud(*merged, msg->header, lidar_frame_fallback_, lidar_pub_);
    ++lidar_publish_count_;
  }

  CloudT::Ptr filterCloud(
    const PointCloud2 & msg,
    const Bounds & bounds,
    double voxel_leaf_size)
  {
    auto cloud = std::make_shared<CloudT>();
    pcl::fromROSMsg(msg, *cloud);
    if (cloud->empty()) {
      return cloud;
    }

    cloud = passThrough(cloud, "x", bounds.x_min, bounds.x_max);
    cloud = passThrough(cloud, "y", bounds.y_min, bounds.y_max);
    cloud = passThrough(cloud, "z", bounds.z_min, bounds.z_max);

    if (voxel_leaf_size > 0.0 && !cloud->empty()) {
      pcl::VoxelGrid<PointT> voxel;
      voxel.setInputCloud(cloud);
      voxel.setLeafSize(
        static_cast<float>(voxel_leaf_size),
        static_cast<float>(voxel_leaf_size),
        static_cast<float>(voxel_leaf_size));

      auto filtered = std::make_shared<CloudT>();
      voxel.filter(*filtered);
      cloud = filtered;
    }

    return cloud;
  }

  CloudT::Ptr passThrough(
    const CloudT::Ptr & input,
    const std::string & field,
    double min_value,
    double max_value)
  {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName(field);
    pass.setFilterLimits(static_cast<float>(min_value), static_cast<float>(max_value));

    auto output = std::make_shared<CloudT>();
    pass.filter(*output);
    return output;
  }

  CloudT::Ptr mergeBuffer(const std::deque<CloudT::Ptr> & buffer)
  {
    auto merged = std::make_shared<CloudT>();
    for (const auto & cloud : buffer) {
      *merged += *cloud;
    }
    merged->is_dense = false;
    return merged;
  }

  void publishCloud(
    const CloudT & cloud,
    const std_msgs::msg::Header & input_header,
    const std::string & fallback_frame,
    const rclcpp::Publisher<PointCloud2>::SharedPtr & publisher)
  {
    PointCloud2 out;
    pcl::toROSMsg(cloud, out);
    out.header = input_header;
    if (out.header.frame_id.empty()) {
      out.header.frame_id = fallback_frame;
    }
    publisher->publish(out);
  }

  void publishInputCloud(
    const PointCloud2 & input,
    const std::string & fallback_frame,
    const rclcpp::Publisher<PointCloud2>::SharedPtr & publisher)
  {
    auto out = input;
    if (out.header.frame_id.empty()) {
      out.header.frame_id = fallback_frame;
    }
    publisher->publish(out);
  }

  void warnThrottled(const char * message)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", message);
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    std::lock_guard<std::mutex> lock(config_mutex_);
    auto old_camera_accum_frames = camera_accum_frames_;
    auto old_lidar_accum_frames = lidar_accum_frames_;

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "camera_x_min") {
        camera_bounds_.x_min = parameter.as_double();
      } else if (name == "camera_x_max") {
        camera_bounds_.x_max = parameter.as_double();
      } else if (name == "camera_y_min") {
        camera_bounds_.y_min = parameter.as_double();
      } else if (name == "camera_y_max") {
        camera_bounds_.y_max = parameter.as_double();
      } else if (name == "camera_z_min") {
        camera_bounds_.z_min = parameter.as_double();
      } else if (name == "camera_z_max") {
        camera_bounds_.z_max = parameter.as_double();
      } else if (name == "lidar_x_min") {
        lidar_bounds_.x_min = parameter.as_double();
      } else if (name == "lidar_x_max") {
        lidar_bounds_.x_max = parameter.as_double();
      } else if (name == "lidar_y_min") {
        lidar_bounds_.y_min = parameter.as_double();
      } else if (name == "lidar_y_max") {
        lidar_bounds_.y_max = parameter.as_double();
      } else if (name == "lidar_z_min") {
        lidar_bounds_.z_min = parameter.as_double();
      } else if (name == "lidar_z_max") {
        lidar_bounds_.z_max = parameter.as_double();
      } else if (name == "camera_voxel_leaf_size") {
        camera_voxel_leaf_size_ = std::max(0.0, parameter.as_double());
      } else if (name == "lidar_voxel_leaf_size") {
        lidar_voxel_leaf_size_ = std::max(0.0, parameter.as_double());
      } else if (name == "camera_accum_frames") {
        camera_accum_frames_ = static_cast<int>(std::max<int64_t>(0, parameter.as_int()));
      } else if (name == "lidar_accum_frames") {
        lidar_accum_frames_ = static_cast<int>(std::max<int64_t>(0, parameter.as_int()));
      }
    }

    if (old_camera_accum_frames != camera_accum_frames_) {
      camera_buffer_.clear();
    }
    if (old_lidar_accum_frames != lidar_accum_frames_) {
      lidar_buffer_.clear();
    }

    RCLCPP_INFO(
      get_logger(),
      "params updated: camera xyz=[%.3f %.3f] [%.3f %.3f] [%.3f %.3f], "
      "camera voxel=%.3f accum=%d; lidar xyz=[%.3f %.3f] [%.3f %.3f] [%.3f %.3f], "
      "lidar voxel=%.3f accum=%d",
      camera_bounds_.x_min, camera_bounds_.x_max,
      camera_bounds_.y_min, camera_bounds_.y_max,
      camera_bounds_.z_min, camera_bounds_.z_max,
      camera_voxel_leaf_size_, camera_accum_frames_,
      lidar_bounds_.x_min, lidar_bounds_.x_max,
      lidar_bounds_.y_min, lidar_bounds_.y_max,
      lidar_bounds_.z_min, lidar_bounds_.z_max,
      lidar_voxel_leaf_size_, lidar_accum_frames_);

    return result;
  }

  std::string camera_topic_;
  std::string lidar_topic_;
  std::string filtered_camera_topic_;
  std::string filtered_lidar_topic_;
  std::string camera_frame_fallback_;
  std::string lidar_frame_fallback_;
  Bounds camera_bounds_;
  Bounds lidar_bounds_;
  double camera_voxel_leaf_size_;
  double lidar_voxel_leaf_size_;
  int camera_accum_frames_;
  int lidar_accum_frames_;
  size_t camera_input_count_ = 0;
  size_t lidar_input_count_ = 0;
  size_t camera_publish_count_ = 0;
  size_t lidar_publish_count_ = 0;

  std::deque<CloudT::Ptr> camera_buffer_;
  std::deque<CloudT::Ptr> lidar_buffer_;
  std::mutex config_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::CallbackGroup::SharedPtr camera_callback_group_;
  rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
  rclcpp::Subscription<PointCloud2>::SharedPtr camera_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr camera_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr lidar_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointcloudFilterForward>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
