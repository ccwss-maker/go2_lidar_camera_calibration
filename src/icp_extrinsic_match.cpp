#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class IcpExtrinsicMatch : public rclcpp::Node
{
public:
  IcpExtrinsicMatch()
  : Node("icp_extrinsic_match")
  {
    camera_topic_ = declare_parameter<std::string>(
      "camera_cloud_topic", "/icp/filtered_camera_points");
    lidar_topic_ = declare_parameter<std::string>(
      "lidar_cloud_topic", "/icp/filtered_lidar_points");
    aligned_cloud_topic_ = declare_parameter<std::string>(
      "aligned_cloud_topic", "/icp/aligned_camera_points");
    transform_topic_ = declare_parameter<std::string>(
      "transform_topic", "/icp/extrinsic_transform");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    source_frame_ = declare_parameter<std::string>("source_frame", "camera_depth_optical_frame");

    initial_x_ = declare_parameter<double>("initial_x", 0.0);
    initial_y_ = declare_parameter<double>("initial_y", 0.0);
    initial_z_ = declare_parameter<double>("initial_z", 0.2);
    initial_roll_ = declare_parameter<double>("initial_roll", -1.571);
    initial_pitch_ = declare_parameter<double>("initial_pitch", 0.0);
    initial_yaw_ = declare_parameter<double>("initial_yaw", -1.571);

    max_correspondence_distance_ = declare_parameter<double>("max_correspondence_distance", 0.5);
    icp_camera_voxel_leaf_size_ = declare_parameter<double>("icp_camera_voxel_leaf_size", 0.08);
    icp_lidar_voxel_leaf_size_ = declare_parameter<double>("icp_lidar_voxel_leaf_size", 0.08);
    transformation_epsilon_ = declare_parameter<double>("transformation_epsilon", 1e-8);
    euclidean_fitness_epsilon_ = declare_parameter<double>("euclidean_fitness_epsilon", 1e-5);
    max_iterations_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int64_t>("max_iterations", 50)));
    min_camera_points_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int64_t>("min_camera_points", 50)));
    min_lidar_points_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int64_t>("min_lidar_points", 50)));
    sync_queue_size_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int64_t>("sync_queue_size", 20)));
    sync_slop_seconds_ = declare_parameter<double>("sync_slop_seconds", 0.2);
    icp_min_interval_seconds_ = declare_parameter<double>("icp_min_interval_seconds", 1.0);
    fitness_good_threshold_ = declare_parameter<double>("fitness_good_threshold", 0.01);
    fitness_bad_threshold_ = declare_parameter<double>("fitness_bad_threshold", 0.05);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);

    aligned_pub_ = create_publisher<PointCloud2>(aligned_cloud_topic_, rclcpp::SensorDataQoS());
    transform_pub_ = create_publisher<TransformStamped>(transform_topic_, rclcpp::QoS(10));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    auto qos = rclcpp::SensorDataQoS();
    camera_sub_ = create_subscription<PointCloud2>(
      camera_topic_, qos,
      std::bind(&IcpExtrinsicMatch::onCameraCloud, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<PointCloud2>(
      lidar_topic_, qos,
      std::bind(&IcpExtrinsicMatch::onLidarCloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "ICP extrinsic match started\n"
      " - camera source: %s\n"
      " - lidar target: %s\n"
      " - aligned cloud: %s\n"
      " - transform topic: %s\n"
      " - initial %s -> %s: xyz=(%.4f %.4f %.4f), rpy=(%.4f %.4f %.4f)",
      camera_topic_.c_str(), lidar_topic_.c_str(), aligned_cloud_topic_.c_str(),
      transform_topic_.c_str(), target_frame_.c_str(), source_frame_.c_str(),
      initial_x_, initial_y_, initial_z_, initial_roll_, initial_pitch_, initial_yaw_);
  }

private:
  struct BufferedCloud
  {
    PointCloud2::ConstSharedPtr msg;
    rclcpp::Time receive_time;
  };

  void onCameraCloud(const PointCloud2::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(sync_mutex_);
      camera_buffer_.push_back(BufferedCloud{msg, now()});
      trimBuffer(camera_buffer_);
    }
    tryStartIcp();
  }

  void onLidarCloud(const PointCloud2::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(sync_mutex_);
      lidar_buffer_.push_back(BufferedCloud{msg, now()});
      trimBuffer(lidar_buffer_);
    }
    tryStartIcp();
  }

  void trimBuffer(std::deque<BufferedCloud> & buffer)
  {
    while (static_cast<int>(buffer.size()) > sync_queue_size_) {
      buffer.pop_front();
    }
  }

  void tryStartIcp()
  {
    PointCloud2::ConstSharedPtr camera_msg;
    PointCloud2::ConstSharedPtr lidar_msg;
    double best_dt = sync_slop_seconds_;
    double nearest_dt = std::numeric_limits<double>::infinity();

    {
      std::lock_guard<std::mutex> lock(sync_mutex_);
      if (icp_busy_ || camera_buffer_.empty() || lidar_buffer_.empty()) {
        return;
      }
      if ((now() - last_icp_start_time_).seconds() < icp_min_interval_seconds_) {
        return;
      }

      size_t best_camera_index = 0;
      size_t best_lidar_index = 0;
      bool found = false;
      for (size_t i = 0; i < camera_buffer_.size(); ++i) {
        for (size_t j = 0; j < lidar_buffer_.size(); ++j) {
          const auto dt = receiveDiffSeconds(camera_buffer_[i], lidar_buffer_[j]);
          nearest_dt = std::min(nearest_dt, dt);
          if (dt <= best_dt) {
            best_dt = dt;
            best_camera_index = i;
            best_lidar_index = j;
            found = true;
          }
        }
      }
      if (!found) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ICP waiting for synced clouds: camera_buffer=%zu lidar_buffer=%zu slop=%.3fs nearest_receive_dt=%.3fs",
          camera_buffer_.size(), lidar_buffer_.size(), sync_slop_seconds_, nearest_dt);
        return;
      }

      camera_msg = camera_buffer_[best_camera_index].msg;
      lidar_msg = lidar_buffer_[best_lidar_index].msg;
      camera_buffer_.erase(camera_buffer_.begin(), camera_buffer_.begin() + best_camera_index + 1);
      lidar_buffer_.erase(lidar_buffer_.begin(), lidar_buffer_.begin() + best_lidar_index + 1);
      icp_busy_ = true;
      last_icp_start_time_ = now();
    }

    std::thread([this, camera_msg, lidar_msg, best_dt]() {
      runIcp(camera_msg, lidar_msg, best_dt);
      std::lock_guard<std::mutex> lock(sync_mutex_);
      icp_busy_ = false;
    }).detach();
  }

  void runIcp(
    const PointCloud2::ConstSharedPtr & camera_msg,
    const PointCloud2::ConstSharedPtr & lidar_msg,
    double sync_dt)
  {
    (void)sync_dt;
    auto camera_cloud = std::make_shared<CloudT>();
    auto lidar_cloud = std::make_shared<CloudT>();
    pcl::fromROSMsg(*camera_msg, *camera_cloud);
    pcl::fromROSMsg(*lidar_msg, *lidar_cloud);

    camera_cloud = voxelDownsample(camera_cloud, icp_camera_voxel_leaf_size_);
    lidar_cloud = voxelDownsample(lidar_cloud, icp_lidar_voxel_leaf_size_);

    if (static_cast<int>(camera_cloud->size()) < min_camera_points_ ||
      static_cast<int>(lidar_cloud->size()) < min_lidar_points_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ICP skipped: points too few camera=%zu/%d lidar=%zu/%d",
        camera_cloud->size(), min_camera_points_, lidar_cloud->size(), min_lidar_points_);
      return;
    }

    const auto initial = initialTransform();
    auto camera_initial_base = std::make_shared<CloudT>();
    pcl::transformPointCloud(*camera_cloud, *camera_initial_base, initial);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(camera_initial_base);
    icp.setInputTarget(lidar_cloud);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setMaximumIterations(max_iterations_);
    icp.setTransformationEpsilon(transformation_epsilon_);
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

    CloudT aligned;
    icp.align(aligned);

    const Eigen::Matrix4f correction = icp.getFinalTransformation();
    const Eigen::Matrix4f final_base_from_camera = correction * initial;

    PointCloud2 aligned_msg;
    pcl::toROSMsg(aligned, aligned_msg);
    aligned_msg.header = lidar_msg->header;
    if (aligned_msg.header.frame_id.empty()) {
      aligned_msg.header.frame_id = target_frame_;
    }
    aligned_pub_->publish(aligned_msg);

    const auto transform_msg = makeTransformMsg(final_base_from_camera, lidar_msg->header.stamp);
    transform_pub_->publish(transform_msg);
    if (tf_broadcaster_) {
      tf_broadcaster_->sendTransform(transform_msg);
    }

    printResult(icp, final_base_from_camera);
  }

  Eigen::Matrix4f initialTransform() const
  {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    const Eigen::AngleAxisf roll(static_cast<float>(initial_roll_), Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf pitch(static_cast<float>(initial_pitch_), Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yaw(static_cast<float>(initial_yaw_), Eigen::Vector3f::UnitZ());
    const Eigen::Matrix3f rotation = (yaw * pitch * roll).matrix();
    transform.block<3, 3>(0, 0) = rotation;
    transform(0, 3) = static_cast<float>(initial_x_);
    transform(1, 3) = static_cast<float>(initial_y_);
    transform(2, 3) = static_cast<float>(initial_z_);
    return transform;
  }

  CloudT::Ptr voxelDownsample(const CloudT::Ptr & input, double leaf_size) const
  {
    if (leaf_size <= 0.0 || input->empty()) {
      return input;
    }
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(
      static_cast<float>(leaf_size),
      static_cast<float>(leaf_size),
      static_cast<float>(leaf_size));
    auto output = std::make_shared<CloudT>();
    voxel.filter(*output);
    return output;
  }

  double stampDiffSeconds(
    const PointCloud2::ConstSharedPtr & camera_msg,
    const PointCloud2::ConstSharedPtr & lidar_msg) const
  {
    const rclcpp::Time camera_time(camera_msg->header.stamp);
    const rclcpp::Time lidar_time(lidar_msg->header.stamp);
    return std::abs((camera_time - lidar_time).seconds());
  }

  double receiveDiffSeconds(const BufferedCloud & camera, const BufferedCloud & lidar) const
  {
    return std::abs((camera.receive_time - lidar.receive_time).seconds());
  }

  TransformStamped makeTransformMsg(
    const Eigen::Matrix4f & transform,
    const builtin_interfaces::msg::Time & stamp) const
  {
    TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = target_frame_;
    msg.child_frame_id = source_frame_;
    msg.transform.translation.x = transform(0, 3);
    msg.transform.translation.y = transform(1, 3);
    msg.transform.translation.z = transform(2, 3);

    Eigen::Quaternionf quat(transform.block<3, 3>(0, 0));
    quat.normalize();
    msg.transform.rotation.x = quat.x();
    msg.transform.rotation.y = quat.y();
    msg.transform.rotation.z = quat.z();
    msg.transform.rotation.w = quat.w();
    return msg;
  }

  void printResult(
    pcl::IterativeClosestPoint<PointT, PointT> & icp,
    const Eigen::Matrix4f & final_base_from_camera)
  {
    Eigen::Matrix3f rotation = final_base_from_camera.block<3, 3>(0, 0);
    Eigen::Vector3f rpy = staticTransformPublisherRpy(rotation);
    const double fitness = icp.getFitnessScore();
    const double score = qualityScore(fitness, icp.hasConverged());
    const char * quality = qualityLabel(score, icp.hasConverged());

    RCLCPP_INFO(
      get_logger(),
      "[ICP RESULT] xyz=(%.6f %.6f %.6f) rpy=(%.6f %.6f %.6f) score=%.1f quality=%s\n"
      "Matrix:\n"
      "  %.6f  %.6f  %.6f  %.6f\n"
      "  %.6f  %.6f  %.6f  %.6f\n"
      "  %.6f  %.6f  %.6f  %.6f\n"
      "  %.6f  %.6f  %.6f  %.6f",
      final_base_from_camera(0, 3), final_base_from_camera(1, 3), final_base_from_camera(2, 3),
      rpy.x(), rpy.y(), rpy.z(), score, quality,
      final_base_from_camera(0, 0), final_base_from_camera(0, 1),
      final_base_from_camera(0, 2), final_base_from_camera(0, 3),
      final_base_from_camera(1, 0), final_base_from_camera(1, 1),
      final_base_from_camera(1, 2), final_base_from_camera(1, 3),
      final_base_from_camera(2, 0), final_base_from_camera(2, 1),
      final_base_from_camera(2, 2), final_base_from_camera(2, 3),
      final_base_from_camera(3, 0), final_base_from_camera(3, 1),
      final_base_from_camera(3, 2), final_base_from_camera(3, 3));
  }

  Eigen::Vector3f staticTransformPublisherRpy(const Eigen::Matrix3f & rotation) const
  {
    const float sy = std::sqrt(
      rotation(0, 0) * rotation(0, 0) + rotation(1, 0) * rotation(1, 0));
    const bool singular = sy < 1e-6f;

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    if (!singular) {
      roll = std::atan2(rotation(2, 1), rotation(2, 2));
      pitch = std::atan2(-rotation(2, 0), sy);
      yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
      roll = std::atan2(-rotation(1, 2), rotation(1, 1));
      pitch = std::atan2(-rotation(2, 0), sy);
      yaw = 0.0f;
    }

    return Eigen::Vector3f(roll, pitch, yaw);
  }

  double qualityScore(double fitness, bool converged) const
  {
    if (!converged || !std::isfinite(fitness)) {
      return 0.0;
    }
    if (fitness <= fitness_good_threshold_) {
      return 100.0;
    }
    if (fitness >= fitness_bad_threshold_) {
      return 0.0;
    }
    const double range = fitness_bad_threshold_ - fitness_good_threshold_;
    return 100.0 * (fitness_bad_threshold_ - fitness) / range;
  }

  const char * qualityLabel(double score, bool converged) const
  {
    if (!converged) {
      return "BAD_NOT_CONVERGED";
    }
    if (score >= 80.0) {
      return "GOOD";
    }
    if (score >= 50.0) {
      return "OK";
    }
    return "BAD";
  }

  std::string camera_topic_;
  std::string lidar_topic_;
  std::string aligned_cloud_topic_;
  std::string transform_topic_;
  std::string target_frame_;
  std::string source_frame_;
  double initial_x_;
  double initial_y_;
  double initial_z_;
  double initial_roll_;
  double initial_pitch_;
  double initial_yaw_;
  double max_correspondence_distance_;
  double icp_camera_voxel_leaf_size_;
  double icp_lidar_voxel_leaf_size_;
  double transformation_epsilon_;
  double euclidean_fitness_epsilon_;
  int max_iterations_;
  int min_camera_points_;
  int min_lidar_points_;
  int sync_queue_size_;
  double sync_slop_seconds_;
  double icp_min_interval_seconds_;
  double fitness_good_threshold_;
  double fitness_bad_threshold_;
  bool publish_tf_;

  rclcpp::Subscription<PointCloud2>::SharedPtr camera_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr lidar_sub_;
  std::deque<BufferedCloud> camera_buffer_;
  std::deque<BufferedCloud> lidar_buffer_;
  std::mutex sync_mutex_;
  bool icp_busy_ = false;
  rclcpp::Time last_icp_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<TransformStamped>::SharedPtr transform_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IcpExtrinsicMatch>());
  rclcpp::shutdown();
  return 0;
}
