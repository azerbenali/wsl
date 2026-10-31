// Copyright 2026 Azer Ben Ali
// SPDX-License-Identifier: Apache-2.0
//
// ROS 2 front end of the point-cloud pipeline.
//
// Concurrency: a single-threaded executor runs every callback; the TF listener
// spins its own thread, so the bounded TF wait inside the cloud callback cannot
// starve TF updates. The subscription keeps only the newest cloud: if
// processing ever exceeds the sensor period, stale clouds are dropped instead
// of queueing, which bounds output latency.
//
// QoS: sensor input is subscribed best-effort (compatible with both reliable
// and best-effort publishers); outputs are published reliable so that RViz and
// tools with default QoS can also subscribe.

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wsl_perception/cloud_pipeline.hpp"

namespace wsl_perception
{
namespace
{

using sensor_msgs::msg::PointCloud2;

struct DoubleParam
{
  const char * name;
  double wsl_perception::PipelineConfig::*field;
  double min;
  double max;
  const char * description;
};

struct IntParam
{
  const char * name;
  int wsl_perception::PipelineConfig::*field;
  int min;
  int max;
  const char * description;
};

constexpr std::array<DoubleParam, 13> kDoubleParams{{
  {"min_range_m", &wsl_perception::PipelineConfig::min_range_m, 0.0, 10.0,
   "[m] Points closer to the sensor origin are dropped (robot body, near-field noise)."},
  {"max_range_m", &wsl_perception::PipelineConfig::max_range_m, 0.1, 200.0,
   "[m] Points farther from the sensor origin are dropped."},
  {"roi_min_x_m", &wsl_perception::PipelineConfig::roi_min_x_m, -100.0, 100.0,
   "[m] ROI box, target frame."},
  {"roi_max_x_m", &wsl_perception::PipelineConfig::roi_max_x_m, -100.0, 100.0,
   "[m] ROI box, target frame."},
  {"roi_min_y_m", &wsl_perception::PipelineConfig::roi_min_y_m, -100.0, 100.0,
   "[m] ROI box, target frame."},
  {"roi_max_y_m", &wsl_perception::PipelineConfig::roi_max_y_m, -100.0, 100.0,
   "[m] ROI box, target frame."},
  {"roi_min_z_m", &wsl_perception::PipelineConfig::roi_min_z_m, -10.0, 10.0,
   "[m] ROI box, target frame."},
  {"roi_max_z_m", &wsl_perception::PipelineConfig::roi_max_z_m, -10.0, 10.0,
   "[m] ROI box, target frame. Points above the robot's height cannot block it."},
  {"voxel_leaf_size_m", &wsl_perception::PipelineConfig::voxel_leaf_size_m, 0.01, 2.0,
   "[m] Edge length of the voxel grid used for downsampling."},
  {"ground_max_height_m", &wsl_perception::PipelineConfig::ground_max_height_m, -1.0, 2.0,
   "[m] Only points below this height are ground-plane candidates."},
  {"ground_distance_threshold_m", &wsl_perception::PipelineConfig::ground_distance_threshold_m,
   0.005, 0.5, "[m] Maximum point-to-plane distance for a ground point."},
  {"ground_max_tilt_rad", &wsl_perception::PipelineConfig::ground_max_tilt_rad, 0.01, 1.0,
   "[rad] Maximum angle between the ground normal and the target-frame z axis."},
  {"cluster_tolerance_m", &wsl_perception::PipelineConfig::cluster_tolerance_m, 0.02, 5.0,
   "[m] Euclidean distance that joins two obstacle points into one cluster."},
}};

constexpr DoubleParam kExpectedRate{
  "expected_rate_hz", nullptr, 0.1, 100.0,
  "[Hz] Nominal LiDAR rate; diagnostics warn below 80% of it."};
constexpr DoubleParam kTfTimeout{
  "tf_timeout_s", nullptr, 0.0, 1.0,
  "[s] Maximum wait for the sensor-to-target transform per cloud."};

constexpr std::array<IntParam, 4> kIntParams{{
  {"ground_ransac_iterations", &wsl_perception::PipelineConfig::ground_ransac_iterations, 1, 10000,
   "RANSAC iterations for the ground plane."},
  {"ground_min_inliers", &wsl_perception::PipelineConfig::ground_min_inliers, 3, 1000000,
   "Minimum inliers to accept a ground plane."},
  {"cluster_min_points", &wsl_perception::PipelineConfig::cluster_min_points, 1, 1000000,
   "Clusters with fewer points are discarded as noise."},
  {"cluster_max_points", &wsl_perception::PipelineConfig::cluster_max_points, 1, 10000000,
   "Clusters with more points are discarded (e.g. long walls)."},
}};

// Rolling statistics over one diagnostics period.
struct WindowStats
{
  std::size_t clouds{0};
  std::size_t rejected{0};
  std::size_t ground_found{0};
  double total_ms_sum{0.0};
  double total_ms_max{0.0};

  void add(const PipelineResult & result)
  {
    ++clouds;
    ground_found += result.ground_found ? 1 : 0;
    total_ms_sum += result.timings.total_ms;
    total_ms_max = std::max(total_ms_max, result.timings.total_ms);
  }
};

}  // namespace

class PerceptionNode : public rclcpp::Node
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options)
  : Node("perception", options), diagnostics_(this)
  {
    PipelineConfig config;
    for (const auto & p : kDoubleParams) {
      config.*p.field = declare_parameter(p.name, config.*p.field, describe(p));
    }
    for (const auto & p : kIntParams) {
      config.*p.field = static_cast<int>(declare_parameter(p.name, config.*p.field, describe(p)));
    }
    target_frame_ = declare_parameter("target_frame", "base_footprint");
    if (target_frame_.empty()) {
      throw std::invalid_argument("target_frame must not be empty");
    }
    expected_rate_hz_ = declare_parameter(kExpectedRate.name, 10.0, describe(kExpectedRate));
    tf_timeout_ =
      rclcpp::Duration::from_seconds(declare_parameter(kTfTimeout.name, 0.1, describe(kTfTimeout)));
    window_start_sim_ = now();
    // Throws std::invalid_argument on inconsistent configuration: fail at startup.
    pipeline_.emplace(config);

    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) { return onSetParameters(params); });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
    filtered_pub_ = create_publisher<PointCloud2>("perception/filtered_points", reliable);
    obstacle_pub_ = create_publisher<PointCloud2>("perception/obstacle_points", reliable);
    cluster_pub_ =
      create_publisher<vision_msgs::msg::Detection3DArray>("perception/clusters", reliable);
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("perception/markers", reliable);
    cloud_sub_ = create_subscription<PointCloud2>(
      "lidar/points", rclcpp::SensorDataQoS().keep_last(1),
      [this](PointCloud2::ConstSharedPtr msg) { onCloud(*msg); });

    diagnostics_.setHardwareID("lidar");
    diagnostics_.add("point cloud pipeline", this, &PerceptionNode::diagnose);

    RCLCPP_INFO(
      get_logger(),
      "Perception ready: %s -> %s | range [%.2f, %.2f] m | voxel %.3f m | ROI z [%.2f, %.2f] m | "
      "cluster tol %.2f m, size [%d, %d] | expected %.1f Hz",
      cloud_sub_->get_topic_name(), target_frame_.c_str(), config.min_range_m, config.max_range_m,
      config.voxel_leaf_size_m, config.roi_min_z_m, config.roi_max_z_m, config.cluster_tolerance_m,
      config.cluster_min_points, config.cluster_max_points, expected_rate_hz_);
  }

private:
  template <typename Param>
  static rcl_interfaces::msg::ParameterDescriptor describe(const Param & p)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = p.description;
    if constexpr (std::is_same_v<Param, DoubleParam>) {
      descriptor.floating_point_range.resize(1);
      descriptor.floating_point_range[0].from_value = p.min;
      descriptor.floating_point_range[0].to_value = p.max;
    } else {
      descriptor.integer_range.resize(1);
      descriptor.integer_range[0].from_value = p.min;
      descriptor.integer_range[0].to_value = p.max;
    }
    return descriptor;
  }

  // Applies a runtime parameter change only if the resulting configuration is
  // consistent as a whole (e.g. min_range_m < max_range_m).
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    PipelineConfig candidate = pipeline_->config();
    for (const auto & param : params) {
      const auto & name = param.get_name();
      if (name == "target_frame" || name == "expected_rate_hz" || name == "tf_timeout_s") {
        result.successful = false;
        result.reason = name + " is read-only at runtime";
        return result;
      }
      for (const auto & p : kDoubleParams) {
        if (name == p.name) {
          candidate.*p.field = param.as_double();
        }
      }
      for (const auto & p : kIntParams) {
        if (name == p.name) {
          candidate.*p.field = static_cast<int>(param.as_int());
        }
      }
    }
    try {
      pipeline_.emplace(candidate);
      RCLCPP_INFO(get_logger(), "Pipeline reconfigured (%zu parameter(s))", params.size());
    } catch (const std::invalid_argument & e) {
      result.successful = false;
      result.reason = e.what();
    }
    return result;
  }

  std::optional<Eigen::Isometry3f> lookupSensorTransform(const std_msgs::msg::Header & header)
  {
    try {
      const auto transform =
        tf_buffer_->lookupTransform(target_frame_, header.frame_id, header.stamp, tf_timeout_);
      return tf2::transformToEigen(transform).cast<float>();
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot transform %s -> %s at t=%.3f s: %s",
        header.frame_id.c_str(), target_frame_.c_str(), rclcpp::Time(header.stamp).seconds(),
        e.what());
      return std::nullopt;
    }
  }

  void onCloud(const PointCloud2 & msg)
  {
    const rclcpp::Time stamp(msg.header.stamp, RCL_ROS_TIME);
    if (last_stamp_ && stamp < *last_stamp_) {
      RCLCPP_WARN(
        get_logger(), "Cloud timestamp moved backwards from %.3f s to %.3f s (simulation reset?)",
        last_stamp_->seconds(), stamp.seconds());
      window_ = WindowStats{};
    } else if (last_stamp_ && stamp == *last_stamp_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping cloud with duplicate stamp %.3f s",
        stamp.seconds());
      ++window_.rejected;
      return;
    }
    last_stamp_ = stamp;
    last_cloud_wall_ = std::chrono::steady_clock::now();

    const auto sensor_to_target = lookupSensorTransform(msg.header);
    if (!sensor_to_target) {
      ++window_.rejected;
      return;
    }

    const PipelineResult result = pipeline_->process(msg, *sensor_to_target);
    if (result.error != CloudError::kNone) {
      ++window_.rejected;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected cloud from %s at t=%.3f s: %s",
        msg.header.frame_id.c_str(), stamp.seconds(), toString(result.error).data());
      return;
    }
    window_.add(result);
    last_result_summary_ = {
      result.extraction.input_points, result.filtered->size(), result.obstacles->size(),
      result.clusters.size()};

    std_msgs::msg::Header header;
    header.stamp = msg.header.stamp;
    header.frame_id = target_frame_;
    publishCloud(*filtered_pub_, *result.filtered, header);
    publishCloud(*obstacle_pub_, *result.obstacles, header);
    publishClusters(result, header);
  }

  // Serialising a cloud costs as much as several pipeline stages; skip it
  // when nobody listens.
  static void publishCloud(
    rclcpp::Publisher<PointCloud2> & pub, const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const std_msgs::msg::Header & header)
  {
    if (pub.get_subscription_count() == 0) {
      return;
    }
    auto out = std::make_unique<PointCloud2>();
    pcl::toROSMsg(cloud, *out);
    out->header = header;
    pub.publish(std::move(out));
  }

  void publishClusters(const PipelineResult & result, const std_msgs::msg::Header & header)
  {
    if (cluster_pub_->get_subscription_count() > 0) {
      vision_msgs::msg::Detection3DArray detections;
      detections.header = header;
      detections.detections.reserve(result.clusters.size());
      for (std::size_t i = 0; i < result.clusters.size(); ++i) {
        const Cluster & c = result.clusters[i];
        vision_msgs::msg::Detection3D detection;
        detection.header = header;
        detection.id = std::to_string(i);
        const Eigen::Vector3f centre = 0.5f * (c.min + c.max);
        const Eigen::Vector3f size = c.max - c.min;
        detection.bbox.center.position.x = centre.x();
        detection.bbox.center.position.y = centre.y();
        detection.bbox.center.position.z = centre.z();
        detection.bbox.center.orientation.w = 1.0;
        detection.bbox.size.x = size.x();
        detection.bbox.size.y = size.y();
        detection.bbox.size.z = size.z();
        detections.detections.push_back(std::move(detection));
      }
      cluster_pub_->publish(detections);
    }

    if (marker_pub_->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker clear;
      clear.header = header;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(clear);
      for (std::size_t i = 0; i < result.clusters.size(); ++i) {
        const Cluster & c = result.clusters[i];
        visualization_msgs::msg::Marker box;
        box.header = header;
        box.ns = "clusters";
        box.id = static_cast<int>(i);
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.action = visualization_msgs::msg::Marker::ADD;
        const Eigen::Vector3f centre = 0.5f * (c.min + c.max);
        // Flat clusters (single scan line) would otherwise be invisible.
        const Eigen::Vector3f size = (c.max - c.min).cwiseMax(Eigen::Vector3f::Constant(0.05f));
        box.pose.position.x = centre.x();
        box.pose.position.y = centre.y();
        box.pose.position.z = centre.z();
        box.pose.orientation.w = 1.0;
        box.scale.x = size.x();
        box.scale.y = size.y();
        box.scale.z = size.z();
        box.color.r = 0.1f;
        box.color.g = 0.8f;
        box.color.b = 0.7f;
        box.color.a = 0.45f;
        markers.markers.push_back(box);
      }
      marker_pub_->publish(markers);
    }
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const auto now = std::chrono::steady_clock::now();
    const double window_s = std::chrono::duration<double>(now - window_start_wall_).count();
    const double sim_window_s = (this->now() - window_start_sim_).seconds();
    window_start_wall_ = now;
    window_start_sim_ = this->now();

    // Rates are in simulated time so they stay meaningful at RTF != 1.
    const double rate_hz = sim_window_s > 0.0 ? window_.clouds / sim_window_s : 0.0;
    const double mean_ms = window_.clouds ? window_.total_ms_sum / window_.clouds : 0.0;
    const double since_last_s =
      last_cloud_wall_ ? std::chrono::duration<double>(now - *last_cloud_wall_).count() : -1.0;

    if (!last_cloud_wall_ || since_last_s > 2.0) {
      status.summary(DiagnosticStatus::ERROR, "no point clouds received");
    } else if (window_.clouds == 0) {
      status.summary(DiagnosticStatus::ERROR, "all point clouds rejected");
    } else if (rate_hz < 0.8 * expected_rate_hz_) {
      status.summary(DiagnosticStatus::WARN, "input rate below 80% of expected");
    } else if (mean_ms > 1000.0 / expected_rate_hz_) {
      status.summary(DiagnosticStatus::WARN, "processing slower than sensor period");
    } else {
      status.summary(DiagnosticStatus::OK, "ok");
    }
    status.add("input rate [Hz, sim time]", rate_hz);
    status.add("expected rate [Hz]", expected_rate_hz_);
    status.add("processing mean [ms]", mean_ms);
    status.add("processing max [ms]", window_.total_ms_max);
    status.add("clouds processed", window_.clouds);
    status.add("clouds rejected", window_.rejected);
    status.add("ground found", window_.ground_found);
    status.add("raw points (last)", last_result_summary_[0]);
    status.add("filtered points (last)", last_result_summary_[1]);
    status.add("obstacle points (last)", last_result_summary_[2]);
    status.add("clusters (last)", last_result_summary_[3]);
    status.add("window [s, wall]", window_s);
    window_ = WindowStats{};
  }

  std::optional<CloudPipeline> pipeline_;
  std::string target_frame_;
  double expected_rate_hz_{10.0};
  rclcpp::Duration tf_timeout_{0, 0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr cluster_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  diagnostic_updater::Updater diagnostics_;

  std::optional<rclcpp::Time> last_stamp_;
  std::optional<std::chrono::steady_clock::time_point> last_cloud_wall_;
  WindowStats window_;
  std::array<std::size_t, 4> last_result_summary_{};
  std::chrono::steady_clock::time_point window_start_wall_{std::chrono::steady_clock::now()};
  rclcpp::Time window_start_sim_{0, 0, RCL_ROS_TIME};
};

}  // namespace wsl_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<wsl_perception::PerceptionNode>(rclcpp::NodeOptions{}));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception"), "Startup failed: %s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
