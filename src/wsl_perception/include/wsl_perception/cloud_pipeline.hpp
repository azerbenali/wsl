// Copyright 2026 Azer Ben Ali
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "wsl_perception/point_cloud_conversion.hpp"

namespace wsl_perception
{

// All lengths in metres, angles in radians. Range limits are measured from the
// sensor origin; the region of interest is an axis-aligned box in the target
// (robot base) frame, whose z = 0 is nominally the ground.
struct PipelineConfig
{
  double min_range_m{0.5};
  double max_range_m{25.0};

  double roi_min_x_m{-20.0};
  double roi_max_x_m{20.0};
  double roi_min_y_m{-20.0};
  double roi_max_y_m{20.0};
  double roi_min_z_m{-0.3};
  double roi_max_z_m{2.5};

  double voxel_leaf_size_m{0.08};

  // Ground is fitted only to points below this height, so shelf decks or
  // pallet tops (also horizontal planes) cannot win the RANSAC vote.
  double ground_max_height_m{0.3};
  double ground_distance_threshold_m{0.05};
  // Maximum angle between the fitted plane normal and the base z axis.
  double ground_max_tilt_rad{0.15};
  int ground_ransac_iterations{100};
  int ground_min_inliers{50};

  double cluster_tolerance_m{0.25};
  int cluster_min_points{10};
  int cluster_max_points{10000};

  // Throws std::invalid_argument naming the offending parameter.
  void validate() const;
};

struct Cluster
{
  Eigen::Vector3f centroid{Eigen::Vector3f::Zero()};
  Eigen::Vector3f min{Eigen::Vector3f::Zero()};
  Eigen::Vector3f max{Eigen::Vector3f::Zero()};
  std::size_t num_points{0};
};

struct StageTimings
{
  double extract_ms{0.0};
  double transform_roi_ms{0.0};
  double voxel_ms{0.0};
  double ground_ms{0.0};
  double cluster_ms{0.0};
  double total_ms{0.0};
};

struct PipelineResult
{
  CloudError error{CloudError::kNone};
  ExtractionStats extraction;
  // Range-filtered, ROI-cropped and voxelised cloud in the target frame.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered{new pcl::PointCloud<pcl::PointXYZ>};
  // Subset of `filtered` that is not ground.
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles{new pcl::PointCloud<pcl::PointXYZ>};
  bool ground_found{false};
  // Plane n.p + d = 0 with n pointing up (n.z > 0) in the target frame.
  Eigen::Vector4f ground_plane{Eigen::Vector4f::Zero()};
  std::size_t ground_points{0};
  std::vector<Cluster> clusters;
  StageTimings timings;
};

// Stateless point-cloud processing chain:
//   extract (validate, drop NaN/Inf, range) -> transform to target frame ->
//   ROI crop -> voxel grid -> RANSAC ground plane -> Euclidean clustering.
// ROI cropping runs before the voxel grid because it is a cheap per-point test
// that shrinks the voxel grid's sort.
class CloudPipeline
{
public:
  explicit CloudPipeline(const PipelineConfig & config);

  const PipelineConfig & config() const { return config_; }

  PipelineResult process(
    const sensor_msgs::msg::PointCloud2 & msg, const Eigen::Isometry3f & sensor_to_target) const;

  // Ground segmentation and clustering on a cloud already in the target frame.
  void segment(PipelineResult & result) const;

private:
  PipelineConfig config_;
};

}  // namespace wsl_perception
