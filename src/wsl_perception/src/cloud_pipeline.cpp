// Copyright 2026 Azer Ben Ali
// SPDX-License-Identifier: Apache-2.0
#include "wsl_perception/cloud_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace wsl_perception
{
namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point since)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - since).count();
}

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

}  // namespace

void PipelineConfig::validate() const
{
  require(min_range_m >= 0.0, "min_range_m must be >= 0");
  require(max_range_m > min_range_m, "max_range_m must be > min_range_m");
  require(roi_max_x_m > roi_min_x_m, "roi_max_x_m must be > roi_min_x_m");
  require(roi_max_y_m > roi_min_y_m, "roi_max_y_m must be > roi_min_y_m");
  require(roi_max_z_m > roi_min_z_m, "roi_max_z_m must be > roi_min_z_m");
  require(voxel_leaf_size_m > 0.0, "voxel_leaf_size_m must be > 0");
  // pcl::VoxelGrid indexes voxels with a 32-bit integer and silently skips
  // filtering when the grid would overflow it.
  const double voxels_x = std::ceil((roi_max_x_m - roi_min_x_m) / voxel_leaf_size_m);
  const double voxels_y = std::ceil((roi_max_y_m - roi_min_y_m) / voxel_leaf_size_m);
  const double voxels_z = std::ceil((roi_max_z_m - roi_min_z_m) / voxel_leaf_size_m);
  require(
    voxels_x * voxels_y * voxels_z < 2147483647.0,
    "voxel_leaf_size_m is too small for the ROI (voxel index overflows int32)");
  require(ground_max_height_m > roi_min_z_m, "ground_max_height_m must be > roi_min_z_m");
  require(ground_distance_threshold_m > 0.0, "ground_distance_threshold_m must be > 0");
  require(
    ground_max_tilt_rad > 0.0 && ground_max_tilt_rad < M_PI_2,
    "ground_max_tilt_rad must be in (0, pi/2)");
  require(ground_ransac_iterations > 0, "ground_ransac_iterations must be > 0");
  require(ground_min_inliers >= 3, "ground_min_inliers must be >= 3");
  require(cluster_tolerance_m > 0.0, "cluster_tolerance_m must be > 0");
  require(cluster_min_points >= 1, "cluster_min_points must be >= 1");
  require(
    cluster_max_points >= cluster_min_points, "cluster_max_points must be >= cluster_min_points");
}

CloudPipeline::CloudPipeline(const PipelineConfig & config) : config_(config)
{
  config_.validate();
}

PipelineResult CloudPipeline::process(
  const sensor_msgs::msg::PointCloud2 & msg, const Eigen::Isometry3f & sensor_to_target) const
{
  const auto start = Clock::now();
  PipelineResult result;

  Cloud in_range;
  result.error = extractPointsInRange(
    msg, config_.min_range_m, config_.max_range_m, in_range, result.extraction);
  result.timings.extract_ms = elapsedMs(start);
  if (result.error != CloudError::kNone) {
    result.timings.total_ms = elapsedMs(start);
    return result;
  }

  auto stage = Clock::now();
  Cloud cropped;
  cropped.reserve(in_range.size());
  const Eigen::Matrix3f rotation = sensor_to_target.linear();
  const Eigen::Vector3f translation = sensor_to_target.translation();
  for (const auto & p : in_range) {
    const Eigen::Vector3f q = rotation * p.getVector3fMap() + translation;
    if (
      q.x() >= config_.roi_min_x_m && q.x() <= config_.roi_max_x_m &&
      q.y() >= config_.roi_min_y_m && q.y() <= config_.roi_max_y_m &&
      q.z() >= config_.roi_min_z_m && q.z() <= config_.roi_max_z_m) {
      cropped.push_back(pcl::PointXYZ(q.x(), q.y(), q.z()));
    }
  }
  result.timings.transform_roi_ms = elapsedMs(stage);

  stage = Clock::now();
  if (!cropped.empty()) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    const auto leaf = static_cast<float>(config_.voxel_leaf_size_m);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(cropped.makeShared());
    voxel.filter(*result.filtered);
  }
  result.timings.voxel_ms = elapsedMs(stage);

  segment(result);
  result.timings.total_ms = elapsedMs(start);
  return result;
}

void CloudPipeline::segment(PipelineResult & result) const
{
  const Cloud & filtered = *result.filtered;
  auto stage = Clock::now();

  pcl::PointIndices::Ptr candidates(new pcl::PointIndices);
  for (std::size_t i = 0; i < filtered.size(); ++i) {
    if (filtered[i].z <= config_.ground_max_height_m) {
      candidates->indices.push_back(static_cast<int>(i));
    }
  }

  result.ground_found = false;
  if (candidates->indices.size() >= static_cast<std::size_t>(config_.ground_min_inliers)) {
    // PCL seeds its RANSAC generator with a constant unless setRandom(true)
    // is called, so segmentation is deterministic for a given input.
    pcl::SACSegmentation<pcl::PointXYZ> sac;
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setAxis(Eigen::Vector3f::UnitZ());
    sac.setEpsAngle(config_.ground_max_tilt_rad);
    sac.setDistanceThreshold(config_.ground_distance_threshold_m);
    sac.setMaxIterations(config_.ground_ransac_iterations);
    sac.setInputCloud(result.filtered);
    sac.setIndices(candidates);

    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    sac.segment(inliers, coefficients);
    if (
      coefficients.values.size() == 4 &&
      inliers.indices.size() >= static_cast<std::size_t>(config_.ground_min_inliers)) {
      Eigen::Vector4f plane(
        coefficients.values[0], coefficients.values[1], coefficients.values[2],
        coefficients.values[3]);
      plane /= plane.head<3>().norm();
      if (plane.z() < 0.0f) {
        plane = -plane;
      }
      // Coefficient refinement can rotate the plane past the RANSAC angle
      // constraint; reject rather than label a ramp or wall as ground.
      const double tilt = std::acos(std::min(1.0f, plane.z()));
      if (tilt <= config_.ground_max_tilt_rad) {
        result.ground_found = true;
        result.ground_plane = plane;
      }
    }
  }

  result.obstacles->clear();
  result.obstacles->reserve(filtered.size());
  result.ground_points = 0;
  const auto threshold = static_cast<float>(config_.ground_distance_threshold_m);
  for (const auto & p : filtered) {
    // Points below the plane are noise or reflections, not obstacles.
    const bool is_ground =
      result.ground_found &&
      result.ground_plane.head<3>().dot(p.getVector3fMap()) + result.ground_plane.w() <= threshold;
    if (is_ground) {
      ++result.ground_points;
    } else {
      result.obstacles->push_back(p);
    }
  }
  result.obstacles->width = static_cast<std::uint32_t>(result.obstacles->size());
  result.obstacles->height = 1;
  result.obstacles->is_dense = true;
  result.timings.ground_ms = elapsedMs(stage);

  stage = Clock::now();
  result.clusters.clear();
  if (result.obstacles->size() >= static_cast<std::size_t>(config_.cluster_min_points)) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(result.obstacles);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(config_.cluster_tolerance_m);
    extraction.setMinClusterSize(config_.cluster_min_points);
    extraction.setMaxClusterSize(config_.cluster_max_points);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(result.obstacles);
    extraction.extract(cluster_indices);

    result.clusters.reserve(cluster_indices.size());
    for (const auto & indices : cluster_indices) {
      Cluster cluster;
      cluster.min.setConstant(std::numeric_limits<float>::max());
      cluster.max.setConstant(std::numeric_limits<float>::lowest());
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      for (const int index : indices.indices) {
        const Eigen::Vector3f p = (*result.obstacles)[index].getVector3fMap();
        cluster.min = cluster.min.cwiseMin(p);
        cluster.max = cluster.max.cwiseMax(p);
        sum += p.cast<double>();
      }
      cluster.num_points = indices.indices.size();
      cluster.centroid = (sum / static_cast<double>(cluster.num_points)).cast<float>();
      result.clusters.push_back(cluster);
    }
  }
  result.timings.cluster_ms = elapsedMs(stage);
}

}  // namespace wsl_perception
