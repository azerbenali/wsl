// Copyright 2026 Azer Ben Ali
// SPDX-License-Identifier: Apache-2.0
//
// Analytic multi-beam LiDAR model used by tests and the pipeline benchmark.
// Rays are cast in the sensor frame against a horizontal ground plane and
// axis-aligned boxes; rays without a return produce +inf, as Gazebo's
// gpu_lidar does.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace wsl_perception::test
{

struct Box
{
  Eigen::Vector3f min;
  Eigen::Vector3f max;
};

struct Scene
{
  // Ground height in the sensor frame (e.g. -0.5 for a sensor 0.5 m up).
  float ground_z{-0.5f};
  bool has_ground{true};
  std::vector<Box> boxes;
};

struct ScanPattern
{
  int channels{16};
  int samples{900};
  float vertical_half_fov_rad{15.0f * static_cast<float>(M_PI) / 180.0f};
  float max_range_m{30.0f};
};

inline sensor_msgs::msg::PointCloud2 makeXyzCloud(
  std::size_t width, std::size_t height, const std::string & frame_id = "lidar_link")
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp.sec = 100;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(width * height);
  msg.width = static_cast<uint32_t>(width);
  msg.height = static_cast<uint32_t>(height);
  msg.row_step = msg.width * msg.point_step;
  return msg;
}

inline sensor_msgs::msg::PointCloud2 makeCloud(
  const std::vector<Eigen::Vector3f> & points, const std::string & frame_id = "lidar_link")
{
  auto msg = makeXyzCloud(points.size(), 1, frame_id);
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  for (const auto & p : points) {
    *x = p.x();
    *y = p.y();
    *z = p.z();
    ++x;
    ++y;
    ++z;
  }
  return msg;
}

// Distance along the unit ray `dir` from the origin to the box, or +inf.
inline float rayBox(const Eigen::Vector3f & dir, const Box & box)
{
  float t_near = 0.0f;
  float t_far = std::numeric_limits<float>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(dir[axis]) < 1e-9f) {
      if (box.min[axis] > 0.0f || box.max[axis] < 0.0f) {
        return std::numeric_limits<float>::infinity();
      }
      continue;
    }
    float t0 = box.min[axis] / dir[axis];
    float t1 = box.max[axis] / dir[axis];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    t_near = std::max(t_near, t0);
    t_far = std::min(t_far, t1);
    if (t_near > t_far) {
      return std::numeric_limits<float>::infinity();
    }
  }
  return t_near > 0.0f ? t_near : std::numeric_limits<float>::infinity();
}

inline sensor_msgs::msg::PointCloud2 simulateScan(
  const Scene & scene, const ScanPattern & pattern = ScanPattern{})
{
  auto msg = makeXyzCloud(pattern.samples, pattern.channels);
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  const float inf = std::numeric_limits<float>::infinity();
  for (int ring = 0; ring < pattern.channels; ++ring) {
    const float elevation = pattern.channels == 1 ? 0.0f
                                                  : -pattern.vertical_half_fov_rad +
                                                      2.0f * pattern.vertical_half_fov_rad * ring /
                                                        (pattern.channels - 1);
    for (int col = 0; col < pattern.samples; ++col) {
      const float azimuth =
        -static_cast<float>(M_PI) + 2.0f * static_cast<float>(M_PI) * col / pattern.samples;
      const Eigen::Vector3f dir(
        std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth),
        std::sin(elevation));
      float t = inf;
      if (scene.has_ground && dir.z() < 0.0f) {
        t = scene.ground_z / dir.z();
      }
      for (const auto & box : scene.boxes) {
        t = std::min(t, rayBox(dir, box));
      }
      const bool hit = t <= pattern.max_range_m;
      *x = hit ? t * dir.x() : inf;
      *y = hit ? t * dir.y() : inf;
      *z = hit ? t * dir.z() : inf;
      ++x;
      ++y;
      ++z;
    }
  }
  return msg;
}

}  // namespace wsl_perception::test
