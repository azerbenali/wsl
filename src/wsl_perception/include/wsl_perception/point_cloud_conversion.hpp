// Copyright 2026 Azer Ben Ali
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace wsl_perception
{

enum class CloudError : std::uint8_t
{
  kNone,
  kEmpty,
  kMissingFrameId,
  kMissingXyzFields,
  kUnsupportedFieldType,
  kBigEndian,
  kInconsistentLayout,
};

std::string_view toString(CloudError error);

struct ExtractionStats
{
  std::size_t input_points{0};
  std::size_t non_finite{0};
  std::size_t out_of_range{0};
};

// Reads x/y/z directly from the PointCloud2 byte buffer, dropping non-finite
// points and points whose distance from the sensor origin is outside
// [min_range_m, max_range_m]. Doing validation, NaN/Inf removal and the range
// test in one pass avoids materialising the full 32-byte-per-point cloud as an
// intermediate PCL cloud.
//
// The message is treated as untrusted: the layout (point_step, row_step,
// field offsets, data size) is checked before any byte is read.
CloudError extractPointsInRange(
  const sensor_msgs::msg::PointCloud2 & msg, double min_range_m, double max_range_m,
  pcl::PointCloud<pcl::PointXYZ> & out, ExtractionStats & stats);

}  // namespace wsl_perception
