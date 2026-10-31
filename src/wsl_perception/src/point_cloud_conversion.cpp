// Copyright 2026 Azer Ben Ali
// SPDX-License-Identifier: Apache-2.0
#include "wsl_perception/point_cloud_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>

namespace wsl_perception
{
namespace
{

using sensor_msgs::msg::PointField;

std::optional<std::uint32_t> floatFieldOffset(
  const sensor_msgs::msg::PointCloud2 & msg, std::string_view name, bool & wrong_type)
{
  for (const auto & field : msg.fields) {
    if (field.name != name) {
      continue;
    }
    if (field.datatype != PointField::FLOAT32 || field.count != 1) {
      wrong_type = true;
      return std::nullopt;
    }
    return field.offset;
  }
  return std::nullopt;
}

float readFloat(const std::uint8_t * src)
{
  float value;
  std::memcpy(&value, src, sizeof(value));  // unaligned-safe
  return value;
}

}  // namespace

std::string_view toString(CloudError error)
{
  switch (error) {
    case CloudError::kNone:
      return "ok";
    case CloudError::kEmpty:
      return "empty cloud";
    case CloudError::kMissingFrameId:
      return "missing header.frame_id";
    case CloudError::kMissingXyzFields:
      return "missing x/y/z fields";
    case CloudError::kUnsupportedFieldType:
      return "x/y/z fields are not single FLOAT32 values";
    case CloudError::kBigEndian:
      return "big-endian data is not supported";
    case CloudError::kInconsistentLayout:
      return "point_step/row_step/data size are inconsistent";
  }
  return "unknown";
}

CloudError extractPointsInRange(
  const sensor_msgs::msg::PointCloud2 & msg, double min_range_m, double max_range_m,
  pcl::PointCloud<pcl::PointXYZ> & out, ExtractionStats & stats)
{
  out.clear();
  stats = ExtractionStats{};

  if (msg.header.frame_id.empty()) {
    return CloudError::kMissingFrameId;
  }

  bool wrong_type = false;
  const auto x_offset = floatFieldOffset(msg, "x", wrong_type);
  const auto y_offset = floatFieldOffset(msg, "y", wrong_type);
  const auto z_offset = floatFieldOffset(msg, "z", wrong_type);
  if (wrong_type) {
    return CloudError::kUnsupportedFieldType;
  }
  if (!x_offset || !y_offset || !z_offset) {
    return CloudError::kMissingXyzFields;
  }
  if (msg.is_bigendian) {
    return CloudError::kBigEndian;
  }

  const std::size_t width = msg.width;
  const std::size_t height = msg.height;
  const std::size_t point_step = msg.point_step;
  const std::size_t row_step = msg.row_step;
  if (width == 0 || height == 0) {
    return CloudError::kEmpty;
  }
  const std::size_t max_offset = std::max({*x_offset, *y_offset, *z_offset});
  // Overflow-safe form of: width * point_step <= row_step and height * row_step <= data.size().
  // Evaluation order matters: a zero row_step fails the second test before the division.
  if (
    point_step < max_offset + sizeof(float) || row_step / point_step < width ||
    msg.data.size() / row_step < height) {
    return CloudError::kInconsistentLayout;
  }

  stats.input_points = width * height;
  out.reserve(stats.input_points);

  const double min_sq = min_range_m * min_range_m;
  const double max_sq = max_range_m * max_range_m;
  for (std::size_t row = 0; row < height; ++row) {
    const std::uint8_t * row_ptr = msg.data.data() + row * row_step;
    for (std::size_t col = 0; col < width; ++col) {
      const std::uint8_t * point = row_ptr + col * point_step;
      const float x = readFloat(point + *x_offset);
      const float y = readFloat(point + *y_offset);
      const float z = readFloat(point + *z_offset);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++stats.non_finite;
        continue;
      }
      const double range_sq = double(x) * x + double(y) * y + double(z) * z;
      if (range_sq < min_sq || range_sq > max_sq) {
        ++stats.out_of_range;
        continue;
      }
      out.push_back(pcl::PointXYZ(x, y, z));
    }
  }

  // The output is unorganised; width/height follow PCL's convention for that.
  out.width = static_cast<std::uint32_t>(out.size());
  out.height = 1;
  out.is_dense = true;
  return CloudError::kNone;
}

}  // namespace wsl_perception
