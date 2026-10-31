// Copyright 2026 Azer Ben Ali
// SPDX-License-Identifier: Apache-2.0
//
// libFuzzer entry point for the PointCloud2 parser. The first 24 bytes choose
// the declared layout, the rest become the payload, so the fuzzer can explore
// every combination of lying width/height/step/offset values.
//   colcon build --packages-select wsl_perception --cmake-args \
//     -DWSL_BUILD_FUZZERS=ON -DCMAKE_CXX_COMPILER=clang++
//   ./build/wsl_perception/fuzz_point_cloud_conversion -max_total_time=60
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "wsl_perception/point_cloud_conversion.hpp"

namespace
{

uint32_t take32(const uint8_t *& data, size_t & size)
{
  uint32_t value = 0;
  const size_t n = size < 4 ? size : 4;
  std::memcpy(&value, data, n);
  data += n;
  size -= n;
  return value;
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "lidar_link";
  // Bound the declared dimensions so the fuzzer explores layout bugs rather
  // than legitimately huge allocations.
  msg.width = take32(data, size) % 4096;
  msg.height = take32(data, size) % 64;
  msg.point_step = take32(data, size) % 128;
  msg.row_step = take32(data, size);
  const uint32_t offsets = take32(data, size);
  const uint32_t types = take32(data, size);
  const char * names[] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = (offsets >> (8 * i)) & 0xFF;
    field.datatype = ((types >> (8 * i)) & 1) ? sensor_msgs::msg::PointField::FLOAT32
                                              : sensor_msgs::msg::PointField::FLOAT64;
    field.count = 1;
    msg.fields.push_back(field);
  }
  msg.data.assign(data, data + size);

  pcl::PointCloud<pcl::PointXYZ> out;
  wsl_perception::ExtractionStats stats;
  if (
    wsl_perception::extractPointsInRange(msg, 0.0, 1e30, out, stats) ==
    wsl_perception::CloudError::kNone) {
    for (const auto & p : out) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        std::abort();
      }
    }
  }
  return 0;
}
