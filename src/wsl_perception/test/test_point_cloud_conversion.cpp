// Copyright 2026 Azer Ben Ali
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "support/synthetic_scan.hpp"
#include "wsl_perception/point_cloud_conversion.hpp"

namespace wsl_perception
{
namespace
{

using test::makeCloud;
using test::makeXyzCloud;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

CloudError extract(const sensor_msgs::msg::PointCloud2 & msg, Cloud & out, ExtractionStats & stats)
{
  return extractPointsInRange(msg, 0.5, 10.0, out, stats);
}

TEST(PointCloudConversion, EmptyCloudIsRejected)
{
  Cloud out;
  ExtractionStats stats;
  EXPECT_EQ(extract(makeXyzCloud(0, 1), out, stats), CloudError::kEmpty);
  EXPECT_EQ(extract(makeXyzCloud(5, 0), out, stats), CloudError::kEmpty);
}

TEST(PointCloudConversion, MissingFrameIdIsRejected)
{
  Cloud out;
  ExtractionStats stats;
  EXPECT_EQ(extract(makeCloud({{1, 0, 0}}, ""), out, stats), CloudError::kMissingFrameId);
}

TEST(PointCloudConversion, MissingZFieldIsRejected)
{
  auto msg = makeCloud({{1, 0, 0}});
  msg.fields[2].name = "ring";
  Cloud out;
  ExtractionStats stats;
  EXPECT_EQ(extract(msg, out, stats), CloudError::kMissingXyzFields);
}

TEST(PointCloudConversion, NonFloatFieldIsRejected)
{
  auto msg = makeCloud({{1, 0, 0}});
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
  Cloud out;
  ExtractionStats stats;
  EXPECT_EQ(extract(msg, out, stats), CloudError::kUnsupportedFieldType);
}

TEST(PointCloudConversion, BigEndianIsRejected)
{
  auto msg = makeCloud({{1, 0, 0}});
  msg.is_bigendian = true;
  Cloud out;
  ExtractionStats stats;
  EXPECT_EQ(extract(msg, out, stats), CloudError::kBigEndian);
}

TEST(PointCloudConversion, InconsistentLayoutsAreRejectedWithoutReadingOutOfBounds)
{
  Cloud out;
  ExtractionStats stats;

  auto truncated = makeCloud({{1, 0, 0}, {2, 0, 0}});
  truncated.data.resize(truncated.data.size() - 1);
  EXPECT_EQ(extract(truncated, out, stats), CloudError::kInconsistentLayout);

  auto zero_row_step = makeCloud({{1, 0, 0}});
  zero_row_step.row_step = 0;
  EXPECT_EQ(extract(zero_row_step, out, stats), CloudError::kInconsistentLayout);

  auto zero_point_step = makeCloud({{1, 0, 0}});
  zero_point_step.point_step = 0;
  EXPECT_EQ(extract(zero_point_step, out, stats), CloudError::kInconsistentLayout);

  auto field_past_point = makeCloud({{1, 0, 0}});
  field_past_point.fields[2].offset = field_past_point.point_step - 2;
  EXPECT_EQ(extract(field_past_point, out, stats), CloudError::kInconsistentLayout);

  auto huge_width = makeCloud({{1, 0, 0}});
  huge_width.width = std::numeric_limits<uint32_t>::max();
  EXPECT_EQ(extract(huge_width, out, stats), CloudError::kInconsistentLayout);
}

TEST(PointCloudConversion, NonFinitePointsAreDroppedAndCounted)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const auto msg = makeCloud({{1, 0, 0}, {nan, 0, 0}, {0, inf, 0}, {0, 0, -inf}, {0, 2, 0}});
  Cloud out;
  ExtractionStats stats;
  ASSERT_EQ(extract(msg, out, stats), CloudError::kNone);
  EXPECT_EQ(stats.input_points, 5u);
  EXPECT_EQ(stats.non_finite, 3u);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_TRUE(out.is_dense);
}

TEST(PointCloudConversion, RangeLimitsAreInclusive)
{
  const auto msg =
    makeCloud({{0.5f, 0, 0}, {0.49f, 0, 0}, {0, 0, 10.0f}, {0, 0, 10.01f}, {3, 4, 0}, {0, 0, 0}});
  Cloud out;
  ExtractionStats stats;
  ASSERT_EQ(extract(msg, out, stats), CloudError::kNone);
  EXPECT_EQ(out.size(), 3u);  // 0.5, 10.0 and the 3-4-5 point
  EXPECT_EQ(stats.out_of_range, 3u);
  for (const auto & p : out) {
    const float r = p.getVector3fMap().norm();
    EXPECT_GE(r, 0.5f);
    EXPECT_LE(r, 10.0f);
  }
}

TEST(PointCloudConversion, HonoursFieldOffsetsAndRowPadding)
{
  // Gazebo-like layout: 32-byte points, xyz not at the start, padded rows.
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "lidar_link";
  msg.height = 2;
  msg.width = 2;
  msg.point_step = 32;
  msg.row_step = 2 * 32 + 8;
  for (const auto & [name, offset] : std::vector<std::pair<std::string, uint32_t>>{
         {"intensity", 0}, {"x", 8}, {"y", 12}, {"z", 16}}) {
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    msg.fields.push_back(f);
  }
  msg.data.assign(msg.row_step * msg.height, 0xFF);  // 0xFF bytes decode as NaN
  auto write = [&](int row, int col, float x, float y, float z) {
    uint8_t * p = msg.data.data() + row * msg.row_step + col * msg.point_step;
    std::memcpy(p + 8, &x, 4);
    std::memcpy(p + 12, &y, 4);
    std::memcpy(p + 16, &z, 4);
  };
  write(0, 0, 1, 0, 0);
  write(0, 1, 2, 0, 0);
  write(1, 0, 3, 0, 0);
  write(1, 1, 4, 0, 0);

  Cloud out;
  ExtractionStats stats;
  ASSERT_EQ(extract(msg, out, stats), CloudError::kNone);
  ASSERT_EQ(out.size(), 4u);
  for (int i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(out[i].x, static_cast<float>(i + 1));
  }
}

TEST(PointCloudConversion, SimulatedScanMissesBecomeNonFinite)
{
  test::Scene scene;  // ground only: every upward ray misses
  const auto msg = test::simulateScan(scene);
  Cloud out;
  ExtractionStats stats;
  ASSERT_EQ(extractPointsInRange(msg, 0.3, 30.0, out, stats), CloudError::kNone);
  EXPECT_EQ(stats.input_points, 16u * 900u);
  EXPECT_EQ(stats.non_finite, 8u * 900u);  // the upper 8 rings see nothing
  EXPECT_EQ(out.size() + stats.out_of_range + stats.non_finite, stats.input_points);
}

}  // namespace
}  // namespace wsl_perception
