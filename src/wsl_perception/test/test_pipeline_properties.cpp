// Copyright 2026 Azer Ben Ali
// SPDX-License-Identifier: Apache-2.0
//
// Randomised property tests. Each trial draws a valid configuration and a
// cloud mixing realistic returns with NaN, Inf, extreme and duplicate points,
// then checks invariants that must hold for every input. The generator is
// seeded, so a failure reproduces exactly; the trial index is reported.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#include "support/synthetic_scan.hpp"
#include "wsl_perception/cloud_pipeline.hpp"

namespace wsl_perception
{
namespace
{

constexpr int kTrials = 300;

PipelineConfig randomConfig(std::mt19937 & rng)
{
  auto uniform = [&](double lo, double hi) {
    return std::uniform_real_distribution<>(lo, hi)(rng);
  };
  PipelineConfig c;
  c.min_range_m = uniform(0.0, 2.0);
  c.max_range_m = c.min_range_m + uniform(0.5, 40.0);
  c.roi_min_x_m = uniform(-20.0, 0.0);
  c.roi_max_x_m = c.roi_min_x_m + uniform(0.5, 30.0);
  c.roi_min_y_m = uniform(-20.0, 0.0);
  c.roi_max_y_m = c.roi_min_y_m + uniform(0.5, 30.0);
  c.roi_min_z_m = uniform(-1.0, 0.0);
  c.roi_max_z_m = c.roi_min_z_m + uniform(0.3, 4.0);
  c.voxel_leaf_size_m = uniform(0.03, 0.5);
  c.ground_max_height_m = c.roi_min_z_m + uniform(0.1, 1.0);
  c.ground_distance_threshold_m = uniform(0.01, 0.2);
  c.cluster_tolerance_m = uniform(0.05, 1.0);
  c.cluster_min_points = std::uniform_int_distribution<>(1, 20)(rng);
  c.cluster_max_points = c.cluster_min_points + std::uniform_int_distribution<>(0, 5000)(rng);
  c.validate();
  return c;
}

sensor_msgs::msg::PointCloud2 randomCloud(std::mt19937 & rng)
{
  test::Scene scene;
  scene.has_ground = std::bernoulli_distribution(0.8)(rng);
  std::uniform_real_distribution<float> pos(-15.0f, 15.0f), extent(0.1f, 3.0f);
  const int boxes = std::uniform_int_distribution<>(0, 8)(rng);
  for (int i = 0; i < boxes; ++i) {
    const Eigen::Vector3f min(pos(rng), pos(rng), -0.5f);
    scene.boxes.push_back({min, min + Eigen::Vector3f(extent(rng), extent(rng), extent(rng))});
  }
  test::ScanPattern pattern;
  pattern.channels = std::uniform_int_distribution<>(1, 16)(rng);
  pattern.samples = std::uniform_int_distribution<>(1, 360)(rng);
  auto msg = test::simulateScan(scene, pattern);

  // Corrupt a fraction of points with special and extreme values.
  const float specials[] = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    -std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::max(),
    std::numeric_limits<float>::lowest(),
    0.0f,
    1e-30f};
  std::uniform_int_distribution<std::size_t> point(0, msg.width * msg.height - 1);
  std::uniform_int_distribution<std::size_t> special(0, std::size(specials) - 1);
  std::uniform_int_distribution<uint32_t> axis(0, 2);
  const int corruptions = std::uniform_int_distribution<>(0, 50)(rng);
  for (int i = 0; i < corruptions; ++i) {
    const std::size_t offset = point(rng) * msg.point_step + 4 * axis(rng);
    std::memcpy(&msg.data[offset], &specials[special(rng)], sizeof(float));
  }
  return msg;
}

Eigen::Isometry3f randomPose(std::mt19937 & rng)
{
  std::uniform_real_distribution<float> angle(-0.1f, 0.1f), shift(-0.5f, 0.5f);
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.rotate(Eigen::AngleAxisf(angle(rng), Eigen::Vector3f::UnitX()));
  t.rotate(Eigen::AngleAxisf(angle(rng), Eigen::Vector3f::UnitY()));
  t.rotate(Eigen::AngleAxisf(3.0f * angle(rng), Eigen::Vector3f::UnitZ()));
  t.translation() = Eigen::Vector3f(shift(rng), shift(rng), 0.5f + shift(rng));
  return t;
}

bool inRoi(const pcl::PointXYZ & p, const PipelineConfig & c)
{
  // Voxel centroids are averages of in-ROI points, so they stay in the
  // (convex) box up to float rounding.
  constexpr float eps = 1e-4f;
  return p.x >= c.roi_min_x_m - eps && p.x <= c.roi_max_x_m + eps && p.y >= c.roi_min_y_m - eps &&
         p.y <= c.roi_max_y_m + eps && p.z >= c.roi_min_z_m - eps && p.z <= c.roi_max_z_m + eps;
}

TEST(PipelineProperties, ExtractionKeepsOnlyFiniteInRangePoints)
{
  std::mt19937 rng(20260101);
  for (int trial = 0; trial < kTrials; ++trial) {
    SCOPED_TRACE(testing::Message() << "trial " << trial);
    const PipelineConfig config = randomConfig(rng);
    const auto msg = randomCloud(rng);
    pcl::PointCloud<pcl::PointXYZ> out;
    ExtractionStats stats;
    ASSERT_EQ(
      extractPointsInRange(msg, config.min_range_m, config.max_range_m, out, stats),
      CloudError::kNone);
    ASSERT_EQ(out.size() + stats.non_finite + stats.out_of_range, stats.input_points);
    for (const auto & p : out) {
      ASSERT_TRUE(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
      const double r = std::sqrt(double(p.x) * p.x + double(p.y) * p.y + double(p.z) * p.z);
      ASSERT_GE(r, config.min_range_m * (1 - 1e-9));
      ASSERT_LE(r, config.max_range_m * (1 + 1e-9));
    }
  }
}

TEST(PipelineProperties, PipelineInvariantsHoldForRandomInputs)
{
  std::mt19937 rng(42);
  for (int trial = 0; trial < kTrials; ++trial) {
    SCOPED_TRACE(testing::Message() << "trial " << trial);
    const PipelineConfig config = randomConfig(rng);
    const auto msg = randomCloud(rng);
    const auto result = CloudPipeline{config}.process(msg, randomPose(rng));
    ASSERT_EQ(result.error, CloudError::kNone);

    const auto & filtered = *result.filtered;
    ASSERT_LE(filtered.size(), result.extraction.input_points);
    for (const auto & p : filtered) {
      ASSERT_TRUE(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
      ASSERT_TRUE(inRoi(p, config)) << p;
    }
    ASSERT_EQ(result.ground_points + result.obstacles->size(), filtered.size());
    if (!result.ground_found) {
      ASSERT_EQ(result.ground_points, 0u);
    } else {
      ASSERT_NEAR(result.ground_plane.head<3>().norm(), 1.0f, 1e-4f);
      ASSERT_LE(std::acos(result.ground_plane.z()), config.ground_max_tilt_rad + 1e-4);
    }

    std::size_t clustered = 0;
    for (const auto & c : result.clusters) {
      ASSERT_GE(c.num_points, static_cast<std::size_t>(config.cluster_min_points));
      ASSERT_LE(c.num_points, static_cast<std::size_t>(config.cluster_max_points));
      ASSERT_TRUE((c.centroid.array() >= c.min.array() - 1e-4f).all());
      ASSERT_TRUE((c.centroid.array() <= c.max.array() + 1e-4f).all());
      clustered += c.num_points;
    }
    ASSERT_LE(clustered, result.obstacles->size());
  }
}

TEST(PipelineProperties, ArbitraryBytesNeverCrashTheParser)
{
  // Light-weight fuzzing of the untrusted-layout path; the libFuzzer target
  // (WSL_BUILD_FUZZERS) explores the same entry point far more deeply.
  std::mt19937 rng(7);
  std::uniform_int_distribution<int> byte(0, 255), small(0, 64);
  for (int trial = 0; trial < 2000; ++trial) {
    auto msg = test::makeXyzCloud(small(rng), small(rng) % 4);
    msg.point_step = small(rng);
    msg.row_step = small(rng) * small(rng);
    for (auto & field : msg.fields) {
      field.offset = small(rng);
    }
    msg.data.resize(small(rng) * small(rng));
    for (auto & b : msg.data) {
      b = static_cast<uint8_t>(byte(rng));
    }
    pcl::PointCloud<pcl::PointXYZ> out;
    ExtractionStats stats;
    if (extractPointsInRange(msg, 0.0, 1e9, out, stats) == CloudError::kNone) {
      for (const auto & p : out) {
        ASSERT_TRUE(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
      }
    }
  }
}

}  // namespace
}  // namespace wsl_perception
