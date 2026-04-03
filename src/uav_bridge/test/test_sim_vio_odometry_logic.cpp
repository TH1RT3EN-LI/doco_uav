#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

#include "uav_bridge/sim_vio_odometry_logic.hpp"

namespace
{

using uav_bridge::SimVioOdometryConfig;
using uav_bridge::SimVioOdometryLogic;
using uav_bridge::SimVioOdometrySample;

SimVioOdometrySample makeSample(
  int64_t stamp_ns,
  double x,
  double y,
  double z)
{
  SimVioOdometrySample sample;
  sample.stamp_ns = stamp_ns;
  sample.frame_id = "uav_ground_truth_odom";
  sample.child_frame_id = "uav_ground_truth_base_link";
  sample.position = {x, y, z};
  sample.orientation_xyzw = {0.0, 0.0, 0.0, 1.0};
  sample.linear_velocity = {1.0, 2.0, 3.0};
  sample.angular_velocity = {0.1, 0.2, 0.3};
  sample.pose_covariance[0] = 0.02;
  sample.pose_covariance[7] = 0.02;
  sample.pose_covariance[14] = 0.02;
  sample.pose_covariance[21] = 0.05;
  sample.pose_covariance[28] = 0.05;
  sample.pose_covariance[35] = 0.05;
  sample.twist_covariance[0] = 0.10;
  sample.twist_covariance[7] = 0.10;
  sample.twist_covariance[14] = 0.10;
  return sample;
}

std::vector<SimVioOdometrySample> collectOutputs(
  SimVioOdometryLogic & logic,
  int64_t start_ns,
  int64_t end_ns,
  int64_t step_ns)
{
  std::vector<SimVioOdometrySample> outputs;
  for (int64_t now_ns = start_ns; now_ns <= end_ns; now_ns += step_ns) {
    const auto output = logic.step(now_ns);
    if (output.has_value()) {
      outputs.push_back(*output);
    }
  }
  return outputs;
}

void expectSamplesNear(const SimVioOdometrySample & lhs, const SimVioOdometrySample & rhs)
{
  EXPECT_EQ(lhs.stamp_ns, rhs.stamp_ns);
  EXPECT_EQ(lhs.frame_id, rhs.frame_id);
  EXPECT_EQ(lhs.child_frame_id, rhs.child_frame_id);
  for (size_t i = 0; i < lhs.position.size(); ++i) {
    EXPECT_NEAR(lhs.position[i], rhs.position[i], 1.0e-12);
    EXPECT_NEAR(lhs.linear_velocity[i], rhs.linear_velocity[i], 1.0e-12);
    EXPECT_NEAR(lhs.angular_velocity[i], rhs.angular_velocity[i], 1.0e-12);
  }
  for (size_t i = 0; i < lhs.orientation_xyzw.size(); ++i) {
    EXPECT_NEAR(lhs.orientation_xyzw[i], rhs.orientation_xyzw[i], 1.0e-12);
  }
}

}  // namespace

TEST(SimVioOdometryLogicTest, ZeroNoiseZeroDelayTracksGroundTruthAndRewritesFrames)
{
  SimVioOdometryConfig config;
  config.output_delay_s = 0.0;
  config.warmup_duration_s = 0.0;
  config.position_noise_stddev_xy_m = 0.0;
  config.position_noise_stddev_z_m = 0.0;
  config.roll_pitch_noise_stddev_rad = 0.0;
  config.yaw_noise_stddev_rad = 0.0;
  config.linear_velocity_noise_stddev_mps = 0.0;
  config.position_random_walk_xy_m_per_sqrt_s = 0.0;
  config.position_random_walk_z_m_per_sqrt_s = 0.0;
  config.yaw_random_walk_rad_per_sqrt_s = 0.0;
  config.dropout_interval_mean_s = 0.0;
  config.dropout_duration_s = 0.0;

  SimVioOdometryLogic logic(config);
  logic.observeGroundTruth(makeSample(100000000LL, 1.0, 2.0, 3.0));

  const auto output = logic.step(100000000LL);
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->frame_id, "global");
  EXPECT_EQ(output->child_frame_id, "imu");
  EXPECT_DOUBLE_EQ(output->position[0], 1.0);
  EXPECT_DOUBLE_EQ(output->position[1], 2.0);
  EXPECT_DOUBLE_EQ(output->position[2], 3.0);
  EXPECT_DOUBLE_EQ(output->linear_velocity[0], 1.0);
  EXPECT_DOUBLE_EQ(output->linear_velocity[1], 2.0);
  EXPECT_DOUBLE_EQ(output->linear_velocity[2], 3.0);
  EXPECT_DOUBLE_EQ(output->orientation_xyzw[3], 1.0);
}

TEST(SimVioOdometryLogicTest, DelayDefersPublicationButKeepsMeasurementStamp)
{
  SimVioOdometryConfig config;
  config.output_delay_s = 0.04;
  config.warmup_duration_s = 0.0;
  config.position_noise_stddev_xy_m = 0.0;
  config.position_noise_stddev_z_m = 0.0;
  config.roll_pitch_noise_stddev_rad = 0.0;
  config.yaw_noise_stddev_rad = 0.0;
  config.linear_velocity_noise_stddev_mps = 0.0;
  config.position_random_walk_xy_m_per_sqrt_s = 0.0;
  config.position_random_walk_z_m_per_sqrt_s = 0.0;
  config.yaw_random_walk_rad_per_sqrt_s = 0.0;
  config.dropout_interval_mean_s = 0.0;
  config.dropout_duration_s = 0.0;

  SimVioOdometryLogic logic(config);
  logic.observeGroundTruth(makeSample(0LL, 0.0, 0.0, 0.0));
  logic.observeGroundTruth(makeSample(20000000LL, 1.0, 0.0, 0.0));
  logic.observeGroundTruth(makeSample(40000000LL, 2.0, 0.0, 0.0));

  EXPECT_FALSE(logic.step(39000000LL).has_value());
  const auto first = logic.step(40000000LL);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->stamp_ns, 0LL);

  const auto second = logic.step(60000000LL);
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->stamp_ns, 20000000LL);
}

TEST(SimVioOdometryLogicTest, FixedSeedProducesReproducibleSequence)
{
  SimVioOdometryConfig config;
  config.output_delay_s = 0.0;
  config.warmup_duration_s = 0.0;
  config.dropout_interval_mean_s = 0.4;
  config.dropout_duration_s = 0.1;
  config.random_seed = 7U;

  SimVioOdometryLogic lhs(config);
  SimVioOdometryLogic rhs(config);

  for (int i = 0; i < 40; ++i) {
    const int64_t stamp_ns = static_cast<int64_t>(i) * 20000000LL;
    const auto sample = makeSample(stamp_ns, static_cast<double>(i), 0.1 * i, -0.05 * i);
    lhs.observeGroundTruth(sample);
    rhs.observeGroundTruth(sample);
  }

  const auto lhs_outputs = collectOutputs(lhs, 0LL, 1000000000LL, 20000000LL);
  const auto rhs_outputs = collectOutputs(rhs, 0LL, 1000000000LL, 20000000LL);

  ASSERT_EQ(lhs_outputs.size(), rhs_outputs.size());
  for (size_t i = 0; i < lhs_outputs.size(); ++i) {
    expectSamplesNear(lhs_outputs[i], rhs_outputs[i]);
  }
}

TEST(SimVioOdometryLogicTest, ResetClearsBufferedStateAndRestartsSequence)
{
  SimVioOdometryConfig config;
  config.output_delay_s = 0.0;
  config.warmup_duration_s = 0.0;
  config.position_noise_stddev_xy_m = 0.0;
  config.position_noise_stddev_z_m = 0.0;
  config.roll_pitch_noise_stddev_rad = 0.0;
  config.yaw_noise_stddev_rad = 0.0;
  config.linear_velocity_noise_stddev_mps = 0.0;
  config.position_random_walk_xy_m_per_sqrt_s = 0.0;
  config.position_random_walk_z_m_per_sqrt_s = 0.0;
  config.yaw_random_walk_rad_per_sqrt_s = 0.0;
  config.dropout_interval_mean_s = 0.0;
  config.dropout_duration_s = 0.0;

  SimVioOdometryLogic logic(config);
  logic.observeGroundTruth(makeSample(0LL, 1.0, 0.0, 0.0));
  ASSERT_TRUE(logic.step(0LL).has_value());

  logic.reset(0LL);
  EXPECT_FALSE(logic.step(0LL).has_value());

  logic.observeGroundTruth(makeSample(100000000LL, 9.0, 0.0, 0.0));
  const auto output = logic.step(100000000LL);
  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->position[0], 9.0);
}
