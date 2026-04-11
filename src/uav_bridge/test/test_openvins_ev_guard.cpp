#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "uav_bridge/openvins_ev_guard.hpp"

namespace
{

using uav_bridge::OpenVinsEvGuard;
using uav_bridge::OpenVinsEvGuardConfig;
using uav_bridge::OpenVinsEvGuardMode;
using uav_bridge::OpenVinsEvGuardReason;
using uav_bridge::OpenVinsEvGuardSample;
using uav_bridge::refreshHeldVehicleOdometry;

OpenVinsEvGuardSample makeSample(
  uint64_t stamp_us,
  uint64_t receive_stamp_us,
  float x,
  float y,
  float z,
  float yaw_rad = 0.0f)
{
  OpenVinsEvGuardSample sample;
  sample.source_stamp_us = stamp_us;
  sample.receive_stamp_us = receive_stamp_us;
  sample.position_enu_m = {x, y, z};
  sample.orientation_valid = true;
  sample.yaw_enu_rad = yaw_rad;
  sample.velocity_valid = true;
  sample.velocity_enu_mps = {0.0f, 0.0f, 0.0f};
  return sample;
}

OpenVinsEvGuardConfig makeDefaultConfig()
{
  OpenVinsEvGuardConfig config;
  config.enable = true;
  config.auto_recover = true;
  config.nominal_rate_hz = 30.0;
  config.hold_last_budget_s = 0.12;
  config.recovery_good_frames = 5;
  config.max_source_gap_s = 0.20;
  config.max_position_step_m = 0.20f;
  config.max_implied_speed_mps = 1.0f;
  config.max_reported_speed_mps = 1.0f;
  config.max_accel_mps2 = 2.5f;
  config.max_yaw_rate_radps = 2.0f;
  return config;
}

TEST(OpenVinsEvGuardTest, HealthySequencePublishesFresh)
{
  OpenVinsEvGuard guard(makeDefaultConfig());

  auto first = guard.observe(makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f));
  EXPECT_TRUE(first.sample_valid);
  EXPECT_TRUE(first.publish_fresh);
  EXPECT_FALSE(first.publish_hold_last);
  EXPECT_EQ(first.mode, OpenVinsEvGuardMode::Healthy);
  EXPECT_EQ(first.reason, OpenVinsEvGuardReason::Ok);
  EXPECT_TRUE(first.health_ok);

  auto second = guard.observe(makeSample(1033333U, 1033333U, 0.01f, 0.0f, 0.0f));
  EXPECT_TRUE(second.sample_valid);
  EXPECT_TRUE(second.publish_fresh);
  EXPECT_EQ(second.mode, OpenVinsEvGuardMode::Healthy);
  EXPECT_EQ(second.reason, OpenVinsEvGuardReason::Ok);
}

TEST(OpenVinsEvGuardTest, PoseNonfiniteTriggersHoldLastAndImmediateRecovery)
{
  OpenVinsEvGuard guard(makeDefaultConfig());

  EXPECT_TRUE(guard.observe(makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f)).publish_fresh);

  auto bad = makeSample(1033333U, 1033333U, 0.0f, 0.0f, 0.0f);
  bad.position_enu_m[0] = std::numeric_limits<float>::quiet_NaN();
  auto held = guard.observe(bad);
  EXPECT_FALSE(held.sample_valid);
  EXPECT_FALSE(held.publish_fresh);
  EXPECT_TRUE(held.publish_hold_last);
  EXPECT_EQ(held.mode, OpenVinsEvGuardMode::HoldLastValid);
  EXPECT_EQ(held.reason, OpenVinsEvGuardReason::PoseNonfinite);
  EXPECT_FALSE(held.health_ok);

  auto recovered = guard.observe(makeSample(1066666U, 1066666U, 0.015f, 0.0f, 0.0f));
  EXPECT_TRUE(recovered.sample_valid);
  EXPECT_TRUE(recovered.publish_fresh);
  EXPECT_FALSE(recovered.publish_hold_last);
  EXPECT_EQ(recovered.mode, OpenVinsEvGuardMode::Healthy);
  EXPECT_EQ(recovered.reason, OpenVinsEvGuardReason::Ok);
}

TEST(OpenVinsEvGuardTest, OrientationInvalidTriggersHoldLast)
{
  OpenVinsEvGuard guard(makeDefaultConfig());
  EXPECT_TRUE(guard.observe(makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f)).publish_fresh);

  auto bad = makeSample(1033333U, 1033333U, 0.01f, 0.0f, 0.0f);
  bad.orientation_valid = false;
  auto result = guard.observe(bad);
  EXPECT_TRUE(result.publish_hold_last);
  EXPECT_EQ(result.reason, OpenVinsEvGuardReason::OrientationInvalid);
  EXPECT_EQ(result.mode, OpenVinsEvGuardMode::HoldLastValid);
}

TEST(OpenVinsEvGuardTest, TimestampBackwardsTriggersHoldLast)
{
  OpenVinsEvGuard guard(makeDefaultConfig());
  EXPECT_TRUE(guard.observe(makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f)).publish_fresh);

  auto result = guard.observe(makeSample(900000U, 1033333U, 0.01f, 0.0f, 0.0f));
  EXPECT_TRUE(result.publish_hold_last);
  EXPECT_EQ(result.reason, OpenVinsEvGuardReason::TimestampBackwards);
  EXPECT_EQ(result.mode, OpenVinsEvGuardMode::HoldLastValid);
}

TEST(OpenVinsEvGuardTest, PositionStepTriggersHoldLast)
{
  OpenVinsEvGuard guard(makeDefaultConfig());
  EXPECT_TRUE(guard.observe(makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f)).publish_fresh);

  auto result = guard.observe(makeSample(1033333U, 1033333U, 0.21f, 0.0f, 0.0f));
  EXPECT_TRUE(result.publish_hold_last);
  EXPECT_EQ(result.reason, OpenVinsEvGuardReason::PositionStep);
}

TEST(OpenVinsEvGuardTest, ImpliedSpeedTriggersHoldLast)
{
  OpenVinsEvGuard guard(makeDefaultConfig());
  EXPECT_TRUE(guard.observe(makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f)).publish_fresh);

  auto result = guard.observe(makeSample(1033333U, 1033333U, 0.04f, 0.0f, 0.0f));
  EXPECT_TRUE(result.publish_hold_last);
  EXPECT_EQ(result.reason, OpenVinsEvGuardReason::ImpliedSpeed);
}

TEST(OpenVinsEvGuardTest, ReportedSpeedTriggersHoldLast)
{
  OpenVinsEvGuard guard(makeDefaultConfig());
  EXPECT_TRUE(guard.observe(makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f)).publish_fresh);

  auto fast = makeSample(1033333U, 1033333U, 0.01f, 0.0f, 0.0f);
  fast.velocity_enu_mps = {1.1f, 0.0f, 0.0f};
  auto result = guard.observe(fast);
  EXPECT_TRUE(result.publish_hold_last);
  EXPECT_EQ(result.reason, OpenVinsEvGuardReason::ReportedSpeed);
}

TEST(OpenVinsEvGuardTest, AccelJumpTriggersHoldLast)
{
  OpenVinsEvGuard guard(makeDefaultConfig());
  auto first = makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f);
  first.velocity_enu_mps = {0.0f, 0.0f, 0.0f};
  EXPECT_TRUE(guard.observe(first).publish_fresh);

  auto second = makeSample(1033333U, 1033333U, 0.01f, 0.0f, 0.0f);
  second.velocity_enu_mps = {0.1f, 0.0f, 0.0f};
  auto result = guard.observe(second);
  EXPECT_TRUE(result.publish_hold_last);
  EXPECT_EQ(result.reason, OpenVinsEvGuardReason::AccelJump);
}

TEST(OpenVinsEvGuardTest, YawRateJumpTriggersHoldLast)
{
  OpenVinsEvGuard guard(makeDefaultConfig());
  EXPECT_TRUE(guard.observe(makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f, 0.0f)).publish_fresh);

  auto result = guard.observe(makeSample(1033333U, 1033333U, 0.01f, 0.0f, 0.0f, 0.08f));
  EXPECT_TRUE(result.publish_hold_last);
  EXPECT_EQ(result.reason, OpenVinsEvGuardReason::YawRateJump);
}

TEST(OpenVinsEvGuardTest, HoldLastBudgetExceededTransitionsToFaulted)
{
  OpenVinsEvGuard guard(makeDefaultConfig());
  EXPECT_TRUE(guard.observe(makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f)).publish_fresh);

  auto bad = makeSample(1033333U, 1033333U, 0.21f, 0.0f, 0.0f);
  auto held = guard.observe(bad);
  EXPECT_EQ(held.mode, OpenVinsEvGuardMode::HoldLastValid);
  EXPECT_TRUE(held.publish_hold_last);

  auto still_bad = makeSample(1066666U, 1135000U, 0.22f, 0.0f, 0.0f);
  auto faulted = guard.observe(still_bad);
  EXPECT_EQ(faulted.mode, OpenVinsEvGuardMode::Faulted);
  EXPECT_EQ(faulted.reason, OpenVinsEvGuardReason::HoldLastBudgetExceeded);
  EXPECT_FALSE(faulted.publish_hold_last);
  EXPECT_FALSE(faulted.publish_fresh);
  EXPECT_FALSE(faulted.health_ok);
}

TEST(OpenVinsEvGuardTest, FaultedRequiresFiveGoodFramesBeforeRecovery)
{
  OpenVinsEvGuard guard(makeDefaultConfig());
  EXPECT_TRUE(guard.observe(makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f)).publish_fresh);

  auto bad = makeSample(1033333U, 1033333U, 0.21f, 0.0f, 0.0f);
  EXPECT_TRUE(guard.observe(bad).publish_hold_last);
  auto faulted = guard.observe(makeSample(1066666U, 1135000U, 0.22f, 0.0f, 0.0f));
  EXPECT_EQ(faulted.mode, OpenVinsEvGuardMode::Faulted);

  for (int i = 0; i < 4; ++i) {
    const uint64_t stamp = 1100000U + static_cast<uint64_t>(i) * 33333U;
    const auto pending = guard.observe(makeSample(stamp, 1140000U + static_cast<uint64_t>(i) * 33333U, 0.01f, 0.0f, 0.0f));
    EXPECT_EQ(pending.mode, OpenVinsEvGuardMode::Faulted);
    EXPECT_EQ(pending.reason, OpenVinsEvGuardReason::RecoveryPending);
    EXPECT_FALSE(pending.publish_fresh);
    EXPECT_FALSE(pending.bump_reset_counter);
  }

  auto recovered = guard.observe(makeSample(1233332U, 1273332U, 0.015f, 0.0f, 0.0f));
  EXPECT_EQ(recovered.mode, OpenVinsEvGuardMode::Healthy);
  EXPECT_EQ(recovered.reason, OpenVinsEvGuardReason::Ok);
  EXPECT_TRUE(recovered.publish_fresh);
  EXPECT_TRUE(recovered.bump_reset_counter);
  EXPECT_TRUE(recovered.health_ok);
}

TEST(OpenVinsEvGuardTest, FaultedStaysLatchedWhenAutoRecoveryDisabled)
{
  auto config = makeDefaultConfig();
  config.auto_recover = false;
  OpenVinsEvGuard guard(config);
  EXPECT_TRUE(guard.observe(makeSample(1000000U, 1000000U, 0.0f, 0.0f, 0.0f)).publish_fresh);

  auto bad = makeSample(1033333U, 1033333U, 0.21f, 0.0f, 0.0f);
  const auto held = guard.observe(bad);
  EXPECT_EQ(held.mode, OpenVinsEvGuardMode::HoldLastValid);
  EXPECT_TRUE(held.publish_hold_last);
  EXPECT_FALSE(held.publish_fresh);

  const auto still_held = guard.observe(makeSample(1066666U, 1066666U, 0.01f, 0.0f, 0.0f));
  EXPECT_EQ(still_held.mode, OpenVinsEvGuardMode::HoldLastValid);
  EXPECT_TRUE(still_held.publish_hold_last);
  EXPECT_FALSE(still_held.publish_fresh);
  EXPECT_FALSE(still_held.health_ok);

  auto faulted = guard.observe(makeSample(1099999U, 1135000U, 0.22f, 0.0f, 0.0f));
  EXPECT_EQ(faulted.mode, OpenVinsEvGuardMode::Faulted);
  EXPECT_FALSE(faulted.health_ok);

  for (int i = 0; i < 6; ++i) {
    const uint64_t stamp = 1100000U + static_cast<uint64_t>(i) * 33333U;
    const auto still_faulted = guard.observe(
      makeSample(
        stamp,
        1140000U + static_cast<uint64_t>(i) * 33333U,
        0.01f,
        0.0f,
        0.0f));
    EXPECT_EQ(still_faulted.mode, OpenVinsEvGuardMode::Faulted);
    EXPECT_FALSE(still_faulted.publish_fresh);
    EXPECT_FALSE(still_faulted.bump_reset_counter);
    EXPECT_FALSE(still_faulted.health_ok);
  }
}

TEST(OpenVinsEvGuardTest, RefreshHeldVehicleOdometryOnlyUpdatesTimestamp)
{
  px4_msgs::msg::VehicleOdometry cached{};
  cached.timestamp = 10U;
  cached.timestamp_sample = 20U;
  cached.position = {1.0f, 2.0f, 3.0f};
  cached.velocity = {4.0f, 5.0f, 6.0f};
  cached.reset_counter = 7U;
  cached.quality = 100;

  const auto refreshed = refreshHeldVehicleOdometry(cached, 99U);
  EXPECT_EQ(refreshed.timestamp, 99U);
  EXPECT_EQ(refreshed.timestamp_sample, 20U);
  EXPECT_EQ(refreshed.position[0], 1.0f);
  EXPECT_EQ(refreshed.velocity[1], 5.0f);
  EXPECT_EQ(refreshed.reset_counter, 7U);
  EXPECT_EQ(refreshed.quality, 100);
}

}  // namespace
