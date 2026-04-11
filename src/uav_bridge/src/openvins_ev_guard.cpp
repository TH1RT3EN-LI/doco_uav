#include "uav_bridge/openvins_ev_guard.hpp"

#include <cmath>

namespace uav_bridge
{
namespace
{

bool isFiniteVector(const std::array<float, 3> & values)
{
  return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

float vectorNorm(const std::array<float, 3> & values)
{
  return std::sqrt(
    (values[0] * values[0]) +
    (values[1] * values[1]) +
    (values[2] * values[2]));
}

float normalizeAngle(float angle_rad)
{
  while (angle_rad > static_cast<float>(M_PI)) {
    angle_rad -= static_cast<float>(2.0 * M_PI);
  }
  while (angle_rad < static_cast<float>(-M_PI)) {
    angle_rad += static_cast<float>(2.0 * M_PI);
  }
  return angle_rad;
}

double microsToSeconds(uint64_t stamp_delta_us)
{
  return static_cast<double>(stamp_delta_us) / 1000000.0;
}

}  // namespace

OpenVinsEvGuard::OpenVinsEvGuard(const OpenVinsEvGuardConfig & config)
: config_(config)
{
}

OpenVinsEvGuardResult OpenVinsEvGuard::observe(const OpenVinsEvGuardSample & sample)
{
  OpenVinsEvGuardResult result;

  if (!config_.enable) {
    mode_ = OpenVinsEvGuardMode::Healthy;
    reason_ = OpenVinsEvGuardReason::Ok;
    last_valid_sample_ = sample;
    last_valid_receive_stamp_us_ = sample.receive_stamp_us;
    if (sample.source_stamp_us != 0U) {
      last_source_stamp_us_ = sample.source_stamp_us;
    }
    recovery_good_frames_seen_ = 0;
    result.mode = mode_;
    result.reason = reason_;
    result.sample_valid = true;
    result.publish_fresh = true;
    result.health_ok = true;
    return result;
  }

  const OpenVinsEvGuardReason validation_reason = validateSample(sample);
  const bool sample_valid = validation_reason == OpenVinsEvGuardReason::Ok;
  result.sample_valid = sample_valid;

  if (mode_ == OpenVinsEvGuardMode::Faulted) {
    if (!config_.auto_recover) {
      if (!sample_valid) {
        recovery_good_frames_seen_ = 0;
        reason_ = validation_reason;
      }
    } else if (sample_valid) {
      last_valid_sample_ = sample;
      last_valid_receive_stamp_us_ = sample.receive_stamp_us;
      recovery_good_frames_seen_ += 1;
      if (recovery_good_frames_seen_ >= config_.recovery_good_frames) {
        mode_ = OpenVinsEvGuardMode::Healthy;
        reason_ = OpenVinsEvGuardReason::Ok;
        recovery_good_frames_seen_ = 0;
        result.publish_fresh = true;
        result.bump_reset_counter = true;
      } else {
        reason_ = OpenVinsEvGuardReason::RecoveryPending;
      }
    } else {
      recovery_good_frames_seen_ = 0;
      reason_ = validation_reason;
    }
  } else if (mode_ == OpenVinsEvGuardMode::HoldLastValid) {
    const bool has_recent_valid = last_valid_sample_.has_value() &&
      sample.receive_stamp_us >= last_valid_receive_stamp_us_ &&
      microsToSeconds(sample.receive_stamp_us - last_valid_receive_stamp_us_) <=
      config_.hold_last_budget_s;

    if (config_.auto_recover && sample_valid) {
      mode_ = OpenVinsEvGuardMode::Healthy;
      reason_ = OpenVinsEvGuardReason::Ok;
      recovery_good_frames_seen_ = 0;
      last_valid_sample_ = sample;
      last_valid_receive_stamp_us_ = sample.receive_stamp_us;
      result.publish_fresh = true;
    } else if (has_recent_valid) {
      if (!sample_valid) {
        reason_ = validation_reason;
      }
      result.publish_hold_last = true;
    } else {
      mode_ = OpenVinsEvGuardMode::Faulted;
      reason_ = OpenVinsEvGuardReason::HoldLastBudgetExceeded;
      recovery_good_frames_seen_ = 0;
    }
  } else {
    if (sample_valid) {
      mode_ = OpenVinsEvGuardMode::Healthy;
      reason_ = OpenVinsEvGuardReason::Ok;
      recovery_good_frames_seen_ = 0;
      last_valid_sample_ = sample;
      last_valid_receive_stamp_us_ = sample.receive_stamp_us;
      result.publish_fresh = true;
    } else {
      const bool has_recent_valid = last_valid_sample_.has_value() &&
        sample.receive_stamp_us >= last_valid_receive_stamp_us_ &&
        microsToSeconds(sample.receive_stamp_us - last_valid_receive_stamp_us_) <=
        config_.hold_last_budget_s;
      if (has_recent_valid) {
        mode_ = OpenVinsEvGuardMode::HoldLastValid;
        reason_ = validation_reason;
        result.publish_hold_last = true;
      } else {
        mode_ = OpenVinsEvGuardMode::Faulted;
        reason_ = validation_reason;
        recovery_good_frames_seen_ = 0;
      }
    }
  }

  result.mode = mode_;
  result.reason = reason_;
  result.health_ok = mode_ == OpenVinsEvGuardMode::Healthy;
  return result;
}

void OpenVinsEvGuard::reset()
{
  mode_ = OpenVinsEvGuardMode::Healthy;
  reason_ = OpenVinsEvGuardReason::Ok;
  last_valid_sample_.reset();
  last_source_stamp_us_ = 0U;
  last_valid_receive_stamp_us_ = 0U;
  recovery_good_frames_seen_ = 0;
}

OpenVinsEvGuardReason OpenVinsEvGuard::validateSample(const OpenVinsEvGuardSample & sample)
{
  if (!isFiniteVector(sample.position_enu_m)) {
    return OpenVinsEvGuardReason::PoseNonfinite;
  }

  if (!sample.orientation_valid || !std::isfinite(sample.yaw_enu_rad)) {
    return OpenVinsEvGuardReason::OrientationInvalid;
  }

  if (sample.source_stamp_us != 0U && last_source_stamp_us_ != 0U) {
    if (sample.source_stamp_us < last_source_stamp_us_) {
      return OpenVinsEvGuardReason::TimestampBackwards;
    }

    const double gap_s = microsToSeconds(sample.source_stamp_us - last_source_stamp_us_);
    if (gap_s > config_.max_source_gap_s) {
      last_source_stamp_us_ = sample.source_stamp_us;
      return OpenVinsEvGuardReason::SourceGap;
    }
  }

  if (last_valid_sample_.has_value()) {
    const std::array<float, 3> delta_position = {
      sample.position_enu_m[0] - last_valid_sample_->position_enu_m[0],
      sample.position_enu_m[1] - last_valid_sample_->position_enu_m[1],
      sample.position_enu_m[2] - last_valid_sample_->position_enu_m[2]};
    const float position_step_m = vectorNorm(delta_position);
    if (position_step_m > config_.max_position_step_m) {
      if (sample.source_stamp_us != 0U) {
        last_source_stamp_us_ = sample.source_stamp_us;
      }
      return OpenVinsEvGuardReason::PositionStep;
    }

    if (sample.source_stamp_us > last_valid_sample_->source_stamp_us &&
      last_valid_sample_->source_stamp_us != 0U)
    {
      const double dt_s = microsToSeconds(sample.source_stamp_us - last_valid_sample_->source_stamp_us);
      if (dt_s > 1.0e-6) {
        const float implied_speed_mps = position_step_m / static_cast<float>(dt_s);
        if (implied_speed_mps > config_.max_implied_speed_mps) {
          if (sample.source_stamp_us != 0U) {
            last_source_stamp_us_ = sample.source_stamp_us;
          }
          return OpenVinsEvGuardReason::ImpliedSpeed;
        }

        if (sample.velocity_valid && vectorNorm(sample.velocity_enu_mps) > config_.max_reported_speed_mps) {
          if (sample.source_stamp_us != 0U) {
            last_source_stamp_us_ = sample.source_stamp_us;
          }
          return OpenVinsEvGuardReason::ReportedSpeed;
        }

        if (sample.velocity_valid && last_valid_sample_->velocity_valid) {
          const std::array<float, 3> delta_velocity = {
            sample.velocity_enu_mps[0] - last_valid_sample_->velocity_enu_mps[0],
            sample.velocity_enu_mps[1] - last_valid_sample_->velocity_enu_mps[1],
            sample.velocity_enu_mps[2] - last_valid_sample_->velocity_enu_mps[2]};
          const float accel_mps2 = vectorNorm(delta_velocity) / static_cast<float>(dt_s);
          if (accel_mps2 > config_.max_accel_mps2) {
            if (sample.source_stamp_us != 0U) {
              last_source_stamp_us_ = sample.source_stamp_us;
            }
            return OpenVinsEvGuardReason::AccelJump;
          }
        }

        const float yaw_delta_rad = normalizeAngle(sample.yaw_enu_rad - last_valid_sample_->yaw_enu_rad);
        const float yaw_rate_radps = std::abs(yaw_delta_rad) / static_cast<float>(dt_s);
        if (yaw_rate_radps > config_.max_yaw_rate_radps) {
          if (sample.source_stamp_us != 0U) {
            last_source_stamp_us_ = sample.source_stamp_us;
          }
          return OpenVinsEvGuardReason::YawRateJump;
        }
      }
    }
  }

  if (sample.velocity_valid && vectorNorm(sample.velocity_enu_mps) > config_.max_reported_speed_mps) {
    if (sample.source_stamp_us != 0U) {
      last_source_stamp_us_ = sample.source_stamp_us;
    }
    return OpenVinsEvGuardReason::ReportedSpeed;
  }

  if (sample.source_stamp_us != 0U) {
    last_source_stamp_us_ = sample.source_stamp_us;
  }
  return OpenVinsEvGuardReason::Ok;
}

const char * openVinsEvGuardModeName(OpenVinsEvGuardMode mode)
{
  switch (mode) {
    case OpenVinsEvGuardMode::Healthy:
      return "HEALTHY";
    case OpenVinsEvGuardMode::HoldLastValid:
      return "HOLD_LAST_VALID";
    case OpenVinsEvGuardMode::Faulted:
      return "FAULTED";
  }
  return "UNKNOWN";
}

const char * openVinsEvGuardReasonName(OpenVinsEvGuardReason reason)
{
  switch (reason) {
    case OpenVinsEvGuardReason::Ok:
      return "ok";
    case OpenVinsEvGuardReason::PoseNonfinite:
      return "pose_nonfinite";
    case OpenVinsEvGuardReason::OrientationInvalid:
      return "orientation_invalid";
    case OpenVinsEvGuardReason::TimestampBackwards:
      return "timestamp_backwards";
    case OpenVinsEvGuardReason::SourceGap:
      return "source_gap";
    case OpenVinsEvGuardReason::PositionStep:
      return "position_step";
    case OpenVinsEvGuardReason::ImpliedSpeed:
      return "implied_speed";
    case OpenVinsEvGuardReason::ReportedSpeed:
      return "reported_speed";
    case OpenVinsEvGuardReason::AccelJump:
      return "accel_jump";
    case OpenVinsEvGuardReason::YawRateJump:
      return "yaw_rate_jump";
    case OpenVinsEvGuardReason::HoldLastBudgetExceeded:
      return "hold_last_budget_exceeded";
    case OpenVinsEvGuardReason::RecoveryPending:
      return "recovery_pending";
  }
  return "unknown";
}

px4_msgs::msg::VehicleOdometry refreshHeldVehicleOdometry(
  const px4_msgs::msg::VehicleOdometry & cached,
  uint64_t timestamp_us)
{
  auto refreshed = cached;
  refreshed.timestamp = timestamp_us;
  return refreshed;
}

}  // namespace uav_bridge
