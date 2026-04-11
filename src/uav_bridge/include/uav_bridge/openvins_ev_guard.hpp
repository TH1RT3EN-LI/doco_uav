#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace uav_bridge
{

enum class OpenVinsEvGuardMode : uint8_t
{
  Healthy = 0,
  HoldLastValid = 1,
  Faulted = 2,
};

enum class OpenVinsEvGuardReason : uint8_t
{
  Ok = 0,
  PoseNonfinite = 1,
  OrientationInvalid = 2,
  TimestampBackwards = 3,
  SourceGap = 4,
  PositionStep = 5,
  ImpliedSpeed = 6,
  ReportedSpeed = 7,
  AccelJump = 8,
  YawRateJump = 9,
  HoldLastBudgetExceeded = 10,
  RecoveryPending = 11,
};

struct OpenVinsEvGuardConfig
{
  bool enable{true};
  bool auto_recover{true};
  double nominal_rate_hz{30.0};
  double hold_last_budget_s{0.12};
  int recovery_good_frames{5};
  double max_source_gap_s{0.20};
  float max_position_step_m{0.20f};
  float max_implied_speed_mps{1.0f};
  float max_reported_speed_mps{1.0f};
  float max_accel_mps2{2.5f};
  float max_yaw_rate_radps{2.0f};
};

struct OpenVinsEvGuardSample
{
  uint64_t source_stamp_us{0U};
  uint64_t receive_stamp_us{0U};
  std::array<float, 3> position_enu_m{0.0f, 0.0f, 0.0f};
  bool orientation_valid{false};
  float yaw_enu_rad{0.0f};
  bool velocity_valid{false};
  std::array<float, 3> velocity_enu_mps{0.0f, 0.0f, 0.0f};
};

struct OpenVinsEvGuardResult
{
  OpenVinsEvGuardMode mode{OpenVinsEvGuardMode::Healthy};
  OpenVinsEvGuardReason reason{OpenVinsEvGuardReason::Ok};
  bool sample_valid{false};
  bool publish_fresh{false};
  bool publish_hold_last{false};
  bool bump_reset_counter{false};
  bool health_ok{true};
};

class OpenVinsEvGuard
{
public:
  explicit OpenVinsEvGuard(const OpenVinsEvGuardConfig & config);

  OpenVinsEvGuardResult observe(const OpenVinsEvGuardSample & sample);

  void reset();

  OpenVinsEvGuardMode mode() const
  {
    return mode_;
  }

  OpenVinsEvGuardReason reason() const
  {
    return reason_;
  }

private:
  OpenVinsEvGuardReason validateSample(const OpenVinsEvGuardSample & sample);

  OpenVinsEvGuardConfig config_;
  OpenVinsEvGuardMode mode_{OpenVinsEvGuardMode::Healthy};
  OpenVinsEvGuardReason reason_{OpenVinsEvGuardReason::Ok};
  std::optional<OpenVinsEvGuardSample> last_valid_sample_;
  uint64_t last_source_stamp_us_{0U};
  uint64_t last_valid_receive_stamp_us_{0U};
  int recovery_good_frames_seen_{0};
};

const char * openVinsEvGuardModeName(OpenVinsEvGuardMode mode);
const char * openVinsEvGuardReasonName(OpenVinsEvGuardReason reason);

px4_msgs::msg::VehicleOdometry refreshHeldVehicleOdometry(
  const px4_msgs::msg::VehicleOdometry & cached,
  uint64_t timestamp_us);

}  // namespace uav_bridge
