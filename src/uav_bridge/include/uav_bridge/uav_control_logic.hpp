#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

namespace uav_bridge
{

enum class UavControlMode
{
  Hold,
  Takeoff,
  Position,
  Px4PositionHold,
  VelocityBody,
  Landing,
};

enum class Px4TimestampSource
{
  System = 0,
  GazeboSim = 1,
};

class UavControlModeTracker
{
public:
  UavControlMode mode() const
  {
    return mode_;
  }

  void requestHold()
  {
    mode_ = UavControlMode::Hold;
  }

  void requestTakeoff()
  {
    mode_ = UavControlMode::Takeoff;
  }

  void requestPosition()
  {
    if (mode_ != UavControlMode::Landing) {
      mode_ = UavControlMode::Position;
    }
  }

  void requestPx4PositionHold()
  {
    if (mode_ != UavControlMode::Landing) {
      mode_ = UavControlMode::Px4PositionHold;
    }
  }

  void forcePx4PositionHold()
  {
    mode_ = UavControlMode::Px4PositionHold;
  }

  void requestVelocityBody()
  {
    if (mode_ != UavControlMode::Landing) {
      mode_ = UavControlMode::VelocityBody;
    }
  }

  void requestLanding()
  {
    mode_ = UavControlMode::Landing;
  }

  void onVelocityCommandTimeout()
  {
    if (mode_ == UavControlMode::VelocityBody) {
      mode_ = UavControlMode::Hold;
    }
  }

private:
  UavControlMode mode_{UavControlMode::Hold};
};

inline Px4TimestampSource parsePx4TimestampSource(std::string_view value)
{
  return value == "gz_sim" ? Px4TimestampSource::GazeboSim : Px4TimestampSource::System;
}

inline uint64_t selectPx4TimestampMicros(
  Px4TimestampSource source,
  uint64_t ros_clock_time_us,
  uint64_t system_clock_time_us)
{
  return source == Px4TimestampSource::GazeboSim ? ros_clock_time_us : system_clock_time_us;
}

inline uint64_t initializeVelocityBodyCommandTimestamp(
  uint64_t now_us,
  uint64_t last_command_time_us)
{
  if (now_us == 0 || last_command_time_us != 0) {
    return last_command_time_us;
  }
  return now_us;
}

inline bool isVelocityBodyCommandTimedOut(
  uint64_t now_us,
  uint64_t last_command_time_us,
  double timeout_ms)
{
  if (timeout_ms < 0.0 || now_us == 0 || last_command_time_us == 0 || now_us < last_command_time_us) {
    return false;
  }
  return static_cast<double>(now_us - last_command_time_us) / 1000.0 > timeout_ms;
}

inline bool isFiniteVelocityBodyCommand(
  float body_velocity_x_mps,
  float body_velocity_y_mps,
  float body_velocity_z_mps,
  float yaw_rate_radps)
{
  return std::isfinite(body_velocity_x_mps) &&
         std::isfinite(body_velocity_y_mps) &&
         std::isfinite(body_velocity_z_mps) &&
         std::isfinite(yaw_rate_radps);
}

inline bool hasValidVelocityEstimate(
  bool horizontal_velocity_valid,
  bool vertical_velocity_valid,
  const std::array<float, 3> & velocity_ned_mps)
{
  return horizontal_velocity_valid &&
         vertical_velocity_valid &&
         std::isfinite(velocity_ned_mps[0]) &&
         std::isfinite(velocity_ned_mps[1]) &&
         std::isfinite(velocity_ned_mps[2]);
}

inline bool isFiniteVelocitySetpoint(
  const std::array<float, 3> & velocity_ned_mps,
  float yaw_speed_ned_radps)
{
  return std::isfinite(velocity_ned_mps[0]) &&
         std::isfinite(velocity_ned_mps[1]) &&
         std::isfinite(velocity_ned_mps[2]) &&
         std::isfinite(yaw_speed_ned_radps);
}

}  // namespace uav_bridge
