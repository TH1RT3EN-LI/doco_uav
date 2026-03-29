#pragma once

#include <cmath>
#include <limits>
#include <optional>

#include <rclcpp/time.hpp>

#include "uav_mode_supervisor/supervisor_logic.hpp"

namespace uav_mode_supervisor
{

inline bool IsStatusFresh(double age_s, double timeout_s)
{
  return std::isfinite(age_s) && std::isfinite(timeout_s) &&
         timeout_s >= 0.0 && age_s >= 0.0 && age_s <= timeout_s;
}

template<typename StatusT>
struct TimestampedStatusCache
{
  StatusT status{};
  rclcpp::Time source_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time receive_stamp{0, 0, RCL_ROS_TIME};
};

inline rclcpp::Time ResolveStatusSourceStamp(
  const rclcpp::Time & candidate_source_stamp,
  const rclcpp::Time & receive_stamp)
{
  if (candidate_source_stamp.nanoseconds() == 0) {
    return receive_stamp;
  }
  return candidate_source_stamp;
}

template<typename StatusT>
inline bool ShouldAcceptStatusUpdate(
  const TimestampedStatusCache<StatusT> & cache,
  const rclcpp::Time & new_source_stamp)
{
  return cache.source_stamp.nanoseconds() == 0 ||
         new_source_stamp.nanoseconds() >= cache.source_stamp.nanoseconds();
}

template<typename StatusT>
inline bool CacheStatusUpdate(
  TimestampedStatusCache<StatusT> & cache,
  const StatusT & status,
  const rclcpp::Time & source_stamp,
  const rclcpp::Time & receive_stamp)
{
  if (!ShouldAcceptStatusUpdate(cache, source_stamp)) {
    return false;
  }

  cache.status = status;
  cache.source_stamp = source_stamp;
  cache.receive_stamp = receive_stamp;
  return true;
}

template<typename StatusT>
inline void OverwriteStatusCache(
  TimestampedStatusCache<StatusT> & cache,
  const StatusT & status,
  const rclcpp::Time & source_stamp,
  const rclcpp::Time & receive_stamp)
{
  cache.status = status;
  cache.source_stamp = source_stamp;
  cache.receive_stamp = receive_stamp;
}

inline double StatusAgeSeconds(
  const rclcpp::Time & source_stamp,
  const rclcpp::Time & now)
{
  if (source_stamp.nanoseconds() == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (now - source_stamp).seconds();
}

inline bool TimedStatusFresh(
  const rclcpp::Time & source_stamp,
  double timeout_s,
  const rclcpp::Time & now)
{
  return IsStatusFresh(StatusAgeSeconds(source_stamp, now), timeout_s);
}

inline SupervisorLogic::FusionStatus BuildEffectiveFusionStatus(
  const SupervisorLogic::FusionStatus & raw_status,
  const std::optional<bool> & fallback_relocalize_requested,
  bool fresh)
{
  auto status = raw_status;
  status.fresh = fresh;
  if (!status.diagnostics_seen) {
    status.relocalize_requested = fallback_relocalize_requested.value_or(true);
  }
  return status;
}

inline SupervisorLogic::VisualLandingStatus BuildEffectiveVisualLandingStatus(
  const SupervisorLogic::VisualLandingStatus & raw_status,
  bool fresh)
{
  auto status = raw_status;
  status.fresh = fresh;
  return status;
}

inline SupervisorLogic::VisualLandingStatus MakeSyntheticVisualLandingStatus(bool active)
{
  SupervisorLogic::VisualLandingStatus status;
  status.state_seen = true;
  status.fresh = true;
  status.active = active;
  status.target_detected = false;
  status.phase = active ? "HOLD_WAIT" : "READY";
  return status;
}

}  // namespace uav_mode_supervisor
