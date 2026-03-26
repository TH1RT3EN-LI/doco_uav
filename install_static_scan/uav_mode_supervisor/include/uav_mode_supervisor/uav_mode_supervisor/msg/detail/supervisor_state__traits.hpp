// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uav_mode_supervisor:msg/SupervisorState.idl
// generated code does not contain a copyright notice

#ifndef UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__TRAITS_HPP_
#define UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uav_mode_supervisor/msg/detail/supervisor_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace uav_mode_supervisor
{

namespace msg
{

inline void to_flow_style_yaml(
  const SupervisorState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: owner
  {
    out << "owner: ";
    rosidl_generator_traits::value_to_yaml(msg.owner, out);
    out << ", ";
  }

  // member: active_tracking_height_m
  {
    out << "active_tracking_height_m: ";
    rosidl_generator_traits::value_to_yaml(msg.active_tracking_height_m, out);
    out << ", ";
  }

  // member: last_command
  {
    out << "last_command: ";
    rosidl_generator_traits::value_to_yaml(msg.last_command, out);
    out << ", ";
  }

  // member: pending_command
  {
    out << "pending_command: ";
    rosidl_generator_traits::value_to_yaml(msg.pending_command, out);
    out << ", ";
  }

  // member: command_in_progress
  {
    out << "command_in_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.command_in_progress, out);
    out << ", ";
  }

  // member: last_message
  {
    out << "last_message: ";
    rosidl_generator_traits::value_to_yaml(msg.last_message, out);
    out << ", ";
  }

  // member: fusion_diagnostics_seen
  {
    out << "fusion_diagnostics_seen: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_diagnostics_seen, out);
    out << ", ";
  }

  // member: fusion_initialized
  {
    out << "fusion_initialized: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_initialized, out);
    out << ", ";
  }

  // member: fusion_relocalize_requested
  {
    out << "fusion_relocalize_requested: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_relocalize_requested, out);
    out << ", ";
  }

  // member: fusion_ready
  {
    out << "fusion_ready: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_ready, out);
    out << ", ";
  }

  // member: fusion_reason
  {
    out << "fusion_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_reason, out);
    out << ", ";
  }

  // member: visual_state_seen
  {
    out << "visual_state_seen: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_state_seen, out);
    out << ", ";
  }

  // member: visual_active
  {
    out << "visual_active: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_active, out);
    out << ", ";
  }

  // member: visual_target_detected
  {
    out << "visual_target_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_target_detected, out);
    out << ", ";
  }

  // member: visual_phase
  {
    out << "visual_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_phase, out);
    out << ", ";
  }

  // member: visual_committed
  {
    out << "visual_committed: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_committed, out);
    out << ", ";
  }

  // member: visual_capture_observed
  {
    out << "visual_capture_observed: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_capture_observed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SupervisorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: owner
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "owner: ";
    rosidl_generator_traits::value_to_yaml(msg.owner, out);
    out << "\n";
  }

  // member: active_tracking_height_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active_tracking_height_m: ";
    rosidl_generator_traits::value_to_yaml(msg.active_tracking_height_m, out);
    out << "\n";
  }

  // member: last_command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_command: ";
    rosidl_generator_traits::value_to_yaml(msg.last_command, out);
    out << "\n";
  }

  // member: pending_command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pending_command: ";
    rosidl_generator_traits::value_to_yaml(msg.pending_command, out);
    out << "\n";
  }

  // member: command_in_progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command_in_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.command_in_progress, out);
    out << "\n";
  }

  // member: last_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_message: ";
    rosidl_generator_traits::value_to_yaml(msg.last_message, out);
    out << "\n";
  }

  // member: fusion_diagnostics_seen
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fusion_diagnostics_seen: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_diagnostics_seen, out);
    out << "\n";
  }

  // member: fusion_initialized
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fusion_initialized: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_initialized, out);
    out << "\n";
  }

  // member: fusion_relocalize_requested
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fusion_relocalize_requested: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_relocalize_requested, out);
    out << "\n";
  }

  // member: fusion_ready
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fusion_ready: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_ready, out);
    out << "\n";
  }

  // member: fusion_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fusion_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_reason, out);
    out << "\n";
  }

  // member: visual_state_seen
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visual_state_seen: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_state_seen, out);
    out << "\n";
  }

  // member: visual_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visual_active: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_active, out);
    out << "\n";
  }

  // member: visual_target_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visual_target_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_target_detected, out);
    out << "\n";
  }

  // member: visual_phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visual_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_phase, out);
    out << "\n";
  }

  // member: visual_committed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visual_committed: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_committed, out);
    out << "\n";
  }

  // member: visual_capture_observed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visual_capture_observed: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_capture_observed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SupervisorState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace uav_mode_supervisor

namespace rosidl_generator_traits
{

[[deprecated("use uav_mode_supervisor::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uav_mode_supervisor::msg::SupervisorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  uav_mode_supervisor::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uav_mode_supervisor::msg::to_yaml() instead")]]
inline std::string to_yaml(const uav_mode_supervisor::msg::SupervisorState & msg)
{
  return uav_mode_supervisor::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uav_mode_supervisor::msg::SupervisorState>()
{
  return "uav_mode_supervisor::msg::SupervisorState";
}

template<>
inline const char * name<uav_mode_supervisor::msg::SupervisorState>()
{
  return "uav_mode_supervisor/msg/SupervisorState";
}

template<>
struct has_fixed_size<uav_mode_supervisor::msg::SupervisorState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uav_mode_supervisor::msg::SupervisorState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<uav_mode_supervisor::msg::SupervisorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__TRAITS_HPP_
