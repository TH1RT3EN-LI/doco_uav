// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uav_mode_supervisor:msg/SupervisorState.idl
// generated code does not contain a copyright notice

#ifndef UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__BUILDER_HPP_
#define UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uav_mode_supervisor/msg/detail/supervisor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uav_mode_supervisor
{

namespace msg
{

namespace builder
{

class Init_SupervisorState_visual_capture_observed
{
public:
  explicit Init_SupervisorState_visual_capture_observed(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  ::uav_mode_supervisor::msg::SupervisorState visual_capture_observed(::uav_mode_supervisor::msg::SupervisorState::_visual_capture_observed_type arg)
  {
    msg_.visual_capture_observed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_visual_committed
{
public:
  explicit Init_SupervisorState_visual_committed(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_visual_capture_observed visual_committed(::uav_mode_supervisor::msg::SupervisorState::_visual_committed_type arg)
  {
    msg_.visual_committed = std::move(arg);
    return Init_SupervisorState_visual_capture_observed(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_visual_phase
{
public:
  explicit Init_SupervisorState_visual_phase(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_visual_committed visual_phase(::uav_mode_supervisor::msg::SupervisorState::_visual_phase_type arg)
  {
    msg_.visual_phase = std::move(arg);
    return Init_SupervisorState_visual_committed(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_visual_target_detected
{
public:
  explicit Init_SupervisorState_visual_target_detected(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_visual_phase visual_target_detected(::uav_mode_supervisor::msg::SupervisorState::_visual_target_detected_type arg)
  {
    msg_.visual_target_detected = std::move(arg);
    return Init_SupervisorState_visual_phase(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_visual_active
{
public:
  explicit Init_SupervisorState_visual_active(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_visual_target_detected visual_active(::uav_mode_supervisor::msg::SupervisorState::_visual_active_type arg)
  {
    msg_.visual_active = std::move(arg);
    return Init_SupervisorState_visual_target_detected(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_visual_state_seen
{
public:
  explicit Init_SupervisorState_visual_state_seen(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_visual_active visual_state_seen(::uav_mode_supervisor::msg::SupervisorState::_visual_state_seen_type arg)
  {
    msg_.visual_state_seen = std::move(arg);
    return Init_SupervisorState_visual_active(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_fusion_reason
{
public:
  explicit Init_SupervisorState_fusion_reason(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_visual_state_seen fusion_reason(::uav_mode_supervisor::msg::SupervisorState::_fusion_reason_type arg)
  {
    msg_.fusion_reason = std::move(arg);
    return Init_SupervisorState_visual_state_seen(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_fusion_ready
{
public:
  explicit Init_SupervisorState_fusion_ready(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_fusion_reason fusion_ready(::uav_mode_supervisor::msg::SupervisorState::_fusion_ready_type arg)
  {
    msg_.fusion_ready = std::move(arg);
    return Init_SupervisorState_fusion_reason(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_fusion_relocalize_requested
{
public:
  explicit Init_SupervisorState_fusion_relocalize_requested(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_fusion_ready fusion_relocalize_requested(::uav_mode_supervisor::msg::SupervisorState::_fusion_relocalize_requested_type arg)
  {
    msg_.fusion_relocalize_requested = std::move(arg);
    return Init_SupervisorState_fusion_ready(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_fusion_initialized
{
public:
  explicit Init_SupervisorState_fusion_initialized(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_fusion_relocalize_requested fusion_initialized(::uav_mode_supervisor::msg::SupervisorState::_fusion_initialized_type arg)
  {
    msg_.fusion_initialized = std::move(arg);
    return Init_SupervisorState_fusion_relocalize_requested(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_fusion_diagnostics_seen
{
public:
  explicit Init_SupervisorState_fusion_diagnostics_seen(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_fusion_initialized fusion_diagnostics_seen(::uav_mode_supervisor::msg::SupervisorState::_fusion_diagnostics_seen_type arg)
  {
    msg_.fusion_diagnostics_seen = std::move(arg);
    return Init_SupervisorState_fusion_initialized(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_last_message
{
public:
  explicit Init_SupervisorState_last_message(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_fusion_diagnostics_seen last_message(::uav_mode_supervisor::msg::SupervisorState::_last_message_type arg)
  {
    msg_.last_message = std::move(arg);
    return Init_SupervisorState_fusion_diagnostics_seen(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_command_in_progress
{
public:
  explicit Init_SupervisorState_command_in_progress(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_last_message command_in_progress(::uav_mode_supervisor::msg::SupervisorState::_command_in_progress_type arg)
  {
    msg_.command_in_progress = std::move(arg);
    return Init_SupervisorState_last_message(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_pending_command
{
public:
  explicit Init_SupervisorState_pending_command(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_command_in_progress pending_command(::uav_mode_supervisor::msg::SupervisorState::_pending_command_type arg)
  {
    msg_.pending_command = std::move(arg);
    return Init_SupervisorState_command_in_progress(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_last_command
{
public:
  explicit Init_SupervisorState_last_command(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_pending_command last_command(::uav_mode_supervisor::msg::SupervisorState::_last_command_type arg)
  {
    msg_.last_command = std::move(arg);
    return Init_SupervisorState_pending_command(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_active_tracking_height_m
{
public:
  explicit Init_SupervisorState_active_tracking_height_m(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_last_command active_tracking_height_m(::uav_mode_supervisor::msg::SupervisorState::_active_tracking_height_m_type arg)
  {
    msg_.active_tracking_height_m = std::move(arg);
    return Init_SupervisorState_last_command(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_owner
{
public:
  explicit Init_SupervisorState_owner(::uav_mode_supervisor::msg::SupervisorState & msg)
  : msg_(msg)
  {}
  Init_SupervisorState_active_tracking_height_m owner(::uav_mode_supervisor::msg::SupervisorState::_owner_type arg)
  {
    msg_.owner = std::move(arg);
    return Init_SupervisorState_active_tracking_height_m(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

class Init_SupervisorState_header
{
public:
  Init_SupervisorState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SupervisorState_owner header(::uav_mode_supervisor::msg::SupervisorState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SupervisorState_owner(msg_);
  }

private:
  ::uav_mode_supervisor::msg::SupervisorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uav_mode_supervisor::msg::SupervisorState>()
{
  return uav_mode_supervisor::msg::builder::Init_SupervisorState_header();
}

}  // namespace uav_mode_supervisor

#endif  // UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__BUILDER_HPP_
