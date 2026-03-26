// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uav_visual_landing:msg/LandingControllerState.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__BUILDER_HPP_
#define UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uav_visual_landing/msg/detail/landing_controller_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uav_visual_landing
{

namespace msg
{

namespace builder
{

class Init_LandingControllerState_cmd_yaw_rate
{
public:
  explicit Init_LandingControllerState_cmd_yaw_rate(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  ::uav_visual_landing::msg::LandingControllerState cmd_yaw_rate(::uav_visual_landing::msg::LandingControllerState::_cmd_yaw_rate_type arg)
  {
    msg_.cmd_yaw_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_cmd_vz
{
public:
  explicit Init_LandingControllerState_cmd_vz(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_cmd_yaw_rate cmd_vz(::uav_visual_landing::msg::LandingControllerState::_cmd_vz_type arg)
  {
    msg_.cmd_vz = std::move(arg);
    return Init_LandingControllerState_cmd_yaw_rate(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_cmd_vy
{
public:
  explicit Init_LandingControllerState_cmd_vy(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_cmd_vz cmd_vy(::uav_visual_landing::msg::LandingControllerState::_cmd_vy_type arg)
  {
    msg_.cmd_vy = std::move(arg);
    return Init_LandingControllerState_cmd_vz(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_cmd_vx
{
public:
  explicit Init_LandingControllerState_cmd_vx(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_cmd_vy cmd_vx(::uav_visual_landing::msg::LandingControllerState::_cmd_vx_type arg)
  {
    msg_.cmd_vx = std::move(arg);
    return Init_LandingControllerState_cmd_vy(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_xy_control_mode
{
public:
  explicit Init_LandingControllerState_xy_control_mode(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_cmd_vx xy_control_mode(::uav_visual_landing::msg::LandingControllerState::_xy_control_mode_type arg)
  {
    msg_.xy_control_mode = std::move(arg);
    return Init_LandingControllerState_cmd_vx(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_z_error_m
{
public:
  explicit Init_LandingControllerState_z_error_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_xy_control_mode z_error_m(::uav_visual_landing::msg::LandingControllerState::_z_error_m_type arg)
  {
    msg_.z_error_m = std::move(arg);
    return Init_LandingControllerState_xy_control_mode(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_z_target_height_m
{
public:
  explicit Init_LandingControllerState_z_target_height_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_z_error_m z_target_height_m(::uav_visual_landing::msg::LandingControllerState::_z_target_height_m_type arg)
  {
    msg_.z_target_height_m = std::move(arg);
    return Init_LandingControllerState_z_error_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_lateral_error_rate_y_mps
{
public:
  explicit Init_LandingControllerState_lateral_error_rate_y_mps(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_z_target_height_m lateral_error_rate_y_mps(::uav_visual_landing::msg::LandingControllerState::_lateral_error_rate_y_mps_type arg)
  {
    msg_.lateral_error_rate_y_mps = std::move(arg);
    return Init_LandingControllerState_z_target_height_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_lateral_error_rate_x_mps
{
public:
  explicit Init_LandingControllerState_lateral_error_rate_x_mps(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_lateral_error_rate_y_mps lateral_error_rate_x_mps(::uav_visual_landing::msg::LandingControllerState::_lateral_error_rate_x_mps_type arg)
  {
    msg_.lateral_error_rate_x_mps = std::move(arg);
    return Init_LandingControllerState_lateral_error_rate_y_mps(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_lateral_error_m
{
public:
  explicit Init_LandingControllerState_lateral_error_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_lateral_error_rate_x_mps lateral_error_m(::uav_visual_landing::msg::LandingControllerState::_lateral_error_m_type arg)
  {
    msg_.lateral_error_m = std::move(arg);
    return Init_LandingControllerState_lateral_error_rate_x_mps(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_lateral_error_y_m
{
public:
  explicit Init_LandingControllerState_lateral_error_y_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_lateral_error_m lateral_error_y_m(::uav_visual_landing::msg::LandingControllerState::_lateral_error_y_m_type arg)
  {
    msg_.lateral_error_y_m = std::move(arg);
    return Init_LandingControllerState_lateral_error_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_lateral_error_x_m
{
public:
  explicit Init_LandingControllerState_lateral_error_x_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_lateral_error_y_m lateral_error_x_m(::uav_visual_landing::msg::LandingControllerState::_lateral_error_x_m_type arg)
  {
    msg_.lateral_error_x_m = std::move(arg);
    return Init_LandingControllerState_lateral_error_y_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_lateral_error_valid
{
public:
  explicit Init_LandingControllerState_lateral_error_valid(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_lateral_error_x_m lateral_error_valid(::uav_visual_landing::msg::LandingControllerState::_lateral_error_valid_type arg)
  {
    msg_.lateral_error_valid = std::move(arg);
    return Init_LandingControllerState_lateral_error_x_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_err_v_rate_norm_s
{
public:
  explicit Init_LandingControllerState_err_v_rate_norm_s(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_lateral_error_valid err_v_rate_norm_s(::uav_visual_landing::msg::LandingControllerState::_err_v_rate_norm_s_type arg)
  {
    msg_.err_v_rate_norm_s = std::move(arg);
    return Init_LandingControllerState_lateral_error_valid(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_err_u_rate_norm_s
{
public:
  explicit Init_LandingControllerState_err_u_rate_norm_s(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_err_v_rate_norm_s err_u_rate_norm_s(::uav_visual_landing::msg::LandingControllerState::_err_u_rate_norm_s_type arg)
  {
    msg_.err_u_rate_norm_s = std::move(arg);
    return Init_LandingControllerState_err_v_rate_norm_s(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_err_v_norm_filtered
{
public:
  explicit Init_LandingControllerState_err_v_norm_filtered(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_err_u_rate_norm_s err_v_norm_filtered(::uav_visual_landing::msg::LandingControllerState::_err_v_norm_filtered_type arg)
  {
    msg_.err_v_norm_filtered = std::move(arg);
    return Init_LandingControllerState_err_u_rate_norm_s(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_err_u_norm_filtered
{
public:
  explicit Init_LandingControllerState_err_u_norm_filtered(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_err_v_norm_filtered err_u_norm_filtered(::uav_visual_landing::msg::LandingControllerState::_err_u_norm_filtered_type arg)
  {
    msg_.err_u_norm_filtered = std::move(arg);
    return Init_LandingControllerState_err_v_norm_filtered(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_active_max_vxy
{
public:
  explicit Init_LandingControllerState_active_max_vxy(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_err_u_norm_filtered active_max_vxy(::uav_visual_landing::msg::LandingControllerState::_active_max_vxy_type arg)
  {
    msg_.active_max_vxy = std::move(arg);
    return Init_LandingControllerState_err_u_norm_filtered(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_align_exit_lateral_m
{
public:
  explicit Init_LandingControllerState_align_exit_lateral_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_active_max_vxy align_exit_lateral_m(::uav_visual_landing::msg::LandingControllerState::_align_exit_lateral_m_type arg)
  {
    msg_.align_exit_lateral_m = std::move(arg);
    return Init_LandingControllerState_active_max_vxy(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_align_enter_lateral_m
{
public:
  explicit Init_LandingControllerState_align_enter_lateral_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_align_exit_lateral_m align_enter_lateral_m(::uav_visual_landing::msg::LandingControllerState::_align_enter_lateral_m_type arg)
  {
    msg_.align_enter_lateral_m = std::move(arg);
    return Init_LandingControllerState_align_exit_lateral_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_tag_depth_m
{
public:
  explicit Init_LandingControllerState_tag_depth_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_align_enter_lateral_m tag_depth_m(::uav_visual_landing::msg::LandingControllerState::_tag_depth_m_type arg)
  {
    msg_.tag_depth_m = std::move(arg);
    return Init_LandingControllerState_align_enter_lateral_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_tag_depth_valid
{
public:
  explicit Init_LandingControllerState_tag_depth_valid(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_tag_depth_m tag_depth_valid(::uav_visual_landing::msg::LandingControllerState::_tag_depth_valid_type arg)
  {
    msg_.tag_depth_valid = std::move(arg);
    return Init_LandingControllerState_tag_depth_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_control_height_m
{
public:
  explicit Init_LandingControllerState_control_height_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_tag_depth_valid control_height_m(::uav_visual_landing::msg::LandingControllerState::_control_height_m_type arg)
  {
    msg_.control_height_m = std::move(arg);
    return Init_LandingControllerState_tag_depth_valid(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_height_measurement_m
{
public:
  explicit Init_LandingControllerState_height_measurement_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_control_height_m height_measurement_m(::uav_visual_landing::msg::LandingControllerState::_height_measurement_m_type arg)
  {
    msg_.height_measurement_m = std::move(arg);
    return Init_LandingControllerState_control_height_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_raw_flow_fresh
{
public:
  explicit Init_LandingControllerState_raw_flow_fresh(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_height_measurement_m raw_flow_fresh(::uav_visual_landing::msg::LandingControllerState::_raw_flow_fresh_type arg)
  {
    msg_.raw_flow_fresh = std::move(arg);
    return Init_LandingControllerState_height_measurement_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_height_measurement_fresh
{
public:
  explicit Init_LandingControllerState_height_measurement_fresh(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_raw_flow_fresh height_measurement_fresh(::uav_visual_landing::msg::LandingControllerState::_height_measurement_fresh_type arg)
  {
    msg_.height_measurement_fresh = std::move(arg);
    return Init_LandingControllerState_raw_flow_fresh(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_height_measurement_source
{
public:
  explicit Init_LandingControllerState_height_measurement_source(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_height_measurement_fresh height_measurement_source(::uav_visual_landing::msg::LandingControllerState::_height_measurement_source_type arg)
  {
    msg_.height_measurement_source = std::move(arg);
    return Init_LandingControllerState_height_measurement_fresh(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_height_valid
{
public:
  explicit Init_LandingControllerState_height_valid(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_height_measurement_source height_valid(::uav_visual_landing::msg::LandingControllerState::_height_valid_type arg)
  {
    msg_.height_valid = std::move(arg);
    return Init_LandingControllerState_height_measurement_source(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_odom_height_m
{
public:
  explicit Init_LandingControllerState_odom_height_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_height_valid odom_height_m(::uav_visual_landing::msg::LandingControllerState::_odom_height_m_type arg)
  {
    msg_.odom_height_m = std::move(arg);
    return Init_LandingControllerState_height_valid(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_terminal_trigger_source
{
public:
  explicit Init_LandingControllerState_terminal_trigger_source(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_odom_height_m terminal_trigger_source(::uav_visual_landing::msg::LandingControllerState::_terminal_trigger_source_type arg)
  {
    msg_.terminal_trigger_source = std::move(arg);
    return Init_LandingControllerState_odom_height_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_height_source
{
public:
  explicit Init_LandingControllerState_height_source(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_terminal_trigger_source height_source(::uav_visual_landing::msg::LandingControllerState::_height_source_type arg)
  {
    msg_.height_source = std::move(arg);
    return Init_LandingControllerState_terminal_trigger_source(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_target_confidence
{
public:
  explicit Init_LandingControllerState_target_confidence(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_height_source target_confidence(::uav_visual_landing::msg::LandingControllerState::_target_confidence_type arg)
  {
    msg_.target_confidence = std::move(arg);
    return Init_LandingControllerState_height_source(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_observation_age_s
{
public:
  explicit Init_LandingControllerState_observation_age_s(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_target_confidence observation_age_s(::uav_visual_landing::msg::LandingControllerState::_observation_age_s_type arg)
  {
    msg_.observation_age_s = std::move(arg);
    return Init_LandingControllerState_target_confidence(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_target_detected
{
public:
  explicit Init_LandingControllerState_target_detected(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_observation_age_s target_detected(::uav_visual_landing::msg::LandingControllerState::_target_detected_type arg)
  {
    msg_.target_detected = std::move(arg);
    return Init_LandingControllerState_observation_age_s(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_phase
{
public:
  explicit Init_LandingControllerState_phase(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_target_detected phase(::uav_visual_landing::msg::LandingControllerState::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_LandingControllerState_target_detected(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_active
{
public:
  explicit Init_LandingControllerState_active(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_phase active(::uav_visual_landing::msg::LandingControllerState::_active_type arg)
  {
    msg_.active = std::move(arg);
    return Init_LandingControllerState_phase(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_header
{
public:
  Init_LandingControllerState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LandingControllerState_active header(::uav_visual_landing::msg::LandingControllerState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LandingControllerState_active(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uav_visual_landing::msg::LandingControllerState>()
{
  return uav_visual_landing::msg::builder::Init_LandingControllerState_header();
}

}  // namespace uav_visual_landing

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__BUILDER_HPP_
