// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uav_visual_landing:msg/LandingControllerState.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_HPP_
#define UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__uav_visual_landing__msg__LandingControllerState __attribute__((deprecated))
#else
# define DEPRECATED__uav_visual_landing__msg__LandingControllerState __declspec(deprecated)
#endif

namespace uav_visual_landing
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LandingControllerState_
{
  using Type = LandingControllerState_<ContainerAllocator>;

  explicit LandingControllerState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->active = false;
      this->phase = "";
      this->target_detected = false;
      this->observation_age_s = 0.0f;
      this->target_confidence = 0.0f;
      this->height_source = "";
      this->terminal_trigger_source = "";
      this->odom_height_m = 0.0f;
      this->height_valid = false;
      this->height_measurement_source = "";
      this->height_measurement_fresh = false;
      this->raw_flow_fresh = false;
      this->height_measurement_m = 0.0f;
      this->control_height_m = 0.0f;
      this->tag_depth_valid = false;
      this->tag_depth_m = 0.0f;
      this->align_enter_lateral_m = 0.0f;
      this->align_exit_lateral_m = 0.0f;
      this->active_max_vxy = 0.0f;
      this->err_u_norm_filtered = 0.0f;
      this->err_v_norm_filtered = 0.0f;
      this->err_u_rate_norm_s = 0.0f;
      this->err_v_rate_norm_s = 0.0f;
      this->lateral_error_valid = false;
      this->lateral_error_x_m = 0.0f;
      this->lateral_error_y_m = 0.0f;
      this->lateral_error_m = 0.0f;
      this->lateral_error_rate_x_mps = 0.0f;
      this->lateral_error_rate_y_mps = 0.0f;
      this->z_target_height_m = 0.0f;
      this->z_error_m = 0.0f;
      this->xy_control_mode = "";
      this->cmd_vx = 0.0f;
      this->cmd_vy = 0.0f;
      this->cmd_vz = 0.0f;
      this->cmd_yaw_rate = 0.0f;
    }
  }

  explicit LandingControllerState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    phase(_alloc),
    height_source(_alloc),
    terminal_trigger_source(_alloc),
    height_measurement_source(_alloc),
    xy_control_mode(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->active = false;
      this->phase = "";
      this->target_detected = false;
      this->observation_age_s = 0.0f;
      this->target_confidence = 0.0f;
      this->height_source = "";
      this->terminal_trigger_source = "";
      this->odom_height_m = 0.0f;
      this->height_valid = false;
      this->height_measurement_source = "";
      this->height_measurement_fresh = false;
      this->raw_flow_fresh = false;
      this->height_measurement_m = 0.0f;
      this->control_height_m = 0.0f;
      this->tag_depth_valid = false;
      this->tag_depth_m = 0.0f;
      this->align_enter_lateral_m = 0.0f;
      this->align_exit_lateral_m = 0.0f;
      this->active_max_vxy = 0.0f;
      this->err_u_norm_filtered = 0.0f;
      this->err_v_norm_filtered = 0.0f;
      this->err_u_rate_norm_s = 0.0f;
      this->err_v_rate_norm_s = 0.0f;
      this->lateral_error_valid = false;
      this->lateral_error_x_m = 0.0f;
      this->lateral_error_y_m = 0.0f;
      this->lateral_error_m = 0.0f;
      this->lateral_error_rate_x_mps = 0.0f;
      this->lateral_error_rate_y_mps = 0.0f;
      this->z_target_height_m = 0.0f;
      this->z_error_m = 0.0f;
      this->xy_control_mode = "";
      this->cmd_vx = 0.0f;
      this->cmd_vy = 0.0f;
      this->cmd_vz = 0.0f;
      this->cmd_yaw_rate = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _active_type =
    bool;
  _active_type active;
  using _phase_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _phase_type phase;
  using _target_detected_type =
    bool;
  _target_detected_type target_detected;
  using _observation_age_s_type =
    float;
  _observation_age_s_type observation_age_s;
  using _target_confidence_type =
    float;
  _target_confidence_type target_confidence;
  using _height_source_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _height_source_type height_source;
  using _terminal_trigger_source_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _terminal_trigger_source_type terminal_trigger_source;
  using _odom_height_m_type =
    float;
  _odom_height_m_type odom_height_m;
  using _height_valid_type =
    bool;
  _height_valid_type height_valid;
  using _height_measurement_source_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _height_measurement_source_type height_measurement_source;
  using _height_measurement_fresh_type =
    bool;
  _height_measurement_fresh_type height_measurement_fresh;
  using _raw_flow_fresh_type =
    bool;
  _raw_flow_fresh_type raw_flow_fresh;
  using _height_measurement_m_type =
    float;
  _height_measurement_m_type height_measurement_m;
  using _control_height_m_type =
    float;
  _control_height_m_type control_height_m;
  using _tag_depth_valid_type =
    bool;
  _tag_depth_valid_type tag_depth_valid;
  using _tag_depth_m_type =
    float;
  _tag_depth_m_type tag_depth_m;
  using _align_enter_lateral_m_type =
    float;
  _align_enter_lateral_m_type align_enter_lateral_m;
  using _align_exit_lateral_m_type =
    float;
  _align_exit_lateral_m_type align_exit_lateral_m;
  using _active_max_vxy_type =
    float;
  _active_max_vxy_type active_max_vxy;
  using _err_u_norm_filtered_type =
    float;
  _err_u_norm_filtered_type err_u_norm_filtered;
  using _err_v_norm_filtered_type =
    float;
  _err_v_norm_filtered_type err_v_norm_filtered;
  using _err_u_rate_norm_s_type =
    float;
  _err_u_rate_norm_s_type err_u_rate_norm_s;
  using _err_v_rate_norm_s_type =
    float;
  _err_v_rate_norm_s_type err_v_rate_norm_s;
  using _lateral_error_valid_type =
    bool;
  _lateral_error_valid_type lateral_error_valid;
  using _lateral_error_x_m_type =
    float;
  _lateral_error_x_m_type lateral_error_x_m;
  using _lateral_error_y_m_type =
    float;
  _lateral_error_y_m_type lateral_error_y_m;
  using _lateral_error_m_type =
    float;
  _lateral_error_m_type lateral_error_m;
  using _lateral_error_rate_x_mps_type =
    float;
  _lateral_error_rate_x_mps_type lateral_error_rate_x_mps;
  using _lateral_error_rate_y_mps_type =
    float;
  _lateral_error_rate_y_mps_type lateral_error_rate_y_mps;
  using _z_target_height_m_type =
    float;
  _z_target_height_m_type z_target_height_m;
  using _z_error_m_type =
    float;
  _z_error_m_type z_error_m;
  using _xy_control_mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _xy_control_mode_type xy_control_mode;
  using _cmd_vx_type =
    float;
  _cmd_vx_type cmd_vx;
  using _cmd_vy_type =
    float;
  _cmd_vy_type cmd_vy;
  using _cmd_vz_type =
    float;
  _cmd_vz_type cmd_vz;
  using _cmd_yaw_rate_type =
    float;
  _cmd_yaw_rate_type cmd_yaw_rate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__active(
    const bool & _arg)
  {
    this->active = _arg;
    return *this;
  }
  Type & set__phase(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->phase = _arg;
    return *this;
  }
  Type & set__target_detected(
    const bool & _arg)
  {
    this->target_detected = _arg;
    return *this;
  }
  Type & set__observation_age_s(
    const float & _arg)
  {
    this->observation_age_s = _arg;
    return *this;
  }
  Type & set__target_confidence(
    const float & _arg)
  {
    this->target_confidence = _arg;
    return *this;
  }
  Type & set__height_source(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->height_source = _arg;
    return *this;
  }
  Type & set__terminal_trigger_source(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->terminal_trigger_source = _arg;
    return *this;
  }
  Type & set__odom_height_m(
    const float & _arg)
  {
    this->odom_height_m = _arg;
    return *this;
  }
  Type & set__height_valid(
    const bool & _arg)
  {
    this->height_valid = _arg;
    return *this;
  }
  Type & set__height_measurement_source(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->height_measurement_source = _arg;
    return *this;
  }
  Type & set__height_measurement_fresh(
    const bool & _arg)
  {
    this->height_measurement_fresh = _arg;
    return *this;
  }
  Type & set__raw_flow_fresh(
    const bool & _arg)
  {
    this->raw_flow_fresh = _arg;
    return *this;
  }
  Type & set__height_measurement_m(
    const float & _arg)
  {
    this->height_measurement_m = _arg;
    return *this;
  }
  Type & set__control_height_m(
    const float & _arg)
  {
    this->control_height_m = _arg;
    return *this;
  }
  Type & set__tag_depth_valid(
    const bool & _arg)
  {
    this->tag_depth_valid = _arg;
    return *this;
  }
  Type & set__tag_depth_m(
    const float & _arg)
  {
    this->tag_depth_m = _arg;
    return *this;
  }
  Type & set__align_enter_lateral_m(
    const float & _arg)
  {
    this->align_enter_lateral_m = _arg;
    return *this;
  }
  Type & set__align_exit_lateral_m(
    const float & _arg)
  {
    this->align_exit_lateral_m = _arg;
    return *this;
  }
  Type & set__active_max_vxy(
    const float & _arg)
  {
    this->active_max_vxy = _arg;
    return *this;
  }
  Type & set__err_u_norm_filtered(
    const float & _arg)
  {
    this->err_u_norm_filtered = _arg;
    return *this;
  }
  Type & set__err_v_norm_filtered(
    const float & _arg)
  {
    this->err_v_norm_filtered = _arg;
    return *this;
  }
  Type & set__err_u_rate_norm_s(
    const float & _arg)
  {
    this->err_u_rate_norm_s = _arg;
    return *this;
  }
  Type & set__err_v_rate_norm_s(
    const float & _arg)
  {
    this->err_v_rate_norm_s = _arg;
    return *this;
  }
  Type & set__lateral_error_valid(
    const bool & _arg)
  {
    this->lateral_error_valid = _arg;
    return *this;
  }
  Type & set__lateral_error_x_m(
    const float & _arg)
  {
    this->lateral_error_x_m = _arg;
    return *this;
  }
  Type & set__lateral_error_y_m(
    const float & _arg)
  {
    this->lateral_error_y_m = _arg;
    return *this;
  }
  Type & set__lateral_error_m(
    const float & _arg)
  {
    this->lateral_error_m = _arg;
    return *this;
  }
  Type & set__lateral_error_rate_x_mps(
    const float & _arg)
  {
    this->lateral_error_rate_x_mps = _arg;
    return *this;
  }
  Type & set__lateral_error_rate_y_mps(
    const float & _arg)
  {
    this->lateral_error_rate_y_mps = _arg;
    return *this;
  }
  Type & set__z_target_height_m(
    const float & _arg)
  {
    this->z_target_height_m = _arg;
    return *this;
  }
  Type & set__z_error_m(
    const float & _arg)
  {
    this->z_error_m = _arg;
    return *this;
  }
  Type & set__xy_control_mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->xy_control_mode = _arg;
    return *this;
  }
  Type & set__cmd_vx(
    const float & _arg)
  {
    this->cmd_vx = _arg;
    return *this;
  }
  Type & set__cmd_vy(
    const float & _arg)
  {
    this->cmd_vy = _arg;
    return *this;
  }
  Type & set__cmd_vz(
    const float & _arg)
  {
    this->cmd_vz = _arg;
    return *this;
  }
  Type & set__cmd_yaw_rate(
    const float & _arg)
  {
    this->cmd_yaw_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> *;
  using ConstRawPtr =
    const uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uav_visual_landing__msg__LandingControllerState
    std::shared_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uav_visual_landing__msg__LandingControllerState
    std::shared_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LandingControllerState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->active != other.active) {
      return false;
    }
    if (this->phase != other.phase) {
      return false;
    }
    if (this->target_detected != other.target_detected) {
      return false;
    }
    if (this->observation_age_s != other.observation_age_s) {
      return false;
    }
    if (this->target_confidence != other.target_confidence) {
      return false;
    }
    if (this->height_source != other.height_source) {
      return false;
    }
    if (this->terminal_trigger_source != other.terminal_trigger_source) {
      return false;
    }
    if (this->odom_height_m != other.odom_height_m) {
      return false;
    }
    if (this->height_valid != other.height_valid) {
      return false;
    }
    if (this->height_measurement_source != other.height_measurement_source) {
      return false;
    }
    if (this->height_measurement_fresh != other.height_measurement_fresh) {
      return false;
    }
    if (this->raw_flow_fresh != other.raw_flow_fresh) {
      return false;
    }
    if (this->height_measurement_m != other.height_measurement_m) {
      return false;
    }
    if (this->control_height_m != other.control_height_m) {
      return false;
    }
    if (this->tag_depth_valid != other.tag_depth_valid) {
      return false;
    }
    if (this->tag_depth_m != other.tag_depth_m) {
      return false;
    }
    if (this->align_enter_lateral_m != other.align_enter_lateral_m) {
      return false;
    }
    if (this->align_exit_lateral_m != other.align_exit_lateral_m) {
      return false;
    }
    if (this->active_max_vxy != other.active_max_vxy) {
      return false;
    }
    if (this->err_u_norm_filtered != other.err_u_norm_filtered) {
      return false;
    }
    if (this->err_v_norm_filtered != other.err_v_norm_filtered) {
      return false;
    }
    if (this->err_u_rate_norm_s != other.err_u_rate_norm_s) {
      return false;
    }
    if (this->err_v_rate_norm_s != other.err_v_rate_norm_s) {
      return false;
    }
    if (this->lateral_error_valid != other.lateral_error_valid) {
      return false;
    }
    if (this->lateral_error_x_m != other.lateral_error_x_m) {
      return false;
    }
    if (this->lateral_error_y_m != other.lateral_error_y_m) {
      return false;
    }
    if (this->lateral_error_m != other.lateral_error_m) {
      return false;
    }
    if (this->lateral_error_rate_x_mps != other.lateral_error_rate_x_mps) {
      return false;
    }
    if (this->lateral_error_rate_y_mps != other.lateral_error_rate_y_mps) {
      return false;
    }
    if (this->z_target_height_m != other.z_target_height_m) {
      return false;
    }
    if (this->z_error_m != other.z_error_m) {
      return false;
    }
    if (this->xy_control_mode != other.xy_control_mode) {
      return false;
    }
    if (this->cmd_vx != other.cmd_vx) {
      return false;
    }
    if (this->cmd_vy != other.cmd_vy) {
      return false;
    }
    if (this->cmd_vz != other.cmd_vz) {
      return false;
    }
    if (this->cmd_yaw_rate != other.cmd_yaw_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const LandingControllerState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LandingControllerState_

// alias to use template instance with default allocator
using LandingControllerState =
  uav_visual_landing::msg::LandingControllerState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uav_visual_landing

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_HPP_
