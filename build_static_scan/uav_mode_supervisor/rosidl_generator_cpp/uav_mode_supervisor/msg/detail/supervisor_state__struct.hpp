// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uav_mode_supervisor:msg/SupervisorState.idl
// generated code does not contain a copyright notice

#ifndef UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__STRUCT_HPP_
#define UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__STRUCT_HPP_

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
# define DEPRECATED__uav_mode_supervisor__msg__SupervisorState __attribute__((deprecated))
#else
# define DEPRECATED__uav_mode_supervisor__msg__SupervisorState __declspec(deprecated)
#endif

namespace uav_mode_supervisor
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SupervisorState_
{
  using Type = SupervisorState_<ContainerAllocator>;

  explicit SupervisorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->owner = "";
      this->active_tracking_height_m = 0.0f;
      this->last_command = "";
      this->pending_command = "";
      this->command_in_progress = false;
      this->last_message = "";
      this->fusion_diagnostics_seen = false;
      this->fusion_initialized = false;
      this->fusion_relocalize_requested = false;
      this->fusion_ready = false;
      this->fusion_reason = "";
      this->visual_state_seen = false;
      this->visual_active = false;
      this->visual_target_detected = false;
      this->visual_phase = "";
      this->visual_committed = false;
      this->visual_capture_observed = false;
    }
  }

  explicit SupervisorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    owner(_alloc),
    last_command(_alloc),
    pending_command(_alloc),
    last_message(_alloc),
    fusion_reason(_alloc),
    visual_phase(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->owner = "";
      this->active_tracking_height_m = 0.0f;
      this->last_command = "";
      this->pending_command = "";
      this->command_in_progress = false;
      this->last_message = "";
      this->fusion_diagnostics_seen = false;
      this->fusion_initialized = false;
      this->fusion_relocalize_requested = false;
      this->fusion_ready = false;
      this->fusion_reason = "";
      this->visual_state_seen = false;
      this->visual_active = false;
      this->visual_target_detected = false;
      this->visual_phase = "";
      this->visual_committed = false;
      this->visual_capture_observed = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _owner_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _owner_type owner;
  using _active_tracking_height_m_type =
    float;
  _active_tracking_height_m_type active_tracking_height_m;
  using _last_command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _last_command_type last_command;
  using _pending_command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _pending_command_type pending_command;
  using _command_in_progress_type =
    bool;
  _command_in_progress_type command_in_progress;
  using _last_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _last_message_type last_message;
  using _fusion_diagnostics_seen_type =
    bool;
  _fusion_diagnostics_seen_type fusion_diagnostics_seen;
  using _fusion_initialized_type =
    bool;
  _fusion_initialized_type fusion_initialized;
  using _fusion_relocalize_requested_type =
    bool;
  _fusion_relocalize_requested_type fusion_relocalize_requested;
  using _fusion_ready_type =
    bool;
  _fusion_ready_type fusion_ready;
  using _fusion_reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _fusion_reason_type fusion_reason;
  using _visual_state_seen_type =
    bool;
  _visual_state_seen_type visual_state_seen;
  using _visual_active_type =
    bool;
  _visual_active_type visual_active;
  using _visual_target_detected_type =
    bool;
  _visual_target_detected_type visual_target_detected;
  using _visual_phase_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _visual_phase_type visual_phase;
  using _visual_committed_type =
    bool;
  _visual_committed_type visual_committed;
  using _visual_capture_observed_type =
    bool;
  _visual_capture_observed_type visual_capture_observed;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__owner(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->owner = _arg;
    return *this;
  }
  Type & set__active_tracking_height_m(
    const float & _arg)
  {
    this->active_tracking_height_m = _arg;
    return *this;
  }
  Type & set__last_command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->last_command = _arg;
    return *this;
  }
  Type & set__pending_command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->pending_command = _arg;
    return *this;
  }
  Type & set__command_in_progress(
    const bool & _arg)
  {
    this->command_in_progress = _arg;
    return *this;
  }
  Type & set__last_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->last_message = _arg;
    return *this;
  }
  Type & set__fusion_diagnostics_seen(
    const bool & _arg)
  {
    this->fusion_diagnostics_seen = _arg;
    return *this;
  }
  Type & set__fusion_initialized(
    const bool & _arg)
  {
    this->fusion_initialized = _arg;
    return *this;
  }
  Type & set__fusion_relocalize_requested(
    const bool & _arg)
  {
    this->fusion_relocalize_requested = _arg;
    return *this;
  }
  Type & set__fusion_ready(
    const bool & _arg)
  {
    this->fusion_ready = _arg;
    return *this;
  }
  Type & set__fusion_reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->fusion_reason = _arg;
    return *this;
  }
  Type & set__visual_state_seen(
    const bool & _arg)
  {
    this->visual_state_seen = _arg;
    return *this;
  }
  Type & set__visual_active(
    const bool & _arg)
  {
    this->visual_active = _arg;
    return *this;
  }
  Type & set__visual_target_detected(
    const bool & _arg)
  {
    this->visual_target_detected = _arg;
    return *this;
  }
  Type & set__visual_phase(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->visual_phase = _arg;
    return *this;
  }
  Type & set__visual_committed(
    const bool & _arg)
  {
    this->visual_committed = _arg;
    return *this;
  }
  Type & set__visual_capture_observed(
    const bool & _arg)
  {
    this->visual_capture_observed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uav_mode_supervisor__msg__SupervisorState
    std::shared_ptr<uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uav_mode_supervisor__msg__SupervisorState
    std::shared_ptr<uav_mode_supervisor::msg::SupervisorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SupervisorState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->owner != other.owner) {
      return false;
    }
    if (this->active_tracking_height_m != other.active_tracking_height_m) {
      return false;
    }
    if (this->last_command != other.last_command) {
      return false;
    }
    if (this->pending_command != other.pending_command) {
      return false;
    }
    if (this->command_in_progress != other.command_in_progress) {
      return false;
    }
    if (this->last_message != other.last_message) {
      return false;
    }
    if (this->fusion_diagnostics_seen != other.fusion_diagnostics_seen) {
      return false;
    }
    if (this->fusion_initialized != other.fusion_initialized) {
      return false;
    }
    if (this->fusion_relocalize_requested != other.fusion_relocalize_requested) {
      return false;
    }
    if (this->fusion_ready != other.fusion_ready) {
      return false;
    }
    if (this->fusion_reason != other.fusion_reason) {
      return false;
    }
    if (this->visual_state_seen != other.visual_state_seen) {
      return false;
    }
    if (this->visual_active != other.visual_active) {
      return false;
    }
    if (this->visual_target_detected != other.visual_target_detected) {
      return false;
    }
    if (this->visual_phase != other.visual_phase) {
      return false;
    }
    if (this->visual_committed != other.visual_committed) {
      return false;
    }
    if (this->visual_capture_observed != other.visual_capture_observed) {
      return false;
    }
    return true;
  }
  bool operator!=(const SupervisorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SupervisorState_

// alias to use template instance with default allocator
using SupervisorState =
  uav_mode_supervisor::msg::SupervisorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uav_mode_supervisor

#endif  // UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__STRUCT_HPP_
