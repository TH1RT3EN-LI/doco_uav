// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uav_mode_supervisor:msg/SupervisorState.idl
// generated code does not contain a copyright notice

#ifndef UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__STRUCT_H_
#define UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'owner'
// Member 'last_command'
// Member 'pending_command'
// Member 'last_message'
// Member 'fusion_reason'
// Member 'visual_phase'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/SupervisorState in the package uav_mode_supervisor.
typedef struct uav_mode_supervisor__msg__SupervisorState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String owner;
  float active_tracking_height_m;
  rosidl_runtime_c__String last_command;
  rosidl_runtime_c__String pending_command;
  bool command_in_progress;
  rosidl_runtime_c__String last_message;
  bool fusion_diagnostics_seen;
  bool fusion_initialized;
  bool fusion_relocalize_requested;
  bool fusion_ready;
  rosidl_runtime_c__String fusion_reason;
  bool visual_state_seen;
  bool visual_active;
  bool visual_target_detected;
  rosidl_runtime_c__String visual_phase;
  bool visual_committed;
  bool visual_capture_observed;
} uav_mode_supervisor__msg__SupervisorState;

// Struct for a sequence of uav_mode_supervisor__msg__SupervisorState.
typedef struct uav_mode_supervisor__msg__SupervisorState__Sequence
{
  uav_mode_supervisor__msg__SupervisorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uav_mode_supervisor__msg__SupervisorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__STRUCT_H_
