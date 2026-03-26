// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uav_mode_supervisor:srv/CommandSupervisor.idl
// generated code does not contain a copyright notice

#ifndef UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__STRUCT_H_
#define UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CommandSupervisor in the package uav_mode_supervisor.
typedef struct uav_mode_supervisor__srv__CommandSupervisor_Request
{
  rosidl_runtime_c__String command;
  float tracking_target_height_m;
} uav_mode_supervisor__srv__CommandSupervisor_Request;

// Struct for a sequence of uav_mode_supervisor__srv__CommandSupervisor_Request.
typedef struct uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence
{
  uav_mode_supervisor__srv__CommandSupervisor_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CommandSupervisor in the package uav_mode_supervisor.
typedef struct uav_mode_supervisor__srv__CommandSupervisor_Response
{
  bool success;
  rosidl_runtime_c__String message;
} uav_mode_supervisor__srv__CommandSupervisor_Response;

// Struct for a sequence of uav_mode_supervisor__srv__CommandSupervisor_Response.
typedef struct uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence
{
  uav_mode_supervisor__srv__CommandSupervisor_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__STRUCT_H_
