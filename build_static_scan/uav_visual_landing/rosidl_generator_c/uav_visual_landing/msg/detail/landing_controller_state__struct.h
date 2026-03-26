// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uav_visual_landing:msg/LandingControllerState.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_H_
#define UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_H_

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
// Member 'phase'
// Member 'height_source'
// Member 'terminal_trigger_source'
// Member 'height_measurement_source'
// Member 'xy_control_mode'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/LandingControllerState in the package uav_visual_landing.
typedef struct uav_visual_landing__msg__LandingControllerState
{
  std_msgs__msg__Header header;
  bool active;
  rosidl_runtime_c__String phase;
  bool target_detected;
  float observation_age_s;
  float target_confidence;
  rosidl_runtime_c__String height_source;
  rosidl_runtime_c__String terminal_trigger_source;
  float odom_height_m;
  bool height_valid;
  rosidl_runtime_c__String height_measurement_source;
  bool height_measurement_fresh;
  bool raw_flow_fresh;
  float height_measurement_m;
  float control_height_m;
  bool tag_depth_valid;
  float tag_depth_m;
  float align_enter_lateral_m;
  float align_exit_lateral_m;
  float active_max_vxy;
  float err_u_norm_filtered;
  float err_v_norm_filtered;
  float err_u_rate_norm_s;
  float err_v_rate_norm_s;
  bool lateral_error_valid;
  float lateral_error_x_m;
  float lateral_error_y_m;
  float lateral_error_m;
  float lateral_error_rate_x_mps;
  float lateral_error_rate_y_mps;
  float z_target_height_m;
  float z_error_m;
  rosidl_runtime_c__String xy_control_mode;
  float cmd_vx;
  float cmd_vy;
  float cmd_vz;
  float cmd_yaw_rate;
} uav_visual_landing__msg__LandingControllerState;

// Struct for a sequence of uav_visual_landing__msg__LandingControllerState.
typedef struct uav_visual_landing__msg__LandingControllerState__Sequence
{
  uav_visual_landing__msg__LandingControllerState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uav_visual_landing__msg__LandingControllerState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_H_
