// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uav_visual_landing:msg/LandingControllerState.idl
// generated code does not contain a copyright notice
#include "uav_visual_landing/msg/detail/landing_controller_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `phase`
// Member `height_source`
// Member `terminal_trigger_source`
// Member `height_measurement_source`
// Member `xy_control_mode`
#include "rosidl_runtime_c/string_functions.h"

bool
uav_visual_landing__msg__LandingControllerState__init(uav_visual_landing__msg__LandingControllerState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    uav_visual_landing__msg__LandingControllerState__fini(msg);
    return false;
  }
  // active
  // phase
  if (!rosidl_runtime_c__String__init(&msg->phase)) {
    uav_visual_landing__msg__LandingControllerState__fini(msg);
    return false;
  }
  // target_detected
  // observation_age_s
  // target_confidence
  // height_source
  if (!rosidl_runtime_c__String__init(&msg->height_source)) {
    uav_visual_landing__msg__LandingControllerState__fini(msg);
    return false;
  }
  // terminal_trigger_source
  if (!rosidl_runtime_c__String__init(&msg->terminal_trigger_source)) {
    uav_visual_landing__msg__LandingControllerState__fini(msg);
    return false;
  }
  // odom_height_m
  // height_valid
  // height_measurement_source
  if (!rosidl_runtime_c__String__init(&msg->height_measurement_source)) {
    uav_visual_landing__msg__LandingControllerState__fini(msg);
    return false;
  }
  // height_measurement_fresh
  // raw_flow_fresh
  // height_measurement_m
  // control_height_m
  // tag_depth_valid
  // tag_depth_m
  // align_enter_lateral_m
  // align_exit_lateral_m
  // active_max_vxy
  // err_u_norm_filtered
  // err_v_norm_filtered
  // err_u_rate_norm_s
  // err_v_rate_norm_s
  // lateral_error_valid
  // lateral_error_x_m
  // lateral_error_y_m
  // lateral_error_m
  // lateral_error_rate_x_mps
  // lateral_error_rate_y_mps
  // z_target_height_m
  // z_error_m
  // xy_control_mode
  if (!rosidl_runtime_c__String__init(&msg->xy_control_mode)) {
    uav_visual_landing__msg__LandingControllerState__fini(msg);
    return false;
  }
  // cmd_vx
  // cmd_vy
  // cmd_vz
  // cmd_yaw_rate
  return true;
}

void
uav_visual_landing__msg__LandingControllerState__fini(uav_visual_landing__msg__LandingControllerState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // active
  // phase
  rosidl_runtime_c__String__fini(&msg->phase);
  // target_detected
  // observation_age_s
  // target_confidence
  // height_source
  rosidl_runtime_c__String__fini(&msg->height_source);
  // terminal_trigger_source
  rosidl_runtime_c__String__fini(&msg->terminal_trigger_source);
  // odom_height_m
  // height_valid
  // height_measurement_source
  rosidl_runtime_c__String__fini(&msg->height_measurement_source);
  // height_measurement_fresh
  // raw_flow_fresh
  // height_measurement_m
  // control_height_m
  // tag_depth_valid
  // tag_depth_m
  // align_enter_lateral_m
  // align_exit_lateral_m
  // active_max_vxy
  // err_u_norm_filtered
  // err_v_norm_filtered
  // err_u_rate_norm_s
  // err_v_rate_norm_s
  // lateral_error_valid
  // lateral_error_x_m
  // lateral_error_y_m
  // lateral_error_m
  // lateral_error_rate_x_mps
  // lateral_error_rate_y_mps
  // z_target_height_m
  // z_error_m
  // xy_control_mode
  rosidl_runtime_c__String__fini(&msg->xy_control_mode);
  // cmd_vx
  // cmd_vy
  // cmd_vz
  // cmd_yaw_rate
}

bool
uav_visual_landing__msg__LandingControllerState__are_equal(const uav_visual_landing__msg__LandingControllerState * lhs, const uav_visual_landing__msg__LandingControllerState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // active
  if (lhs->active != rhs->active) {
    return false;
  }
  // phase
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->phase), &(rhs->phase)))
  {
    return false;
  }
  // target_detected
  if (lhs->target_detected != rhs->target_detected) {
    return false;
  }
  // observation_age_s
  if (lhs->observation_age_s != rhs->observation_age_s) {
    return false;
  }
  // target_confidence
  if (lhs->target_confidence != rhs->target_confidence) {
    return false;
  }
  // height_source
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->height_source), &(rhs->height_source)))
  {
    return false;
  }
  // terminal_trigger_source
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->terminal_trigger_source), &(rhs->terminal_trigger_source)))
  {
    return false;
  }
  // odom_height_m
  if (lhs->odom_height_m != rhs->odom_height_m) {
    return false;
  }
  // height_valid
  if (lhs->height_valid != rhs->height_valid) {
    return false;
  }
  // height_measurement_source
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->height_measurement_source), &(rhs->height_measurement_source)))
  {
    return false;
  }
  // height_measurement_fresh
  if (lhs->height_measurement_fresh != rhs->height_measurement_fresh) {
    return false;
  }
  // raw_flow_fresh
  if (lhs->raw_flow_fresh != rhs->raw_flow_fresh) {
    return false;
  }
  // height_measurement_m
  if (lhs->height_measurement_m != rhs->height_measurement_m) {
    return false;
  }
  // control_height_m
  if (lhs->control_height_m != rhs->control_height_m) {
    return false;
  }
  // tag_depth_valid
  if (lhs->tag_depth_valid != rhs->tag_depth_valid) {
    return false;
  }
  // tag_depth_m
  if (lhs->tag_depth_m != rhs->tag_depth_m) {
    return false;
  }
  // align_enter_lateral_m
  if (lhs->align_enter_lateral_m != rhs->align_enter_lateral_m) {
    return false;
  }
  // align_exit_lateral_m
  if (lhs->align_exit_lateral_m != rhs->align_exit_lateral_m) {
    return false;
  }
  // active_max_vxy
  if (lhs->active_max_vxy != rhs->active_max_vxy) {
    return false;
  }
  // err_u_norm_filtered
  if (lhs->err_u_norm_filtered != rhs->err_u_norm_filtered) {
    return false;
  }
  // err_v_norm_filtered
  if (lhs->err_v_norm_filtered != rhs->err_v_norm_filtered) {
    return false;
  }
  // err_u_rate_norm_s
  if (lhs->err_u_rate_norm_s != rhs->err_u_rate_norm_s) {
    return false;
  }
  // err_v_rate_norm_s
  if (lhs->err_v_rate_norm_s != rhs->err_v_rate_norm_s) {
    return false;
  }
  // lateral_error_valid
  if (lhs->lateral_error_valid != rhs->lateral_error_valid) {
    return false;
  }
  // lateral_error_x_m
  if (lhs->lateral_error_x_m != rhs->lateral_error_x_m) {
    return false;
  }
  // lateral_error_y_m
  if (lhs->lateral_error_y_m != rhs->lateral_error_y_m) {
    return false;
  }
  // lateral_error_m
  if (lhs->lateral_error_m != rhs->lateral_error_m) {
    return false;
  }
  // lateral_error_rate_x_mps
  if (lhs->lateral_error_rate_x_mps != rhs->lateral_error_rate_x_mps) {
    return false;
  }
  // lateral_error_rate_y_mps
  if (lhs->lateral_error_rate_y_mps != rhs->lateral_error_rate_y_mps) {
    return false;
  }
  // z_target_height_m
  if (lhs->z_target_height_m != rhs->z_target_height_m) {
    return false;
  }
  // z_error_m
  if (lhs->z_error_m != rhs->z_error_m) {
    return false;
  }
  // xy_control_mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->xy_control_mode), &(rhs->xy_control_mode)))
  {
    return false;
  }
  // cmd_vx
  if (lhs->cmd_vx != rhs->cmd_vx) {
    return false;
  }
  // cmd_vy
  if (lhs->cmd_vy != rhs->cmd_vy) {
    return false;
  }
  // cmd_vz
  if (lhs->cmd_vz != rhs->cmd_vz) {
    return false;
  }
  // cmd_yaw_rate
  if (lhs->cmd_yaw_rate != rhs->cmd_yaw_rate) {
    return false;
  }
  return true;
}

bool
uav_visual_landing__msg__LandingControllerState__copy(
  const uav_visual_landing__msg__LandingControllerState * input,
  uav_visual_landing__msg__LandingControllerState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // active
  output->active = input->active;
  // phase
  if (!rosidl_runtime_c__String__copy(
      &(input->phase), &(output->phase)))
  {
    return false;
  }
  // target_detected
  output->target_detected = input->target_detected;
  // observation_age_s
  output->observation_age_s = input->observation_age_s;
  // target_confidence
  output->target_confidence = input->target_confidence;
  // height_source
  if (!rosidl_runtime_c__String__copy(
      &(input->height_source), &(output->height_source)))
  {
    return false;
  }
  // terminal_trigger_source
  if (!rosidl_runtime_c__String__copy(
      &(input->terminal_trigger_source), &(output->terminal_trigger_source)))
  {
    return false;
  }
  // odom_height_m
  output->odom_height_m = input->odom_height_m;
  // height_valid
  output->height_valid = input->height_valid;
  // height_measurement_source
  if (!rosidl_runtime_c__String__copy(
      &(input->height_measurement_source), &(output->height_measurement_source)))
  {
    return false;
  }
  // height_measurement_fresh
  output->height_measurement_fresh = input->height_measurement_fresh;
  // raw_flow_fresh
  output->raw_flow_fresh = input->raw_flow_fresh;
  // height_measurement_m
  output->height_measurement_m = input->height_measurement_m;
  // control_height_m
  output->control_height_m = input->control_height_m;
  // tag_depth_valid
  output->tag_depth_valid = input->tag_depth_valid;
  // tag_depth_m
  output->tag_depth_m = input->tag_depth_m;
  // align_enter_lateral_m
  output->align_enter_lateral_m = input->align_enter_lateral_m;
  // align_exit_lateral_m
  output->align_exit_lateral_m = input->align_exit_lateral_m;
  // active_max_vxy
  output->active_max_vxy = input->active_max_vxy;
  // err_u_norm_filtered
  output->err_u_norm_filtered = input->err_u_norm_filtered;
  // err_v_norm_filtered
  output->err_v_norm_filtered = input->err_v_norm_filtered;
  // err_u_rate_norm_s
  output->err_u_rate_norm_s = input->err_u_rate_norm_s;
  // err_v_rate_norm_s
  output->err_v_rate_norm_s = input->err_v_rate_norm_s;
  // lateral_error_valid
  output->lateral_error_valid = input->lateral_error_valid;
  // lateral_error_x_m
  output->lateral_error_x_m = input->lateral_error_x_m;
  // lateral_error_y_m
  output->lateral_error_y_m = input->lateral_error_y_m;
  // lateral_error_m
  output->lateral_error_m = input->lateral_error_m;
  // lateral_error_rate_x_mps
  output->lateral_error_rate_x_mps = input->lateral_error_rate_x_mps;
  // lateral_error_rate_y_mps
  output->lateral_error_rate_y_mps = input->lateral_error_rate_y_mps;
  // z_target_height_m
  output->z_target_height_m = input->z_target_height_m;
  // z_error_m
  output->z_error_m = input->z_error_m;
  // xy_control_mode
  if (!rosidl_runtime_c__String__copy(
      &(input->xy_control_mode), &(output->xy_control_mode)))
  {
    return false;
  }
  // cmd_vx
  output->cmd_vx = input->cmd_vx;
  // cmd_vy
  output->cmd_vy = input->cmd_vy;
  // cmd_vz
  output->cmd_vz = input->cmd_vz;
  // cmd_yaw_rate
  output->cmd_yaw_rate = input->cmd_yaw_rate;
  return true;
}

uav_visual_landing__msg__LandingControllerState *
uav_visual_landing__msg__LandingControllerState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_visual_landing__msg__LandingControllerState * msg = (uav_visual_landing__msg__LandingControllerState *)allocator.allocate(sizeof(uav_visual_landing__msg__LandingControllerState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uav_visual_landing__msg__LandingControllerState));
  bool success = uav_visual_landing__msg__LandingControllerState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uav_visual_landing__msg__LandingControllerState__destroy(uav_visual_landing__msg__LandingControllerState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uav_visual_landing__msg__LandingControllerState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uav_visual_landing__msg__LandingControllerState__Sequence__init(uav_visual_landing__msg__LandingControllerState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_visual_landing__msg__LandingControllerState * data = NULL;

  if (size) {
    data = (uav_visual_landing__msg__LandingControllerState *)allocator.zero_allocate(size, sizeof(uav_visual_landing__msg__LandingControllerState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uav_visual_landing__msg__LandingControllerState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uav_visual_landing__msg__LandingControllerState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
uav_visual_landing__msg__LandingControllerState__Sequence__fini(uav_visual_landing__msg__LandingControllerState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      uav_visual_landing__msg__LandingControllerState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

uav_visual_landing__msg__LandingControllerState__Sequence *
uav_visual_landing__msg__LandingControllerState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_visual_landing__msg__LandingControllerState__Sequence * array = (uav_visual_landing__msg__LandingControllerState__Sequence *)allocator.allocate(sizeof(uav_visual_landing__msg__LandingControllerState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uav_visual_landing__msg__LandingControllerState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uav_visual_landing__msg__LandingControllerState__Sequence__destroy(uav_visual_landing__msg__LandingControllerState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uav_visual_landing__msg__LandingControllerState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uav_visual_landing__msg__LandingControllerState__Sequence__are_equal(const uav_visual_landing__msg__LandingControllerState__Sequence * lhs, const uav_visual_landing__msg__LandingControllerState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uav_visual_landing__msg__LandingControllerState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uav_visual_landing__msg__LandingControllerState__Sequence__copy(
  const uav_visual_landing__msg__LandingControllerState__Sequence * input,
  uav_visual_landing__msg__LandingControllerState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uav_visual_landing__msg__LandingControllerState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uav_visual_landing__msg__LandingControllerState * data =
      (uav_visual_landing__msg__LandingControllerState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uav_visual_landing__msg__LandingControllerState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uav_visual_landing__msg__LandingControllerState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uav_visual_landing__msg__LandingControllerState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
