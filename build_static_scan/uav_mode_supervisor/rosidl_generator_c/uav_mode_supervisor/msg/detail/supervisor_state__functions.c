// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uav_mode_supervisor:msg/SupervisorState.idl
// generated code does not contain a copyright notice
#include "uav_mode_supervisor/msg/detail/supervisor_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `owner`
// Member `last_command`
// Member `pending_command`
// Member `last_message`
// Member `fusion_reason`
// Member `visual_phase`
#include "rosidl_runtime_c/string_functions.h"

bool
uav_mode_supervisor__msg__SupervisorState__init(uav_mode_supervisor__msg__SupervisorState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    uav_mode_supervisor__msg__SupervisorState__fini(msg);
    return false;
  }
  // owner
  if (!rosidl_runtime_c__String__init(&msg->owner)) {
    uav_mode_supervisor__msg__SupervisorState__fini(msg);
    return false;
  }
  // active_tracking_height_m
  // last_command
  if (!rosidl_runtime_c__String__init(&msg->last_command)) {
    uav_mode_supervisor__msg__SupervisorState__fini(msg);
    return false;
  }
  // pending_command
  if (!rosidl_runtime_c__String__init(&msg->pending_command)) {
    uav_mode_supervisor__msg__SupervisorState__fini(msg);
    return false;
  }
  // command_in_progress
  // last_message
  if (!rosidl_runtime_c__String__init(&msg->last_message)) {
    uav_mode_supervisor__msg__SupervisorState__fini(msg);
    return false;
  }
  // fusion_diagnostics_seen
  // fusion_initialized
  // fusion_relocalize_requested
  // fusion_ready
  // fusion_reason
  if (!rosidl_runtime_c__String__init(&msg->fusion_reason)) {
    uav_mode_supervisor__msg__SupervisorState__fini(msg);
    return false;
  }
  // visual_state_seen
  // visual_active
  // visual_target_detected
  // visual_phase
  if (!rosidl_runtime_c__String__init(&msg->visual_phase)) {
    uav_mode_supervisor__msg__SupervisorState__fini(msg);
    return false;
  }
  // visual_committed
  // visual_capture_observed
  return true;
}

void
uav_mode_supervisor__msg__SupervisorState__fini(uav_mode_supervisor__msg__SupervisorState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // owner
  rosidl_runtime_c__String__fini(&msg->owner);
  // active_tracking_height_m
  // last_command
  rosidl_runtime_c__String__fini(&msg->last_command);
  // pending_command
  rosidl_runtime_c__String__fini(&msg->pending_command);
  // command_in_progress
  // last_message
  rosidl_runtime_c__String__fini(&msg->last_message);
  // fusion_diagnostics_seen
  // fusion_initialized
  // fusion_relocalize_requested
  // fusion_ready
  // fusion_reason
  rosidl_runtime_c__String__fini(&msg->fusion_reason);
  // visual_state_seen
  // visual_active
  // visual_target_detected
  // visual_phase
  rosidl_runtime_c__String__fini(&msg->visual_phase);
  // visual_committed
  // visual_capture_observed
}

bool
uav_mode_supervisor__msg__SupervisorState__are_equal(const uav_mode_supervisor__msg__SupervisorState * lhs, const uav_mode_supervisor__msg__SupervisorState * rhs)
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
  // owner
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->owner), &(rhs->owner)))
  {
    return false;
  }
  // active_tracking_height_m
  if (lhs->active_tracking_height_m != rhs->active_tracking_height_m) {
    return false;
  }
  // last_command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->last_command), &(rhs->last_command)))
  {
    return false;
  }
  // pending_command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->pending_command), &(rhs->pending_command)))
  {
    return false;
  }
  // command_in_progress
  if (lhs->command_in_progress != rhs->command_in_progress) {
    return false;
  }
  // last_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->last_message), &(rhs->last_message)))
  {
    return false;
  }
  // fusion_diagnostics_seen
  if (lhs->fusion_diagnostics_seen != rhs->fusion_diagnostics_seen) {
    return false;
  }
  // fusion_initialized
  if (lhs->fusion_initialized != rhs->fusion_initialized) {
    return false;
  }
  // fusion_relocalize_requested
  if (lhs->fusion_relocalize_requested != rhs->fusion_relocalize_requested) {
    return false;
  }
  // fusion_ready
  if (lhs->fusion_ready != rhs->fusion_ready) {
    return false;
  }
  // fusion_reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->fusion_reason), &(rhs->fusion_reason)))
  {
    return false;
  }
  // visual_state_seen
  if (lhs->visual_state_seen != rhs->visual_state_seen) {
    return false;
  }
  // visual_active
  if (lhs->visual_active != rhs->visual_active) {
    return false;
  }
  // visual_target_detected
  if (lhs->visual_target_detected != rhs->visual_target_detected) {
    return false;
  }
  // visual_phase
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->visual_phase), &(rhs->visual_phase)))
  {
    return false;
  }
  // visual_committed
  if (lhs->visual_committed != rhs->visual_committed) {
    return false;
  }
  // visual_capture_observed
  if (lhs->visual_capture_observed != rhs->visual_capture_observed) {
    return false;
  }
  return true;
}

bool
uav_mode_supervisor__msg__SupervisorState__copy(
  const uav_mode_supervisor__msg__SupervisorState * input,
  uav_mode_supervisor__msg__SupervisorState * output)
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
  // owner
  if (!rosidl_runtime_c__String__copy(
      &(input->owner), &(output->owner)))
  {
    return false;
  }
  // active_tracking_height_m
  output->active_tracking_height_m = input->active_tracking_height_m;
  // last_command
  if (!rosidl_runtime_c__String__copy(
      &(input->last_command), &(output->last_command)))
  {
    return false;
  }
  // pending_command
  if (!rosidl_runtime_c__String__copy(
      &(input->pending_command), &(output->pending_command)))
  {
    return false;
  }
  // command_in_progress
  output->command_in_progress = input->command_in_progress;
  // last_message
  if (!rosidl_runtime_c__String__copy(
      &(input->last_message), &(output->last_message)))
  {
    return false;
  }
  // fusion_diagnostics_seen
  output->fusion_diagnostics_seen = input->fusion_diagnostics_seen;
  // fusion_initialized
  output->fusion_initialized = input->fusion_initialized;
  // fusion_relocalize_requested
  output->fusion_relocalize_requested = input->fusion_relocalize_requested;
  // fusion_ready
  output->fusion_ready = input->fusion_ready;
  // fusion_reason
  if (!rosidl_runtime_c__String__copy(
      &(input->fusion_reason), &(output->fusion_reason)))
  {
    return false;
  }
  // visual_state_seen
  output->visual_state_seen = input->visual_state_seen;
  // visual_active
  output->visual_active = input->visual_active;
  // visual_target_detected
  output->visual_target_detected = input->visual_target_detected;
  // visual_phase
  if (!rosidl_runtime_c__String__copy(
      &(input->visual_phase), &(output->visual_phase)))
  {
    return false;
  }
  // visual_committed
  output->visual_committed = input->visual_committed;
  // visual_capture_observed
  output->visual_capture_observed = input->visual_capture_observed;
  return true;
}

uav_mode_supervisor__msg__SupervisorState *
uav_mode_supervisor__msg__SupervisorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_mode_supervisor__msg__SupervisorState * msg = (uav_mode_supervisor__msg__SupervisorState *)allocator.allocate(sizeof(uav_mode_supervisor__msg__SupervisorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uav_mode_supervisor__msg__SupervisorState));
  bool success = uav_mode_supervisor__msg__SupervisorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uav_mode_supervisor__msg__SupervisorState__destroy(uav_mode_supervisor__msg__SupervisorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uav_mode_supervisor__msg__SupervisorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uav_mode_supervisor__msg__SupervisorState__Sequence__init(uav_mode_supervisor__msg__SupervisorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_mode_supervisor__msg__SupervisorState * data = NULL;

  if (size) {
    data = (uav_mode_supervisor__msg__SupervisorState *)allocator.zero_allocate(size, sizeof(uav_mode_supervisor__msg__SupervisorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uav_mode_supervisor__msg__SupervisorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uav_mode_supervisor__msg__SupervisorState__fini(&data[i - 1]);
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
uav_mode_supervisor__msg__SupervisorState__Sequence__fini(uav_mode_supervisor__msg__SupervisorState__Sequence * array)
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
      uav_mode_supervisor__msg__SupervisorState__fini(&array->data[i]);
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

uav_mode_supervisor__msg__SupervisorState__Sequence *
uav_mode_supervisor__msg__SupervisorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_mode_supervisor__msg__SupervisorState__Sequence * array = (uav_mode_supervisor__msg__SupervisorState__Sequence *)allocator.allocate(sizeof(uav_mode_supervisor__msg__SupervisorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uav_mode_supervisor__msg__SupervisorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uav_mode_supervisor__msg__SupervisorState__Sequence__destroy(uav_mode_supervisor__msg__SupervisorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uav_mode_supervisor__msg__SupervisorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uav_mode_supervisor__msg__SupervisorState__Sequence__are_equal(const uav_mode_supervisor__msg__SupervisorState__Sequence * lhs, const uav_mode_supervisor__msg__SupervisorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uav_mode_supervisor__msg__SupervisorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uav_mode_supervisor__msg__SupervisorState__Sequence__copy(
  const uav_mode_supervisor__msg__SupervisorState__Sequence * input,
  uav_mode_supervisor__msg__SupervisorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uav_mode_supervisor__msg__SupervisorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uav_mode_supervisor__msg__SupervisorState * data =
      (uav_mode_supervisor__msg__SupervisorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uav_mode_supervisor__msg__SupervisorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uav_mode_supervisor__msg__SupervisorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uav_mode_supervisor__msg__SupervisorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
