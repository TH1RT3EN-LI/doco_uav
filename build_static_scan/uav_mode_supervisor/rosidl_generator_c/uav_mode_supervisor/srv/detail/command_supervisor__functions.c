// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uav_mode_supervisor:srv/CommandSupervisor.idl
// generated code does not contain a copyright notice
#include "uav_mode_supervisor/srv/detail/command_supervisor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `command`
#include "rosidl_runtime_c/string_functions.h"

bool
uav_mode_supervisor__srv__CommandSupervisor_Request__init(uav_mode_supervisor__srv__CommandSupervisor_Request * msg)
{
  if (!msg) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__init(&msg->command)) {
    uav_mode_supervisor__srv__CommandSupervisor_Request__fini(msg);
    return false;
  }
  // tracking_target_height_m
  return true;
}

void
uav_mode_supervisor__srv__CommandSupervisor_Request__fini(uav_mode_supervisor__srv__CommandSupervisor_Request * msg)
{
  if (!msg) {
    return;
  }
  // command
  rosidl_runtime_c__String__fini(&msg->command);
  // tracking_target_height_m
}

bool
uav_mode_supervisor__srv__CommandSupervisor_Request__are_equal(const uav_mode_supervisor__srv__CommandSupervisor_Request * lhs, const uav_mode_supervisor__srv__CommandSupervisor_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command), &(rhs->command)))
  {
    return false;
  }
  // tracking_target_height_m
  if (lhs->tracking_target_height_m != rhs->tracking_target_height_m) {
    return false;
  }
  return true;
}

bool
uav_mode_supervisor__srv__CommandSupervisor_Request__copy(
  const uav_mode_supervisor__srv__CommandSupervisor_Request * input,
  uav_mode_supervisor__srv__CommandSupervisor_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__copy(
      &(input->command), &(output->command)))
  {
    return false;
  }
  // tracking_target_height_m
  output->tracking_target_height_m = input->tracking_target_height_m;
  return true;
}

uav_mode_supervisor__srv__CommandSupervisor_Request *
uav_mode_supervisor__srv__CommandSupervisor_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_mode_supervisor__srv__CommandSupervisor_Request * msg = (uav_mode_supervisor__srv__CommandSupervisor_Request *)allocator.allocate(sizeof(uav_mode_supervisor__srv__CommandSupervisor_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uav_mode_supervisor__srv__CommandSupervisor_Request));
  bool success = uav_mode_supervisor__srv__CommandSupervisor_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uav_mode_supervisor__srv__CommandSupervisor_Request__destroy(uav_mode_supervisor__srv__CommandSupervisor_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uav_mode_supervisor__srv__CommandSupervisor_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence__init(uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_mode_supervisor__srv__CommandSupervisor_Request * data = NULL;

  if (size) {
    data = (uav_mode_supervisor__srv__CommandSupervisor_Request *)allocator.zero_allocate(size, sizeof(uav_mode_supervisor__srv__CommandSupervisor_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uav_mode_supervisor__srv__CommandSupervisor_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uav_mode_supervisor__srv__CommandSupervisor_Request__fini(&data[i - 1]);
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
uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence__fini(uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence * array)
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
      uav_mode_supervisor__srv__CommandSupervisor_Request__fini(&array->data[i]);
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

uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence *
uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence * array = (uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence *)allocator.allocate(sizeof(uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence__destroy(uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence__are_equal(const uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence * lhs, const uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uav_mode_supervisor__srv__CommandSupervisor_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence__copy(
  const uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence * input,
  uav_mode_supervisor__srv__CommandSupervisor_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uav_mode_supervisor__srv__CommandSupervisor_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uav_mode_supervisor__srv__CommandSupervisor_Request * data =
      (uav_mode_supervisor__srv__CommandSupervisor_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uav_mode_supervisor__srv__CommandSupervisor_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uav_mode_supervisor__srv__CommandSupervisor_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uav_mode_supervisor__srv__CommandSupervisor_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
uav_mode_supervisor__srv__CommandSupervisor_Response__init(uav_mode_supervisor__srv__CommandSupervisor_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    uav_mode_supervisor__srv__CommandSupervisor_Response__fini(msg);
    return false;
  }
  return true;
}

void
uav_mode_supervisor__srv__CommandSupervisor_Response__fini(uav_mode_supervisor__srv__CommandSupervisor_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
uav_mode_supervisor__srv__CommandSupervisor_Response__are_equal(const uav_mode_supervisor__srv__CommandSupervisor_Response * lhs, const uav_mode_supervisor__srv__CommandSupervisor_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
uav_mode_supervisor__srv__CommandSupervisor_Response__copy(
  const uav_mode_supervisor__srv__CommandSupervisor_Response * input,
  uav_mode_supervisor__srv__CommandSupervisor_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

uav_mode_supervisor__srv__CommandSupervisor_Response *
uav_mode_supervisor__srv__CommandSupervisor_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_mode_supervisor__srv__CommandSupervisor_Response * msg = (uav_mode_supervisor__srv__CommandSupervisor_Response *)allocator.allocate(sizeof(uav_mode_supervisor__srv__CommandSupervisor_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uav_mode_supervisor__srv__CommandSupervisor_Response));
  bool success = uav_mode_supervisor__srv__CommandSupervisor_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uav_mode_supervisor__srv__CommandSupervisor_Response__destroy(uav_mode_supervisor__srv__CommandSupervisor_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uav_mode_supervisor__srv__CommandSupervisor_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence__init(uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_mode_supervisor__srv__CommandSupervisor_Response * data = NULL;

  if (size) {
    data = (uav_mode_supervisor__srv__CommandSupervisor_Response *)allocator.zero_allocate(size, sizeof(uav_mode_supervisor__srv__CommandSupervisor_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uav_mode_supervisor__srv__CommandSupervisor_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uav_mode_supervisor__srv__CommandSupervisor_Response__fini(&data[i - 1]);
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
uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence__fini(uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence * array)
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
      uav_mode_supervisor__srv__CommandSupervisor_Response__fini(&array->data[i]);
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

uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence *
uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence * array = (uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence *)allocator.allocate(sizeof(uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence__destroy(uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence__are_equal(const uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence * lhs, const uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uav_mode_supervisor__srv__CommandSupervisor_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence__copy(
  const uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence * input,
  uav_mode_supervisor__srv__CommandSupervisor_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uav_mode_supervisor__srv__CommandSupervisor_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uav_mode_supervisor__srv__CommandSupervisor_Response * data =
      (uav_mode_supervisor__srv__CommandSupervisor_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uav_mode_supervisor__srv__CommandSupervisor_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uav_mode_supervisor__srv__CommandSupervisor_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uav_mode_supervisor__srv__CommandSupervisor_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
