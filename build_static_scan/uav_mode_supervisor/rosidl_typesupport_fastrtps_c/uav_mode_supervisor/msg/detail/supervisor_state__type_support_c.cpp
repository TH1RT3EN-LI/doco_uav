// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from uav_mode_supervisor:msg/SupervisorState.idl
// generated code does not contain a copyright notice
#include "uav_mode_supervisor/msg/detail/supervisor_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "uav_mode_supervisor/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "uav_mode_supervisor/msg/detail/supervisor_state__struct.h"
#include "uav_mode_supervisor/msg/detail/supervisor_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // fusion_reason, last_command, last_message, owner, pending_command, visual_phase
#include "rosidl_runtime_c/string_functions.h"  // fusion_reason, last_command, last_message, owner, pending_command, visual_phase
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_uav_mode_supervisor
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_uav_mode_supervisor
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_uav_mode_supervisor
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _SupervisorState__ros_msg_type = uav_mode_supervisor__msg__SupervisorState;

static bool _SupervisorState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SupervisorState__ros_msg_type * ros_message = static_cast<const _SupervisorState__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: owner
  {
    const rosidl_runtime_c__String * str = &ros_message->owner;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: active_tracking_height_m
  {
    cdr << ros_message->active_tracking_height_m;
  }

  // Field name: last_command
  {
    const rosidl_runtime_c__String * str = &ros_message->last_command;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: pending_command
  {
    const rosidl_runtime_c__String * str = &ros_message->pending_command;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: command_in_progress
  {
    cdr << (ros_message->command_in_progress ? true : false);
  }

  // Field name: last_message
  {
    const rosidl_runtime_c__String * str = &ros_message->last_message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: fusion_diagnostics_seen
  {
    cdr << (ros_message->fusion_diagnostics_seen ? true : false);
  }

  // Field name: fusion_initialized
  {
    cdr << (ros_message->fusion_initialized ? true : false);
  }

  // Field name: fusion_relocalize_requested
  {
    cdr << (ros_message->fusion_relocalize_requested ? true : false);
  }

  // Field name: fusion_ready
  {
    cdr << (ros_message->fusion_ready ? true : false);
  }

  // Field name: fusion_reason
  {
    const rosidl_runtime_c__String * str = &ros_message->fusion_reason;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: visual_state_seen
  {
    cdr << (ros_message->visual_state_seen ? true : false);
  }

  // Field name: visual_active
  {
    cdr << (ros_message->visual_active ? true : false);
  }

  // Field name: visual_target_detected
  {
    cdr << (ros_message->visual_target_detected ? true : false);
  }

  // Field name: visual_phase
  {
    const rosidl_runtime_c__String * str = &ros_message->visual_phase;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: visual_committed
  {
    cdr << (ros_message->visual_committed ? true : false);
  }

  // Field name: visual_capture_observed
  {
    cdr << (ros_message->visual_capture_observed ? true : false);
  }

  return true;
}

static bool _SupervisorState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SupervisorState__ros_msg_type * ros_message = static_cast<_SupervisorState__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: owner
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->owner.data) {
      rosidl_runtime_c__String__init(&ros_message->owner);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->owner,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'owner'\n");
      return false;
    }
  }

  // Field name: active_tracking_height_m
  {
    cdr >> ros_message->active_tracking_height_m;
  }

  // Field name: last_command
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->last_command.data) {
      rosidl_runtime_c__String__init(&ros_message->last_command);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->last_command,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'last_command'\n");
      return false;
    }
  }

  // Field name: pending_command
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->pending_command.data) {
      rosidl_runtime_c__String__init(&ros_message->pending_command);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->pending_command,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'pending_command'\n");
      return false;
    }
  }

  // Field name: command_in_progress
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->command_in_progress = tmp ? true : false;
  }

  // Field name: last_message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->last_message.data) {
      rosidl_runtime_c__String__init(&ros_message->last_message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->last_message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'last_message'\n");
      return false;
    }
  }

  // Field name: fusion_diagnostics_seen
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->fusion_diagnostics_seen = tmp ? true : false;
  }

  // Field name: fusion_initialized
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->fusion_initialized = tmp ? true : false;
  }

  // Field name: fusion_relocalize_requested
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->fusion_relocalize_requested = tmp ? true : false;
  }

  // Field name: fusion_ready
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->fusion_ready = tmp ? true : false;
  }

  // Field name: fusion_reason
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->fusion_reason.data) {
      rosidl_runtime_c__String__init(&ros_message->fusion_reason);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->fusion_reason,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'fusion_reason'\n");
      return false;
    }
  }

  // Field name: visual_state_seen
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->visual_state_seen = tmp ? true : false;
  }

  // Field name: visual_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->visual_active = tmp ? true : false;
  }

  // Field name: visual_target_detected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->visual_target_detected = tmp ? true : false;
  }

  // Field name: visual_phase
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->visual_phase.data) {
      rosidl_runtime_c__String__init(&ros_message->visual_phase);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->visual_phase,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'visual_phase'\n");
      return false;
    }
  }

  // Field name: visual_committed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->visual_committed = tmp ? true : false;
  }

  // Field name: visual_capture_observed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->visual_capture_observed = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uav_mode_supervisor
size_t get_serialized_size_uav_mode_supervisor__msg__SupervisorState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SupervisorState__ros_msg_type * ros_message = static_cast<const _SupervisorState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name owner
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->owner.size + 1);
  // field.name active_tracking_height_m
  {
    size_t item_size = sizeof(ros_message->active_tracking_height_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name last_command
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->last_command.size + 1);
  // field.name pending_command
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->pending_command.size + 1);
  // field.name command_in_progress
  {
    size_t item_size = sizeof(ros_message->command_in_progress);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name last_message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->last_message.size + 1);
  // field.name fusion_diagnostics_seen
  {
    size_t item_size = sizeof(ros_message->fusion_diagnostics_seen);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fusion_initialized
  {
    size_t item_size = sizeof(ros_message->fusion_initialized);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fusion_relocalize_requested
  {
    size_t item_size = sizeof(ros_message->fusion_relocalize_requested);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fusion_ready
  {
    size_t item_size = sizeof(ros_message->fusion_ready);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fusion_reason
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->fusion_reason.size + 1);
  // field.name visual_state_seen
  {
    size_t item_size = sizeof(ros_message->visual_state_seen);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name visual_active
  {
    size_t item_size = sizeof(ros_message->visual_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name visual_target_detected
  {
    size_t item_size = sizeof(ros_message->visual_target_detected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name visual_phase
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->visual_phase.size + 1);
  // field.name visual_committed
  {
    size_t item_size = sizeof(ros_message->visual_committed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name visual_capture_observed
  {
    size_t item_size = sizeof(ros_message->visual_capture_observed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SupervisorState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_uav_mode_supervisor__msg__SupervisorState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uav_mode_supervisor
size_t max_serialized_size_uav_mode_supervisor__msg__SupervisorState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: owner
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: active_tracking_height_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: last_command
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: pending_command
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: command_in_progress
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: last_message
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: fusion_diagnostics_seen
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fusion_initialized
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fusion_relocalize_requested
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fusion_ready
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fusion_reason
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: visual_state_seen
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: visual_active
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: visual_target_detected
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: visual_phase
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: visual_committed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: visual_capture_observed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = uav_mode_supervisor__msg__SupervisorState;
    is_plain =
      (
      offsetof(DataType, visual_capture_observed) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SupervisorState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_uav_mode_supervisor__msg__SupervisorState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SupervisorState = {
  "uav_mode_supervisor::msg",
  "SupervisorState",
  _SupervisorState__cdr_serialize,
  _SupervisorState__cdr_deserialize,
  _SupervisorState__get_serialized_size,
  _SupervisorState__max_serialized_size
};

static rosidl_message_type_support_t _SupervisorState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SupervisorState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, uav_mode_supervisor, msg, SupervisorState)() {
  return &_SupervisorState__type_support;
}

#if defined(__cplusplus)
}
#endif
