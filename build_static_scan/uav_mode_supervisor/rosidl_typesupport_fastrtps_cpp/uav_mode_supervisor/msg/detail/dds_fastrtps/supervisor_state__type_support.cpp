// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from uav_mode_supervisor:msg/SupervisorState.idl
// generated code does not contain a copyright notice
#include "uav_mode_supervisor/msg/detail/supervisor_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "uav_mode_supervisor/msg/detail/supervisor_state__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace uav_mode_supervisor
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
cdr_serialize(
  const uav_mode_supervisor::msg::SupervisorState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: owner
  cdr << ros_message.owner;
  // Member: active_tracking_height_m
  cdr << ros_message.active_tracking_height_m;
  // Member: last_command
  cdr << ros_message.last_command;
  // Member: pending_command
  cdr << ros_message.pending_command;
  // Member: command_in_progress
  cdr << (ros_message.command_in_progress ? true : false);
  // Member: last_message
  cdr << ros_message.last_message;
  // Member: fusion_diagnostics_seen
  cdr << (ros_message.fusion_diagnostics_seen ? true : false);
  // Member: fusion_initialized
  cdr << (ros_message.fusion_initialized ? true : false);
  // Member: fusion_relocalize_requested
  cdr << (ros_message.fusion_relocalize_requested ? true : false);
  // Member: fusion_ready
  cdr << (ros_message.fusion_ready ? true : false);
  // Member: fusion_reason
  cdr << ros_message.fusion_reason;
  // Member: visual_state_seen
  cdr << (ros_message.visual_state_seen ? true : false);
  // Member: visual_active
  cdr << (ros_message.visual_active ? true : false);
  // Member: visual_target_detected
  cdr << (ros_message.visual_target_detected ? true : false);
  // Member: visual_phase
  cdr << ros_message.visual_phase;
  // Member: visual_committed
  cdr << (ros_message.visual_committed ? true : false);
  // Member: visual_capture_observed
  cdr << (ros_message.visual_capture_observed ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  uav_mode_supervisor::msg::SupervisorState & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: owner
  cdr >> ros_message.owner;

  // Member: active_tracking_height_m
  cdr >> ros_message.active_tracking_height_m;

  // Member: last_command
  cdr >> ros_message.last_command;

  // Member: pending_command
  cdr >> ros_message.pending_command;

  // Member: command_in_progress
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.command_in_progress = tmp ? true : false;
  }

  // Member: last_message
  cdr >> ros_message.last_message;

  // Member: fusion_diagnostics_seen
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fusion_diagnostics_seen = tmp ? true : false;
  }

  // Member: fusion_initialized
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fusion_initialized = tmp ? true : false;
  }

  // Member: fusion_relocalize_requested
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fusion_relocalize_requested = tmp ? true : false;
  }

  // Member: fusion_ready
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fusion_ready = tmp ? true : false;
  }

  // Member: fusion_reason
  cdr >> ros_message.fusion_reason;

  // Member: visual_state_seen
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.visual_state_seen = tmp ? true : false;
  }

  // Member: visual_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.visual_active = tmp ? true : false;
  }

  // Member: visual_target_detected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.visual_target_detected = tmp ? true : false;
  }

  // Member: visual_phase
  cdr >> ros_message.visual_phase;

  // Member: visual_committed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.visual_committed = tmp ? true : false;
  }

  // Member: visual_capture_observed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.visual_capture_observed = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
get_serialized_size(
  const uav_mode_supervisor::msg::SupervisorState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: owner
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.owner.size() + 1);
  // Member: active_tracking_height_m
  {
    size_t item_size = sizeof(ros_message.active_tracking_height_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: last_command
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.last_command.size() + 1);
  // Member: pending_command
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.pending_command.size() + 1);
  // Member: command_in_progress
  {
    size_t item_size = sizeof(ros_message.command_in_progress);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: last_message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.last_message.size() + 1);
  // Member: fusion_diagnostics_seen
  {
    size_t item_size = sizeof(ros_message.fusion_diagnostics_seen);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fusion_initialized
  {
    size_t item_size = sizeof(ros_message.fusion_initialized);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fusion_relocalize_requested
  {
    size_t item_size = sizeof(ros_message.fusion_relocalize_requested);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fusion_ready
  {
    size_t item_size = sizeof(ros_message.fusion_ready);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fusion_reason
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.fusion_reason.size() + 1);
  // Member: visual_state_seen
  {
    size_t item_size = sizeof(ros_message.visual_state_seen);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: visual_active
  {
    size_t item_size = sizeof(ros_message.visual_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: visual_target_detected
  {
    size_t item_size = sizeof(ros_message.visual_target_detected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: visual_phase
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.visual_phase.size() + 1);
  // Member: visual_committed
  {
    size_t item_size = sizeof(ros_message.visual_committed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: visual_capture_observed
  {
    size_t item_size = sizeof(ros_message.visual_capture_observed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
max_serialized_size_SupervisorState(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: owner
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

  // Member: active_tracking_height_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: last_command
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

  // Member: pending_command
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

  // Member: command_in_progress
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: last_message
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

  // Member: fusion_diagnostics_seen
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fusion_initialized
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fusion_relocalize_requested
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fusion_ready
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fusion_reason
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

  // Member: visual_state_seen
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: visual_active
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: visual_target_detected
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: visual_phase
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

  // Member: visual_committed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: visual_capture_observed
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
    using DataType = uav_mode_supervisor::msg::SupervisorState;
    is_plain =
      (
      offsetof(DataType, visual_capture_observed) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SupervisorState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const uav_mode_supervisor::msg::SupervisorState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SupervisorState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<uav_mode_supervisor::msg::SupervisorState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SupervisorState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const uav_mode_supervisor::msg::SupervisorState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SupervisorState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SupervisorState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SupervisorState__callbacks = {
  "uav_mode_supervisor::msg",
  "SupervisorState",
  _SupervisorState__cdr_serialize,
  _SupervisorState__cdr_deserialize,
  _SupervisorState__get_serialized_size,
  _SupervisorState__max_serialized_size
};

static rosidl_message_type_support_t _SupervisorState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SupervisorState__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace uav_mode_supervisor

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_uav_mode_supervisor
const rosidl_message_type_support_t *
get_message_type_support_handle<uav_mode_supervisor::msg::SupervisorState>()
{
  return &uav_mode_supervisor::msg::typesupport_fastrtps_cpp::_SupervisorState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uav_mode_supervisor, msg, SupervisorState)() {
  return &uav_mode_supervisor::msg::typesupport_fastrtps_cpp::_SupervisorState__handle;
}

#ifdef __cplusplus
}
#endif
