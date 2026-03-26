// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from uav_mode_supervisor:msg/SupervisorState.idl
// generated code does not contain a copyright notice

#ifndef UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "uav_mode_supervisor/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "uav_mode_supervisor/msg/detail/supervisor_state__struct.hpp"

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

#include "fastcdr/Cdr.h"

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
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  uav_mode_supervisor::msg::SupervisorState & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
get_serialized_size(
  const uav_mode_supervisor::msg::SupervisorState & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
max_serialized_size_SupervisorState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace uav_mode_supervisor

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uav_mode_supervisor, msg, SupervisorState)();

#ifdef __cplusplus
}
#endif

#endif  // UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
