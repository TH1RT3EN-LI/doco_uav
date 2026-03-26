// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from uav_mode_supervisor:srv/CommandSupervisor.idl
// generated code does not contain a copyright notice

#ifndef UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "uav_mode_supervisor/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "uav_mode_supervisor/srv/detail/command_supervisor__struct.hpp"

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

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
cdr_serialize(
  const uav_mode_supervisor::srv::CommandSupervisor_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  uav_mode_supervisor::srv::CommandSupervisor_Request & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
get_serialized_size(
  const uav_mode_supervisor::srv::CommandSupervisor_Request & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
max_serialized_size_CommandSupervisor_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace uav_mode_supervisor

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uav_mode_supervisor, srv, CommandSupervisor_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "uav_mode_supervisor/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
// already included above
// #include "uav_mode_supervisor/srv/detail/command_supervisor__struct.hpp"

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

// already included above
// #include "fastcdr/Cdr.h"

namespace uav_mode_supervisor
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
cdr_serialize(
  const uav_mode_supervisor::srv::CommandSupervisor_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  uav_mode_supervisor::srv::CommandSupervisor_Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
get_serialized_size(
  const uav_mode_supervisor::srv::CommandSupervisor_Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
max_serialized_size_CommandSupervisor_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace uav_mode_supervisor

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uav_mode_supervisor, srv, CommandSupervisor_Response)();

#ifdef __cplusplus
}
#endif

#include "rmw/types.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "uav_mode_supervisor/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_mode_supervisor
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uav_mode_supervisor, srv, CommandSupervisor)();

#ifdef __cplusplus
}
#endif

#endif  // UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
