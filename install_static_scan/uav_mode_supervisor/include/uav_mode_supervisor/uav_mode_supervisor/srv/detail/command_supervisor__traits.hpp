// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uav_mode_supervisor:srv/CommandSupervisor.idl
// generated code does not contain a copyright notice

#ifndef UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__TRAITS_HPP_
#define UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uav_mode_supervisor/srv/detail/command_supervisor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace uav_mode_supervisor
{

namespace srv
{

inline void to_flow_style_yaml(
  const CommandSupervisor_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: tracking_target_height_m
  {
    out << "tracking_target_height_m: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_target_height_m, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CommandSupervisor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: tracking_target_height_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracking_target_height_m: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_target_height_m, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CommandSupervisor_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace uav_mode_supervisor

namespace rosidl_generator_traits
{

[[deprecated("use uav_mode_supervisor::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uav_mode_supervisor::srv::CommandSupervisor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  uav_mode_supervisor::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uav_mode_supervisor::srv::to_yaml() instead")]]
inline std::string to_yaml(const uav_mode_supervisor::srv::CommandSupervisor_Request & msg)
{
  return uav_mode_supervisor::srv::to_yaml(msg);
}

template<>
inline const char * data_type<uav_mode_supervisor::srv::CommandSupervisor_Request>()
{
  return "uav_mode_supervisor::srv::CommandSupervisor_Request";
}

template<>
inline const char * name<uav_mode_supervisor::srv::CommandSupervisor_Request>()
{
  return "uav_mode_supervisor/srv/CommandSupervisor_Request";
}

template<>
struct has_fixed_size<uav_mode_supervisor::srv::CommandSupervisor_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uav_mode_supervisor::srv::CommandSupervisor_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<uav_mode_supervisor::srv::CommandSupervisor_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace uav_mode_supervisor
{

namespace srv
{

inline void to_flow_style_yaml(
  const CommandSupervisor_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CommandSupervisor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CommandSupervisor_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace uav_mode_supervisor

namespace rosidl_generator_traits
{

[[deprecated("use uav_mode_supervisor::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uav_mode_supervisor::srv::CommandSupervisor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  uav_mode_supervisor::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uav_mode_supervisor::srv::to_yaml() instead")]]
inline std::string to_yaml(const uav_mode_supervisor::srv::CommandSupervisor_Response & msg)
{
  return uav_mode_supervisor::srv::to_yaml(msg);
}

template<>
inline const char * data_type<uav_mode_supervisor::srv::CommandSupervisor_Response>()
{
  return "uav_mode_supervisor::srv::CommandSupervisor_Response";
}

template<>
inline const char * name<uav_mode_supervisor::srv::CommandSupervisor_Response>()
{
  return "uav_mode_supervisor/srv/CommandSupervisor_Response";
}

template<>
struct has_fixed_size<uav_mode_supervisor::srv::CommandSupervisor_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uav_mode_supervisor::srv::CommandSupervisor_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<uav_mode_supervisor::srv::CommandSupervisor_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<uav_mode_supervisor::srv::CommandSupervisor>()
{
  return "uav_mode_supervisor::srv::CommandSupervisor";
}

template<>
inline const char * name<uav_mode_supervisor::srv::CommandSupervisor>()
{
  return "uav_mode_supervisor/srv/CommandSupervisor";
}

template<>
struct has_fixed_size<uav_mode_supervisor::srv::CommandSupervisor>
  : std::integral_constant<
    bool,
    has_fixed_size<uav_mode_supervisor::srv::CommandSupervisor_Request>::value &&
    has_fixed_size<uav_mode_supervisor::srv::CommandSupervisor_Response>::value
  >
{
};

template<>
struct has_bounded_size<uav_mode_supervisor::srv::CommandSupervisor>
  : std::integral_constant<
    bool,
    has_bounded_size<uav_mode_supervisor::srv::CommandSupervisor_Request>::value &&
    has_bounded_size<uav_mode_supervisor::srv::CommandSupervisor_Response>::value
  >
{
};

template<>
struct is_service<uav_mode_supervisor::srv::CommandSupervisor>
  : std::true_type
{
};

template<>
struct is_service_request<uav_mode_supervisor::srv::CommandSupervisor_Request>
  : std::true_type
{
};

template<>
struct is_service_response<uav_mode_supervisor::srv::CommandSupervisor_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__TRAITS_HPP_
