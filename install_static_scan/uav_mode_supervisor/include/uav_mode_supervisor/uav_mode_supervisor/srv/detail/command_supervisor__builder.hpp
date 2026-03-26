// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uav_mode_supervisor:srv/CommandSupervisor.idl
// generated code does not contain a copyright notice

#ifndef UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__BUILDER_HPP_
#define UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uav_mode_supervisor/srv/detail/command_supervisor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uav_mode_supervisor
{

namespace srv
{

namespace builder
{

class Init_CommandSupervisor_Request_tracking_target_height_m
{
public:
  explicit Init_CommandSupervisor_Request_tracking_target_height_m(::uav_mode_supervisor::srv::CommandSupervisor_Request & msg)
  : msg_(msg)
  {}
  ::uav_mode_supervisor::srv::CommandSupervisor_Request tracking_target_height_m(::uav_mode_supervisor::srv::CommandSupervisor_Request::_tracking_target_height_m_type arg)
  {
    msg_.tracking_target_height_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uav_mode_supervisor::srv::CommandSupervisor_Request msg_;
};

class Init_CommandSupervisor_Request_command
{
public:
  Init_CommandSupervisor_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandSupervisor_Request_tracking_target_height_m command(::uav_mode_supervisor::srv::CommandSupervisor_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_CommandSupervisor_Request_tracking_target_height_m(msg_);
  }

private:
  ::uav_mode_supervisor::srv::CommandSupervisor_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::uav_mode_supervisor::srv::CommandSupervisor_Request>()
{
  return uav_mode_supervisor::srv::builder::Init_CommandSupervisor_Request_command();
}

}  // namespace uav_mode_supervisor


namespace uav_mode_supervisor
{

namespace srv
{

namespace builder
{

class Init_CommandSupervisor_Response_message
{
public:
  explicit Init_CommandSupervisor_Response_message(::uav_mode_supervisor::srv::CommandSupervisor_Response & msg)
  : msg_(msg)
  {}
  ::uav_mode_supervisor::srv::CommandSupervisor_Response message(::uav_mode_supervisor::srv::CommandSupervisor_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uav_mode_supervisor::srv::CommandSupervisor_Response msg_;
};

class Init_CommandSupervisor_Response_success
{
public:
  Init_CommandSupervisor_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandSupervisor_Response_message success(::uav_mode_supervisor::srv::CommandSupervisor_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CommandSupervisor_Response_message(msg_);
  }

private:
  ::uav_mode_supervisor::srv::CommandSupervisor_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::uav_mode_supervisor::srv::CommandSupervisor_Response>()
{
  return uav_mode_supervisor::srv::builder::Init_CommandSupervisor_Response_success();
}

}  // namespace uav_mode_supervisor

#endif  // UAV_MODE_SUPERVISOR__SRV__DETAIL__COMMAND_SUPERVISOR__BUILDER_HPP_
