// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mycar_interfaces:msg/SystemStatus.idl
// generated code does not contain a copyright notice

#ifndef MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__BUILDER_HPP_
#define MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mycar_interfaces/msg/detail/system_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mycar_interfaces
{

namespace msg
{

namespace builder
{

class Init_SystemStatus_detail
{
public:
  explicit Init_SystemStatus_detail(::mycar_interfaces::msg::SystemStatus & msg)
  : msg_(msg)
  {}
  ::mycar_interfaces::msg::SystemStatus detail(::mycar_interfaces::msg::SystemStatus::_detail_type arg)
  {
    msg_.detail = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycar_interfaces::msg::SystemStatus msg_;
};

class Init_SystemStatus_waypoint_index
{
public:
  explicit Init_SystemStatus_waypoint_index(::mycar_interfaces::msg::SystemStatus & msg)
  : msg_(msg)
  {}
  Init_SystemStatus_detail waypoint_index(::mycar_interfaces::msg::SystemStatus::_waypoint_index_type arg)
  {
    msg_.waypoint_index = std::move(arg);
    return Init_SystemStatus_detail(msg_);
  }

private:
  ::mycar_interfaces::msg::SystemStatus msg_;
};

class Init_SystemStatus_confidence
{
public:
  explicit Init_SystemStatus_confidence(::mycar_interfaces::msg::SystemStatus & msg)
  : msg_(msg)
  {}
  Init_SystemStatus_waypoint_index confidence(::mycar_interfaces::msg::SystemStatus::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_SystemStatus_waypoint_index(msg_);
  }

private:
  ::mycar_interfaces::msg::SystemStatus msg_;
};

class Init_SystemStatus_target_pose
{
public:
  explicit Init_SystemStatus_target_pose(::mycar_interfaces::msg::SystemStatus & msg)
  : msg_(msg)
  {}
  Init_SystemStatus_confidence target_pose(::mycar_interfaces::msg::SystemStatus::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return Init_SystemStatus_confidence(msg_);
  }

private:
  ::mycar_interfaces::msg::SystemStatus msg_;
};

class Init_SystemStatus_target_visible
{
public:
  explicit Init_SystemStatus_target_visible(::mycar_interfaces::msg::SystemStatus & msg)
  : msg_(msg)
  {}
  Init_SystemStatus_target_pose target_visible(::mycar_interfaces::msg::SystemStatus::_target_visible_type arg)
  {
    msg_.target_visible = std::move(arg);
    return Init_SystemStatus_target_pose(msg_);
  }

private:
  ::mycar_interfaces::msg::SystemStatus msg_;
};

class Init_SystemStatus_mode
{
public:
  explicit Init_SystemStatus_mode(::mycar_interfaces::msg::SystemStatus & msg)
  : msg_(msg)
  {}
  Init_SystemStatus_target_visible mode(::mycar_interfaces::msg::SystemStatus::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_SystemStatus_target_visible(msg_);
  }

private:
  ::mycar_interfaces::msg::SystemStatus msg_;
};

class Init_SystemStatus_header
{
public:
  Init_SystemStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SystemStatus_mode header(::mycar_interfaces::msg::SystemStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SystemStatus_mode(msg_);
  }

private:
  ::mycar_interfaces::msg::SystemStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycar_interfaces::msg::SystemStatus>()
{
  return mycar_interfaces::msg::builder::Init_SystemStatus_header();
}

}  // namespace mycar_interfaces

#endif  // MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__BUILDER_HPP_
