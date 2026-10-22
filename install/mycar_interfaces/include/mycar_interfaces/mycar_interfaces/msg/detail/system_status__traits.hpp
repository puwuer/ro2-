// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mycar_interfaces:msg/SystemStatus.idl
// generated code does not contain a copyright notice

#ifndef MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__TRAITS_HPP_
#define MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mycar_interfaces/msg/detail/system_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace mycar_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SystemStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: target_visible
  {
    out << "target_visible: ";
    rosidl_generator_traits::value_to_yaml(msg.target_visible, out);
    out << ", ";
  }

  // member: target_pose
  {
    out << "target_pose: ";
    to_flow_style_yaml(msg.target_pose, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: waypoint_index
  {
    out << "waypoint_index: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoint_index, out);
    out << ", ";
  }

  // member: detail
  {
    out << "detail: ";
    rosidl_generator_traits::value_to_yaml(msg.detail, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SystemStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: target_visible
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_visible: ";
    rosidl_generator_traits::value_to_yaml(msg.target_visible, out);
    out << "\n";
  }

  // member: target_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_pose:\n";
    to_block_style_yaml(msg.target_pose, out, indentation + 2);
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: waypoint_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "waypoint_index: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoint_index, out);
    out << "\n";
  }

  // member: detail
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detail: ";
    rosidl_generator_traits::value_to_yaml(msg.detail, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SystemStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mycar_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mycar_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mycar_interfaces::msg::SystemStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mycar_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mycar_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const mycar_interfaces::msg::SystemStatus & msg)
{
  return mycar_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mycar_interfaces::msg::SystemStatus>()
{
  return "mycar_interfaces::msg::SystemStatus";
}

template<>
inline const char * name<mycar_interfaces::msg::SystemStatus>()
{
  return "mycar_interfaces/msg/SystemStatus";
}

template<>
struct has_fixed_size<mycar_interfaces::msg::SystemStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mycar_interfaces::msg::SystemStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mycar_interfaces::msg::SystemStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__TRAITS_HPP_
