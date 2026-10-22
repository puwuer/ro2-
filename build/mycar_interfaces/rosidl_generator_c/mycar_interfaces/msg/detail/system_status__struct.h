// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mycar_interfaces:msg/SystemStatus.idl
// generated code does not contain a copyright notice

#ifndef MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__STRUCT_H_
#define MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'PATROL'.
enum
{
  mycar_interfaces__msg__SystemStatus__PATROL = 0
};

/// Constant 'TRACK'.
enum
{
  mycar_interfaces__msg__SystemStatus__TRACK = 1
};

/// Constant 'SEARCH'.
enum
{
  mycar_interfaces__msg__SystemStatus__SEARCH = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'detail'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/SystemStatus in the package mycar_interfaces.
/**
  * 系统工作模式常量
 */
typedef struct mycar_interfaces__msg__SystemStatus
{
  std_msgs__msg__Header header;
  /// 当前工作模式: PATROL / TRACK / SEARCH
  uint8_t mode;
  /// 是否检测到跟踪目标(人)
  bool target_visible;
  /// 目标在 map 坐标系下的位置, target_visible=false 时忽略
  geometry_msgs__msg__PoseStamped target_pose;
  /// 检测置信度 0~1
  float confidence;
  /// 当前(或最近)巡检点索引, 非巡检模式时保留最后一次的值
  int32_t waypoint_index;
  /// 最近一次状态切换原因/说明, 仅用于调试显示
  rosidl_runtime_c__String detail;
} mycar_interfaces__msg__SystemStatus;

// Struct for a sequence of mycar_interfaces__msg__SystemStatus.
typedef struct mycar_interfaces__msg__SystemStatus__Sequence
{
  mycar_interfaces__msg__SystemStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycar_interfaces__msg__SystemStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__STRUCT_H_
