// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__OBJECT__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'coreinfo'
#include "perception_ros2_msg/msg/core_info__struct.h"
// Member 'hassupplmentinfo'
#include "std_msgs/msg/bool__struct.h"
// Member 'supplementinfo'
#include "perception_ros2_msg/msg/supplement_info__struct.h"

// Struct defined in msg/Object in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__Object
{
  perception_ros2_msg__msg__CoreInfo coreinfo;
  std_msgs__msg__Bool hassupplmentinfo;
  perception_ros2_msg__msg__SupplementInfo supplementinfo;
} perception_ros2_msg__msg__Object;

// Struct for a sequence of perception_ros2_msg__msg__Object.
typedef struct perception_ros2_msg__msg__Object__Sequence
{
  perception_ros2_msg__msg__Object * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__Object__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__OBJECT__STRUCT_H_
