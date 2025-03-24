// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/Indices.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__INDICES__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__INDICES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'indices'
#include "std_msgs/msg/int32__struct.h"

// Struct defined in msg/Indices in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__Indices
{
  std_msgs__msg__Int32__Sequence indices;
} perception_ros2_msg__msg__Indices;

// Struct for a sequence of perception_ros2_msg__msg__Indices.
typedef struct perception_ros2_msg__msg__Indices__Sequence
{
  perception_ros2_msg__msg__Indices * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__Indices__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__INDICES__STRUCT_H_
