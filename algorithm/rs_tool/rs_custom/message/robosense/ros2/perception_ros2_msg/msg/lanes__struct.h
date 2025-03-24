// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/Lanes.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__LANES__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__LANES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'lanes'
#include "perception_ros2_msg/msg/lane__struct.h"

// Struct defined in msg/Lanes in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__Lanes
{
  perception_ros2_msg__msg__Lane__Sequence lanes;
} perception_ros2_msg__msg__Lanes;

// Struct for a sequence of perception_ros2_msg__msg__Lanes.
typedef struct perception_ros2_msg__msg__Lanes__Sequence
{
  perception_ros2_msg__msg__Lanes * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__Lanes__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__LANES__STRUCT_H_
