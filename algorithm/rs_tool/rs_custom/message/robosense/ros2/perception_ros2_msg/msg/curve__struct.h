// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/Curve.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__CURVE__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__CURVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'x_start'
// Member 'x_end'
// Member 'a'
// Member 'b'
// Member 'c'
// Member 'd'
#include "std_msgs/msg/float32__struct.h"

// Struct defined in msg/Curve in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__Curve
{
  std_msgs__msg__Float32 x_start;
  std_msgs__msg__Float32 x_end;
  std_msgs__msg__Float32 a;
  std_msgs__msg__Float32 b;
  std_msgs__msg__Float32 c;
  std_msgs__msg__Float32 d;
} perception_ros2_msg__msg__Curve;

// Struct for a sequence of perception_ros2_msg__msg__Curve.
typedef struct perception_ros2_msg__msg__Curve__Sequence
{
  perception_ros2_msg__msg__Curve * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__Curve__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__CURVE__STRUCT_H_
