// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/Point4f.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__POINT4F__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__POINT4F__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'x'
// Member 'y'
// Member 'z'
#include "std_msgs/msg/float32__struct.h"
// Member 'i'
#include "std_msgs/msg/u_int16__struct.h"

// Struct defined in msg/Point4f in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__Point4f
{
  std_msgs__msg__Float32 x;
  std_msgs__msg__Float32 y;
  std_msgs__msg__Float32 z;
  std_msgs__msg__UInt16 i;
} perception_ros2_msg__msg__Point4f;

// Struct for a sequence of perception_ros2_msg__msg__Point4f.
typedef struct perception_ros2_msg__msg__Point4f__Sequence
{
  perception_ros2_msg__msg__Point4f * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__Point4f__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__POINT4F__STRUCT_H_
