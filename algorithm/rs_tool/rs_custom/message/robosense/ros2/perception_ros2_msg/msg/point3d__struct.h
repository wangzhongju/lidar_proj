// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/Point3d.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__POINT3D__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__POINT3D__STRUCT_H_

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
#include "std_msgs/msg/float64__struct.h"

// Struct defined in msg/Point3d in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__Point3d
{
  std_msgs__msg__Float64 x;
  std_msgs__msg__Float64 y;
  std_msgs__msg__Float64 z;
} perception_ros2_msg__msg__Point3d;

// Struct for a sequence of perception_ros2_msg__msg__Point3d.
typedef struct perception_ros2_msg__msg__Point3d__Sequence
{
  perception_ros2_msg__msg__Point3d * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__Point3d__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__POINT3D__STRUCT_H_
