// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/FreeSpaceInfos.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__FREE_SPACE_INFOS__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__FREE_SPACE_INFOS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'fs_pts'
#include "perception_ros2_msg/msg/point3f__struct.h"
// Member 'fs_confidence'
#include "std_msgs/msg/float32__struct.h"

// Struct defined in msg/FreeSpaceInfos in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__FreeSpaceInfos
{
  perception_ros2_msg__msg__Point3f__Sequence fs_pts;
  std_msgs__msg__Float32__Sequence fs_confidence;
} perception_ros2_msg__msg__FreeSpaceInfos;

// Struct for a sequence of perception_ros2_msg__msg__FreeSpaceInfos.
typedef struct perception_ros2_msg__msg__FreeSpaceInfos__Sequence
{
  perception_ros2_msg__msg__FreeSpaceInfos * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__FreeSpaceInfos__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__FREE_SPACE_INFOS__STRUCT_H_
