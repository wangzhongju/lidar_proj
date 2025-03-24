// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/PoseMap.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__POSE_MAP__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__POSE_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'status_poses'
#include "perception_ros2_msg/msg/axis_status_pose__struct.h"

// Struct defined in msg/PoseMap in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__PoseMap
{
  perception_ros2_msg__msg__AxisStatusPose__Sequence status_poses;
} perception_ros2_msg__msg__PoseMap;

// Struct for a sequence of perception_ros2_msg__msg__PoseMap.
typedef struct perception_ros2_msg__msg__PoseMap__Sequence
{
  perception_ros2_msg__msg__PoseMap * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__PoseMap__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__POSE_MAP__STRUCT_H_
