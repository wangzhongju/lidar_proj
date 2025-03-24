// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__POSE__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__POSE__STRUCT_H_

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
// Member 'roll'
// Member 'pitch'
// Member 'yaw'
#include "std_msgs/msg/float32__struct.h"
// Member 'status'
#include "std_msgs/msg/int32__struct.h"

// Struct defined in msg/Pose in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__Pose
{
  std_msgs__msg__Float32 x;
  std_msgs__msg__Float32 y;
  std_msgs__msg__Float32 z;
  std_msgs__msg__Float32 roll;
  std_msgs__msg__Float32 pitch;
  std_msgs__msg__Float32 yaw;
  std_msgs__msg__Int32 status;
} perception_ros2_msg__msg__Pose;

// Struct for a sequence of perception_ros2_msg__msg__Pose.
typedef struct perception_ros2_msg__msg__Pose__Sequence
{
  perception_ros2_msg__msg__Pose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__Pose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__POSE__STRUCT_H_
