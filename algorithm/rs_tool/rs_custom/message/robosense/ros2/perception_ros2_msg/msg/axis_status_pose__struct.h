// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/AxisStatusPose.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__AXIS_STATUS_POSE__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__AXIS_STATUS_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "std_msgs/msg/int32__struct.h"
// Member 'pose'
#include "perception_ros2_msg/msg/pose__struct.h"

// Struct defined in msg/AxisStatusPose in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__AxisStatusPose
{
  std_msgs__msg__Int32 status;
  perception_ros2_msg__msg__Pose pose;
} perception_ros2_msg__msg__AxisStatusPose;

// Struct for a sequence of perception_ros2_msg__msg__AxisStatusPose.
typedef struct perception_ros2_msg__msg__AxisStatusPose__Sequence
{
  perception_ros2_msg__msg__AxisStatusPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__AxisStatusPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__AXIS_STATUS_POSE__STRUCT_H_
