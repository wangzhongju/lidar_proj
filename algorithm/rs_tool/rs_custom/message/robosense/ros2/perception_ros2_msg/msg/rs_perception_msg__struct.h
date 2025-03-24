// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/RsPerceptionMsg.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__RS_PERCEPTION_MSG__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__RS_PERCEPTION_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'lidarframe'
#include "perception_ros2_msg/msg/lidar_frame_msg__struct.h"
// Member 'device_id'
#include "std_msgs/msg/int32__struct.h"

// Struct defined in msg/RsPerceptionMsg in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__RsPerceptionMsg
{
  perception_ros2_msg__msg__LidarFrameMsg lidarframe;
  std_msgs__msg__Int32 device_id;
} perception_ros2_msg__msg__RsPerceptionMsg;

// Struct for a sequence of perception_ros2_msg__msg__RsPerceptionMsg.
typedef struct perception_ros2_msg__msg__RsPerceptionMsg__Sequence
{
  perception_ros2_msg__msg__RsPerceptionMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__RsPerceptionMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__RS_PERCEPTION_MSG__STRUCT_H_
