// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/Lane.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__LANE__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__LANE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'lane_id'
// Member 'measure_status'
#include "std_msgs/msg/int32__struct.h"
// Member 'curve'
#include "perception_ros2_msg/msg/curve__struct.h"
// Member 'end_points'
#include "perception_ros2_msg/msg/end_points__struct.h"
// Member 'confidence'
#include "std_msgs/msg/float32__struct.h"

// Struct defined in msg/Lane in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__Lane
{
  std_msgs__msg__Int32 lane_id;
  perception_ros2_msg__msg__Curve curve;
  perception_ros2_msg__msg__EndPoints end_points;
  std_msgs__msg__Int32 measure_status;
  std_msgs__msg__Float32 confidence;
} perception_ros2_msg__msg__Lane;

// Struct for a sequence of perception_ros2_msg__msg__Lane.
typedef struct perception_ros2_msg__msg__Lane__Sequence
{
  perception_ros2_msg__msg__Lane * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__Lane__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__LANE__STRUCT_H_
