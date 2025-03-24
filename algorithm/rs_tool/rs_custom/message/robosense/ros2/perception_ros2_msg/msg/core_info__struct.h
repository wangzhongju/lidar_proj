// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/CoreInfo.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__CORE_INFO__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__CORE_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'timestamp'
// Member 'exist_confidence'
// Member 'type_confidence'
// Member 'angle_velocity'
// Member 'angle_velocity_cov'
// Member 'angle_acceleration'
// Member 'angle_acceleration_cov'
#include "std_msgs/msg/float32__struct.h"
// Member 'frame_id'
#include "std_msgs/msg/string__struct.h"
// Member 'priority_id'
// Member 'type'
// Member 'attention_type'
// Member 'motion_state'
// Member 'lane_pos'
// Member 'tracker_id'
#include "std_msgs/msg/int32__struct.h"
// Member 'center'
// Member 'center_cov'
// Member 'size'
// Member 'size_cov'
// Member 'direction'
// Member 'direction_cov'
// Member 'velocity'
// Member 'relative_velocity'
// Member 'velocity_cov'
// Member 'relative_velocity_cov'
// Member 'acceleration'
// Member 'acceleration_cov'
// Member 'anchor'
// Member 'nearest_point'
#include "perception_ros2_msg/msg/point3f__struct.h"
// Member 'age'
#include "std_msgs/msg/float64__struct.h"

// Struct defined in msg/CoreInfo in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__CoreInfo
{
  std_msgs__msg__Float32 timestamp;
  std_msgs__msg__String frame_id;
  std_msgs__msg__Int32 priority_id;
  std_msgs__msg__Float32 exist_confidence;
  perception_ros2_msg__msg__Point3f center;
  perception_ros2_msg__msg__Point3f center_cov;
  perception_ros2_msg__msg__Point3f size;
  perception_ros2_msg__msg__Point3f size_cov;
  perception_ros2_msg__msg__Point3f direction;
  perception_ros2_msg__msg__Point3f direction_cov;
  std_msgs__msg__Int32 type;
  std_msgs__msg__Float32 type_confidence;
  std_msgs__msg__Int32 attention_type;
  std_msgs__msg__Int32 motion_state;
  std_msgs__msg__Int32 lane_pos;
  std_msgs__msg__Int32 tracker_id;
  std_msgs__msg__Float64 age;
  perception_ros2_msg__msg__Point3f velocity;
  perception_ros2_msg__msg__Point3f relative_velocity;
  perception_ros2_msg__msg__Point3f velocity_cov;
  perception_ros2_msg__msg__Point3f relative_velocity_cov;
  perception_ros2_msg__msg__Point3f acceleration;
  perception_ros2_msg__msg__Point3f acceleration_cov;
  std_msgs__msg__Float32 angle_velocity;
  std_msgs__msg__Float32 angle_velocity_cov;
  std_msgs__msg__Float32 angle_acceleration;
  std_msgs__msg__Float32 angle_acceleration_cov;
  perception_ros2_msg__msg__Point3f anchor;
  perception_ros2_msg__msg__Point3f nearest_point;
} perception_ros2_msg__msg__CoreInfo;

// Struct for a sequence of perception_ros2_msg__msg__CoreInfo.
typedef struct perception_ros2_msg__msg__CoreInfo__Sequence
{
  perception_ros2_msg__msg__CoreInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__CoreInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__CORE_INFO__STRUCT_H_
