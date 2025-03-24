// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/SupplementInfo.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__SUPPLEMENT_INFO__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__SUPPLEMENT_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'unique_id'
#include "std_msgs/msg/u_int32__struct.h"
// Member 'polygon'
// Member 'geo_center'
// Member 'geo_size'
// Member 'trajectory'
// Member 'history_velocity'
#include "perception_ros2_msg/msg/point3f__struct.h"
// Member 'left_point_index'
// Member 'right_point_index'
// Member 'cloud_indices'
// Member 'size_type'
// Member 'mode'
// Member 'tracking_state'
// Member 'history_type'
// Member 'gps_mode'
#include "std_msgs/msg/int32__struct.h"
// Member 'latent_types'
#include "std_msgs/msg/float32__struct.h"
// Member 'in_roi'
#include "std_msgs/msg/bool__struct.h"
// Member 'gps_info'
#include "perception_ros2_msg/msg/point3d__struct.h"

// Struct defined in msg/SupplementInfo in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__SupplementInfo
{
  std_msgs__msg__UInt32 unique_id;
  perception_ros2_msg__msg__Point3f__Sequence polygon;
  std_msgs__msg__Int32 left_point_index;
  std_msgs__msg__Int32 right_point_index;
  std_msgs__msg__Int32__Sequence cloud_indices;
  std_msgs__msg__Float32__Sequence latent_types;
  std_msgs__msg__Int32 size_type;
  std_msgs__msg__Int32 mode;
  std_msgs__msg__Bool in_roi;
  std_msgs__msg__Int32 tracking_state;
  perception_ros2_msg__msg__Point3f geo_center;
  perception_ros2_msg__msg__Point3f geo_size;
  perception_ros2_msg__msg__Point3f__Sequence trajectory;
  perception_ros2_msg__msg__Point3f__Sequence history_velocity;
  std_msgs__msg__Int32__Sequence history_type;
  std_msgs__msg__Int32 gps_mode;
  perception_ros2_msg__msg__Point3d gps_info;
} perception_ros2_msg__msg__SupplementInfo;

// Struct for a sequence of perception_ros2_msg__msg__SupplementInfo.
typedef struct perception_ros2_msg__msg__SupplementInfo__Sequence
{
  perception_ros2_msg__msg__SupplementInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__SupplementInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__SUPPLEMENT_INFO__STRUCT_H_
