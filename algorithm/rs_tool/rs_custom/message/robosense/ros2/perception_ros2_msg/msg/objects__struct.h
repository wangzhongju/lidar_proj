// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/Objects.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__OBJECTS__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__OBJECTS__STRUCT_H_

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
#include "std_msgs/msg/float32__struct.h"
// Member 'device_id'
#include "std_msgs/msg/int32__struct.h"
// Member 'objects'
#include "perception_ros2_msg/msg/object__struct.h"

// Struct defined in msg/Objects in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__Objects
{
  std_msgs__msg__Float32 timestamp;
  std_msgs__msg__Int32 device_id;
  perception_ros2_msg__msg__Object__Sequence objects;
} perception_ros2_msg__msg__Objects;

// Struct for a sequence of perception_ros2_msg__msg__Objects.
typedef struct perception_ros2_msg__msg__Objects__Sequence
{
  perception_ros2_msg__msg__Objects * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__Objects__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__OBJECTS__STRUCT_H_
