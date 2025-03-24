// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/RoadEdges.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__ROAD_EDGES__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__ROAD_EDGES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'curbs'
#include "perception_ros2_msg/msg/road_edge__struct.h"

// Struct defined in msg/RoadEdges in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__RoadEdges
{
  perception_ros2_msg__msg__RoadEdge__Sequence curbs;
} perception_ros2_msg__msg__RoadEdges;

// Struct for a sequence of perception_ros2_msg__msg__RoadEdges.
typedef struct perception_ros2_msg__msg__RoadEdges__Sequence
{
  perception_ros2_msg__msg__RoadEdges * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__RoadEdges__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__ROAD_EDGES__STRUCT_H_
