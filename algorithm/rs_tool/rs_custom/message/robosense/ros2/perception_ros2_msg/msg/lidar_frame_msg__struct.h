// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/LidarFrameMsg.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__LIDAR_FRAME_MSG__STRUCT_H_
#define PERCEPTION_ROS2_MSG__MSG__LIDAR_FRAME_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'frame_id'
#include "std_msgs/msg/string__struct.h"
// Member 'timestamp'
#include "std_msgs/msg/float64__struct.h"
// Member 'global_pose'
#include "perception_ros2_msg/msg/pose__struct.h"
// Member 'gps_origin'
#include "perception_ros2_msg/msg/point3d__struct.h"
// Member 'status_pose_map'
#include "perception_ros2_msg/msg/pose_map__struct.h"
// Member 'status'
#include "std_msgs/msg/int32__struct.h"
// Member 'valid_indices'
// Member 'non_ground_indices'
// Member 'ground_indices'
// Member 'background_indices'
#include "perception_ros2_msg/msg/indices__struct.h"
// Member 'objects'
// Member 'attention_objects'
#include "perception_ros2_msg/msg/objects__struct.h"
// Member 'has_pointcloud'
// Member 'has_attention_objects'
// Member 'has_freespace'
// Member 'has_lanes'
// Member 'has_roadedges'
// Member 'has_sematice_indices'
#include "std_msgs/msg/bool__struct.h"
// Member 'scan_pointcloud'
#include "perception_ros2_msg/msg/point4f__struct.h"
// Member 'freespace_infos'
#include "perception_ros2_msg/msg/free_space_infos__struct.h"
// Member 'lanes'
#include "perception_ros2_msg/msg/lanes__struct.h"
// Member 'roadedges'
#include "perception_ros2_msg/msg/road_edges__struct.h"

// Struct defined in msg/LidarFrameMsg in the package perception_ros2_msg.
typedef struct perception_ros2_msg__msg__LidarFrameMsg
{
  std_msgs__msg__String frame_id;
  std_msgs__msg__Float64 timestamp;
  perception_ros2_msg__msg__Pose global_pose;
  perception_ros2_msg__msg__Point3d gps_origin;
  perception_ros2_msg__msg__PoseMap status_pose_map;
  std_msgs__msg__Int32 status;
  perception_ros2_msg__msg__Indices valid_indices;
  perception_ros2_msg__msg__Objects objects;
  std_msgs__msg__Bool has_pointcloud;
  perception_ros2_msg__msg__Point4f__Sequence scan_pointcloud;
  std_msgs__msg__Bool has_attention_objects;
  perception_ros2_msg__msg__Objects attention_objects;
  std_msgs__msg__Bool has_freespace;
  perception_ros2_msg__msg__FreeSpaceInfos freespace_infos;
  std_msgs__msg__Bool has_lanes;
  perception_ros2_msg__msg__Lanes lanes;
  std_msgs__msg__Bool has_roadedges;
  perception_ros2_msg__msg__RoadEdges roadedges;
  std_msgs__msg__Bool has_sematice_indices;
  perception_ros2_msg__msg__Indices non_ground_indices;
  perception_ros2_msg__msg__Indices ground_indices;
  perception_ros2_msg__msg__Indices background_indices;
} perception_ros2_msg__msg__LidarFrameMsg;

// Struct for a sequence of perception_ros2_msg__msg__LidarFrameMsg.
typedef struct perception_ros2_msg__msg__LidarFrameMsg__Sequence
{
  perception_ros2_msg__msg__LidarFrameMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_ros2_msg__msg__LidarFrameMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__LIDAR_FRAME_MSG__STRUCT_H_
