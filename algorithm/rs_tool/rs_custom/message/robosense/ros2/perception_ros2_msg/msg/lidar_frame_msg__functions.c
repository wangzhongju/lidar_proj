// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/LidarFrameMsg.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/lidar_frame_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `frame_id`
#include "std_msgs/msg/string__functions.h"
// Member `timestamp`
#include "std_msgs/msg/float64__functions.h"
// Member `global_pose`
#include "perception_ros2_msg/msg/pose__functions.h"
// Member `gps_origin`
#include "perception_ros2_msg/msg/point3d__functions.h"
// Member `status_pose_map`
#include "perception_ros2_msg/msg/pose_map__functions.h"
// Member `status`
#include "std_msgs/msg/int32__functions.h"
// Member `valid_indices`
// Member `non_ground_indices`
// Member `ground_indices`
// Member `background_indices`
#include "perception_ros2_msg/msg/indices__functions.h"
// Member `objects`
// Member `attention_objects`
#include "perception_ros2_msg/msg/objects__functions.h"
// Member `has_pointcloud`
// Member `has_attention_objects`
// Member `has_freespace`
// Member `has_lanes`
// Member `has_roadedges`
// Member `has_sematice_indices`
#include "std_msgs/msg/bool__functions.h"
// Member `scan_pointcloud`
#include "perception_ros2_msg/msg/point4f__functions.h"
// Member `freespace_infos`
#include "perception_ros2_msg/msg/free_space_infos__functions.h"
// Member `lanes`
#include "perception_ros2_msg/msg/lanes__functions.h"
// Member `roadedges`
#include "perception_ros2_msg/msg/road_edges__functions.h"

bool
perception_ros2_msg__msg__LidarFrameMsg__init(perception_ros2_msg__msg__LidarFrameMsg * msg)
{
  if (!msg) {
    return false;
  }
  // frame_id
  if (!std_msgs__msg__String__init(&msg->frame_id)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // timestamp
  if (!std_msgs__msg__Float64__init(&msg->timestamp)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // global_pose
  if (!perception_ros2_msg__msg__Pose__init(&msg->global_pose)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // gps_origin
  if (!perception_ros2_msg__msg__Point3d__init(&msg->gps_origin)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // status_pose_map
  if (!perception_ros2_msg__msg__PoseMap__init(&msg->status_pose_map)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // status
  if (!std_msgs__msg__Int32__init(&msg->status)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // valid_indices
  if (!perception_ros2_msg__msg__Indices__init(&msg->valid_indices)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // objects
  if (!perception_ros2_msg__msg__Objects__init(&msg->objects)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // has_pointcloud
  if (!std_msgs__msg__Bool__init(&msg->has_pointcloud)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // scan_pointcloud
  if (!perception_ros2_msg__msg__Point4f__Sequence__init(&msg->scan_pointcloud, 0)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // has_attention_objects
  if (!std_msgs__msg__Bool__init(&msg->has_attention_objects)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // attention_objects
  if (!perception_ros2_msg__msg__Objects__init(&msg->attention_objects)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // has_freespace
  if (!std_msgs__msg__Bool__init(&msg->has_freespace)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // freespace_infos
  if (!perception_ros2_msg__msg__FreeSpaceInfos__init(&msg->freespace_infos)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // has_lanes
  if (!std_msgs__msg__Bool__init(&msg->has_lanes)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // lanes
  if (!perception_ros2_msg__msg__Lanes__init(&msg->lanes)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // has_roadedges
  if (!std_msgs__msg__Bool__init(&msg->has_roadedges)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // roadedges
  if (!perception_ros2_msg__msg__RoadEdges__init(&msg->roadedges)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // has_sematice_indices
  if (!std_msgs__msg__Bool__init(&msg->has_sematice_indices)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // non_ground_indices
  if (!perception_ros2_msg__msg__Indices__init(&msg->non_ground_indices)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // ground_indices
  if (!perception_ros2_msg__msg__Indices__init(&msg->ground_indices)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  // background_indices
  if (!perception_ros2_msg__msg__Indices__init(&msg->background_indices)) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__LidarFrameMsg__fini(perception_ros2_msg__msg__LidarFrameMsg * msg)
{
  if (!msg) {
    return;
  }
  // frame_id
  std_msgs__msg__String__fini(&msg->frame_id);
  // timestamp
  std_msgs__msg__Float64__fini(&msg->timestamp);
  // global_pose
  perception_ros2_msg__msg__Pose__fini(&msg->global_pose);
  // gps_origin
  perception_ros2_msg__msg__Point3d__fini(&msg->gps_origin);
  // status_pose_map
  perception_ros2_msg__msg__PoseMap__fini(&msg->status_pose_map);
  // status
  std_msgs__msg__Int32__fini(&msg->status);
  // valid_indices
  perception_ros2_msg__msg__Indices__fini(&msg->valid_indices);
  // objects
  perception_ros2_msg__msg__Objects__fini(&msg->objects);
  // has_pointcloud
  std_msgs__msg__Bool__fini(&msg->has_pointcloud);
  // scan_pointcloud
  perception_ros2_msg__msg__Point4f__Sequence__fini(&msg->scan_pointcloud);
  // has_attention_objects
  std_msgs__msg__Bool__fini(&msg->has_attention_objects);
  // attention_objects
  perception_ros2_msg__msg__Objects__fini(&msg->attention_objects);
  // has_freespace
  std_msgs__msg__Bool__fini(&msg->has_freespace);
  // freespace_infos
  perception_ros2_msg__msg__FreeSpaceInfos__fini(&msg->freespace_infos);
  // has_lanes
  std_msgs__msg__Bool__fini(&msg->has_lanes);
  // lanes
  perception_ros2_msg__msg__Lanes__fini(&msg->lanes);
  // has_roadedges
  std_msgs__msg__Bool__fini(&msg->has_roadedges);
  // roadedges
  perception_ros2_msg__msg__RoadEdges__fini(&msg->roadedges);
  // has_sematice_indices
  std_msgs__msg__Bool__fini(&msg->has_sematice_indices);
  // non_ground_indices
  perception_ros2_msg__msg__Indices__fini(&msg->non_ground_indices);
  // ground_indices
  perception_ros2_msg__msg__Indices__fini(&msg->ground_indices);
  // background_indices
  perception_ros2_msg__msg__Indices__fini(&msg->background_indices);
}

perception_ros2_msg__msg__LidarFrameMsg *
perception_ros2_msg__msg__LidarFrameMsg__create()
{
  perception_ros2_msg__msg__LidarFrameMsg * msg = (perception_ros2_msg__msg__LidarFrameMsg *)malloc(sizeof(perception_ros2_msg__msg__LidarFrameMsg));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__LidarFrameMsg));
  bool success = perception_ros2_msg__msg__LidarFrameMsg__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__LidarFrameMsg__destroy(perception_ros2_msg__msg__LidarFrameMsg * msg)
{
  if (msg) {
    perception_ros2_msg__msg__LidarFrameMsg__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__LidarFrameMsg__Sequence__init(perception_ros2_msg__msg__LidarFrameMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__LidarFrameMsg * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__LidarFrameMsg *)calloc(size, sizeof(perception_ros2_msg__msg__LidarFrameMsg));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__LidarFrameMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__LidarFrameMsg__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
perception_ros2_msg__msg__LidarFrameMsg__Sequence__fini(perception_ros2_msg__msg__LidarFrameMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__LidarFrameMsg__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

perception_ros2_msg__msg__LidarFrameMsg__Sequence *
perception_ros2_msg__msg__LidarFrameMsg__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__LidarFrameMsg__Sequence * array = (perception_ros2_msg__msg__LidarFrameMsg__Sequence *)malloc(sizeof(perception_ros2_msg__msg__LidarFrameMsg__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__LidarFrameMsg__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__LidarFrameMsg__Sequence__destroy(perception_ros2_msg__msg__LidarFrameMsg__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__LidarFrameMsg__Sequence__fini(array);
  }
  free(array);
}
