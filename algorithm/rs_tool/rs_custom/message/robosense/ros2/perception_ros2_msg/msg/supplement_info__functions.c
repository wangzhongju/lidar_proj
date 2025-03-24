// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/SupplementInfo.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/supplement_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `unique_id`
#include "std_msgs/msg/u_int32__functions.h"
// Member `polygon`
// Member `geo_center`
// Member `geo_size`
// Member `trajectory`
// Member `history_velocity`
#include "perception_ros2_msg/msg/point3f__functions.h"
// Member `left_point_index`
// Member `right_point_index`
// Member `cloud_indices`
// Member `size_type`
// Member `mode`
// Member `tracking_state`
// Member `history_type`
// Member `gps_mode`
#include "std_msgs/msg/int32__functions.h"
// Member `latent_types`
#include "std_msgs/msg/float32__functions.h"
// Member `in_roi`
#include "std_msgs/msg/bool__functions.h"
// Member `gps_info`
#include "perception_ros2_msg/msg/point3d__functions.h"

bool
perception_ros2_msg__msg__SupplementInfo__init(perception_ros2_msg__msg__SupplementInfo * msg)
{
  if (!msg) {
    return false;
  }
  // unique_id
  if (!std_msgs__msg__UInt32__init(&msg->unique_id)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // polygon
  if (!perception_ros2_msg__msg__Point3f__Sequence__init(&msg->polygon, 0)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // left_point_index
  if (!std_msgs__msg__Int32__init(&msg->left_point_index)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // right_point_index
  if (!std_msgs__msg__Int32__init(&msg->right_point_index)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // cloud_indices
  if (!std_msgs__msg__Int32__Sequence__init(&msg->cloud_indices, 0)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // latent_types
  if (!std_msgs__msg__Float32__Sequence__init(&msg->latent_types, 0)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // size_type
  if (!std_msgs__msg__Int32__init(&msg->size_type)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // mode
  if (!std_msgs__msg__Int32__init(&msg->mode)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // in_roi
  if (!std_msgs__msg__Bool__init(&msg->in_roi)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // tracking_state
  if (!std_msgs__msg__Int32__init(&msg->tracking_state)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // geo_center
  if (!perception_ros2_msg__msg__Point3f__init(&msg->geo_center)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // geo_size
  if (!perception_ros2_msg__msg__Point3f__init(&msg->geo_size)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // trajectory
  if (!perception_ros2_msg__msg__Point3f__Sequence__init(&msg->trajectory, 0)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // history_velocity
  if (!perception_ros2_msg__msg__Point3f__Sequence__init(&msg->history_velocity, 0)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // history_type
  if (!std_msgs__msg__Int32__Sequence__init(&msg->history_type, 0)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // gps_mode
  if (!std_msgs__msg__Int32__init(&msg->gps_mode)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  // gps_info
  if (!perception_ros2_msg__msg__Point3d__init(&msg->gps_info)) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__SupplementInfo__fini(perception_ros2_msg__msg__SupplementInfo * msg)
{
  if (!msg) {
    return;
  }
  // unique_id
  std_msgs__msg__UInt32__fini(&msg->unique_id);
  // polygon
  perception_ros2_msg__msg__Point3f__Sequence__fini(&msg->polygon);
  // left_point_index
  std_msgs__msg__Int32__fini(&msg->left_point_index);
  // right_point_index
  std_msgs__msg__Int32__fini(&msg->right_point_index);
  // cloud_indices
  std_msgs__msg__Int32__Sequence__fini(&msg->cloud_indices);
  // latent_types
  std_msgs__msg__Float32__Sequence__fini(&msg->latent_types);
  // size_type
  std_msgs__msg__Int32__fini(&msg->size_type);
  // mode
  std_msgs__msg__Int32__fini(&msg->mode);
  // in_roi
  std_msgs__msg__Bool__fini(&msg->in_roi);
  // tracking_state
  std_msgs__msg__Int32__fini(&msg->tracking_state);
  // geo_center
  perception_ros2_msg__msg__Point3f__fini(&msg->geo_center);
  // geo_size
  perception_ros2_msg__msg__Point3f__fini(&msg->geo_size);
  // trajectory
  perception_ros2_msg__msg__Point3f__Sequence__fini(&msg->trajectory);
  // history_velocity
  perception_ros2_msg__msg__Point3f__Sequence__fini(&msg->history_velocity);
  // history_type
  std_msgs__msg__Int32__Sequence__fini(&msg->history_type);
  // gps_mode
  std_msgs__msg__Int32__fini(&msg->gps_mode);
  // gps_info
  perception_ros2_msg__msg__Point3d__fini(&msg->gps_info);
}

perception_ros2_msg__msg__SupplementInfo *
perception_ros2_msg__msg__SupplementInfo__create()
{
  perception_ros2_msg__msg__SupplementInfo * msg = (perception_ros2_msg__msg__SupplementInfo *)malloc(sizeof(perception_ros2_msg__msg__SupplementInfo));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__SupplementInfo));
  bool success = perception_ros2_msg__msg__SupplementInfo__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__SupplementInfo__destroy(perception_ros2_msg__msg__SupplementInfo * msg)
{
  if (msg) {
    perception_ros2_msg__msg__SupplementInfo__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__SupplementInfo__Sequence__init(perception_ros2_msg__msg__SupplementInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__SupplementInfo * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__SupplementInfo *)calloc(size, sizeof(perception_ros2_msg__msg__SupplementInfo));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__SupplementInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__SupplementInfo__fini(&data[i - 1]);
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
perception_ros2_msg__msg__SupplementInfo__Sequence__fini(perception_ros2_msg__msg__SupplementInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__SupplementInfo__fini(&array->data[i]);
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

perception_ros2_msg__msg__SupplementInfo__Sequence *
perception_ros2_msg__msg__SupplementInfo__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__SupplementInfo__Sequence * array = (perception_ros2_msg__msg__SupplementInfo__Sequence *)malloc(sizeof(perception_ros2_msg__msg__SupplementInfo__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__SupplementInfo__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__SupplementInfo__Sequence__destroy(perception_ros2_msg__msg__SupplementInfo__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__SupplementInfo__Sequence__fini(array);
  }
  free(array);
}
