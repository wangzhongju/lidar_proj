// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/CoreInfo.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/core_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `timestamp`
// Member `exist_confidence`
// Member `type_confidence`
// Member `angle_velocity`
// Member `angle_velocity_cov`
// Member `angle_acceleration`
// Member `angle_acceleration_cov`
#include "std_msgs/msg/float32__functions.h"
// Member `frame_id`
#include "std_msgs/msg/string__functions.h"
// Member `priority_id`
// Member `type`
// Member `attention_type`
// Member `motion_state`
// Member `lane_pos`
// Member `tracker_id`
#include "std_msgs/msg/int32__functions.h"
// Member `center`
// Member `center_cov`
// Member `size`
// Member `size_cov`
// Member `direction`
// Member `direction_cov`
// Member `velocity`
// Member `relative_velocity`
// Member `velocity_cov`
// Member `relative_velocity_cov`
// Member `acceleration`
// Member `acceleration_cov`
// Member `anchor`
// Member `nearest_point`
#include "perception_ros2_msg/msg/point3f__functions.h"
// Member `age`
#include "std_msgs/msg/float64__functions.h"

bool
perception_ros2_msg__msg__CoreInfo__init(perception_ros2_msg__msg__CoreInfo * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  if (!std_msgs__msg__Float32__init(&msg->timestamp)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // frame_id
  if (!std_msgs__msg__String__init(&msg->frame_id)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // priority_id
  if (!std_msgs__msg__Int32__init(&msg->priority_id)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // exist_confidence
  if (!std_msgs__msg__Float32__init(&msg->exist_confidence)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // center
  if (!perception_ros2_msg__msg__Point3f__init(&msg->center)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // center_cov
  if (!perception_ros2_msg__msg__Point3f__init(&msg->center_cov)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // size
  if (!perception_ros2_msg__msg__Point3f__init(&msg->size)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // size_cov
  if (!perception_ros2_msg__msg__Point3f__init(&msg->size_cov)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // direction
  if (!perception_ros2_msg__msg__Point3f__init(&msg->direction)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // direction_cov
  if (!perception_ros2_msg__msg__Point3f__init(&msg->direction_cov)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // type
  if (!std_msgs__msg__Int32__init(&msg->type)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // type_confidence
  if (!std_msgs__msg__Float32__init(&msg->type_confidence)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // attention_type
  if (!std_msgs__msg__Int32__init(&msg->attention_type)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // motion_state
  if (!std_msgs__msg__Int32__init(&msg->motion_state)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // lane_pos
  if (!std_msgs__msg__Int32__init(&msg->lane_pos)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // tracker_id
  if (!std_msgs__msg__Int32__init(&msg->tracker_id)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // age
  if (!std_msgs__msg__Float64__init(&msg->age)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // velocity
  if (!perception_ros2_msg__msg__Point3f__init(&msg->velocity)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // relative_velocity
  if (!perception_ros2_msg__msg__Point3f__init(&msg->relative_velocity)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // velocity_cov
  if (!perception_ros2_msg__msg__Point3f__init(&msg->velocity_cov)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // relative_velocity_cov
  if (!perception_ros2_msg__msg__Point3f__init(&msg->relative_velocity_cov)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // acceleration
  if (!perception_ros2_msg__msg__Point3f__init(&msg->acceleration)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // acceleration_cov
  if (!perception_ros2_msg__msg__Point3f__init(&msg->acceleration_cov)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // angle_velocity
  if (!std_msgs__msg__Float32__init(&msg->angle_velocity)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // angle_velocity_cov
  if (!std_msgs__msg__Float32__init(&msg->angle_velocity_cov)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // angle_acceleration
  if (!std_msgs__msg__Float32__init(&msg->angle_acceleration)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // angle_acceleration_cov
  if (!std_msgs__msg__Float32__init(&msg->angle_acceleration_cov)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // anchor
  if (!perception_ros2_msg__msg__Point3f__init(&msg->anchor)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  // nearest_point
  if (!perception_ros2_msg__msg__Point3f__init(&msg->nearest_point)) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__CoreInfo__fini(perception_ros2_msg__msg__CoreInfo * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  std_msgs__msg__Float32__fini(&msg->timestamp);
  // frame_id
  std_msgs__msg__String__fini(&msg->frame_id);
  // priority_id
  std_msgs__msg__Int32__fini(&msg->priority_id);
  // exist_confidence
  std_msgs__msg__Float32__fini(&msg->exist_confidence);
  // center
  perception_ros2_msg__msg__Point3f__fini(&msg->center);
  // center_cov
  perception_ros2_msg__msg__Point3f__fini(&msg->center_cov);
  // size
  perception_ros2_msg__msg__Point3f__fini(&msg->size);
  // size_cov
  perception_ros2_msg__msg__Point3f__fini(&msg->size_cov);
  // direction
  perception_ros2_msg__msg__Point3f__fini(&msg->direction);
  // direction_cov
  perception_ros2_msg__msg__Point3f__fini(&msg->direction_cov);
  // type
  std_msgs__msg__Int32__fini(&msg->type);
  // type_confidence
  std_msgs__msg__Float32__fini(&msg->type_confidence);
  // attention_type
  std_msgs__msg__Int32__fini(&msg->attention_type);
  // motion_state
  std_msgs__msg__Int32__fini(&msg->motion_state);
  // lane_pos
  std_msgs__msg__Int32__fini(&msg->lane_pos);
  // tracker_id
  std_msgs__msg__Int32__fini(&msg->tracker_id);
  // age
  std_msgs__msg__Float64__fini(&msg->age);
  // velocity
  perception_ros2_msg__msg__Point3f__fini(&msg->velocity);
  // relative_velocity
  perception_ros2_msg__msg__Point3f__fini(&msg->relative_velocity);
  // velocity_cov
  perception_ros2_msg__msg__Point3f__fini(&msg->velocity_cov);
  // relative_velocity_cov
  perception_ros2_msg__msg__Point3f__fini(&msg->relative_velocity_cov);
  // acceleration
  perception_ros2_msg__msg__Point3f__fini(&msg->acceleration);
  // acceleration_cov
  perception_ros2_msg__msg__Point3f__fini(&msg->acceleration_cov);
  // angle_velocity
  std_msgs__msg__Float32__fini(&msg->angle_velocity);
  // angle_velocity_cov
  std_msgs__msg__Float32__fini(&msg->angle_velocity_cov);
  // angle_acceleration
  std_msgs__msg__Float32__fini(&msg->angle_acceleration);
  // angle_acceleration_cov
  std_msgs__msg__Float32__fini(&msg->angle_acceleration_cov);
  // anchor
  perception_ros2_msg__msg__Point3f__fini(&msg->anchor);
  // nearest_point
  perception_ros2_msg__msg__Point3f__fini(&msg->nearest_point);
}

perception_ros2_msg__msg__CoreInfo *
perception_ros2_msg__msg__CoreInfo__create()
{
  perception_ros2_msg__msg__CoreInfo * msg = (perception_ros2_msg__msg__CoreInfo *)malloc(sizeof(perception_ros2_msg__msg__CoreInfo));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__CoreInfo));
  bool success = perception_ros2_msg__msg__CoreInfo__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__CoreInfo__destroy(perception_ros2_msg__msg__CoreInfo * msg)
{
  if (msg) {
    perception_ros2_msg__msg__CoreInfo__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__CoreInfo__Sequence__init(perception_ros2_msg__msg__CoreInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__CoreInfo * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__CoreInfo *)calloc(size, sizeof(perception_ros2_msg__msg__CoreInfo));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__CoreInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__CoreInfo__fini(&data[i - 1]);
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
perception_ros2_msg__msg__CoreInfo__Sequence__fini(perception_ros2_msg__msg__CoreInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__CoreInfo__fini(&array->data[i]);
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

perception_ros2_msg__msg__CoreInfo__Sequence *
perception_ros2_msg__msg__CoreInfo__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__CoreInfo__Sequence * array = (perception_ros2_msg__msg__CoreInfo__Sequence *)malloc(sizeof(perception_ros2_msg__msg__CoreInfo__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__CoreInfo__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__CoreInfo__Sequence__destroy(perception_ros2_msg__msg__CoreInfo__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__CoreInfo__Sequence__fini(array);
  }
  free(array);
}
