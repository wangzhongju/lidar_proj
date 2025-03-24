// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/RsPerceptionMsg.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/rs_perception_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `lidarframe`
#include "perception_ros2_msg/msg/lidar_frame_msg__functions.h"
// Member `device_id`
#include "std_msgs/msg/int32__functions.h"

bool
perception_ros2_msg__msg__RsPerceptionMsg__init(perception_ros2_msg__msg__RsPerceptionMsg * msg)
{
  if (!msg) {
    return false;
  }
  // lidarframe
  if (!perception_ros2_msg__msg__LidarFrameMsg__init(&msg->lidarframe)) {
    perception_ros2_msg__msg__RsPerceptionMsg__fini(msg);
    return false;
  }
  // device_id
  if (!std_msgs__msg__Int32__init(&msg->device_id)) {
    perception_ros2_msg__msg__RsPerceptionMsg__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__RsPerceptionMsg__fini(perception_ros2_msg__msg__RsPerceptionMsg * msg)
{
  if (!msg) {
    return;
  }
  // lidarframe
  perception_ros2_msg__msg__LidarFrameMsg__fini(&msg->lidarframe);
  // device_id
  std_msgs__msg__Int32__fini(&msg->device_id);
}

perception_ros2_msg__msg__RsPerceptionMsg *
perception_ros2_msg__msg__RsPerceptionMsg__create()
{
  perception_ros2_msg__msg__RsPerceptionMsg * msg = (perception_ros2_msg__msg__RsPerceptionMsg *)malloc(sizeof(perception_ros2_msg__msg__RsPerceptionMsg));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__RsPerceptionMsg));
  bool success = perception_ros2_msg__msg__RsPerceptionMsg__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__RsPerceptionMsg__destroy(perception_ros2_msg__msg__RsPerceptionMsg * msg)
{
  if (msg) {
    perception_ros2_msg__msg__RsPerceptionMsg__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__RsPerceptionMsg__Sequence__init(perception_ros2_msg__msg__RsPerceptionMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__RsPerceptionMsg * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__RsPerceptionMsg *)calloc(size, sizeof(perception_ros2_msg__msg__RsPerceptionMsg));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__RsPerceptionMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__RsPerceptionMsg__fini(&data[i - 1]);
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
perception_ros2_msg__msg__RsPerceptionMsg__Sequence__fini(perception_ros2_msg__msg__RsPerceptionMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__RsPerceptionMsg__fini(&array->data[i]);
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

perception_ros2_msg__msg__RsPerceptionMsg__Sequence *
perception_ros2_msg__msg__RsPerceptionMsg__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__RsPerceptionMsg__Sequence * array = (perception_ros2_msg__msg__RsPerceptionMsg__Sequence *)malloc(sizeof(perception_ros2_msg__msg__RsPerceptionMsg__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__RsPerceptionMsg__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__RsPerceptionMsg__Sequence__destroy(perception_ros2_msg__msg__RsPerceptionMsg__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__RsPerceptionMsg__Sequence__fini(array);
  }
  free(array);
}
