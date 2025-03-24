// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/AxisStatusPose.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/axis_status_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `status`
#include "std_msgs/msg/int32__functions.h"
// Member `pose`
#include "perception_ros2_msg/msg/pose__functions.h"

bool
perception_ros2_msg__msg__AxisStatusPose__init(perception_ros2_msg__msg__AxisStatusPose * msg)
{
  if (!msg) {
    return false;
  }
  // status
  if (!std_msgs__msg__Int32__init(&msg->status)) {
    perception_ros2_msg__msg__AxisStatusPose__fini(msg);
    return false;
  }
  // pose
  if (!perception_ros2_msg__msg__Pose__init(&msg->pose)) {
    perception_ros2_msg__msg__AxisStatusPose__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__AxisStatusPose__fini(perception_ros2_msg__msg__AxisStatusPose * msg)
{
  if (!msg) {
    return;
  }
  // status
  std_msgs__msg__Int32__fini(&msg->status);
  // pose
  perception_ros2_msg__msg__Pose__fini(&msg->pose);
}

perception_ros2_msg__msg__AxisStatusPose *
perception_ros2_msg__msg__AxisStatusPose__create()
{
  perception_ros2_msg__msg__AxisStatusPose * msg = (perception_ros2_msg__msg__AxisStatusPose *)malloc(sizeof(perception_ros2_msg__msg__AxisStatusPose));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__AxisStatusPose));
  bool success = perception_ros2_msg__msg__AxisStatusPose__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__AxisStatusPose__destroy(perception_ros2_msg__msg__AxisStatusPose * msg)
{
  if (msg) {
    perception_ros2_msg__msg__AxisStatusPose__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__AxisStatusPose__Sequence__init(perception_ros2_msg__msg__AxisStatusPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__AxisStatusPose * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__AxisStatusPose *)calloc(size, sizeof(perception_ros2_msg__msg__AxisStatusPose));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__AxisStatusPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__AxisStatusPose__fini(&data[i - 1]);
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
perception_ros2_msg__msg__AxisStatusPose__Sequence__fini(perception_ros2_msg__msg__AxisStatusPose__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__AxisStatusPose__fini(&array->data[i]);
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

perception_ros2_msg__msg__AxisStatusPose__Sequence *
perception_ros2_msg__msg__AxisStatusPose__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__AxisStatusPose__Sequence * array = (perception_ros2_msg__msg__AxisStatusPose__Sequence *)malloc(sizeof(perception_ros2_msg__msg__AxisStatusPose__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__AxisStatusPose__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__AxisStatusPose__Sequence__destroy(perception_ros2_msg__msg__AxisStatusPose__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__AxisStatusPose__Sequence__fini(array);
  }
  free(array);
}
