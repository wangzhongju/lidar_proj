// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/FreeSpaceInfos.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/free_space_infos__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `fs_pts`
#include "perception_ros2_msg/msg/point3f__functions.h"
// Member `fs_confidence`
#include "std_msgs/msg/float32__functions.h"

bool
perception_ros2_msg__msg__FreeSpaceInfos__init(perception_ros2_msg__msg__FreeSpaceInfos * msg)
{
  if (!msg) {
    return false;
  }
  // fs_pts
  if (!perception_ros2_msg__msg__Point3f__Sequence__init(&msg->fs_pts, 0)) {
    perception_ros2_msg__msg__FreeSpaceInfos__fini(msg);
    return false;
  }
  // fs_confidence
  if (!std_msgs__msg__Float32__Sequence__init(&msg->fs_confidence, 0)) {
    perception_ros2_msg__msg__FreeSpaceInfos__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__FreeSpaceInfos__fini(perception_ros2_msg__msg__FreeSpaceInfos * msg)
{
  if (!msg) {
    return;
  }
  // fs_pts
  perception_ros2_msg__msg__Point3f__Sequence__fini(&msg->fs_pts);
  // fs_confidence
  std_msgs__msg__Float32__Sequence__fini(&msg->fs_confidence);
}

perception_ros2_msg__msg__FreeSpaceInfos *
perception_ros2_msg__msg__FreeSpaceInfos__create()
{
  perception_ros2_msg__msg__FreeSpaceInfos * msg = (perception_ros2_msg__msg__FreeSpaceInfos *)malloc(sizeof(perception_ros2_msg__msg__FreeSpaceInfos));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__FreeSpaceInfos));
  bool success = perception_ros2_msg__msg__FreeSpaceInfos__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__FreeSpaceInfos__destroy(perception_ros2_msg__msg__FreeSpaceInfos * msg)
{
  if (msg) {
    perception_ros2_msg__msg__FreeSpaceInfos__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__FreeSpaceInfos__Sequence__init(perception_ros2_msg__msg__FreeSpaceInfos__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__FreeSpaceInfos * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__FreeSpaceInfos *)calloc(size, sizeof(perception_ros2_msg__msg__FreeSpaceInfos));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__FreeSpaceInfos__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__FreeSpaceInfos__fini(&data[i - 1]);
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
perception_ros2_msg__msg__FreeSpaceInfos__Sequence__fini(perception_ros2_msg__msg__FreeSpaceInfos__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__FreeSpaceInfos__fini(&array->data[i]);
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

perception_ros2_msg__msg__FreeSpaceInfos__Sequence *
perception_ros2_msg__msg__FreeSpaceInfos__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__FreeSpaceInfos__Sequence * array = (perception_ros2_msg__msg__FreeSpaceInfos__Sequence *)malloc(sizeof(perception_ros2_msg__msg__FreeSpaceInfos__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__FreeSpaceInfos__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__FreeSpaceInfos__Sequence__destroy(perception_ros2_msg__msg__FreeSpaceInfos__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__FreeSpaceInfos__Sequence__fini(array);
  }
  free(array);
}
