// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/Object.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/object__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `coreinfo`
#include "perception_ros2_msg/msg/core_info__functions.h"
// Member `hassupplmentinfo`
#include "std_msgs/msg/bool__functions.h"
// Member `supplementinfo`
#include "perception_ros2_msg/msg/supplement_info__functions.h"

bool
perception_ros2_msg__msg__Object__init(perception_ros2_msg__msg__Object * msg)
{
  if (!msg) {
    return false;
  }
  // coreinfo
  if (!perception_ros2_msg__msg__CoreInfo__init(&msg->coreinfo)) {
    perception_ros2_msg__msg__Object__fini(msg);
    return false;
  }
  // hassupplmentinfo
  if (!std_msgs__msg__Bool__init(&msg->hassupplmentinfo)) {
    perception_ros2_msg__msg__Object__fini(msg);
    return false;
  }
  // supplementinfo
  if (!perception_ros2_msg__msg__SupplementInfo__init(&msg->supplementinfo)) {
    perception_ros2_msg__msg__Object__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__Object__fini(perception_ros2_msg__msg__Object * msg)
{
  if (!msg) {
    return;
  }
  // coreinfo
  perception_ros2_msg__msg__CoreInfo__fini(&msg->coreinfo);
  // hassupplmentinfo
  std_msgs__msg__Bool__fini(&msg->hassupplmentinfo);
  // supplementinfo
  perception_ros2_msg__msg__SupplementInfo__fini(&msg->supplementinfo);
}

perception_ros2_msg__msg__Object *
perception_ros2_msg__msg__Object__create()
{
  perception_ros2_msg__msg__Object * msg = (perception_ros2_msg__msg__Object *)malloc(sizeof(perception_ros2_msg__msg__Object));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__Object));
  bool success = perception_ros2_msg__msg__Object__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__Object__destroy(perception_ros2_msg__msg__Object * msg)
{
  if (msg) {
    perception_ros2_msg__msg__Object__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__Object__Sequence__init(perception_ros2_msg__msg__Object__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__Object * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__Object *)calloc(size, sizeof(perception_ros2_msg__msg__Object));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__Object__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__Object__fini(&data[i - 1]);
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
perception_ros2_msg__msg__Object__Sequence__fini(perception_ros2_msg__msg__Object__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__Object__fini(&array->data[i]);
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

perception_ros2_msg__msg__Object__Sequence *
perception_ros2_msg__msg__Object__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__Object__Sequence * array = (perception_ros2_msg__msg__Object__Sequence *)malloc(sizeof(perception_ros2_msg__msg__Object__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__Object__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__Object__Sequence__destroy(perception_ros2_msg__msg__Object__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__Object__Sequence__fini(array);
  }
  free(array);
}
