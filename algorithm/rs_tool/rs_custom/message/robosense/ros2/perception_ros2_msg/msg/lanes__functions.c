// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/Lanes.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/lanes__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `lanes`
#include "perception_ros2_msg/msg/lane__functions.h"

bool
perception_ros2_msg__msg__Lanes__init(perception_ros2_msg__msg__Lanes * msg)
{
  if (!msg) {
    return false;
  }
  // lanes
  if (!perception_ros2_msg__msg__Lane__Sequence__init(&msg->lanes, 0)) {
    perception_ros2_msg__msg__Lanes__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__Lanes__fini(perception_ros2_msg__msg__Lanes * msg)
{
  if (!msg) {
    return;
  }
  // lanes
  perception_ros2_msg__msg__Lane__Sequence__fini(&msg->lanes);
}

perception_ros2_msg__msg__Lanes *
perception_ros2_msg__msg__Lanes__create()
{
  perception_ros2_msg__msg__Lanes * msg = (perception_ros2_msg__msg__Lanes *)malloc(sizeof(perception_ros2_msg__msg__Lanes));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__Lanes));
  bool success = perception_ros2_msg__msg__Lanes__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__Lanes__destroy(perception_ros2_msg__msg__Lanes * msg)
{
  if (msg) {
    perception_ros2_msg__msg__Lanes__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__Lanes__Sequence__init(perception_ros2_msg__msg__Lanes__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__Lanes * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__Lanes *)calloc(size, sizeof(perception_ros2_msg__msg__Lanes));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__Lanes__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__Lanes__fini(&data[i - 1]);
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
perception_ros2_msg__msg__Lanes__Sequence__fini(perception_ros2_msg__msg__Lanes__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__Lanes__fini(&array->data[i]);
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

perception_ros2_msg__msg__Lanes__Sequence *
perception_ros2_msg__msg__Lanes__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__Lanes__Sequence * array = (perception_ros2_msg__msg__Lanes__Sequence *)malloc(sizeof(perception_ros2_msg__msg__Lanes__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__Lanes__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__Lanes__Sequence__destroy(perception_ros2_msg__msg__Lanes__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__Lanes__Sequence__fini(array);
  }
  free(array);
}
