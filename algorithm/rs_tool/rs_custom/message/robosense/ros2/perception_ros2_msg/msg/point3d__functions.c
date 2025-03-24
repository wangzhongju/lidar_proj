// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/Point3d.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/point3d__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `x`
// Member `y`
// Member `z`
#include "std_msgs/msg/float64__functions.h"

bool
perception_ros2_msg__msg__Point3d__init(perception_ros2_msg__msg__Point3d * msg)
{
  if (!msg) {
    return false;
  }
  // x
  if (!std_msgs__msg__Float64__init(&msg->x)) {
    perception_ros2_msg__msg__Point3d__fini(msg);
    return false;
  }
  // y
  if (!std_msgs__msg__Float64__init(&msg->y)) {
    perception_ros2_msg__msg__Point3d__fini(msg);
    return false;
  }
  // z
  if (!std_msgs__msg__Float64__init(&msg->z)) {
    perception_ros2_msg__msg__Point3d__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__Point3d__fini(perception_ros2_msg__msg__Point3d * msg)
{
  if (!msg) {
    return;
  }
  // x
  std_msgs__msg__Float64__fini(&msg->x);
  // y
  std_msgs__msg__Float64__fini(&msg->y);
  // z
  std_msgs__msg__Float64__fini(&msg->z);
}

perception_ros2_msg__msg__Point3d *
perception_ros2_msg__msg__Point3d__create()
{
  perception_ros2_msg__msg__Point3d * msg = (perception_ros2_msg__msg__Point3d *)malloc(sizeof(perception_ros2_msg__msg__Point3d));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__Point3d));
  bool success = perception_ros2_msg__msg__Point3d__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__Point3d__destroy(perception_ros2_msg__msg__Point3d * msg)
{
  if (msg) {
    perception_ros2_msg__msg__Point3d__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__Point3d__Sequence__init(perception_ros2_msg__msg__Point3d__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__Point3d * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__Point3d *)calloc(size, sizeof(perception_ros2_msg__msg__Point3d));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__Point3d__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__Point3d__fini(&data[i - 1]);
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
perception_ros2_msg__msg__Point3d__Sequence__fini(perception_ros2_msg__msg__Point3d__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__Point3d__fini(&array->data[i]);
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

perception_ros2_msg__msg__Point3d__Sequence *
perception_ros2_msg__msg__Point3d__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__Point3d__Sequence * array = (perception_ros2_msg__msg__Point3d__Sequence *)malloc(sizeof(perception_ros2_msg__msg__Point3d__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__Point3d__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__Point3d__Sequence__destroy(perception_ros2_msg__msg__Point3d__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__Point3d__Sequence__fini(array);
  }
  free(array);
}
