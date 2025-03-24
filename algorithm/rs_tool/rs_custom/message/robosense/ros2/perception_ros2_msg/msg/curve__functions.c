// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/Curve.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/curve__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `x_start`
// Member `x_end`
// Member `a`
// Member `b`
// Member `c`
// Member `d`
#include "std_msgs/msg/float32__functions.h"

bool
perception_ros2_msg__msg__Curve__init(perception_ros2_msg__msg__Curve * msg)
{
  if (!msg) {
    return false;
  }
  // x_start
  if (!std_msgs__msg__Float32__init(&msg->x_start)) {
    perception_ros2_msg__msg__Curve__fini(msg);
    return false;
  }
  // x_end
  if (!std_msgs__msg__Float32__init(&msg->x_end)) {
    perception_ros2_msg__msg__Curve__fini(msg);
    return false;
  }
  // a
  if (!std_msgs__msg__Float32__init(&msg->a)) {
    perception_ros2_msg__msg__Curve__fini(msg);
    return false;
  }
  // b
  if (!std_msgs__msg__Float32__init(&msg->b)) {
    perception_ros2_msg__msg__Curve__fini(msg);
    return false;
  }
  // c
  if (!std_msgs__msg__Float32__init(&msg->c)) {
    perception_ros2_msg__msg__Curve__fini(msg);
    return false;
  }
  // d
  if (!std_msgs__msg__Float32__init(&msg->d)) {
    perception_ros2_msg__msg__Curve__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__Curve__fini(perception_ros2_msg__msg__Curve * msg)
{
  if (!msg) {
    return;
  }
  // x_start
  std_msgs__msg__Float32__fini(&msg->x_start);
  // x_end
  std_msgs__msg__Float32__fini(&msg->x_end);
  // a
  std_msgs__msg__Float32__fini(&msg->a);
  // b
  std_msgs__msg__Float32__fini(&msg->b);
  // c
  std_msgs__msg__Float32__fini(&msg->c);
  // d
  std_msgs__msg__Float32__fini(&msg->d);
}

perception_ros2_msg__msg__Curve *
perception_ros2_msg__msg__Curve__create()
{
  perception_ros2_msg__msg__Curve * msg = (perception_ros2_msg__msg__Curve *)malloc(sizeof(perception_ros2_msg__msg__Curve));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__Curve));
  bool success = perception_ros2_msg__msg__Curve__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__Curve__destroy(perception_ros2_msg__msg__Curve * msg)
{
  if (msg) {
    perception_ros2_msg__msg__Curve__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__Curve__Sequence__init(perception_ros2_msg__msg__Curve__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__Curve * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__Curve *)calloc(size, sizeof(perception_ros2_msg__msg__Curve));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__Curve__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__Curve__fini(&data[i - 1]);
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
perception_ros2_msg__msg__Curve__Sequence__fini(perception_ros2_msg__msg__Curve__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__Curve__fini(&array->data[i]);
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

perception_ros2_msg__msg__Curve__Sequence *
perception_ros2_msg__msg__Curve__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__Curve__Sequence * array = (perception_ros2_msg__msg__Curve__Sequence *)malloc(sizeof(perception_ros2_msg__msg__Curve__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__Curve__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__Curve__Sequence__destroy(perception_ros2_msg__msg__Curve__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__Curve__Sequence__fini(array);
  }
  free(array);
}
