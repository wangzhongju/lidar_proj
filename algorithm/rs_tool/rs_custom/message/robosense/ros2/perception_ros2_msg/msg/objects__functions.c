// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/Objects.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/objects__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `timestamp`
#include "std_msgs/msg/float32__functions.h"
// Member `device_id`
#include "std_msgs/msg/int32__functions.h"
// Member `objects`
#include "perception_ros2_msg/msg/object__functions.h"

bool
perception_ros2_msg__msg__Objects__init(perception_ros2_msg__msg__Objects * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  if (!std_msgs__msg__Float32__init(&msg->timestamp)) {
    perception_ros2_msg__msg__Objects__fini(msg);
    return false;
  }
  // device_id
  if (!std_msgs__msg__Int32__init(&msg->device_id)) {
    perception_ros2_msg__msg__Objects__fini(msg);
    return false;
  }
  // objects
  if (!perception_ros2_msg__msg__Object__Sequence__init(&msg->objects, 0)) {
    perception_ros2_msg__msg__Objects__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__Objects__fini(perception_ros2_msg__msg__Objects * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  std_msgs__msg__Float32__fini(&msg->timestamp);
  // device_id
  std_msgs__msg__Int32__fini(&msg->device_id);
  // objects
  perception_ros2_msg__msg__Object__Sequence__fini(&msg->objects);
}

perception_ros2_msg__msg__Objects *
perception_ros2_msg__msg__Objects__create()
{
  perception_ros2_msg__msg__Objects * msg = (perception_ros2_msg__msg__Objects *)malloc(sizeof(perception_ros2_msg__msg__Objects));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__Objects));
  bool success = perception_ros2_msg__msg__Objects__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__Objects__destroy(perception_ros2_msg__msg__Objects * msg)
{
  if (msg) {
    perception_ros2_msg__msg__Objects__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__Objects__Sequence__init(perception_ros2_msg__msg__Objects__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__Objects * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__Objects *)calloc(size, sizeof(perception_ros2_msg__msg__Objects));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__Objects__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__Objects__fini(&data[i - 1]);
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
perception_ros2_msg__msg__Objects__Sequence__fini(perception_ros2_msg__msg__Objects__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__Objects__fini(&array->data[i]);
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

perception_ros2_msg__msg__Objects__Sequence *
perception_ros2_msg__msg__Objects__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__Objects__Sequence * array = (perception_ros2_msg__msg__Objects__Sequence *)malloc(sizeof(perception_ros2_msg__msg__Objects__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__Objects__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__Objects__Sequence__destroy(perception_ros2_msg__msg__Objects__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__Objects__Sequence__fini(array);
  }
  free(array);
}
