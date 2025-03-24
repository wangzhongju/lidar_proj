// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_ros2_msg:msg/RoadEdge.idl
// generated code does not contain a copyright notice
#include "perception_ros2_msg/msg/road_edge__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `roadedge_id`
// Member `measure_status`
#include "std_msgs/msg/int32__functions.h"
// Member `curve`
#include "perception_ros2_msg/msg/curve__functions.h"
// Member `end_points`
#include "perception_ros2_msg/msg/end_points__functions.h"
// Member `confidence`
#include "std_msgs/msg/float32__functions.h"

bool
perception_ros2_msg__msg__RoadEdge__init(perception_ros2_msg__msg__RoadEdge * msg)
{
  if (!msg) {
    return false;
  }
  // roadedge_id
  if (!std_msgs__msg__Int32__init(&msg->roadedge_id)) {
    perception_ros2_msg__msg__RoadEdge__fini(msg);
    return false;
  }
  // curve
  if (!perception_ros2_msg__msg__Curve__init(&msg->curve)) {
    perception_ros2_msg__msg__RoadEdge__fini(msg);
    return false;
  }
  // end_points
  if (!perception_ros2_msg__msg__EndPoints__init(&msg->end_points)) {
    perception_ros2_msg__msg__RoadEdge__fini(msg);
    return false;
  }
  // measure_status
  if (!std_msgs__msg__Int32__init(&msg->measure_status)) {
    perception_ros2_msg__msg__RoadEdge__fini(msg);
    return false;
  }
  // confidence
  if (!std_msgs__msg__Float32__init(&msg->confidence)) {
    perception_ros2_msg__msg__RoadEdge__fini(msg);
    return false;
  }
  return true;
}

void
perception_ros2_msg__msg__RoadEdge__fini(perception_ros2_msg__msg__RoadEdge * msg)
{
  if (!msg) {
    return;
  }
  // roadedge_id
  std_msgs__msg__Int32__fini(&msg->roadedge_id);
  // curve
  perception_ros2_msg__msg__Curve__fini(&msg->curve);
  // end_points
  perception_ros2_msg__msg__EndPoints__fini(&msg->end_points);
  // measure_status
  std_msgs__msg__Int32__fini(&msg->measure_status);
  // confidence
  std_msgs__msg__Float32__fini(&msg->confidence);
}

perception_ros2_msg__msg__RoadEdge *
perception_ros2_msg__msg__RoadEdge__create()
{
  perception_ros2_msg__msg__RoadEdge * msg = (perception_ros2_msg__msg__RoadEdge *)malloc(sizeof(perception_ros2_msg__msg__RoadEdge));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_ros2_msg__msg__RoadEdge));
  bool success = perception_ros2_msg__msg__RoadEdge__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
perception_ros2_msg__msg__RoadEdge__destroy(perception_ros2_msg__msg__RoadEdge * msg)
{
  if (msg) {
    perception_ros2_msg__msg__RoadEdge__fini(msg);
  }
  free(msg);
}


bool
perception_ros2_msg__msg__RoadEdge__Sequence__init(perception_ros2_msg__msg__RoadEdge__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  perception_ros2_msg__msg__RoadEdge * data = NULL;
  if (size) {
    data = (perception_ros2_msg__msg__RoadEdge *)calloc(size, sizeof(perception_ros2_msg__msg__RoadEdge));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_ros2_msg__msg__RoadEdge__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_ros2_msg__msg__RoadEdge__fini(&data[i - 1]);
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
perception_ros2_msg__msg__RoadEdge__Sequence__fini(perception_ros2_msg__msg__RoadEdge__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      perception_ros2_msg__msg__RoadEdge__fini(&array->data[i]);
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

perception_ros2_msg__msg__RoadEdge__Sequence *
perception_ros2_msg__msg__RoadEdge__Sequence__create(size_t size)
{
  perception_ros2_msg__msg__RoadEdge__Sequence * array = (perception_ros2_msg__msg__RoadEdge__Sequence *)malloc(sizeof(perception_ros2_msg__msg__RoadEdge__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = perception_ros2_msg__msg__RoadEdge__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
perception_ros2_msg__msg__RoadEdge__Sequence__destroy(perception_ros2_msg__msg__RoadEdge__Sequence * array)
{
  if (array) {
    perception_ros2_msg__msg__RoadEdge__Sequence__fini(array);
  }
  free(array);
}
