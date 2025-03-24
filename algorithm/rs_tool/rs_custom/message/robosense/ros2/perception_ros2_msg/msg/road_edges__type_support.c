// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_ros2_msg:msg/RoadEdges.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_ros2_msg/msg/road_edges__rosidl_typesupport_introspection_c.h"
#include "perception_ros2_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_ros2_msg/msg/road_edges__functions.h"
#include "perception_ros2_msg/msg/road_edges__struct.h"


// Include directives for member types
// Member `curbs`
#include "perception_ros2_msg/msg/road_edge.h"
// Member `curbs`
#include "perception_ros2_msg/msg/road_edge__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_ros2_msg__msg__RoadEdges__init(message_memory);
}

void RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_fini_function(void * message_memory)
{
  perception_ros2_msg__msg__RoadEdges__fini(message_memory);
}

size_t RoadEdges__rosidl_typesupport_introspection_c__size_function__RoadEdge__curbs(
  const void * untyped_member)
{
  const perception_ros2_msg__msg__RoadEdge__Sequence * member =
    (const perception_ros2_msg__msg__RoadEdge__Sequence *)(untyped_member);
  return member->size;
}

const void * RoadEdges__rosidl_typesupport_introspection_c__get_const_function__RoadEdge__curbs(
  const void * untyped_member, size_t index)
{
  const perception_ros2_msg__msg__RoadEdge__Sequence * member =
    (const perception_ros2_msg__msg__RoadEdge__Sequence *)(untyped_member);
  return &member->data[index];
}

void * RoadEdges__rosidl_typesupport_introspection_c__get_function__RoadEdge__curbs(
  void * untyped_member, size_t index)
{
  perception_ros2_msg__msg__RoadEdge__Sequence * member =
    (perception_ros2_msg__msg__RoadEdge__Sequence *)(untyped_member);
  return &member->data[index];
}

bool RoadEdges__rosidl_typesupport_introspection_c__resize_function__RoadEdge__curbs(
  void * untyped_member, size_t size)
{
  perception_ros2_msg__msg__RoadEdge__Sequence * member =
    (perception_ros2_msg__msg__RoadEdge__Sequence *)(untyped_member);
  perception_ros2_msg__msg__RoadEdge__Sequence__fini(member);
  return perception_ros2_msg__msg__RoadEdge__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_message_member_array[1] = {
  {
    "curbs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__RoadEdges, curbs),  // bytes offset in struct
    NULL,  // default value
    RoadEdges__rosidl_typesupport_introspection_c__size_function__RoadEdge__curbs,  // size() function pointer
    RoadEdges__rosidl_typesupport_introspection_c__get_const_function__RoadEdge__curbs,  // get_const(index) function pointer
    RoadEdges__rosidl_typesupport_introspection_c__get_function__RoadEdge__curbs,  // get(index) function pointer
    RoadEdges__rosidl_typesupport_introspection_c__resize_function__RoadEdge__curbs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_message_members = {
  "perception_ros2_msg__msg",  // message namespace
  "RoadEdges",  // message name
  1,  // number of fields
  sizeof(perception_ros2_msg__msg__RoadEdges),
  RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_message_member_array,  // message members
  RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_init_function,  // function to initialize message memory (memory has to be allocated)
  RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_message_type_support_handle = {
  0,
  &RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_ros2_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, RoadEdges)() {
  RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, RoadEdge)();
  if (!RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_message_type_support_handle.typesupport_identifier) {
    RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &RoadEdges__rosidl_typesupport_introspection_c__RoadEdges_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
