// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_ros2_msg:msg/Lanes.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_ros2_msg/msg/lanes__rosidl_typesupport_introspection_c.h"
#include "perception_ros2_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_ros2_msg/msg/lanes__functions.h"
#include "perception_ros2_msg/msg/lanes__struct.h"


// Include directives for member types
// Member `lanes`
#include "perception_ros2_msg/msg/lane.h"
// Member `lanes`
#include "perception_ros2_msg/msg/lane__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Lanes__rosidl_typesupport_introspection_c__Lanes_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_ros2_msg__msg__Lanes__init(message_memory);
}

void Lanes__rosidl_typesupport_introspection_c__Lanes_fini_function(void * message_memory)
{
  perception_ros2_msg__msg__Lanes__fini(message_memory);
}

size_t Lanes__rosidl_typesupport_introspection_c__size_function__Lane__lanes(
  const void * untyped_member)
{
  const perception_ros2_msg__msg__Lane__Sequence * member =
    (const perception_ros2_msg__msg__Lane__Sequence *)(untyped_member);
  return member->size;
}

const void * Lanes__rosidl_typesupport_introspection_c__get_const_function__Lane__lanes(
  const void * untyped_member, size_t index)
{
  const perception_ros2_msg__msg__Lane__Sequence * member =
    (const perception_ros2_msg__msg__Lane__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Lanes__rosidl_typesupport_introspection_c__get_function__Lane__lanes(
  void * untyped_member, size_t index)
{
  perception_ros2_msg__msg__Lane__Sequence * member =
    (perception_ros2_msg__msg__Lane__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Lanes__rosidl_typesupport_introspection_c__resize_function__Lane__lanes(
  void * untyped_member, size_t size)
{
  perception_ros2_msg__msg__Lane__Sequence * member =
    (perception_ros2_msg__msg__Lane__Sequence *)(untyped_member);
  perception_ros2_msg__msg__Lane__Sequence__fini(member);
  return perception_ros2_msg__msg__Lane__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Lanes__rosidl_typesupport_introspection_c__Lanes_message_member_array[1] = {
  {
    "lanes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__Lanes, lanes),  // bytes offset in struct
    NULL,  // default value
    Lanes__rosidl_typesupport_introspection_c__size_function__Lane__lanes,  // size() function pointer
    Lanes__rosidl_typesupport_introspection_c__get_const_function__Lane__lanes,  // get_const(index) function pointer
    Lanes__rosidl_typesupport_introspection_c__get_function__Lane__lanes,  // get(index) function pointer
    Lanes__rosidl_typesupport_introspection_c__resize_function__Lane__lanes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Lanes__rosidl_typesupport_introspection_c__Lanes_message_members = {
  "perception_ros2_msg__msg",  // message namespace
  "Lanes",  // message name
  1,  // number of fields
  sizeof(perception_ros2_msg__msg__Lanes),
  Lanes__rosidl_typesupport_introspection_c__Lanes_message_member_array,  // message members
  Lanes__rosidl_typesupport_introspection_c__Lanes_init_function,  // function to initialize message memory (memory has to be allocated)
  Lanes__rosidl_typesupport_introspection_c__Lanes_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Lanes__rosidl_typesupport_introspection_c__Lanes_message_type_support_handle = {
  0,
  &Lanes__rosidl_typesupport_introspection_c__Lanes_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_ros2_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Lanes)() {
  Lanes__rosidl_typesupport_introspection_c__Lanes_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Lane)();
  if (!Lanes__rosidl_typesupport_introspection_c__Lanes_message_type_support_handle.typesupport_identifier) {
    Lanes__rosidl_typesupport_introspection_c__Lanes_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Lanes__rosidl_typesupport_introspection_c__Lanes_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
