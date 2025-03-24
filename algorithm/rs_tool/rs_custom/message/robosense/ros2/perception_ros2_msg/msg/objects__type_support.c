// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_ros2_msg:msg/Objects.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_ros2_msg/msg/objects__rosidl_typesupport_introspection_c.h"
#include "perception_ros2_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_ros2_msg/msg/objects__functions.h"
#include "perception_ros2_msg/msg/objects__struct.h"


// Include directives for member types
// Member `timestamp`
#include "std_msgs/msg/float32.h"
// Member `timestamp`
#include "std_msgs/msg/float32__rosidl_typesupport_introspection_c.h"
// Member `device_id`
#include "std_msgs/msg/int32.h"
// Member `device_id`
#include "std_msgs/msg/int32__rosidl_typesupport_introspection_c.h"
// Member `objects`
#include "perception_ros2_msg/msg/object.h"
// Member `objects`
#include "perception_ros2_msg/msg/object__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Objects__rosidl_typesupport_introspection_c__Objects_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_ros2_msg__msg__Objects__init(message_memory);
}

void Objects__rosidl_typesupport_introspection_c__Objects_fini_function(void * message_memory)
{
  perception_ros2_msg__msg__Objects__fini(message_memory);
}

size_t Objects__rosidl_typesupport_introspection_c__size_function__Object__objects(
  const void * untyped_member)
{
  const perception_ros2_msg__msg__Object__Sequence * member =
    (const perception_ros2_msg__msg__Object__Sequence *)(untyped_member);
  return member->size;
}

const void * Objects__rosidl_typesupport_introspection_c__get_const_function__Object__objects(
  const void * untyped_member, size_t index)
{
  const perception_ros2_msg__msg__Object__Sequence * member =
    (const perception_ros2_msg__msg__Object__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Objects__rosidl_typesupport_introspection_c__get_function__Object__objects(
  void * untyped_member, size_t index)
{
  perception_ros2_msg__msg__Object__Sequence * member =
    (perception_ros2_msg__msg__Object__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Objects__rosidl_typesupport_introspection_c__resize_function__Object__objects(
  void * untyped_member, size_t size)
{
  perception_ros2_msg__msg__Object__Sequence * member =
    (perception_ros2_msg__msg__Object__Sequence *)(untyped_member);
  perception_ros2_msg__msg__Object__Sequence__fini(member);
  return perception_ros2_msg__msg__Object__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Objects__rosidl_typesupport_introspection_c__Objects_message_member_array[3] = {
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__Objects, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "device_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__Objects, device_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__Objects, objects),  // bytes offset in struct
    NULL,  // default value
    Objects__rosidl_typesupport_introspection_c__size_function__Object__objects,  // size() function pointer
    Objects__rosidl_typesupport_introspection_c__get_const_function__Object__objects,  // get_const(index) function pointer
    Objects__rosidl_typesupport_introspection_c__get_function__Object__objects,  // get(index) function pointer
    Objects__rosidl_typesupport_introspection_c__resize_function__Object__objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Objects__rosidl_typesupport_introspection_c__Objects_message_members = {
  "perception_ros2_msg__msg",  // message namespace
  "Objects",  // message name
  3,  // number of fields
  sizeof(perception_ros2_msg__msg__Objects),
  Objects__rosidl_typesupport_introspection_c__Objects_message_member_array,  // message members
  Objects__rosidl_typesupport_introspection_c__Objects_init_function,  // function to initialize message memory (memory has to be allocated)
  Objects__rosidl_typesupport_introspection_c__Objects_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Objects__rosidl_typesupport_introspection_c__Objects_message_type_support_handle = {
  0,
  &Objects__rosidl_typesupport_introspection_c__Objects_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_ros2_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Objects)() {
  Objects__rosidl_typesupport_introspection_c__Objects_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32)();
  Objects__rosidl_typesupport_introspection_c__Objects_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  Objects__rosidl_typesupport_introspection_c__Objects_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Object)();
  if (!Objects__rosidl_typesupport_introspection_c__Objects_message_type_support_handle.typesupport_identifier) {
    Objects__rosidl_typesupport_introspection_c__Objects_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Objects__rosidl_typesupport_introspection_c__Objects_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
