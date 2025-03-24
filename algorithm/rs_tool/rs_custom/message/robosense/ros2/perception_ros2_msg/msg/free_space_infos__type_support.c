// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_ros2_msg:msg/FreeSpaceInfos.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_ros2_msg/msg/free_space_infos__rosidl_typesupport_introspection_c.h"
#include "perception_ros2_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_ros2_msg/msg/free_space_infos__functions.h"
#include "perception_ros2_msg/msg/free_space_infos__struct.h"


// Include directives for member types
// Member `fs_pts`
#include "perception_ros2_msg/msg/point3f.h"
// Member `fs_pts`
#include "perception_ros2_msg/msg/point3f__rosidl_typesupport_introspection_c.h"
// Member `fs_confidence`
#include "std_msgs/msg/float32.h"
// Member `fs_confidence`
#include "std_msgs/msg/float32__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_ros2_msg__msg__FreeSpaceInfos__init(message_memory);
}

void FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_fini_function(void * message_memory)
{
  perception_ros2_msg__msg__FreeSpaceInfos__fini(message_memory);
}

size_t FreeSpaceInfos__rosidl_typesupport_introspection_c__size_function__Point3f__fs_pts(
  const void * untyped_member)
{
  const perception_ros2_msg__msg__Point3f__Sequence * member =
    (const perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return member->size;
}

const void * FreeSpaceInfos__rosidl_typesupport_introspection_c__get_const_function__Point3f__fs_pts(
  const void * untyped_member, size_t index)
{
  const perception_ros2_msg__msg__Point3f__Sequence * member =
    (const perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return &member->data[index];
}

void * FreeSpaceInfos__rosidl_typesupport_introspection_c__get_function__Point3f__fs_pts(
  void * untyped_member, size_t index)
{
  perception_ros2_msg__msg__Point3f__Sequence * member =
    (perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return &member->data[index];
}

bool FreeSpaceInfos__rosidl_typesupport_introspection_c__resize_function__Point3f__fs_pts(
  void * untyped_member, size_t size)
{
  perception_ros2_msg__msg__Point3f__Sequence * member =
    (perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  perception_ros2_msg__msg__Point3f__Sequence__fini(member);
  return perception_ros2_msg__msg__Point3f__Sequence__init(member, size);
}

size_t FreeSpaceInfos__rosidl_typesupport_introspection_c__size_function__Float32__fs_confidence(
  const void * untyped_member)
{
  const std_msgs__msg__Float32__Sequence * member =
    (const std_msgs__msg__Float32__Sequence *)(untyped_member);
  return member->size;
}

const void * FreeSpaceInfos__rosidl_typesupport_introspection_c__get_const_function__Float32__fs_confidence(
  const void * untyped_member, size_t index)
{
  const std_msgs__msg__Float32__Sequence * member =
    (const std_msgs__msg__Float32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * FreeSpaceInfos__rosidl_typesupport_introspection_c__get_function__Float32__fs_confidence(
  void * untyped_member, size_t index)
{
  std_msgs__msg__Float32__Sequence * member =
    (std_msgs__msg__Float32__Sequence *)(untyped_member);
  return &member->data[index];
}

bool FreeSpaceInfos__rosidl_typesupport_introspection_c__resize_function__Float32__fs_confidence(
  void * untyped_member, size_t size)
{
  std_msgs__msg__Float32__Sequence * member =
    (std_msgs__msg__Float32__Sequence *)(untyped_member);
  std_msgs__msg__Float32__Sequence__fini(member);
  return std_msgs__msg__Float32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_message_member_array[2] = {
  {
    "fs_pts",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__FreeSpaceInfos, fs_pts),  // bytes offset in struct
    NULL,  // default value
    FreeSpaceInfos__rosidl_typesupport_introspection_c__size_function__Point3f__fs_pts,  // size() function pointer
    FreeSpaceInfos__rosidl_typesupport_introspection_c__get_const_function__Point3f__fs_pts,  // get_const(index) function pointer
    FreeSpaceInfos__rosidl_typesupport_introspection_c__get_function__Point3f__fs_pts,  // get(index) function pointer
    FreeSpaceInfos__rosidl_typesupport_introspection_c__resize_function__Point3f__fs_pts  // resize(index) function pointer
  },
  {
    "fs_confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__FreeSpaceInfos, fs_confidence),  // bytes offset in struct
    NULL,  // default value
    FreeSpaceInfos__rosidl_typesupport_introspection_c__size_function__Float32__fs_confidence,  // size() function pointer
    FreeSpaceInfos__rosidl_typesupport_introspection_c__get_const_function__Float32__fs_confidence,  // get_const(index) function pointer
    FreeSpaceInfos__rosidl_typesupport_introspection_c__get_function__Float32__fs_confidence,  // get(index) function pointer
    FreeSpaceInfos__rosidl_typesupport_introspection_c__resize_function__Float32__fs_confidence  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_message_members = {
  "perception_ros2_msg__msg",  // message namespace
  "FreeSpaceInfos",  // message name
  2,  // number of fields
  sizeof(perception_ros2_msg__msg__FreeSpaceInfos),
  FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_message_member_array,  // message members
  FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_init_function,  // function to initialize message memory (memory has to be allocated)
  FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_message_type_support_handle = {
  0,
  &FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_ros2_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, FreeSpaceInfos)() {
  FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Point3f)();
  FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32)();
  if (!FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_message_type_support_handle.typesupport_identifier) {
    FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &FreeSpaceInfos__rosidl_typesupport_introspection_c__FreeSpaceInfos_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
