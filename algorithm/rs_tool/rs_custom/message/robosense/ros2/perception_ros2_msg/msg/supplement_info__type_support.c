// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_ros2_msg:msg/SupplementInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_ros2_msg/msg/supplement_info__rosidl_typesupport_introspection_c.h"
#include "perception_ros2_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_ros2_msg/msg/supplement_info__functions.h"
#include "perception_ros2_msg/msg/supplement_info__struct.h"


// Include directives for member types
// Member `unique_id`
#include "std_msgs/msg/u_int32.h"
// Member `unique_id`
#include "std_msgs/msg/u_int32__rosidl_typesupport_introspection_c.h"
// Member `polygon`
// Member `geo_center`
// Member `geo_size`
// Member `trajectory`
// Member `history_velocity`
#include "perception_ros2_msg/msg/point3f.h"
// Member `polygon`
// Member `geo_center`
// Member `geo_size`
// Member `trajectory`
// Member `history_velocity`
#include "perception_ros2_msg/msg/point3f__rosidl_typesupport_introspection_c.h"
// Member `left_point_index`
// Member `right_point_index`
// Member `cloud_indices`
// Member `size_type`
// Member `mode`
// Member `tracking_state`
// Member `history_type`
// Member `gps_mode`
#include "std_msgs/msg/int32.h"
// Member `left_point_index`
// Member `right_point_index`
// Member `cloud_indices`
// Member `size_type`
// Member `mode`
// Member `tracking_state`
// Member `history_type`
// Member `gps_mode`
#include "std_msgs/msg/int32__rosidl_typesupport_introspection_c.h"
// Member `latent_types`
#include "std_msgs/msg/float32.h"
// Member `latent_types`
#include "std_msgs/msg/float32__rosidl_typesupport_introspection_c.h"
// Member `in_roi`
#include "std_msgs/msg/bool.h"
// Member `in_roi`
#include "std_msgs/msg/bool__rosidl_typesupport_introspection_c.h"
// Member `gps_info`
#include "perception_ros2_msg/msg/point3d.h"
// Member `gps_info`
#include "perception_ros2_msg/msg/point3d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_ros2_msg__msg__SupplementInfo__init(message_memory);
}

void SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_fini_function(void * message_memory)
{
  perception_ros2_msg__msg__SupplementInfo__fini(message_memory);
}

size_t SupplementInfo__rosidl_typesupport_introspection_c__size_function__Point3f__polygon(
  const void * untyped_member)
{
  const perception_ros2_msg__msg__Point3f__Sequence * member =
    (const perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return member->size;
}

const void * SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Point3f__polygon(
  const void * untyped_member, size_t index)
{
  const perception_ros2_msg__msg__Point3f__Sequence * member =
    (const perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return &member->data[index];
}

void * SupplementInfo__rosidl_typesupport_introspection_c__get_function__Point3f__polygon(
  void * untyped_member, size_t index)
{
  perception_ros2_msg__msg__Point3f__Sequence * member =
    (perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return &member->data[index];
}

bool SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Point3f__polygon(
  void * untyped_member, size_t size)
{
  perception_ros2_msg__msg__Point3f__Sequence * member =
    (perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  perception_ros2_msg__msg__Point3f__Sequence__fini(member);
  return perception_ros2_msg__msg__Point3f__Sequence__init(member, size);
}

size_t SupplementInfo__rosidl_typesupport_introspection_c__size_function__Int32__cloud_indices(
  const void * untyped_member)
{
  const std_msgs__msg__Int32__Sequence * member =
    (const std_msgs__msg__Int32__Sequence *)(untyped_member);
  return member->size;
}

const void * SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Int32__cloud_indices(
  const void * untyped_member, size_t index)
{
  const std_msgs__msg__Int32__Sequence * member =
    (const std_msgs__msg__Int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * SupplementInfo__rosidl_typesupport_introspection_c__get_function__Int32__cloud_indices(
  void * untyped_member, size_t index)
{
  std_msgs__msg__Int32__Sequence * member =
    (std_msgs__msg__Int32__Sequence *)(untyped_member);
  return &member->data[index];
}

bool SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Int32__cloud_indices(
  void * untyped_member, size_t size)
{
  std_msgs__msg__Int32__Sequence * member =
    (std_msgs__msg__Int32__Sequence *)(untyped_member);
  std_msgs__msg__Int32__Sequence__fini(member);
  return std_msgs__msg__Int32__Sequence__init(member, size);
}

size_t SupplementInfo__rosidl_typesupport_introspection_c__size_function__Float32__latent_types(
  const void * untyped_member)
{
  const std_msgs__msg__Float32__Sequence * member =
    (const std_msgs__msg__Float32__Sequence *)(untyped_member);
  return member->size;
}

const void * SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Float32__latent_types(
  const void * untyped_member, size_t index)
{
  const std_msgs__msg__Float32__Sequence * member =
    (const std_msgs__msg__Float32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * SupplementInfo__rosidl_typesupport_introspection_c__get_function__Float32__latent_types(
  void * untyped_member, size_t index)
{
  std_msgs__msg__Float32__Sequence * member =
    (std_msgs__msg__Float32__Sequence *)(untyped_member);
  return &member->data[index];
}

bool SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Float32__latent_types(
  void * untyped_member, size_t size)
{
  std_msgs__msg__Float32__Sequence * member =
    (std_msgs__msg__Float32__Sequence *)(untyped_member);
  std_msgs__msg__Float32__Sequence__fini(member);
  return std_msgs__msg__Float32__Sequence__init(member, size);
}

size_t SupplementInfo__rosidl_typesupport_introspection_c__size_function__Point3f__trajectory(
  const void * untyped_member)
{
  const perception_ros2_msg__msg__Point3f__Sequence * member =
    (const perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return member->size;
}

const void * SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Point3f__trajectory(
  const void * untyped_member, size_t index)
{
  const perception_ros2_msg__msg__Point3f__Sequence * member =
    (const perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return &member->data[index];
}

void * SupplementInfo__rosidl_typesupport_introspection_c__get_function__Point3f__trajectory(
  void * untyped_member, size_t index)
{
  perception_ros2_msg__msg__Point3f__Sequence * member =
    (perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return &member->data[index];
}

bool SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Point3f__trajectory(
  void * untyped_member, size_t size)
{
  perception_ros2_msg__msg__Point3f__Sequence * member =
    (perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  perception_ros2_msg__msg__Point3f__Sequence__fini(member);
  return perception_ros2_msg__msg__Point3f__Sequence__init(member, size);
}

size_t SupplementInfo__rosidl_typesupport_introspection_c__size_function__Point3f__history_velocity(
  const void * untyped_member)
{
  const perception_ros2_msg__msg__Point3f__Sequence * member =
    (const perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return member->size;
}

const void * SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Point3f__history_velocity(
  const void * untyped_member, size_t index)
{
  const perception_ros2_msg__msg__Point3f__Sequence * member =
    (const perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return &member->data[index];
}

void * SupplementInfo__rosidl_typesupport_introspection_c__get_function__Point3f__history_velocity(
  void * untyped_member, size_t index)
{
  perception_ros2_msg__msg__Point3f__Sequence * member =
    (perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  return &member->data[index];
}

bool SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Point3f__history_velocity(
  void * untyped_member, size_t size)
{
  perception_ros2_msg__msg__Point3f__Sequence * member =
    (perception_ros2_msg__msg__Point3f__Sequence *)(untyped_member);
  perception_ros2_msg__msg__Point3f__Sequence__fini(member);
  return perception_ros2_msg__msg__Point3f__Sequence__init(member, size);
}

size_t SupplementInfo__rosidl_typesupport_introspection_c__size_function__Int32__history_type(
  const void * untyped_member)
{
  const std_msgs__msg__Int32__Sequence * member =
    (const std_msgs__msg__Int32__Sequence *)(untyped_member);
  return member->size;
}

const void * SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Int32__history_type(
  const void * untyped_member, size_t index)
{
  const std_msgs__msg__Int32__Sequence * member =
    (const std_msgs__msg__Int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * SupplementInfo__rosidl_typesupport_introspection_c__get_function__Int32__history_type(
  void * untyped_member, size_t index)
{
  std_msgs__msg__Int32__Sequence * member =
    (std_msgs__msg__Int32__Sequence *)(untyped_member);
  return &member->data[index];
}

bool SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Int32__history_type(
  void * untyped_member, size_t size)
{
  std_msgs__msg__Int32__Sequence * member =
    (std_msgs__msg__Int32__Sequence *)(untyped_member);
  std_msgs__msg__Int32__Sequence__fini(member);
  return std_msgs__msg__Int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[17] = {
  {
    "unique_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, unique_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "polygon",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, polygon),  // bytes offset in struct
    NULL,  // default value
    SupplementInfo__rosidl_typesupport_introspection_c__size_function__Point3f__polygon,  // size() function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Point3f__polygon,  // get_const(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_function__Point3f__polygon,  // get(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Point3f__polygon  // resize(index) function pointer
  },
  {
    "left_point_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, left_point_index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_point_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, right_point_index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cloud_indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, cloud_indices),  // bytes offset in struct
    NULL,  // default value
    SupplementInfo__rosidl_typesupport_introspection_c__size_function__Int32__cloud_indices,  // size() function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Int32__cloud_indices,  // get_const(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_function__Int32__cloud_indices,  // get(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Int32__cloud_indices  // resize(index) function pointer
  },
  {
    "latent_types",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, latent_types),  // bytes offset in struct
    NULL,  // default value
    SupplementInfo__rosidl_typesupport_introspection_c__size_function__Float32__latent_types,  // size() function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Float32__latent_types,  // get_const(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_function__Float32__latent_types,  // get(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Float32__latent_types  // resize(index) function pointer
  },
  {
    "size_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, size_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "in_roi",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, in_roi),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tracking_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, tracking_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "geo_center",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, geo_center),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "geo_size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, geo_size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trajectory",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, trajectory),  // bytes offset in struct
    NULL,  // default value
    SupplementInfo__rosidl_typesupport_introspection_c__size_function__Point3f__trajectory,  // size() function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Point3f__trajectory,  // get_const(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_function__Point3f__trajectory,  // get(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Point3f__trajectory  // resize(index) function pointer
  },
  {
    "history_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, history_velocity),  // bytes offset in struct
    NULL,  // default value
    SupplementInfo__rosidl_typesupport_introspection_c__size_function__Point3f__history_velocity,  // size() function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Point3f__history_velocity,  // get_const(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_function__Point3f__history_velocity,  // get(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Point3f__history_velocity  // resize(index) function pointer
  },
  {
    "history_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, history_type),  // bytes offset in struct
    NULL,  // default value
    SupplementInfo__rosidl_typesupport_introspection_c__size_function__Int32__history_type,  // size() function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_const_function__Int32__history_type,  // get_const(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__get_function__Int32__history_type,  // get(index) function pointer
    SupplementInfo__rosidl_typesupport_introspection_c__resize_function__Int32__history_type  // resize(index) function pointer
  },
  {
    "gps_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, gps_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gps_info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__SupplementInfo, gps_info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_members = {
  "perception_ros2_msg__msg",  // message namespace
  "SupplementInfo",  // message name
  17,  // number of fields
  sizeof(perception_ros2_msg__msg__SupplementInfo),
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array,  // message members
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_type_support_handle = {
  0,
  &SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_ros2_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, SupplementInfo)() {
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, UInt32)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Point3f)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Bool)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[9].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[10].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Point3f)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[11].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Point3f)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[12].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Point3f)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[13].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Point3f)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[14].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[15].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_member_array[16].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Point3d)();
  if (!SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_type_support_handle.typesupport_identifier) {
    SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SupplementInfo__rosidl_typesupport_introspection_c__SupplementInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
