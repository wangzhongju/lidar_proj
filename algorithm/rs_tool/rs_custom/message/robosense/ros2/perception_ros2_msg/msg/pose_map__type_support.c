// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_ros2_msg:msg/PoseMap.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_ros2_msg/msg/pose_map__rosidl_typesupport_introspection_c.h"
#include "perception_ros2_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_ros2_msg/msg/pose_map__functions.h"
#include "perception_ros2_msg/msg/pose_map__struct.h"


// Include directives for member types
// Member `status_poses`
#include "perception_ros2_msg/msg/axis_status_pose.h"
// Member `status_poses`
#include "perception_ros2_msg/msg/axis_status_pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PoseMap__rosidl_typesupport_introspection_c__PoseMap_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_ros2_msg__msg__PoseMap__init(message_memory);
}

void PoseMap__rosidl_typesupport_introspection_c__PoseMap_fini_function(void * message_memory)
{
  perception_ros2_msg__msg__PoseMap__fini(message_memory);
}

size_t PoseMap__rosidl_typesupport_introspection_c__size_function__AxisStatusPose__status_poses(
  const void * untyped_member)
{
  const perception_ros2_msg__msg__AxisStatusPose__Sequence * member =
    (const perception_ros2_msg__msg__AxisStatusPose__Sequence *)(untyped_member);
  return member->size;
}

const void * PoseMap__rosidl_typesupport_introspection_c__get_const_function__AxisStatusPose__status_poses(
  const void * untyped_member, size_t index)
{
  const perception_ros2_msg__msg__AxisStatusPose__Sequence * member =
    (const perception_ros2_msg__msg__AxisStatusPose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * PoseMap__rosidl_typesupport_introspection_c__get_function__AxisStatusPose__status_poses(
  void * untyped_member, size_t index)
{
  perception_ros2_msg__msg__AxisStatusPose__Sequence * member =
    (perception_ros2_msg__msg__AxisStatusPose__Sequence *)(untyped_member);
  return &member->data[index];
}

bool PoseMap__rosidl_typesupport_introspection_c__resize_function__AxisStatusPose__status_poses(
  void * untyped_member, size_t size)
{
  perception_ros2_msg__msg__AxisStatusPose__Sequence * member =
    (perception_ros2_msg__msg__AxisStatusPose__Sequence *)(untyped_member);
  perception_ros2_msg__msg__AxisStatusPose__Sequence__fini(member);
  return perception_ros2_msg__msg__AxisStatusPose__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember PoseMap__rosidl_typesupport_introspection_c__PoseMap_message_member_array[1] = {
  {
    "status_poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__PoseMap, status_poses),  // bytes offset in struct
    NULL,  // default value
    PoseMap__rosidl_typesupport_introspection_c__size_function__AxisStatusPose__status_poses,  // size() function pointer
    PoseMap__rosidl_typesupport_introspection_c__get_const_function__AxisStatusPose__status_poses,  // get_const(index) function pointer
    PoseMap__rosidl_typesupport_introspection_c__get_function__AxisStatusPose__status_poses,  // get(index) function pointer
    PoseMap__rosidl_typesupport_introspection_c__resize_function__AxisStatusPose__status_poses  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PoseMap__rosidl_typesupport_introspection_c__PoseMap_message_members = {
  "perception_ros2_msg__msg",  // message namespace
  "PoseMap",  // message name
  1,  // number of fields
  sizeof(perception_ros2_msg__msg__PoseMap),
  PoseMap__rosidl_typesupport_introspection_c__PoseMap_message_member_array,  // message members
  PoseMap__rosidl_typesupport_introspection_c__PoseMap_init_function,  // function to initialize message memory (memory has to be allocated)
  PoseMap__rosidl_typesupport_introspection_c__PoseMap_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PoseMap__rosidl_typesupport_introspection_c__PoseMap_message_type_support_handle = {
  0,
  &PoseMap__rosidl_typesupport_introspection_c__PoseMap_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_ros2_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, PoseMap)() {
  PoseMap__rosidl_typesupport_introspection_c__PoseMap_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, AxisStatusPose)();
  if (!PoseMap__rosidl_typesupport_introspection_c__PoseMap_message_type_support_handle.typesupport_identifier) {
    PoseMap__rosidl_typesupport_introspection_c__PoseMap_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PoseMap__rosidl_typesupport_introspection_c__PoseMap_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
