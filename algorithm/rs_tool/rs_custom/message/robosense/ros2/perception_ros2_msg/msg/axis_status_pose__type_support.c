// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_ros2_msg:msg/AxisStatusPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_ros2_msg/msg/axis_status_pose__rosidl_typesupport_introspection_c.h"
#include "perception_ros2_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_ros2_msg/msg/axis_status_pose__functions.h"
#include "perception_ros2_msg/msg/axis_status_pose__struct.h"


// Include directives for member types
// Member `status`
#include "std_msgs/msg/int32.h"
// Member `status`
#include "std_msgs/msg/int32__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "perception_ros2_msg/msg/pose.h"
// Member `pose`
#include "perception_ros2_msg/msg/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_ros2_msg__msg__AxisStatusPose__init(message_memory);
}

void AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_fini_function(void * message_memory)
{
  perception_ros2_msg__msg__AxisStatusPose__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__AxisStatusPose, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__AxisStatusPose, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_message_members = {
  "perception_ros2_msg__msg",  // message namespace
  "AxisStatusPose",  // message name
  2,  // number of fields
  sizeof(perception_ros2_msg__msg__AxisStatusPose),
  AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_message_member_array,  // message members
  AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_init_function,  // function to initialize message memory (memory has to be allocated)
  AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_message_type_support_handle = {
  0,
  &AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_ros2_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, AxisStatusPose)() {
  AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Pose)();
  if (!AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_message_type_support_handle.typesupport_identifier) {
    AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &AxisStatusPose__rosidl_typesupport_introspection_c__AxisStatusPose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
