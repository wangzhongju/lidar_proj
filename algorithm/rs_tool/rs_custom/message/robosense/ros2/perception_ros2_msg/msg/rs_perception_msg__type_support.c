// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_ros2_msg:msg/RsPerceptionMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_ros2_msg/msg/rs_perception_msg__rosidl_typesupport_introspection_c.h"
#include "perception_ros2_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_ros2_msg/msg/rs_perception_msg__functions.h"
#include "perception_ros2_msg/msg/rs_perception_msg__struct.h"


// Include directives for member types
// Member `lidarframe`
#include "perception_ros2_msg/msg/lidar_frame_msg.h"
// Member `lidarframe`
#include "perception_ros2_msg/msg/lidar_frame_msg__rosidl_typesupport_introspection_c.h"
// Member `device_id`
#include "std_msgs/msg/int32.h"
// Member `device_id`
#include "std_msgs/msg/int32__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_ros2_msg__msg__RsPerceptionMsg__init(message_memory);
}

void RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_fini_function(void * message_memory)
{
  perception_ros2_msg__msg__RsPerceptionMsg__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_message_member_array[2] = {
  {
    "lidarframe",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__RsPerceptionMsg, lidarframe),  // bytes offset in struct
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
    offsetof(perception_ros2_msg__msg__RsPerceptionMsg, device_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_message_members = {
  "perception_ros2_msg__msg",  // message namespace
  "RsPerceptionMsg",  // message name
  2,  // number of fields
  sizeof(perception_ros2_msg__msg__RsPerceptionMsg),
  RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_message_member_array,  // message members
  RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_message_type_support_handle = {
  0,
  &RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_ros2_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, RsPerceptionMsg)() {
  RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, LidarFrameMsg)();
  RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  if (!RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_message_type_support_handle.typesupport_identifier) {
    RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &RsPerceptionMsg__rosidl_typesupport_introspection_c__RsPerceptionMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
