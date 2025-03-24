// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_ros2_msg:msg/Point2f.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_ros2_msg/msg/point2f__rosidl_typesupport_introspection_c.h"
#include "perception_ros2_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_ros2_msg/msg/point2f__functions.h"
#include "perception_ros2_msg/msg/point2f__struct.h"


// Include directives for member types
// Member `x`
// Member `y`
#include "std_msgs/msg/float32.h"
// Member `x`
// Member `y`
#include "std_msgs/msg/float32__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Point2f__rosidl_typesupport_introspection_c__Point2f_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_ros2_msg__msg__Point2f__init(message_memory);
}

void Point2f__rosidl_typesupport_introspection_c__Point2f_fini_function(void * message_memory)
{
  perception_ros2_msg__msg__Point2f__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Point2f__rosidl_typesupport_introspection_c__Point2f_message_member_array[2] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__Point2f, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__Point2f, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Point2f__rosidl_typesupport_introspection_c__Point2f_message_members = {
  "perception_ros2_msg__msg",  // message namespace
  "Point2f",  // message name
  2,  // number of fields
  sizeof(perception_ros2_msg__msg__Point2f),
  Point2f__rosidl_typesupport_introspection_c__Point2f_message_member_array,  // message members
  Point2f__rosidl_typesupport_introspection_c__Point2f_init_function,  // function to initialize message memory (memory has to be allocated)
  Point2f__rosidl_typesupport_introspection_c__Point2f_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Point2f__rosidl_typesupport_introspection_c__Point2f_message_type_support_handle = {
  0,
  &Point2f__rosidl_typesupport_introspection_c__Point2f_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_ros2_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Point2f)() {
  Point2f__rosidl_typesupport_introspection_c__Point2f_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32)();
  Point2f__rosidl_typesupport_introspection_c__Point2f_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32)();
  if (!Point2f__rosidl_typesupport_introspection_c__Point2f_message_type_support_handle.typesupport_identifier) {
    Point2f__rosidl_typesupport_introspection_c__Point2f_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Point2f__rosidl_typesupport_introspection_c__Point2f_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
