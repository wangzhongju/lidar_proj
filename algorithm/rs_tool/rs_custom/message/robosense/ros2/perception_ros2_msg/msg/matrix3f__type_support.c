// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_ros2_msg:msg/Matrix3f.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_ros2_msg/msg/matrix3f__rosidl_typesupport_introspection_c.h"
#include "perception_ros2_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_ros2_msg/msg/matrix3f__functions.h"
#include "perception_ros2_msg/msg/matrix3f__struct.h"


// Include directives for member types
// Member `matrix3x3`
#include "std_msgs/msg/float32.h"
// Member `matrix3x3`
#include "std_msgs/msg/float32__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_ros2_msg__msg__Matrix3f__init(message_memory);
}

void Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_fini_function(void * message_memory)
{
  perception_ros2_msg__msg__Matrix3f__fini(message_memory);
}

size_t Matrix3f__rosidl_typesupport_introspection_c__size_function__Float32__matrix3x3(
  const void * untyped_member)
{
  const std_msgs__msg__Float32__Sequence * member =
    (const std_msgs__msg__Float32__Sequence *)(untyped_member);
  return member->size;
}

const void * Matrix3f__rosidl_typesupport_introspection_c__get_const_function__Float32__matrix3x3(
  const void * untyped_member, size_t index)
{
  const std_msgs__msg__Float32__Sequence * member =
    (const std_msgs__msg__Float32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Matrix3f__rosidl_typesupport_introspection_c__get_function__Float32__matrix3x3(
  void * untyped_member, size_t index)
{
  std_msgs__msg__Float32__Sequence * member =
    (std_msgs__msg__Float32__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Matrix3f__rosidl_typesupport_introspection_c__resize_function__Float32__matrix3x3(
  void * untyped_member, size_t size)
{
  std_msgs__msg__Float32__Sequence * member =
    (std_msgs__msg__Float32__Sequence *)(untyped_member);
  std_msgs__msg__Float32__Sequence__fini(member);
  return std_msgs__msg__Float32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_message_member_array[1] = {
  {
    "matrix3x3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__Matrix3f, matrix3x3),  // bytes offset in struct
    NULL,  // default value
    Matrix3f__rosidl_typesupport_introspection_c__size_function__Float32__matrix3x3,  // size() function pointer
    Matrix3f__rosidl_typesupport_introspection_c__get_const_function__Float32__matrix3x3,  // get_const(index) function pointer
    Matrix3f__rosidl_typesupport_introspection_c__get_function__Float32__matrix3x3,  // get(index) function pointer
    Matrix3f__rosidl_typesupport_introspection_c__resize_function__Float32__matrix3x3  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_message_members = {
  "perception_ros2_msg__msg",  // message namespace
  "Matrix3f",  // message name
  1,  // number of fields
  sizeof(perception_ros2_msg__msg__Matrix3f),
  Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_message_member_array,  // message members
  Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_init_function,  // function to initialize message memory (memory has to be allocated)
  Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_message_type_support_handle = {
  0,
  &Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_ros2_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Matrix3f)() {
  Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32)();
  if (!Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_message_type_support_handle.typesupport_identifier) {
    Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Matrix3f__rosidl_typesupport_introspection_c__Matrix3f_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
