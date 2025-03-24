// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from perception_ros2_msg:msg/Matrix3f.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "perception_ros2_msg/msg/matrix3f__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace perception_ros2_msg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Matrix3f_init_function(
  void * message_memory, rosidl_generator_cpp::MessageInitialization _init)
{
  new (message_memory) perception_ros2_msg::msg::Matrix3f(_init);
}

void Matrix3f_fini_function(void * message_memory)
{
  auto typed_message = static_cast<perception_ros2_msg::msg::Matrix3f *>(message_memory);
  typed_message->~Matrix3f();
}

size_t size_function__Matrix3f__matrix3x3(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std_msgs::msg::Float32> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Matrix3f__matrix3x3(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std_msgs::msg::Float32> *>(untyped_member);
  return &member[index];
}

void * get_function__Matrix3f__matrix3x3(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std_msgs::msg::Float32> *>(untyped_member);
  return &member[index];
}

void resize_function__Matrix3f__matrix3x3(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std_msgs::msg::Float32> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Matrix3f_message_member_array[1] = {
  {
    "matrix3x3",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Float32>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::Matrix3f, matrix3x3),  // bytes offset in struct
    nullptr,  // default value
    size_function__Matrix3f__matrix3x3,  // size() function pointer
    get_const_function__Matrix3f__matrix3x3,  // get_const(index) function pointer
    get_function__Matrix3f__matrix3x3,  // get(index) function pointer
    resize_function__Matrix3f__matrix3x3  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Matrix3f_message_members = {
  "perception_ros2_msg::msg",  // message namespace
  "Matrix3f",  // message name
  1,  // number of fields
  sizeof(perception_ros2_msg::msg::Matrix3f),
  Matrix3f_message_member_array,  // message members
  Matrix3f_init_function,  // function to initialize message memory (memory has to be allocated)
  Matrix3f_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Matrix3f_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Matrix3f_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace perception_ros2_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<perception_ros2_msg::msg::Matrix3f>()
{
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::Matrix3f_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, perception_ros2_msg, msg, Matrix3f)() {
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::Matrix3f_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
