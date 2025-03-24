// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from perception_ros2_msg:msg/Indices.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "perception_ros2_msg/msg/indices__struct.hpp"
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

void Indices_init_function(
  void * message_memory, rosidl_generator_cpp::MessageInitialization _init)
{
  new (message_memory) perception_ros2_msg::msg::Indices(_init);
}

void Indices_fini_function(void * message_memory)
{
  auto typed_message = static_cast<perception_ros2_msg::msg::Indices *>(message_memory);
  typed_message->~Indices();
}

size_t size_function__Indices__indices(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std_msgs::msg::Int32> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Indices__indices(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std_msgs::msg::Int32> *>(untyped_member);
  return &member[index];
}

void * get_function__Indices__indices(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std_msgs::msg::Int32> *>(untyped_member);
  return &member[index];
}

void resize_function__Indices__indices(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std_msgs::msg::Int32> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Indices_message_member_array[1] = {
  {
    "indices",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Int32>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::Indices, indices),  // bytes offset in struct
    nullptr,  // default value
    size_function__Indices__indices,  // size() function pointer
    get_const_function__Indices__indices,  // get_const(index) function pointer
    get_function__Indices__indices,  // get(index) function pointer
    resize_function__Indices__indices  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Indices_message_members = {
  "perception_ros2_msg::msg",  // message namespace
  "Indices",  // message name
  1,  // number of fields
  sizeof(perception_ros2_msg::msg::Indices),
  Indices_message_member_array,  // message members
  Indices_init_function,  // function to initialize message memory (memory has to be allocated)
  Indices_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Indices_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Indices_message_members,
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
get_message_type_support_handle<perception_ros2_msg::msg::Indices>()
{
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::Indices_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, perception_ros2_msg, msg, Indices)() {
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::Indices_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
