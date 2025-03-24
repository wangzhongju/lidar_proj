// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from perception_ros2_msg:msg/Lanes.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "perception_ros2_msg/msg/lanes__struct.hpp"
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

void Lanes_init_function(
  void * message_memory, rosidl_generator_cpp::MessageInitialization _init)
{
  new (message_memory) perception_ros2_msg::msg::Lanes(_init);
}

void Lanes_fini_function(void * message_memory)
{
  auto typed_message = static_cast<perception_ros2_msg::msg::Lanes *>(message_memory);
  typed_message->~Lanes();
}

size_t size_function__Lanes__lanes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<perception_ros2_msg::msg::Lane> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Lanes__lanes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<perception_ros2_msg::msg::Lane> *>(untyped_member);
  return &member[index];
}

void * get_function__Lanes__lanes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<perception_ros2_msg::msg::Lane> *>(untyped_member);
  return &member[index];
}

void resize_function__Lanes__lanes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<perception_ros2_msg::msg::Lane> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Lanes_message_member_array[1] = {
  {
    "lanes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<perception_ros2_msg::msg::Lane>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::Lanes, lanes),  // bytes offset in struct
    nullptr,  // default value
    size_function__Lanes__lanes,  // size() function pointer
    get_const_function__Lanes__lanes,  // get_const(index) function pointer
    get_function__Lanes__lanes,  // get(index) function pointer
    resize_function__Lanes__lanes  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Lanes_message_members = {
  "perception_ros2_msg::msg",  // message namespace
  "Lanes",  // message name
  1,  // number of fields
  sizeof(perception_ros2_msg::msg::Lanes),
  Lanes_message_member_array,  // message members
  Lanes_init_function,  // function to initialize message memory (memory has to be allocated)
  Lanes_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Lanes_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Lanes_message_members,
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
get_message_type_support_handle<perception_ros2_msg::msg::Lanes>()
{
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::Lanes_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, perception_ros2_msg, msg, Lanes)() {
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::Lanes_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
