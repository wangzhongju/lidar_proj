// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from perception_ros2_msg:msg/RoadEdges.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "perception_ros2_msg/msg/road_edges__struct.hpp"
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

void RoadEdges_init_function(
  void * message_memory, rosidl_generator_cpp::MessageInitialization _init)
{
  new (message_memory) perception_ros2_msg::msg::RoadEdges(_init);
}

void RoadEdges_fini_function(void * message_memory)
{
  auto typed_message = static_cast<perception_ros2_msg::msg::RoadEdges *>(message_memory);
  typed_message->~RoadEdges();
}

size_t size_function__RoadEdges__curbs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<perception_ros2_msg::msg::RoadEdge> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RoadEdges__curbs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<perception_ros2_msg::msg::RoadEdge> *>(untyped_member);
  return &member[index];
}

void * get_function__RoadEdges__curbs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<perception_ros2_msg::msg::RoadEdge> *>(untyped_member);
  return &member[index];
}

void resize_function__RoadEdges__curbs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<perception_ros2_msg::msg::RoadEdge> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RoadEdges_message_member_array[1] = {
  {
    "curbs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<perception_ros2_msg::msg::RoadEdge>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::RoadEdges, curbs),  // bytes offset in struct
    nullptr,  // default value
    size_function__RoadEdges__curbs,  // size() function pointer
    get_const_function__RoadEdges__curbs,  // get_const(index) function pointer
    get_function__RoadEdges__curbs,  // get(index) function pointer
    resize_function__RoadEdges__curbs  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RoadEdges_message_members = {
  "perception_ros2_msg::msg",  // message namespace
  "RoadEdges",  // message name
  1,  // number of fields
  sizeof(perception_ros2_msg::msg::RoadEdges),
  RoadEdges_message_member_array,  // message members
  RoadEdges_init_function,  // function to initialize message memory (memory has to be allocated)
  RoadEdges_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RoadEdges_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RoadEdges_message_members,
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
get_message_type_support_handle<perception_ros2_msg::msg::RoadEdges>()
{
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::RoadEdges_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, perception_ros2_msg, msg, RoadEdges)() {
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::RoadEdges_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
