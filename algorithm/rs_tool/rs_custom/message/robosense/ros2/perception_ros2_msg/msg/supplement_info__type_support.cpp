// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from perception_ros2_msg:msg/SupplementInfo.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "perception_ros2_msg/msg/supplement_info__struct.hpp"
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

void SupplementInfo_init_function(
  void * message_memory, rosidl_generator_cpp::MessageInitialization _init)
{
  new (message_memory) perception_ros2_msg::msg::SupplementInfo(_init);
}

void SupplementInfo_fini_function(void * message_memory)
{
  auto typed_message = static_cast<perception_ros2_msg::msg::SupplementInfo *>(message_memory);
  typed_message->~SupplementInfo();
}

size_t size_function__SupplementInfo__polygon(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SupplementInfo__polygon(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  return &member[index];
}

void * get_function__SupplementInfo__polygon(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  return &member[index];
}

void resize_function__SupplementInfo__polygon(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SupplementInfo__cloud_indices(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std_msgs::msg::Int32> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SupplementInfo__cloud_indices(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std_msgs::msg::Int32> *>(untyped_member);
  return &member[index];
}

void * get_function__SupplementInfo__cloud_indices(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std_msgs::msg::Int32> *>(untyped_member);
  return &member[index];
}

void resize_function__SupplementInfo__cloud_indices(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std_msgs::msg::Int32> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SupplementInfo__latent_types(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std_msgs::msg::Float32> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SupplementInfo__latent_types(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std_msgs::msg::Float32> *>(untyped_member);
  return &member[index];
}

void * get_function__SupplementInfo__latent_types(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std_msgs::msg::Float32> *>(untyped_member);
  return &member[index];
}

void resize_function__SupplementInfo__latent_types(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std_msgs::msg::Float32> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SupplementInfo__trajectory(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SupplementInfo__trajectory(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  return &member[index];
}

void * get_function__SupplementInfo__trajectory(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  return &member[index];
}

void resize_function__SupplementInfo__trajectory(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SupplementInfo__history_velocity(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SupplementInfo__history_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  return &member[index];
}

void * get_function__SupplementInfo__history_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  return &member[index];
}

void resize_function__SupplementInfo__history_velocity(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<perception_ros2_msg::msg::Point3f> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SupplementInfo__history_type(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std_msgs::msg::Int32> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SupplementInfo__history_type(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std_msgs::msg::Int32> *>(untyped_member);
  return &member[index];
}

void * get_function__SupplementInfo__history_type(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std_msgs::msg::Int32> *>(untyped_member);
  return &member[index];
}

void resize_function__SupplementInfo__history_type(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std_msgs::msg::Int32> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SupplementInfo_message_member_array[17] = {
  {
    "unique_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::UInt32>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, unique_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "polygon",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<perception_ros2_msg::msg::Point3f>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, polygon),  // bytes offset in struct
    nullptr,  // default value
    size_function__SupplementInfo__polygon,  // size() function pointer
    get_const_function__SupplementInfo__polygon,  // get_const(index) function pointer
    get_function__SupplementInfo__polygon,  // get(index) function pointer
    resize_function__SupplementInfo__polygon  // resize(index) function pointer
  },
  {
    "left_point_index",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Int32>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, left_point_index),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_point_index",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Int32>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, right_point_index),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cloud_indices",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Int32>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, cloud_indices),  // bytes offset in struct
    nullptr,  // default value
    size_function__SupplementInfo__cloud_indices,  // size() function pointer
    get_const_function__SupplementInfo__cloud_indices,  // get_const(index) function pointer
    get_function__SupplementInfo__cloud_indices,  // get(index) function pointer
    resize_function__SupplementInfo__cloud_indices  // resize(index) function pointer
  },
  {
    "latent_types",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Float32>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, latent_types),  // bytes offset in struct
    nullptr,  // default value
    size_function__SupplementInfo__latent_types,  // size() function pointer
    get_const_function__SupplementInfo__latent_types,  // get_const(index) function pointer
    get_function__SupplementInfo__latent_types,  // get(index) function pointer
    resize_function__SupplementInfo__latent_types  // resize(index) function pointer
  },
  {
    "size_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Int32>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, size_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Int32>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "in_roi",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Bool>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, in_roi),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tracking_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Int32>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, tracking_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "geo_center",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<perception_ros2_msg::msg::Point3f>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, geo_center),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "geo_size",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<perception_ros2_msg::msg::Point3f>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, geo_size),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "trajectory",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<perception_ros2_msg::msg::Point3f>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, trajectory),  // bytes offset in struct
    nullptr,  // default value
    size_function__SupplementInfo__trajectory,  // size() function pointer
    get_const_function__SupplementInfo__trajectory,  // get_const(index) function pointer
    get_function__SupplementInfo__trajectory,  // get(index) function pointer
    resize_function__SupplementInfo__trajectory  // resize(index) function pointer
  },
  {
    "history_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<perception_ros2_msg::msg::Point3f>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, history_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__SupplementInfo__history_velocity,  // size() function pointer
    get_const_function__SupplementInfo__history_velocity,  // get_const(index) function pointer
    get_function__SupplementInfo__history_velocity,  // get(index) function pointer
    resize_function__SupplementInfo__history_velocity  // resize(index) function pointer
  },
  {
    "history_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Int32>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, history_type),  // bytes offset in struct
    nullptr,  // default value
    size_function__SupplementInfo__history_type,  // size() function pointer
    get_const_function__SupplementInfo__history_type,  // get_const(index) function pointer
    get_function__SupplementInfo__history_type,  // get(index) function pointer
    resize_function__SupplementInfo__history_type  // resize(index) function pointer
  },
  {
    "gps_mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Int32>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, gps_mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gps_info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<perception_ros2_msg::msg::Point3d>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::SupplementInfo, gps_info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SupplementInfo_message_members = {
  "perception_ros2_msg::msg",  // message namespace
  "SupplementInfo",  // message name
  17,  // number of fields
  sizeof(perception_ros2_msg::msg::SupplementInfo),
  SupplementInfo_message_member_array,  // message members
  SupplementInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  SupplementInfo_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SupplementInfo_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SupplementInfo_message_members,
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
get_message_type_support_handle<perception_ros2_msg::msg::SupplementInfo>()
{
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::SupplementInfo_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, perception_ros2_msg, msg, SupplementInfo)() {
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::SupplementInfo_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
