// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from perception_ros2_msg:msg/RsPerceptionMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "perception_ros2_msg/msg/rs_perception_msg__struct.hpp"
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

void RsPerceptionMsg_init_function(
  void * message_memory, rosidl_generator_cpp::MessageInitialization _init)
{
  new (message_memory) perception_ros2_msg::msg::RsPerceptionMsg(_init);
}

void RsPerceptionMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<perception_ros2_msg::msg::RsPerceptionMsg *>(message_memory);
  typed_message->~RsPerceptionMsg();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RsPerceptionMsg_message_member_array[2] = {
  {
    "lidarframe",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<perception_ros2_msg::msg::LidarFrameMsg>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::RsPerceptionMsg, lidarframe),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "device_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Int32>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg::msg::RsPerceptionMsg, device_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RsPerceptionMsg_message_members = {
  "perception_ros2_msg::msg",  // message namespace
  "RsPerceptionMsg",  // message name
  2,  // number of fields
  sizeof(perception_ros2_msg::msg::RsPerceptionMsg),
  RsPerceptionMsg_message_member_array,  // message members
  RsPerceptionMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  RsPerceptionMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RsPerceptionMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RsPerceptionMsg_message_members,
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
get_message_type_support_handle<perception_ros2_msg::msg::RsPerceptionMsg>()
{
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::RsPerceptionMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, perception_ros2_msg, msg, RsPerceptionMsg)() {
  return &::perception_ros2_msg::msg::rosidl_typesupport_introspection_cpp::RsPerceptionMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
