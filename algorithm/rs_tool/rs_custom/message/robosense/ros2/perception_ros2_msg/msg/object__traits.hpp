// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from perception_ros2_msg:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__OBJECT__TRAITS_HPP_
#define PERCEPTION_ROS2_MSG__MSG__OBJECT__TRAITS_HPP_

#include "perception_ros2_msg/msg/object__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'coreinfo'
#include "perception_ros2_msg/msg/core_info__traits.hpp"
// Member 'hassupplmentinfo'
#include "std_msgs/msg/bool__traits.hpp"
// Member 'supplementinfo'
#include "perception_ros2_msg/msg/supplement_info__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<perception_ros2_msg::msg::Object>()
{
  return "perception_ros2_msg::msg::Object";
}

template<>
struct has_fixed_size<perception_ros2_msg::msg::Object>
  : std::integral_constant<bool, has_fixed_size<perception_ros2_msg::msg::CoreInfo>::value && has_fixed_size<perception_ros2_msg::msg::SupplementInfo>::value && has_fixed_size<std_msgs::msg::Bool>::value> {};

template<>
struct has_bounded_size<perception_ros2_msg::msg::Object>
  : std::integral_constant<bool, has_bounded_size<perception_ros2_msg::msg::CoreInfo>::value && has_bounded_size<perception_ros2_msg::msg::SupplementInfo>::value && has_bounded_size<std_msgs::msg::Bool>::value> {};

template<>
struct is_message<perception_ros2_msg::msg::Object>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PERCEPTION_ROS2_MSG__MSG__OBJECT__TRAITS_HPP_
