// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from perception_ros2_msg:msg/Point3f.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__POINT3F__TRAITS_HPP_
#define PERCEPTION_ROS2_MSG__MSG__POINT3F__TRAITS_HPP_

#include "perception_ros2_msg/msg/point3f__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'x'
// Member 'y'
// Member 'z'
#include "std_msgs/msg/float32__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<perception_ros2_msg::msg::Point3f>()
{
  return "perception_ros2_msg::msg::Point3f";
}

template<>
struct has_fixed_size<perception_ros2_msg::msg::Point3f>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Float32>::value> {};

template<>
struct has_bounded_size<perception_ros2_msg::msg::Point3f>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Float32>::value> {};

template<>
struct is_message<perception_ros2_msg::msg::Point3f>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PERCEPTION_ROS2_MSG__MSG__POINT3F__TRAITS_HPP_
