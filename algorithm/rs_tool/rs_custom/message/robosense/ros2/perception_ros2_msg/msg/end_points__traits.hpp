// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from perception_ros2_msg:msg/EndPoints.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__END_POINTS__TRAITS_HPP_
#define PERCEPTION_ROS2_MSG__MSG__END_POINTS__TRAITS_HPP_

#include "perception_ros2_msg/msg/end_points__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'start'
// Member 'end'
#include "perception_ros2_msg/msg/point2f__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<perception_ros2_msg::msg::EndPoints>()
{
  return "perception_ros2_msg::msg::EndPoints";
}

template<>
struct has_fixed_size<perception_ros2_msg::msg::EndPoints>
  : std::integral_constant<bool, has_fixed_size<perception_ros2_msg::msg::Point2f>::value> {};

template<>
struct has_bounded_size<perception_ros2_msg::msg::EndPoints>
  : std::integral_constant<bool, has_bounded_size<perception_ros2_msg::msg::Point2f>::value> {};

template<>
struct is_message<perception_ros2_msg::msg::EndPoints>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PERCEPTION_ROS2_MSG__MSG__END_POINTS__TRAITS_HPP_
