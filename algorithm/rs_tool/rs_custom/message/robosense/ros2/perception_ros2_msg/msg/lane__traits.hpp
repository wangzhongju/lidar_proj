// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from perception_ros2_msg:msg/Lane.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__LANE__TRAITS_HPP_
#define PERCEPTION_ROS2_MSG__MSG__LANE__TRAITS_HPP_

#include "perception_ros2_msg/msg/lane__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'lane_id'
// Member 'measure_status'
#include "std_msgs/msg/int32__traits.hpp"
// Member 'curve'
#include "perception_ros2_msg/msg/curve__traits.hpp"
// Member 'end_points'
#include "perception_ros2_msg/msg/end_points__traits.hpp"
// Member 'confidence'
#include "std_msgs/msg/float32__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<perception_ros2_msg::msg::Lane>()
{
  return "perception_ros2_msg::msg::Lane";
}

template<>
struct has_fixed_size<perception_ros2_msg::msg::Lane>
  : std::integral_constant<bool, has_fixed_size<perception_ros2_msg::msg::Curve>::value && has_fixed_size<perception_ros2_msg::msg::EndPoints>::value && has_fixed_size<std_msgs::msg::Float32>::value && has_fixed_size<std_msgs::msg::Int32>::value> {};

template<>
struct has_bounded_size<perception_ros2_msg::msg::Lane>
  : std::integral_constant<bool, has_bounded_size<perception_ros2_msg::msg::Curve>::value && has_bounded_size<perception_ros2_msg::msg::EndPoints>::value && has_bounded_size<std_msgs::msg::Float32>::value && has_bounded_size<std_msgs::msg::Int32>::value> {};

template<>
struct is_message<perception_ros2_msg::msg::Lane>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PERCEPTION_ROS2_MSG__MSG__LANE__TRAITS_HPP_
