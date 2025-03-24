// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from perception_ros2_msg:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__POSE__TRAITS_HPP_
#define PERCEPTION_ROS2_MSG__MSG__POSE__TRAITS_HPP_

#include "perception_ros2_msg/msg/pose__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'x'
// Member 'y'
// Member 'z'
// Member 'roll'
// Member 'pitch'
// Member 'yaw'
#include "std_msgs/msg/float32__traits.hpp"
// Member 'status'
#include "std_msgs/msg/int32__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<perception_ros2_msg::msg::Pose>()
{
  return "perception_ros2_msg::msg::Pose";
}

template<>
struct has_fixed_size<perception_ros2_msg::msg::Pose>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Float32>::value && has_fixed_size<std_msgs::msg::Int32>::value> {};

template<>
struct has_bounded_size<perception_ros2_msg::msg::Pose>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Float32>::value && has_bounded_size<std_msgs::msg::Int32>::value> {};

template<>
struct is_message<perception_ros2_msg::msg::Pose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PERCEPTION_ROS2_MSG__MSG__POSE__TRAITS_HPP_
