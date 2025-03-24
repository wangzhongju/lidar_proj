// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from perception_ros2_msg:msg/AxisStatusPose.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__AXIS_STATUS_POSE__TRAITS_HPP_
#define PERCEPTION_ROS2_MSG__MSG__AXIS_STATUS_POSE__TRAITS_HPP_

#include "perception_ros2_msg/msg/axis_status_pose__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'status'
#include "std_msgs/msg/int32__traits.hpp"
// Member 'pose'
#include "perception_ros2_msg/msg/pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<perception_ros2_msg::msg::AxisStatusPose>()
{
  return "perception_ros2_msg::msg::AxisStatusPose";
}

template<>
struct has_fixed_size<perception_ros2_msg::msg::AxisStatusPose>
  : std::integral_constant<bool, has_fixed_size<perception_ros2_msg::msg::Pose>::value && has_fixed_size<std_msgs::msg::Int32>::value> {};

template<>
struct has_bounded_size<perception_ros2_msg::msg::AxisStatusPose>
  : std::integral_constant<bool, has_bounded_size<perception_ros2_msg::msg::Pose>::value && has_bounded_size<std_msgs::msg::Int32>::value> {};

template<>
struct is_message<perception_ros2_msg::msg::AxisStatusPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PERCEPTION_ROS2_MSG__MSG__AXIS_STATUS_POSE__TRAITS_HPP_
