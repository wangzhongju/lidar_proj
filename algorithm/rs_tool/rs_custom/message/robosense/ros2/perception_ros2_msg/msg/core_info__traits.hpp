// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from perception_ros2_msg:msg/CoreInfo.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__CORE_INFO__TRAITS_HPP_
#define PERCEPTION_ROS2_MSG__MSG__CORE_INFO__TRAITS_HPP_

#include "perception_ros2_msg/msg/core_info__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'timestamp'
// Member 'exist_confidence'
// Member 'type_confidence'
// Member 'angle_velocity'
// Member 'angle_velocity_cov'
// Member 'angle_acceleration'
// Member 'angle_acceleration_cov'
#include "std_msgs/msg/float32__traits.hpp"
// Member 'frame_id'
#include "std_msgs/msg/string__traits.hpp"
// Member 'priority_id'
// Member 'type'
// Member 'attention_type'
// Member 'motion_state'
// Member 'lane_pos'
// Member 'tracker_id'
#include "std_msgs/msg/int32__traits.hpp"
// Member 'center'
// Member 'center_cov'
// Member 'size'
// Member 'size_cov'
// Member 'direction'
// Member 'direction_cov'
// Member 'velocity'
// Member 'relative_velocity'
// Member 'velocity_cov'
// Member 'relative_velocity_cov'
// Member 'acceleration'
// Member 'acceleration_cov'
// Member 'anchor'
// Member 'nearest_point'
#include "perception_ros2_msg/msg/point3f__traits.hpp"
// Member 'age'
#include "std_msgs/msg/float64__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<perception_ros2_msg::msg::CoreInfo>()
{
  return "perception_ros2_msg::msg::CoreInfo";
}

template<>
struct has_fixed_size<perception_ros2_msg::msg::CoreInfo>
  : std::integral_constant<bool, has_fixed_size<perception_ros2_msg::msg::Point3f>::value && has_fixed_size<std_msgs::msg::Float32>::value && has_fixed_size<std_msgs::msg::Float64>::value && has_fixed_size<std_msgs::msg::Int32>::value && has_fixed_size<std_msgs::msg::String>::value> {};

template<>
struct has_bounded_size<perception_ros2_msg::msg::CoreInfo>
  : std::integral_constant<bool, has_bounded_size<perception_ros2_msg::msg::Point3f>::value && has_bounded_size<std_msgs::msg::Float32>::value && has_bounded_size<std_msgs::msg::Float64>::value && has_bounded_size<std_msgs::msg::Int32>::value && has_bounded_size<std_msgs::msg::String>::value> {};

template<>
struct is_message<perception_ros2_msg::msg::CoreInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PERCEPTION_ROS2_MSG__MSG__CORE_INFO__TRAITS_HPP_
