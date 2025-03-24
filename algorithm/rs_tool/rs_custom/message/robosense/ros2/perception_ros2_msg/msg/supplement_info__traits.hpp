// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from perception_ros2_msg:msg/SupplementInfo.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__SUPPLEMENT_INFO__TRAITS_HPP_
#define PERCEPTION_ROS2_MSG__MSG__SUPPLEMENT_INFO__TRAITS_HPP_

#include "perception_ros2_msg/msg/supplement_info__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'unique_id'
#include "std_msgs/msg/u_int32__traits.hpp"
// Member 'left_point_index'
// Member 'right_point_index'
// Member 'size_type'
// Member 'mode'
// Member 'tracking_state'
// Member 'gps_mode'
#include "std_msgs/msg/int32__traits.hpp"
// Member 'in_roi'
#include "std_msgs/msg/bool__traits.hpp"
// Member 'geo_center'
// Member 'geo_size'
#include "perception_ros2_msg/msg/point3f__traits.hpp"
// Member 'gps_info'
#include "perception_ros2_msg/msg/point3d__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<perception_ros2_msg::msg::SupplementInfo>()
{
  return "perception_ros2_msg::msg::SupplementInfo";
}

template<>
struct has_fixed_size<perception_ros2_msg::msg::SupplementInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<perception_ros2_msg::msg::SupplementInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<perception_ros2_msg::msg::SupplementInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PERCEPTION_ROS2_MSG__MSG__SUPPLEMENT_INFO__TRAITS_HPP_
