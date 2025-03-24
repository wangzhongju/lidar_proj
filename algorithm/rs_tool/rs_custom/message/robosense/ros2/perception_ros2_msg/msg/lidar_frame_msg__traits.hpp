// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from perception_ros2_msg:msg/LidarFrameMsg.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__LIDAR_FRAME_MSG__TRAITS_HPP_
#define PERCEPTION_ROS2_MSG__MSG__LIDAR_FRAME_MSG__TRAITS_HPP_

#include "perception_ros2_msg/msg/lidar_frame_msg__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'frame_id'
#include "std_msgs/msg/string__traits.hpp"
// Member 'timestamp'
#include "std_msgs/msg/float64__traits.hpp"
// Member 'global_pose'
#include "perception_ros2_msg/msg/pose__traits.hpp"
// Member 'gps_origin'
#include "perception_ros2_msg/msg/point3d__traits.hpp"
// Member 'status_pose_map'
#include "perception_ros2_msg/msg/pose_map__traits.hpp"
// Member 'status'
#include "std_msgs/msg/int32__traits.hpp"
// Member 'valid_indices'
// Member 'non_ground_indices'
// Member 'ground_indices'
// Member 'background_indices'
#include "perception_ros2_msg/msg/indices__traits.hpp"
// Member 'objects'
// Member 'attention_objects'
#include "perception_ros2_msg/msg/objects__traits.hpp"
// Member 'has_pointcloud'
// Member 'has_attention_objects'
// Member 'has_freespace'
// Member 'has_lanes'
// Member 'has_roadedges'
// Member 'has_sematice_indices'
#include "std_msgs/msg/bool__traits.hpp"
// Member 'freespace_infos'
#include "perception_ros2_msg/msg/free_space_infos__traits.hpp"
// Member 'lanes'
#include "perception_ros2_msg/msg/lanes__traits.hpp"
// Member 'roadedges'
#include "perception_ros2_msg/msg/road_edges__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<perception_ros2_msg::msg::LidarFrameMsg>()
{
  return "perception_ros2_msg::msg::LidarFrameMsg";
}

template<>
struct has_fixed_size<perception_ros2_msg::msg::LidarFrameMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<perception_ros2_msg::msg::LidarFrameMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<perception_ros2_msg::msg::LidarFrameMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PERCEPTION_ROS2_MSG__MSG__LIDAR_FRAME_MSG__TRAITS_HPP_
