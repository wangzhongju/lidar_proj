// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/SupplementInfo.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__SUPPLEMENT_INFO__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__SUPPLEMENT_INFO__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'unique_id'
#include "std_msgs/msg/u_int32__struct.hpp"
// Member 'polygon'
// Member 'geo_center'
// Member 'geo_size'
// Member 'trajectory'
// Member 'history_velocity'
#include "perception_ros2_msg/msg/point3f__struct.hpp"
// Member 'left_point_index'
// Member 'right_point_index'
// Member 'cloud_indices'
// Member 'size_type'
// Member 'mode'
// Member 'tracking_state'
// Member 'history_type'
// Member 'gps_mode'
#include "std_msgs/msg/int32__struct.hpp"
// Member 'latent_types'
#include "std_msgs/msg/float32__struct.hpp"
// Member 'in_roi'
#include "std_msgs/msg/bool__struct.hpp"
// Member 'gps_info'
#include "perception_ros2_msg/msg/point3d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__SupplementInfo __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__SupplementInfo __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SupplementInfo_
{
  using Type = SupplementInfo_<ContainerAllocator>;

  explicit SupplementInfo_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : unique_id(_init),
    left_point_index(_init),
    right_point_index(_init),
    size_type(_init),
    mode(_init),
    in_roi(_init),
    tracking_state(_init),
    geo_center(_init),
    geo_size(_init),
    gps_mode(_init),
    gps_info(_init)
  {
    (void)_init;
  }

  explicit SupplementInfo_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : unique_id(_alloc, _init),
    left_point_index(_alloc, _init),
    right_point_index(_alloc, _init),
    size_type(_alloc, _init),
    mode(_alloc, _init),
    in_roi(_alloc, _init),
    tracking_state(_alloc, _init),
    geo_center(_alloc, _init),
    geo_size(_alloc, _init),
    gps_mode(_alloc, _init),
    gps_info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _unique_id_type =
    std_msgs::msg::UInt32_<ContainerAllocator>;
  _unique_id_type unique_id;
  using _polygon_type =
    std::vector<perception_ros2_msg::msg::Point3f_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Point3f_<ContainerAllocator>>::other>;
  _polygon_type polygon;
  using _left_point_index_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _left_point_index_type left_point_index;
  using _right_point_index_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _right_point_index_type right_point_index;
  using _cloud_indices_type =
    std::vector<std_msgs::msg::Int32_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::Int32_<ContainerAllocator>>::other>;
  _cloud_indices_type cloud_indices;
  using _latent_types_type =
    std::vector<std_msgs::msg::Float32_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::Float32_<ContainerAllocator>>::other>;
  _latent_types_type latent_types;
  using _size_type_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _size_type_type size_type;
  using _mode_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _mode_type mode;
  using _in_roi_type =
    std_msgs::msg::Bool_<ContainerAllocator>;
  _in_roi_type in_roi;
  using _tracking_state_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _tracking_state_type tracking_state;
  using _geo_center_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _geo_center_type geo_center;
  using _geo_size_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _geo_size_type geo_size;
  using _trajectory_type =
    std::vector<perception_ros2_msg::msg::Point3f_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Point3f_<ContainerAllocator>>::other>;
  _trajectory_type trajectory;
  using _history_velocity_type =
    std::vector<perception_ros2_msg::msg::Point3f_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Point3f_<ContainerAllocator>>::other>;
  _history_velocity_type history_velocity;
  using _history_type_type =
    std::vector<std_msgs::msg::Int32_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::Int32_<ContainerAllocator>>::other>;
  _history_type_type history_type;
  using _gps_mode_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _gps_mode_type gps_mode;
  using _gps_info_type =
    perception_ros2_msg::msg::Point3d_<ContainerAllocator>;
  _gps_info_type gps_info;

  // setters for named parameter idiom
  Type & set__unique_id(
    const std_msgs::msg::UInt32_<ContainerAllocator> & _arg)
  {
    this->unique_id = _arg;
    return *this;
  }
  Type & set__polygon(
    const std::vector<perception_ros2_msg::msg::Point3f_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Point3f_<ContainerAllocator>>::other> & _arg)
  {
    this->polygon = _arg;
    return *this;
  }
  Type & set__left_point_index(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->left_point_index = _arg;
    return *this;
  }
  Type & set__right_point_index(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->right_point_index = _arg;
    return *this;
  }
  Type & set__cloud_indices(
    const std::vector<std_msgs::msg::Int32_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::Int32_<ContainerAllocator>>::other> & _arg)
  {
    this->cloud_indices = _arg;
    return *this;
  }
  Type & set__latent_types(
    const std::vector<std_msgs::msg::Float32_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::Float32_<ContainerAllocator>>::other> & _arg)
  {
    this->latent_types = _arg;
    return *this;
  }
  Type & set__size_type(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->size_type = _arg;
    return *this;
  }
  Type & set__mode(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__in_roi(
    const std_msgs::msg::Bool_<ContainerAllocator> & _arg)
  {
    this->in_roi = _arg;
    return *this;
  }
  Type & set__tracking_state(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->tracking_state = _arg;
    return *this;
  }
  Type & set__geo_center(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->geo_center = _arg;
    return *this;
  }
  Type & set__geo_size(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->geo_size = _arg;
    return *this;
  }
  Type & set__trajectory(
    const std::vector<perception_ros2_msg::msg::Point3f_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Point3f_<ContainerAllocator>>::other> & _arg)
  {
    this->trajectory = _arg;
    return *this;
  }
  Type & set__history_velocity(
    const std::vector<perception_ros2_msg::msg::Point3f_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Point3f_<ContainerAllocator>>::other> & _arg)
  {
    this->history_velocity = _arg;
    return *this;
  }
  Type & set__history_type(
    const std::vector<std_msgs::msg::Int32_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::Int32_<ContainerAllocator>>::other> & _arg)
  {
    this->history_type = _arg;
    return *this;
  }
  Type & set__gps_mode(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->gps_mode = _arg;
    return *this;
  }
  Type & set__gps_info(
    const perception_ros2_msg::msg::Point3d_<ContainerAllocator> & _arg)
  {
    this->gps_info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__SupplementInfo
    std::shared_ptr<perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__SupplementInfo
    std::shared_ptr<perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SupplementInfo_ & other) const
  {
    if (this->unique_id != other.unique_id) {
      return false;
    }
    if (this->polygon != other.polygon) {
      return false;
    }
    if (this->left_point_index != other.left_point_index) {
      return false;
    }
    if (this->right_point_index != other.right_point_index) {
      return false;
    }
    if (this->cloud_indices != other.cloud_indices) {
      return false;
    }
    if (this->latent_types != other.latent_types) {
      return false;
    }
    if (this->size_type != other.size_type) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->in_roi != other.in_roi) {
      return false;
    }
    if (this->tracking_state != other.tracking_state) {
      return false;
    }
    if (this->geo_center != other.geo_center) {
      return false;
    }
    if (this->geo_size != other.geo_size) {
      return false;
    }
    if (this->trajectory != other.trajectory) {
      return false;
    }
    if (this->history_velocity != other.history_velocity) {
      return false;
    }
    if (this->history_type != other.history_type) {
      return false;
    }
    if (this->gps_mode != other.gps_mode) {
      return false;
    }
    if (this->gps_info != other.gps_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const SupplementInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SupplementInfo_

// alias to use template instance with default allocator
using SupplementInfo =
  perception_ros2_msg::msg::SupplementInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__SUPPLEMENT_INFO__STRUCT_HPP_
