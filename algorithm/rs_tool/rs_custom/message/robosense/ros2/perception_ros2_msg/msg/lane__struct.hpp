// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/Lane.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__LANE__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__LANE__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'lane_id'
// Member 'measure_status'
#include "std_msgs/msg/int32__struct.hpp"
// Member 'curve'
#include "perception_ros2_msg/msg/curve__struct.hpp"
// Member 'end_points'
#include "perception_ros2_msg/msg/end_points__struct.hpp"
// Member 'confidence'
#include "std_msgs/msg/float32__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__Lane __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__Lane __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Lane_
{
  using Type = Lane_<ContainerAllocator>;

  explicit Lane_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : lane_id(_init),
    curve(_init),
    end_points(_init),
    measure_status(_init),
    confidence(_init)
  {
    (void)_init;
  }

  explicit Lane_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : lane_id(_alloc, _init),
    curve(_alloc, _init),
    end_points(_alloc, _init),
    measure_status(_alloc, _init),
    confidence(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _lane_id_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _lane_id_type lane_id;
  using _curve_type =
    perception_ros2_msg::msg::Curve_<ContainerAllocator>;
  _curve_type curve;
  using _end_points_type =
    perception_ros2_msg::msg::EndPoints_<ContainerAllocator>;
  _end_points_type end_points;
  using _measure_status_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _measure_status_type measure_status;
  using _confidence_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _confidence_type confidence;

  // setters for named parameter idiom
  Type & set__lane_id(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->lane_id = _arg;
    return *this;
  }
  Type & set__curve(
    const perception_ros2_msg::msg::Curve_<ContainerAllocator> & _arg)
  {
    this->curve = _arg;
    return *this;
  }
  Type & set__end_points(
    const perception_ros2_msg::msg::EndPoints_<ContainerAllocator> & _arg)
  {
    this->end_points = _arg;
    return *this;
  }
  Type & set__measure_status(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->measure_status = _arg;
    return *this;
  }
  Type & set__confidence(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::Lane_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::Lane_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Lane_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Lane_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Lane_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Lane_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Lane_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Lane_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Lane_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Lane_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__Lane
    std::shared_ptr<perception_ros2_msg::msg::Lane_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__Lane
    std::shared_ptr<perception_ros2_msg::msg::Lane_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Lane_ & other) const
  {
    if (this->lane_id != other.lane_id) {
      return false;
    }
    if (this->curve != other.curve) {
      return false;
    }
    if (this->end_points != other.end_points) {
      return false;
    }
    if (this->measure_status != other.measure_status) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const Lane_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Lane_

// alias to use template instance with default allocator
using Lane =
  perception_ros2_msg::msg::Lane_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__LANE__STRUCT_HPP_
