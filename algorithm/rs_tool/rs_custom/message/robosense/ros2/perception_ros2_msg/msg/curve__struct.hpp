// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/Curve.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__CURVE__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__CURVE__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'x_start'
// Member 'x_end'
// Member 'a'
// Member 'b'
// Member 'c'
// Member 'd'
#include "std_msgs/msg/float32__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__Curve __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__Curve __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Curve_
{
  using Type = Curve_<ContainerAllocator>;

  explicit Curve_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : x_start(_init),
    x_end(_init),
    a(_init),
    b(_init),
    c(_init),
    d(_init)
  {
    (void)_init;
  }

  explicit Curve_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : x_start(_alloc, _init),
    x_end(_alloc, _init),
    a(_alloc, _init),
    b(_alloc, _init),
    c(_alloc, _init),
    d(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _x_start_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _x_start_type x_start;
  using _x_end_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _x_end_type x_end;
  using _a_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _a_type a;
  using _b_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _b_type b;
  using _c_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _c_type c;
  using _d_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _d_type d;

  // setters for named parameter idiom
  Type & set__x_start(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->x_start = _arg;
    return *this;
  }
  Type & set__x_end(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->x_end = _arg;
    return *this;
  }
  Type & set__a(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__b(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->b = _arg;
    return *this;
  }
  Type & set__c(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->c = _arg;
    return *this;
  }
  Type & set__d(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->d = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::Curve_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::Curve_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Curve_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Curve_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Curve_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Curve_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Curve_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Curve_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Curve_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Curve_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__Curve
    std::shared_ptr<perception_ros2_msg::msg::Curve_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__Curve
    std::shared_ptr<perception_ros2_msg::msg::Curve_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Curve_ & other) const
  {
    if (this->x_start != other.x_start) {
      return false;
    }
    if (this->x_end != other.x_end) {
      return false;
    }
    if (this->a != other.a) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    if (this->c != other.c) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    return true;
  }
  bool operator!=(const Curve_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Curve_

// alias to use template instance with default allocator
using Curve =
  perception_ros2_msg::msg::Curve_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__CURVE__STRUCT_HPP_
