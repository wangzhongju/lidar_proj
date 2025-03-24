// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/Point4f.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__POINT4F__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__POINT4F__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'x'
// Member 'y'
// Member 'z'
#include "std_msgs/msg/float32__struct.hpp"
// Member 'i'
#include "std_msgs/msg/u_int16__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__Point4f __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__Point4f __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Point4f_
{
  using Type = Point4f_<ContainerAllocator>;

  explicit Point4f_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : x(_init),
    y(_init),
    z(_init),
    i(_init)
  {
    (void)_init;
  }

  explicit Point4f_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : x(_alloc, _init),
    y(_alloc, _init),
    z(_alloc, _init),
    i(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _x_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _x_type x;
  using _y_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _y_type y;
  using _z_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _z_type z;
  using _i_type =
    std_msgs::msg::UInt16_<ContainerAllocator>;
  _i_type i;

  // setters for named parameter idiom
  Type & set__x(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__i(
    const std_msgs::msg::UInt16_<ContainerAllocator> & _arg)
  {
    this->i = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::Point4f_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::Point4f_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Point4f_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Point4f_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Point4f_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Point4f_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Point4f_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Point4f_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Point4f_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Point4f_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__Point4f
    std::shared_ptr<perception_ros2_msg::msg::Point4f_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__Point4f
    std::shared_ptr<perception_ros2_msg::msg::Point4f_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Point4f_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->i != other.i) {
      return false;
    }
    return true;
  }
  bool operator!=(const Point4f_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Point4f_

// alias to use template instance with default allocator
using Point4f =
  perception_ros2_msg::msg::Point4f_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__POINT4F__STRUCT_HPP_
