// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/Lanes.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__LANES__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__LANES__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'lanes'
#include "perception_ros2_msg/msg/lane__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__Lanes __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__Lanes __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Lanes_
{
  using Type = Lanes_<ContainerAllocator>;

  explicit Lanes_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Lanes_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _lanes_type =
    std::vector<perception_ros2_msg::msg::Lane_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Lane_<ContainerAllocator>>::other>;
  _lanes_type lanes;

  // setters for named parameter idiom
  Type & set__lanes(
    const std::vector<perception_ros2_msg::msg::Lane_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Lane_<ContainerAllocator>>::other> & _arg)
  {
    this->lanes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::Lanes_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::Lanes_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Lanes_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Lanes_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Lanes_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Lanes_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Lanes_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Lanes_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Lanes_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Lanes_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__Lanes
    std::shared_ptr<perception_ros2_msg::msg::Lanes_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__Lanes
    std::shared_ptr<perception_ros2_msg::msg::Lanes_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Lanes_ & other) const
  {
    if (this->lanes != other.lanes) {
      return false;
    }
    return true;
  }
  bool operator!=(const Lanes_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Lanes_

// alias to use template instance with default allocator
using Lanes =
  perception_ros2_msg::msg::Lanes_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__LANES__STRUCT_HPP_
