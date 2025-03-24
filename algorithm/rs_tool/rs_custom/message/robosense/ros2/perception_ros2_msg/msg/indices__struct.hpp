// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/Indices.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__INDICES__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__INDICES__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'indices'
#include "std_msgs/msg/int32__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__Indices __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__Indices __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Indices_
{
  using Type = Indices_<ContainerAllocator>;

  explicit Indices_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Indices_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _indices_type =
    std::vector<std_msgs::msg::Int32_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::Int32_<ContainerAllocator>>::other>;
  _indices_type indices;

  // setters for named parameter idiom
  Type & set__indices(
    const std::vector<std_msgs::msg::Int32_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::Int32_<ContainerAllocator>>::other> & _arg)
  {
    this->indices = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::Indices_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::Indices_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Indices_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Indices_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Indices_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Indices_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Indices_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Indices_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Indices_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Indices_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__Indices
    std::shared_ptr<perception_ros2_msg::msg::Indices_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__Indices
    std::shared_ptr<perception_ros2_msg::msg::Indices_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Indices_ & other) const
  {
    if (this->indices != other.indices) {
      return false;
    }
    return true;
  }
  bool operator!=(const Indices_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Indices_

// alias to use template instance with default allocator
using Indices =
  perception_ros2_msg::msg::Indices_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__INDICES__STRUCT_HPP_
