// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/EndPoints.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__END_POINTS__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__END_POINTS__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'start'
// Member 'end'
#include "perception_ros2_msg/msg/point2f__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__EndPoints __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__EndPoints __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EndPoints_
{
  using Type = EndPoints_<ContainerAllocator>;

  explicit EndPoints_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : start(_init),
    end(_init)
  {
    (void)_init;
  }

  explicit EndPoints_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : start(_alloc, _init),
    end(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _start_type =
    perception_ros2_msg::msg::Point2f_<ContainerAllocator>;
  _start_type start;
  using _end_type =
    perception_ros2_msg::msg::Point2f_<ContainerAllocator>;
  _end_type end;

  // setters for named parameter idiom
  Type & set__start(
    const perception_ros2_msg::msg::Point2f_<ContainerAllocator> & _arg)
  {
    this->start = _arg;
    return *this;
  }
  Type & set__end(
    const perception_ros2_msg::msg::Point2f_<ContainerAllocator> & _arg)
  {
    this->end = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::EndPoints_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::EndPoints_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::EndPoints_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::EndPoints_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::EndPoints_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::EndPoints_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::EndPoints_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::EndPoints_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::EndPoints_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::EndPoints_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__EndPoints
    std::shared_ptr<perception_ros2_msg::msg::EndPoints_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__EndPoints
    std::shared_ptr<perception_ros2_msg::msg::EndPoints_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EndPoints_ & other) const
  {
    if (this->start != other.start) {
      return false;
    }
    if (this->end != other.end) {
      return false;
    }
    return true;
  }
  bool operator!=(const EndPoints_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EndPoints_

// alias to use template instance with default allocator
using EndPoints =
  perception_ros2_msg::msg::EndPoints_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__END_POINTS__STRUCT_HPP_
