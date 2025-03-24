// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__OBJECT__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__OBJECT__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'coreinfo'
#include "perception_ros2_msg/msg/core_info__struct.hpp"
// Member 'hassupplmentinfo'
#include "std_msgs/msg/bool__struct.hpp"
// Member 'supplementinfo'
#include "perception_ros2_msg/msg/supplement_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__Object __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__Object __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Object_
{
  using Type = Object_<ContainerAllocator>;

  explicit Object_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : coreinfo(_init),
    hassupplmentinfo(_init),
    supplementinfo(_init)
  {
    (void)_init;
  }

  explicit Object_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : coreinfo(_alloc, _init),
    hassupplmentinfo(_alloc, _init),
    supplementinfo(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _coreinfo_type =
    perception_ros2_msg::msg::CoreInfo_<ContainerAllocator>;
  _coreinfo_type coreinfo;
  using _hassupplmentinfo_type =
    std_msgs::msg::Bool_<ContainerAllocator>;
  _hassupplmentinfo_type hassupplmentinfo;
  using _supplementinfo_type =
    perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator>;
  _supplementinfo_type supplementinfo;

  // setters for named parameter idiom
  Type & set__coreinfo(
    const perception_ros2_msg::msg::CoreInfo_<ContainerAllocator> & _arg)
  {
    this->coreinfo = _arg;
    return *this;
  }
  Type & set__hassupplmentinfo(
    const std_msgs::msg::Bool_<ContainerAllocator> & _arg)
  {
    this->hassupplmentinfo = _arg;
    return *this;
  }
  Type & set__supplementinfo(
    const perception_ros2_msg::msg::SupplementInfo_<ContainerAllocator> & _arg)
  {
    this->supplementinfo = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::Object_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::Object_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Object_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Object_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Object_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Object_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Object_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Object_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Object_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Object_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__Object
    std::shared_ptr<perception_ros2_msg::msg::Object_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__Object
    std::shared_ptr<perception_ros2_msg::msg::Object_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Object_ & other) const
  {
    if (this->coreinfo != other.coreinfo) {
      return false;
    }
    if (this->hassupplmentinfo != other.hassupplmentinfo) {
      return false;
    }
    if (this->supplementinfo != other.supplementinfo) {
      return false;
    }
    return true;
  }
  bool operator!=(const Object_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Object_

// alias to use template instance with default allocator
using Object =
  perception_ros2_msg::msg::Object_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__OBJECT__STRUCT_HPP_
