// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/FreeSpaceInfos.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__FREE_SPACE_INFOS__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__FREE_SPACE_INFOS__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'fs_pts'
#include "perception_ros2_msg/msg/point3f__struct.hpp"
// Member 'fs_confidence'
#include "std_msgs/msg/float32__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__FreeSpaceInfos __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__FreeSpaceInfos __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FreeSpaceInfos_
{
  using Type = FreeSpaceInfos_<ContainerAllocator>;

  explicit FreeSpaceInfos_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit FreeSpaceInfos_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _fs_pts_type =
    std::vector<perception_ros2_msg::msg::Point3f_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Point3f_<ContainerAllocator>>::other>;
  _fs_pts_type fs_pts;
  using _fs_confidence_type =
    std::vector<std_msgs::msg::Float32_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::Float32_<ContainerAllocator>>::other>;
  _fs_confidence_type fs_confidence;

  // setters for named parameter idiom
  Type & set__fs_pts(
    const std::vector<perception_ros2_msg::msg::Point3f_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Point3f_<ContainerAllocator>>::other> & _arg)
  {
    this->fs_pts = _arg;
    return *this;
  }
  Type & set__fs_confidence(
    const std::vector<std_msgs::msg::Float32_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::Float32_<ContainerAllocator>>::other> & _arg)
  {
    this->fs_confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__FreeSpaceInfos
    std::shared_ptr<perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__FreeSpaceInfos
    std::shared_ptr<perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FreeSpaceInfos_ & other) const
  {
    if (this->fs_pts != other.fs_pts) {
      return false;
    }
    if (this->fs_confidence != other.fs_confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const FreeSpaceInfos_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FreeSpaceInfos_

// alias to use template instance with default allocator
using FreeSpaceInfos =
  perception_ros2_msg::msg::FreeSpaceInfos_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__FREE_SPACE_INFOS__STRUCT_HPP_
