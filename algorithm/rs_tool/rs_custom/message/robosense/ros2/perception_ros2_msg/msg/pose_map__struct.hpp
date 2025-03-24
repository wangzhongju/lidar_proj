// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/PoseMap.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__POSE_MAP__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__POSE_MAP__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'status_poses'
#include "perception_ros2_msg/msg/axis_status_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__PoseMap __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__PoseMap __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PoseMap_
{
  using Type = PoseMap_<ContainerAllocator>;

  explicit PoseMap_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit PoseMap_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _status_poses_type =
    std::vector<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator>>::other>;
  _status_poses_type status_poses;

  // setters for named parameter idiom
  Type & set__status_poses(
    const std::vector<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator>>::other> & _arg)
  {
    this->status_poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::PoseMap_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::PoseMap_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::PoseMap_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::PoseMap_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::PoseMap_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::PoseMap_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::PoseMap_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::PoseMap_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::PoseMap_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::PoseMap_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__PoseMap
    std::shared_ptr<perception_ros2_msg::msg::PoseMap_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__PoseMap
    std::shared_ptr<perception_ros2_msg::msg::PoseMap_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseMap_ & other) const
  {
    if (this->status_poses != other.status_poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseMap_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseMap_

// alias to use template instance with default allocator
using PoseMap =
  perception_ros2_msg::msg::PoseMap_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__POSE_MAP__STRUCT_HPP_
