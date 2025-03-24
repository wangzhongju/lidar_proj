// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/AxisStatusPose.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__AXIS_STATUS_POSE__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__AXIS_STATUS_POSE__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'status'
#include "std_msgs/msg/int32__struct.hpp"
// Member 'pose'
#include "perception_ros2_msg/msg/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__AxisStatusPose __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__AxisStatusPose __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AxisStatusPose_
{
  using Type = AxisStatusPose_<ContainerAllocator>;

  explicit AxisStatusPose_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : status(_init),
    pose(_init)
  {
    (void)_init;
  }

  explicit AxisStatusPose_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : status(_alloc, _init),
    pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _status_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _status_type status;
  using _pose_type =
    perception_ros2_msg::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__status(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__pose(
    const perception_ros2_msg::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__AxisStatusPose
    std::shared_ptr<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__AxisStatusPose
    std::shared_ptr<perception_ros2_msg::msg::AxisStatusPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AxisStatusPose_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const AxisStatusPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AxisStatusPose_

// alias to use template instance with default allocator
using AxisStatusPose =
  perception_ros2_msg::msg::AxisStatusPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__AXIS_STATUS_POSE__STRUCT_HPP_
