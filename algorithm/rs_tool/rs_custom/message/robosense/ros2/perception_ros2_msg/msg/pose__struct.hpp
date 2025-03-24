// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__POSE__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__POSE__STRUCT_HPP_

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
// Member 'roll'
// Member 'pitch'
// Member 'yaw'
#include "std_msgs/msg/float32__struct.hpp"
// Member 'status'
#include "std_msgs/msg/int32__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__Pose __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__Pose __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pose_
{
  using Type = Pose_<ContainerAllocator>;

  explicit Pose_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : x(_init),
    y(_init),
    z(_init),
    roll(_init),
    pitch(_init),
    yaw(_init),
    status(_init)
  {
    (void)_init;
  }

  explicit Pose_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : x(_alloc, _init),
    y(_alloc, _init),
    z(_alloc, _init),
    roll(_alloc, _init),
    pitch(_alloc, _init),
    yaw(_alloc, _init),
    status(_alloc, _init)
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
  using _roll_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _roll_type roll;
  using _pitch_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _pitch_type pitch;
  using _yaw_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _yaw_type yaw;
  using _status_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _status_type status;

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
  Type & set__roll(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__status(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::Pose_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::Pose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Pose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Pose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Pose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Pose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Pose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Pose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Pose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Pose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__Pose
    std::shared_ptr<perception_ros2_msg::msg::Pose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__Pose
    std::shared_ptr<perception_ros2_msg::msg::Pose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pose_ & other) const
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
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pose_

// alias to use template instance with default allocator
using Pose =
  perception_ros2_msg::msg::Pose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__POSE__STRUCT_HPP_
