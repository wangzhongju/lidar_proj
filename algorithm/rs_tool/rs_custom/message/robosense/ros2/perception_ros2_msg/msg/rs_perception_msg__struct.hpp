// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/RsPerceptionMsg.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__RS_PERCEPTION_MSG__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__RS_PERCEPTION_MSG__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'lidarframe'
#include "perception_ros2_msg/msg/lidar_frame_msg__struct.hpp"
// Member 'device_id'
#include "std_msgs/msg/int32__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__RsPerceptionMsg __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__RsPerceptionMsg __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RsPerceptionMsg_
{
  using Type = RsPerceptionMsg_<ContainerAllocator>;

  explicit RsPerceptionMsg_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : lidarframe(_init),
    device_id(_init)
  {
    (void)_init;
  }

  explicit RsPerceptionMsg_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : lidarframe(_alloc, _init),
    device_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _lidarframe_type =
    perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator>;
  _lidarframe_type lidarframe;
  using _device_id_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _device_id_type device_id;

  // setters for named parameter idiom
  Type & set__lidarframe(
    const perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator> & _arg)
  {
    this->lidarframe = _arg;
    return *this;
  }
  Type & set__device_id(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->device_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__RsPerceptionMsg
    std::shared_ptr<perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__RsPerceptionMsg
    std::shared_ptr<perception_ros2_msg::msg::RsPerceptionMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RsPerceptionMsg_ & other) const
  {
    if (this->lidarframe != other.lidarframe) {
      return false;
    }
    if (this->device_id != other.device_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const RsPerceptionMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RsPerceptionMsg_

// alias to use template instance with default allocator
using RsPerceptionMsg =
  perception_ros2_msg::msg::RsPerceptionMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__RS_PERCEPTION_MSG__STRUCT_HPP_
