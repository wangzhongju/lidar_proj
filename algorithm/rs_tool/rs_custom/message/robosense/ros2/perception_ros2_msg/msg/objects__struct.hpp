// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/Objects.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__OBJECTS__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__OBJECTS__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'timestamp'
#include "std_msgs/msg/float32__struct.hpp"
// Member 'device_id'
#include "std_msgs/msg/int32__struct.hpp"
// Member 'objects'
#include "perception_ros2_msg/msg/object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__Objects __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__Objects __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Objects_
{
  using Type = Objects_<ContainerAllocator>;

  explicit Objects_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : timestamp(_init),
    device_id(_init)
  {
    (void)_init;
  }

  explicit Objects_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init),
    device_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _timestamp_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _device_id_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _device_id_type device_id;
  using _objects_type =
    std::vector<perception_ros2_msg::msg::Object_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Object_<ContainerAllocator>>::other>;
  _objects_type objects;

  // setters for named parameter idiom
  Type & set__timestamp(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__device_id(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->device_id = _arg;
    return *this;
  }
  Type & set__objects(
    const std::vector<perception_ros2_msg::msg::Object_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Object_<ContainerAllocator>>::other> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::Objects_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::Objects_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Objects_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::Objects_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Objects_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Objects_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::Objects_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::Objects_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Objects_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::Objects_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__Objects
    std::shared_ptr<perception_ros2_msg::msg::Objects_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__Objects
    std::shared_ptr<perception_ros2_msg::msg::Objects_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Objects_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->device_id != other.device_id) {
      return false;
    }
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const Objects_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Objects_

// alias to use template instance with default allocator
using Objects =
  perception_ros2_msg::msg::Objects_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__OBJECTS__STRUCT_HPP_
