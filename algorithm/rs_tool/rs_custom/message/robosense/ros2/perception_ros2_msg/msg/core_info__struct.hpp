// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/CoreInfo.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__CORE_INFO__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__CORE_INFO__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'timestamp'
// Member 'exist_confidence'
// Member 'type_confidence'
// Member 'angle_velocity'
// Member 'angle_velocity_cov'
// Member 'angle_acceleration'
// Member 'angle_acceleration_cov'
#include "std_msgs/msg/float32__struct.hpp"
// Member 'frame_id'
#include "std_msgs/msg/string__struct.hpp"
// Member 'priority_id'
// Member 'type'
// Member 'attention_type'
// Member 'motion_state'
// Member 'lane_pos'
// Member 'tracker_id'
#include "std_msgs/msg/int32__struct.hpp"
// Member 'center'
// Member 'center_cov'
// Member 'size'
// Member 'size_cov'
// Member 'direction'
// Member 'direction_cov'
// Member 'velocity'
// Member 'relative_velocity'
// Member 'velocity_cov'
// Member 'relative_velocity_cov'
// Member 'acceleration'
// Member 'acceleration_cov'
// Member 'anchor'
// Member 'nearest_point'
#include "perception_ros2_msg/msg/point3f__struct.hpp"
// Member 'age'
#include "std_msgs/msg/float64__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__CoreInfo __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__CoreInfo __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CoreInfo_
{
  using Type = CoreInfo_<ContainerAllocator>;

  explicit CoreInfo_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : timestamp(_init),
    frame_id(_init),
    priority_id(_init),
    exist_confidence(_init),
    center(_init),
    center_cov(_init),
    size(_init),
    size_cov(_init),
    direction(_init),
    direction_cov(_init),
    type(_init),
    type_confidence(_init),
    attention_type(_init),
    motion_state(_init),
    lane_pos(_init),
    tracker_id(_init),
    age(_init),
    velocity(_init),
    relative_velocity(_init),
    velocity_cov(_init),
    relative_velocity_cov(_init),
    acceleration(_init),
    acceleration_cov(_init),
    angle_velocity(_init),
    angle_velocity_cov(_init),
    angle_acceleration(_init),
    angle_acceleration_cov(_init),
    anchor(_init),
    nearest_point(_init)
  {
    (void)_init;
  }

  explicit CoreInfo_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init),
    frame_id(_alloc, _init),
    priority_id(_alloc, _init),
    exist_confidence(_alloc, _init),
    center(_alloc, _init),
    center_cov(_alloc, _init),
    size(_alloc, _init),
    size_cov(_alloc, _init),
    direction(_alloc, _init),
    direction_cov(_alloc, _init),
    type(_alloc, _init),
    type_confidence(_alloc, _init),
    attention_type(_alloc, _init),
    motion_state(_alloc, _init),
    lane_pos(_alloc, _init),
    tracker_id(_alloc, _init),
    age(_alloc, _init),
    velocity(_alloc, _init),
    relative_velocity(_alloc, _init),
    velocity_cov(_alloc, _init),
    relative_velocity_cov(_alloc, _init),
    acceleration(_alloc, _init),
    acceleration_cov(_alloc, _init),
    angle_velocity(_alloc, _init),
    angle_velocity_cov(_alloc, _init),
    angle_acceleration(_alloc, _init),
    angle_acceleration_cov(_alloc, _init),
    anchor(_alloc, _init),
    nearest_point(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _timestamp_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _frame_id_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _frame_id_type frame_id;
  using _priority_id_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _priority_id_type priority_id;
  using _exist_confidence_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _exist_confidence_type exist_confidence;
  using _center_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _center_type center;
  using _center_cov_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _center_cov_type center_cov;
  using _size_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _size_type size;
  using _size_cov_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _size_cov_type size_cov;
  using _direction_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _direction_type direction;
  using _direction_cov_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _direction_cov_type direction_cov;
  using _type_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _type_type type;
  using _type_confidence_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _type_confidence_type type_confidence;
  using _attention_type_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _attention_type_type attention_type;
  using _motion_state_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _motion_state_type motion_state;
  using _lane_pos_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _lane_pos_type lane_pos;
  using _tracker_id_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _tracker_id_type tracker_id;
  using _age_type =
    std_msgs::msg::Float64_<ContainerAllocator>;
  _age_type age;
  using _velocity_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _velocity_type velocity;
  using _relative_velocity_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _relative_velocity_type relative_velocity;
  using _velocity_cov_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _velocity_cov_type velocity_cov;
  using _relative_velocity_cov_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _relative_velocity_cov_type relative_velocity_cov;
  using _acceleration_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _acceleration_type acceleration;
  using _acceleration_cov_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _acceleration_cov_type acceleration_cov;
  using _angle_velocity_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _angle_velocity_type angle_velocity;
  using _angle_velocity_cov_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _angle_velocity_cov_type angle_velocity_cov;
  using _angle_acceleration_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _angle_acceleration_type angle_acceleration;
  using _angle_acceleration_cov_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _angle_acceleration_cov_type angle_acceleration_cov;
  using _anchor_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _anchor_type anchor;
  using _nearest_point_type =
    perception_ros2_msg::msg::Point3f_<ContainerAllocator>;
  _nearest_point_type nearest_point;

  // setters for named parameter idiom
  Type & set__timestamp(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__frame_id(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->frame_id = _arg;
    return *this;
  }
  Type & set__priority_id(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->priority_id = _arg;
    return *this;
  }
  Type & set__exist_confidence(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->exist_confidence = _arg;
    return *this;
  }
  Type & set__center(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->center = _arg;
    return *this;
  }
  Type & set__center_cov(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->center_cov = _arg;
    return *this;
  }
  Type & set__size(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->size = _arg;
    return *this;
  }
  Type & set__size_cov(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->size_cov = _arg;
    return *this;
  }
  Type & set__direction(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->direction = _arg;
    return *this;
  }
  Type & set__direction_cov(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->direction_cov = _arg;
    return *this;
  }
  Type & set__type(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__type_confidence(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->type_confidence = _arg;
    return *this;
  }
  Type & set__attention_type(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->attention_type = _arg;
    return *this;
  }
  Type & set__motion_state(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->motion_state = _arg;
    return *this;
  }
  Type & set__lane_pos(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->lane_pos = _arg;
    return *this;
  }
  Type & set__tracker_id(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->tracker_id = _arg;
    return *this;
  }
  Type & set__age(
    const std_msgs::msg::Float64_<ContainerAllocator> & _arg)
  {
    this->age = _arg;
    return *this;
  }
  Type & set__velocity(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__relative_velocity(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->relative_velocity = _arg;
    return *this;
  }
  Type & set__velocity_cov(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->velocity_cov = _arg;
    return *this;
  }
  Type & set__relative_velocity_cov(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->relative_velocity_cov = _arg;
    return *this;
  }
  Type & set__acceleration(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__acceleration_cov(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->acceleration_cov = _arg;
    return *this;
  }
  Type & set__angle_velocity(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->angle_velocity = _arg;
    return *this;
  }
  Type & set__angle_velocity_cov(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->angle_velocity_cov = _arg;
    return *this;
  }
  Type & set__angle_acceleration(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->angle_acceleration = _arg;
    return *this;
  }
  Type & set__angle_acceleration_cov(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->angle_acceleration_cov = _arg;
    return *this;
  }
  Type & set__anchor(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->anchor = _arg;
    return *this;
  }
  Type & set__nearest_point(
    const perception_ros2_msg::msg::Point3f_<ContainerAllocator> & _arg)
  {
    this->nearest_point = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::CoreInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::CoreInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::CoreInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::CoreInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::CoreInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::CoreInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::CoreInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::CoreInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::CoreInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::CoreInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__CoreInfo
    std::shared_ptr<perception_ros2_msg::msg::CoreInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__CoreInfo
    std::shared_ptr<perception_ros2_msg::msg::CoreInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CoreInfo_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->frame_id != other.frame_id) {
      return false;
    }
    if (this->priority_id != other.priority_id) {
      return false;
    }
    if (this->exist_confidence != other.exist_confidence) {
      return false;
    }
    if (this->center != other.center) {
      return false;
    }
    if (this->center_cov != other.center_cov) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    if (this->size_cov != other.size_cov) {
      return false;
    }
    if (this->direction != other.direction) {
      return false;
    }
    if (this->direction_cov != other.direction_cov) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->type_confidence != other.type_confidence) {
      return false;
    }
    if (this->attention_type != other.attention_type) {
      return false;
    }
    if (this->motion_state != other.motion_state) {
      return false;
    }
    if (this->lane_pos != other.lane_pos) {
      return false;
    }
    if (this->tracker_id != other.tracker_id) {
      return false;
    }
    if (this->age != other.age) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->relative_velocity != other.relative_velocity) {
      return false;
    }
    if (this->velocity_cov != other.velocity_cov) {
      return false;
    }
    if (this->relative_velocity_cov != other.relative_velocity_cov) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->acceleration_cov != other.acceleration_cov) {
      return false;
    }
    if (this->angle_velocity != other.angle_velocity) {
      return false;
    }
    if (this->angle_velocity_cov != other.angle_velocity_cov) {
      return false;
    }
    if (this->angle_acceleration != other.angle_acceleration) {
      return false;
    }
    if (this->angle_acceleration_cov != other.angle_acceleration_cov) {
      return false;
    }
    if (this->anchor != other.anchor) {
      return false;
    }
    if (this->nearest_point != other.nearest_point) {
      return false;
    }
    return true;
  }
  bool operator!=(const CoreInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CoreInfo_

// alias to use template instance with default allocator
using CoreInfo =
  perception_ros2_msg::msg::CoreInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__CORE_INFO__STRUCT_HPP_
