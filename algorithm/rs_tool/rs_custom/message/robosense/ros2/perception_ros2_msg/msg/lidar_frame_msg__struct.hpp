// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_ros2_msg:msg/LidarFrameMsg.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__LIDAR_FRAME_MSG__STRUCT_HPP_
#define PERCEPTION_ROS2_MSG__MSG__LIDAR_FRAME_MSG__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'frame_id'
#include "std_msgs/msg/string__struct.hpp"
// Member 'timestamp'
#include "std_msgs/msg/float64__struct.hpp"
// Member 'global_pose'
#include "perception_ros2_msg/msg/pose__struct.hpp"
// Member 'gps_origin'
#include "perception_ros2_msg/msg/point3d__struct.hpp"
// Member 'status_pose_map'
#include "perception_ros2_msg/msg/pose_map__struct.hpp"
// Member 'status'
#include "std_msgs/msg/int32__struct.hpp"
// Member 'valid_indices'
// Member 'non_ground_indices'
// Member 'ground_indices'
// Member 'background_indices'
#include "perception_ros2_msg/msg/indices__struct.hpp"
// Member 'objects'
// Member 'attention_objects'
#include "perception_ros2_msg/msg/objects__struct.hpp"
// Member 'has_pointcloud'
// Member 'has_attention_objects'
// Member 'has_freespace'
// Member 'has_lanes'
// Member 'has_roadedges'
// Member 'has_sematice_indices'
#include "std_msgs/msg/bool__struct.hpp"
// Member 'scan_pointcloud'
#include "perception_ros2_msg/msg/point4f__struct.hpp"
// Member 'freespace_infos'
#include "perception_ros2_msg/msg/free_space_infos__struct.hpp"
// Member 'lanes'
#include "perception_ros2_msg/msg/lanes__struct.hpp"
// Member 'roadedges'
#include "perception_ros2_msg/msg/road_edges__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_ros2_msg__msg__LidarFrameMsg __attribute__((deprecated))
#else
# define DEPRECATED__perception_ros2_msg__msg__LidarFrameMsg __declspec(deprecated)
#endif

namespace perception_ros2_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LidarFrameMsg_
{
  using Type = LidarFrameMsg_<ContainerAllocator>;

  explicit LidarFrameMsg_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : frame_id(_init),
    timestamp(_init),
    global_pose(_init),
    gps_origin(_init),
    status_pose_map(_init),
    status(_init),
    valid_indices(_init),
    objects(_init),
    has_pointcloud(_init),
    has_attention_objects(_init),
    attention_objects(_init),
    has_freespace(_init),
    freespace_infos(_init),
    has_lanes(_init),
    lanes(_init),
    has_roadedges(_init),
    roadedges(_init),
    has_sematice_indices(_init),
    non_ground_indices(_init),
    ground_indices(_init),
    background_indices(_init)
  {
    (void)_init;
  }

  explicit LidarFrameMsg_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : frame_id(_alloc, _init),
    timestamp(_alloc, _init),
    global_pose(_alloc, _init),
    gps_origin(_alloc, _init),
    status_pose_map(_alloc, _init),
    status(_alloc, _init),
    valid_indices(_alloc, _init),
    objects(_alloc, _init),
    has_pointcloud(_alloc, _init),
    has_attention_objects(_alloc, _init),
    attention_objects(_alloc, _init),
    has_freespace(_alloc, _init),
    freespace_infos(_alloc, _init),
    has_lanes(_alloc, _init),
    lanes(_alloc, _init),
    has_roadedges(_alloc, _init),
    roadedges(_alloc, _init),
    has_sematice_indices(_alloc, _init),
    non_ground_indices(_alloc, _init),
    ground_indices(_alloc, _init),
    background_indices(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _frame_id_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _frame_id_type frame_id;
  using _timestamp_type =
    std_msgs::msg::Float64_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _global_pose_type =
    perception_ros2_msg::msg::Pose_<ContainerAllocator>;
  _global_pose_type global_pose;
  using _gps_origin_type =
    perception_ros2_msg::msg::Point3d_<ContainerAllocator>;
  _gps_origin_type gps_origin;
  using _status_pose_map_type =
    perception_ros2_msg::msg::PoseMap_<ContainerAllocator>;
  _status_pose_map_type status_pose_map;
  using _status_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _status_type status;
  using _valid_indices_type =
    perception_ros2_msg::msg::Indices_<ContainerAllocator>;
  _valid_indices_type valid_indices;
  using _objects_type =
    perception_ros2_msg::msg::Objects_<ContainerAllocator>;
  _objects_type objects;
  using _has_pointcloud_type =
    std_msgs::msg::Bool_<ContainerAllocator>;
  _has_pointcloud_type has_pointcloud;
  using _scan_pointcloud_type =
    std::vector<perception_ros2_msg::msg::Point4f_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Point4f_<ContainerAllocator>>::other>;
  _scan_pointcloud_type scan_pointcloud;
  using _has_attention_objects_type =
    std_msgs::msg::Bool_<ContainerAllocator>;
  _has_attention_objects_type has_attention_objects;
  using _attention_objects_type =
    perception_ros2_msg::msg::Objects_<ContainerAllocator>;
  _attention_objects_type attention_objects;
  using _has_freespace_type =
    std_msgs::msg::Bool_<ContainerAllocator>;
  _has_freespace_type has_freespace;
  using _freespace_infos_type =
    perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator>;
  _freespace_infos_type freespace_infos;
  using _has_lanes_type =
    std_msgs::msg::Bool_<ContainerAllocator>;
  _has_lanes_type has_lanes;
  using _lanes_type =
    perception_ros2_msg::msg::Lanes_<ContainerAllocator>;
  _lanes_type lanes;
  using _has_roadedges_type =
    std_msgs::msg::Bool_<ContainerAllocator>;
  _has_roadedges_type has_roadedges;
  using _roadedges_type =
    perception_ros2_msg::msg::RoadEdges_<ContainerAllocator>;
  _roadedges_type roadedges;
  using _has_sematice_indices_type =
    std_msgs::msg::Bool_<ContainerAllocator>;
  _has_sematice_indices_type has_sematice_indices;
  using _non_ground_indices_type =
    perception_ros2_msg::msg::Indices_<ContainerAllocator>;
  _non_ground_indices_type non_ground_indices;
  using _ground_indices_type =
    perception_ros2_msg::msg::Indices_<ContainerAllocator>;
  _ground_indices_type ground_indices;
  using _background_indices_type =
    perception_ros2_msg::msg::Indices_<ContainerAllocator>;
  _background_indices_type background_indices;

  // setters for named parameter idiom
  Type & set__frame_id(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->frame_id = _arg;
    return *this;
  }
  Type & set__timestamp(
    const std_msgs::msg::Float64_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__global_pose(
    const perception_ros2_msg::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->global_pose = _arg;
    return *this;
  }
  Type & set__gps_origin(
    const perception_ros2_msg::msg::Point3d_<ContainerAllocator> & _arg)
  {
    this->gps_origin = _arg;
    return *this;
  }
  Type & set__status_pose_map(
    const perception_ros2_msg::msg::PoseMap_<ContainerAllocator> & _arg)
  {
    this->status_pose_map = _arg;
    return *this;
  }
  Type & set__status(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__valid_indices(
    const perception_ros2_msg::msg::Indices_<ContainerAllocator> & _arg)
  {
    this->valid_indices = _arg;
    return *this;
  }
  Type & set__objects(
    const perception_ros2_msg::msg::Objects_<ContainerAllocator> & _arg)
  {
    this->objects = _arg;
    return *this;
  }
  Type & set__has_pointcloud(
    const std_msgs::msg::Bool_<ContainerAllocator> & _arg)
  {
    this->has_pointcloud = _arg;
    return *this;
  }
  Type & set__scan_pointcloud(
    const std::vector<perception_ros2_msg::msg::Point4f_<ContainerAllocator>, typename ContainerAllocator::template rebind<perception_ros2_msg::msg::Point4f_<ContainerAllocator>>::other> & _arg)
  {
    this->scan_pointcloud = _arg;
    return *this;
  }
  Type & set__has_attention_objects(
    const std_msgs::msg::Bool_<ContainerAllocator> & _arg)
  {
    this->has_attention_objects = _arg;
    return *this;
  }
  Type & set__attention_objects(
    const perception_ros2_msg::msg::Objects_<ContainerAllocator> & _arg)
  {
    this->attention_objects = _arg;
    return *this;
  }
  Type & set__has_freespace(
    const std_msgs::msg::Bool_<ContainerAllocator> & _arg)
  {
    this->has_freespace = _arg;
    return *this;
  }
  Type & set__freespace_infos(
    const perception_ros2_msg::msg::FreeSpaceInfos_<ContainerAllocator> & _arg)
  {
    this->freespace_infos = _arg;
    return *this;
  }
  Type & set__has_lanes(
    const std_msgs::msg::Bool_<ContainerAllocator> & _arg)
  {
    this->has_lanes = _arg;
    return *this;
  }
  Type & set__lanes(
    const perception_ros2_msg::msg::Lanes_<ContainerAllocator> & _arg)
  {
    this->lanes = _arg;
    return *this;
  }
  Type & set__has_roadedges(
    const std_msgs::msg::Bool_<ContainerAllocator> & _arg)
  {
    this->has_roadedges = _arg;
    return *this;
  }
  Type & set__roadedges(
    const perception_ros2_msg::msg::RoadEdges_<ContainerAllocator> & _arg)
  {
    this->roadedges = _arg;
    return *this;
  }
  Type & set__has_sematice_indices(
    const std_msgs::msg::Bool_<ContainerAllocator> & _arg)
  {
    this->has_sematice_indices = _arg;
    return *this;
  }
  Type & set__non_ground_indices(
    const perception_ros2_msg::msg::Indices_<ContainerAllocator> & _arg)
  {
    this->non_ground_indices = _arg;
    return *this;
  }
  Type & set__ground_indices(
    const perception_ros2_msg::msg::Indices_<ContainerAllocator> & _arg)
  {
    this->ground_indices = _arg;
    return *this;
  }
  Type & set__background_indices(
    const perception_ros2_msg::msg::Indices_<ContainerAllocator> & _arg)
  {
    this->background_indices = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_ros2_msg__msg__LidarFrameMsg
    std::shared_ptr<perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_ros2_msg__msg__LidarFrameMsg
    std::shared_ptr<perception_ros2_msg::msg::LidarFrameMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LidarFrameMsg_ & other) const
  {
    if (this->frame_id != other.frame_id) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->global_pose != other.global_pose) {
      return false;
    }
    if (this->gps_origin != other.gps_origin) {
      return false;
    }
    if (this->status_pose_map != other.status_pose_map) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->valid_indices != other.valid_indices) {
      return false;
    }
    if (this->objects != other.objects) {
      return false;
    }
    if (this->has_pointcloud != other.has_pointcloud) {
      return false;
    }
    if (this->scan_pointcloud != other.scan_pointcloud) {
      return false;
    }
    if (this->has_attention_objects != other.has_attention_objects) {
      return false;
    }
    if (this->attention_objects != other.attention_objects) {
      return false;
    }
    if (this->has_freespace != other.has_freespace) {
      return false;
    }
    if (this->freespace_infos != other.freespace_infos) {
      return false;
    }
    if (this->has_lanes != other.has_lanes) {
      return false;
    }
    if (this->lanes != other.lanes) {
      return false;
    }
    if (this->has_roadedges != other.has_roadedges) {
      return false;
    }
    if (this->roadedges != other.roadedges) {
      return false;
    }
    if (this->has_sematice_indices != other.has_sematice_indices) {
      return false;
    }
    if (this->non_ground_indices != other.non_ground_indices) {
      return false;
    }
    if (this->ground_indices != other.ground_indices) {
      return false;
    }
    if (this->background_indices != other.background_indices) {
      return false;
    }
    return true;
  }
  bool operator!=(const LidarFrameMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LidarFrameMsg_

// alias to use template instance with default allocator
using LidarFrameMsg =
  perception_ros2_msg::msg::LidarFrameMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_ros2_msg

#endif  // PERCEPTION_ROS2_MSG__MSG__LIDAR_FRAME_MSG__STRUCT_HPP_
