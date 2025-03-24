// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_ros2_msg:msg/LidarFrameMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_ros2_msg/msg/lidar_frame_msg__rosidl_typesupport_introspection_c.h"
#include "perception_ros2_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_ros2_msg/msg/lidar_frame_msg__functions.h"
#include "perception_ros2_msg/msg/lidar_frame_msg__struct.h"


// Include directives for member types
// Member `frame_id`
#include "std_msgs/msg/string.h"
// Member `frame_id`
#include "std_msgs/msg/string__rosidl_typesupport_introspection_c.h"
// Member `timestamp`
#include "std_msgs/msg/float64.h"
// Member `timestamp`
#include "std_msgs/msg/float64__rosidl_typesupport_introspection_c.h"
// Member `global_pose`
#include "perception_ros2_msg/msg/pose.h"
// Member `global_pose`
#include "perception_ros2_msg/msg/pose__rosidl_typesupport_introspection_c.h"
// Member `gps_origin`
#include "perception_ros2_msg/msg/point3d.h"
// Member `gps_origin`
#include "perception_ros2_msg/msg/point3d__rosidl_typesupport_introspection_c.h"
// Member `status_pose_map`
#include "perception_ros2_msg/msg/pose_map.h"
// Member `status_pose_map`
#include "perception_ros2_msg/msg/pose_map__rosidl_typesupport_introspection_c.h"
// Member `status`
#include "std_msgs/msg/int32.h"
// Member `status`
#include "std_msgs/msg/int32__rosidl_typesupport_introspection_c.h"
// Member `valid_indices`
// Member `non_ground_indices`
// Member `ground_indices`
// Member `background_indices`
#include "perception_ros2_msg/msg/indices.h"
// Member `valid_indices`
// Member `non_ground_indices`
// Member `ground_indices`
// Member `background_indices`
#include "perception_ros2_msg/msg/indices__rosidl_typesupport_introspection_c.h"
// Member `objects`
// Member `attention_objects`
#include "perception_ros2_msg/msg/objects.h"
// Member `objects`
// Member `attention_objects`
#include "perception_ros2_msg/msg/objects__rosidl_typesupport_introspection_c.h"
// Member `has_pointcloud`
// Member `has_attention_objects`
// Member `has_freespace`
// Member `has_lanes`
// Member `has_roadedges`
// Member `has_sematice_indices`
#include "std_msgs/msg/bool.h"
// Member `has_pointcloud`
// Member `has_attention_objects`
// Member `has_freespace`
// Member `has_lanes`
// Member `has_roadedges`
// Member `has_sematice_indices`
#include "std_msgs/msg/bool__rosidl_typesupport_introspection_c.h"
// Member `scan_pointcloud`
#include "perception_ros2_msg/msg/point4f.h"
// Member `scan_pointcloud`
#include "perception_ros2_msg/msg/point4f__rosidl_typesupport_introspection_c.h"
// Member `freespace_infos`
#include "perception_ros2_msg/msg/free_space_infos.h"
// Member `freespace_infos`
#include "perception_ros2_msg/msg/free_space_infos__rosidl_typesupport_introspection_c.h"
// Member `lanes`
#include "perception_ros2_msg/msg/lanes.h"
// Member `lanes`
#include "perception_ros2_msg/msg/lanes__rosidl_typesupport_introspection_c.h"
// Member `roadedges`
#include "perception_ros2_msg/msg/road_edges.h"
// Member `roadedges`
#include "perception_ros2_msg/msg/road_edges__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_ros2_msg__msg__LidarFrameMsg__init(message_memory);
}

void LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_fini_function(void * message_memory)
{
  perception_ros2_msg__msg__LidarFrameMsg__fini(message_memory);
}

size_t LidarFrameMsg__rosidl_typesupport_introspection_c__size_function__Point4f__scan_pointcloud(
  const void * untyped_member)
{
  const perception_ros2_msg__msg__Point4f__Sequence * member =
    (const perception_ros2_msg__msg__Point4f__Sequence *)(untyped_member);
  return member->size;
}

const void * LidarFrameMsg__rosidl_typesupport_introspection_c__get_const_function__Point4f__scan_pointcloud(
  const void * untyped_member, size_t index)
{
  const perception_ros2_msg__msg__Point4f__Sequence * member =
    (const perception_ros2_msg__msg__Point4f__Sequence *)(untyped_member);
  return &member->data[index];
}

void * LidarFrameMsg__rosidl_typesupport_introspection_c__get_function__Point4f__scan_pointcloud(
  void * untyped_member, size_t index)
{
  perception_ros2_msg__msg__Point4f__Sequence * member =
    (perception_ros2_msg__msg__Point4f__Sequence *)(untyped_member);
  return &member->data[index];
}

bool LidarFrameMsg__rosidl_typesupport_introspection_c__resize_function__Point4f__scan_pointcloud(
  void * untyped_member, size_t size)
{
  perception_ros2_msg__msg__Point4f__Sequence * member =
    (perception_ros2_msg__msg__Point4f__Sequence *)(untyped_member);
  perception_ros2_msg__msg__Point4f__Sequence__fini(member);
  return perception_ros2_msg__msg__Point4f__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[22] = {
  {
    "frame_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, frame_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "global_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, global_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gps_origin",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, gps_origin),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status_pose_map",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, status_pose_map),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "valid_indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, valid_indices),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, objects),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_pointcloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, has_pointcloud),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "scan_pointcloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, scan_pointcloud),  // bytes offset in struct
    NULL,  // default value
    LidarFrameMsg__rosidl_typesupport_introspection_c__size_function__Point4f__scan_pointcloud,  // size() function pointer
    LidarFrameMsg__rosidl_typesupport_introspection_c__get_const_function__Point4f__scan_pointcloud,  // get_const(index) function pointer
    LidarFrameMsg__rosidl_typesupport_introspection_c__get_function__Point4f__scan_pointcloud,  // get(index) function pointer
    LidarFrameMsg__rosidl_typesupport_introspection_c__resize_function__Point4f__scan_pointcloud  // resize(index) function pointer
  },
  {
    "has_attention_objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, has_attention_objects),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "attention_objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, attention_objects),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_freespace",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, has_freespace),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "freespace_infos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, freespace_infos),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_lanes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, has_lanes),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lanes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, lanes),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_roadedges",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, has_roadedges),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "roadedges",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, roadedges),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_sematice_indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, has_sematice_indices),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "non_ground_indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, non_ground_indices),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ground_indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, ground_indices),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "background_indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_ros2_msg__msg__LidarFrameMsg, background_indices),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_members = {
  "perception_ros2_msg__msg",  // message namespace
  "LidarFrameMsg",  // message name
  22,  // number of fields
  sizeof(perception_ros2_msg__msg__LidarFrameMsg),
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array,  // message members
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_type_support_handle = {
  0,
  &LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_ros2_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, LidarFrameMsg)() {
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, String)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float64)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Pose)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Point3d)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, PoseMap)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int32)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Indices)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Objects)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Bool)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[9].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Point4f)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[10].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Bool)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[11].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Objects)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[12].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Bool)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[13].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, FreeSpaceInfos)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[14].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Bool)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[15].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Lanes)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[16].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Bool)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[17].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, RoadEdges)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[18].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Bool)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[19].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Indices)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[20].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Indices)();
  LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_member_array[21].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_ros2_msg, msg, Indices)();
  if (!LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_type_support_handle.typesupport_identifier) {
    LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &LidarFrameMsg__rosidl_typesupport_introspection_c__LidarFrameMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
