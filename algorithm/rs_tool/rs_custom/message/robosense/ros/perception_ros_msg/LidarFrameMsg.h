// Generated by gencpp from file perception_ros_msg/LidarFrameMsg.msg
// DO NOT EDIT!


#ifndef PERCEPTION_ROS_MSG_MESSAGE_LIDARFRAMEMSG_H
#define PERCEPTION_ROS_MSG_MESSAGE_LIDARFRAMEMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <perception_ros_msg/Pose.h>
#include <perception_ros_msg/Point3d.h>
#include <perception_ros_msg/PoseMap.h>
#include <std_msgs/Int32.h>
#include <perception_ros_msg/Indices.h>
#include <perception_ros_msg/Objects.h>
#include <std_msgs/Bool.h>
#include <perception_ros_msg/Point4f.h>
#include <std_msgs/Bool.h>
#include <perception_ros_msg/Objects.h>
#include <std_msgs/Bool.h>
#include <perception_ros_msg/FreeSpaceInfos.h>
#include <std_msgs/Bool.h>
#include <perception_ros_msg/Lanes.h>
#include <std_msgs/Bool.h>
#include <perception_ros_msg/RoadEdges.h>
#include <std_msgs/Bool.h>
#include <perception_ros_msg/Indices.h>
#include <perception_ros_msg/Indices.h>
#include <perception_ros_msg/Indices.h>

namespace perception_ros_msg
{
template <class ContainerAllocator>
struct LidarFrameMsg_
{
  typedef LidarFrameMsg_<ContainerAllocator> Type;

  LidarFrameMsg_()
    : frame_id()
    , timestamp()
    , global_pose()
    , gps_origin()
    , status_pose_map()
    , status()
    , valid_indices()
    , objects()
    , has_pointcloud()
    , scan_pointcloud()
    , has_attention_objects()
    , attention_objects()
    , has_freespace()
    , freespace_infos()
    , has_lanes()
    , lanes()
    , has_roadedges()
    , roadedges()
    , has_sematice_indices()
    , non_ground_indices()
    , ground_indices()
    , background_indices()  {
    }
  LidarFrameMsg_(const ContainerAllocator& _alloc)
    : frame_id(_alloc)
    , timestamp(_alloc)
    , global_pose(_alloc)
    , gps_origin(_alloc)
    , status_pose_map(_alloc)
    , status(_alloc)
    , valid_indices(_alloc)
    , objects(_alloc)
    , has_pointcloud(_alloc)
    , scan_pointcloud(_alloc)
    , has_attention_objects(_alloc)
    , attention_objects(_alloc)
    , has_freespace(_alloc)
    , freespace_infos(_alloc)
    , has_lanes(_alloc)
    , lanes(_alloc)
    , has_roadedges(_alloc)
    , roadedges(_alloc)
    , has_sematice_indices(_alloc)
    , non_ground_indices(_alloc)
    , ground_indices(_alloc)
    , background_indices(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::String_<ContainerAllocator>  _frame_id_type;
  _frame_id_type frame_id;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _timestamp_type;
  _timestamp_type timestamp;

   typedef  ::perception_ros_msg::Pose_<ContainerAllocator>  _global_pose_type;
  _global_pose_type global_pose;

   typedef  ::perception_ros_msg::Point3d_<ContainerAllocator>  _gps_origin_type;
  _gps_origin_type gps_origin;

   typedef  ::perception_ros_msg::PoseMap_<ContainerAllocator>  _status_pose_map_type;
  _status_pose_map_type status_pose_map;

   typedef  ::std_msgs::Int32_<ContainerAllocator>  _status_type;
  _status_type status;

   typedef  ::perception_ros_msg::Indices_<ContainerAllocator>  _valid_indices_type;
  _valid_indices_type valid_indices;

   typedef  ::perception_ros_msg::Objects_<ContainerAllocator>  _objects_type;
  _objects_type objects;

   typedef  ::std_msgs::Bool_<ContainerAllocator>  _has_pointcloud_type;
  _has_pointcloud_type has_pointcloud;

   typedef std::vector< ::perception_ros_msg::Point4f_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::perception_ros_msg::Point4f_<ContainerAllocator> >::other >  _scan_pointcloud_type;
  _scan_pointcloud_type scan_pointcloud;

   typedef  ::std_msgs::Bool_<ContainerAllocator>  _has_attention_objects_type;
  _has_attention_objects_type has_attention_objects;

   typedef  ::perception_ros_msg::Objects_<ContainerAllocator>  _attention_objects_type;
  _attention_objects_type attention_objects;

   typedef  ::std_msgs::Bool_<ContainerAllocator>  _has_freespace_type;
  _has_freespace_type has_freespace;

   typedef  ::perception_ros_msg::FreeSpaceInfos_<ContainerAllocator>  _freespace_infos_type;
  _freespace_infos_type freespace_infos;

   typedef  ::std_msgs::Bool_<ContainerAllocator>  _has_lanes_type;
  _has_lanes_type has_lanes;

   typedef  ::perception_ros_msg::Lanes_<ContainerAllocator>  _lanes_type;
  _lanes_type lanes;

   typedef  ::std_msgs::Bool_<ContainerAllocator>  _has_roadedges_type;
  _has_roadedges_type has_roadedges;

   typedef  ::perception_ros_msg::RoadEdges_<ContainerAllocator>  _roadedges_type;
  _roadedges_type roadedges;

   typedef  ::std_msgs::Bool_<ContainerAllocator>  _has_sematice_indices_type;
  _has_sematice_indices_type has_sematice_indices;

   typedef  ::perception_ros_msg::Indices_<ContainerAllocator>  _non_ground_indices_type;
  _non_ground_indices_type non_ground_indices;

   typedef  ::perception_ros_msg::Indices_<ContainerAllocator>  _ground_indices_type;
  _ground_indices_type ground_indices;

   typedef  ::perception_ros_msg::Indices_<ContainerAllocator>  _background_indices_type;
  _background_indices_type background_indices;





  typedef boost::shared_ptr< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> const> ConstPtr;

}; // struct LidarFrameMsg_

typedef ::perception_ros_msg::LidarFrameMsg_<std::allocator<void> > LidarFrameMsg;

typedef boost::shared_ptr< ::perception_ros_msg::LidarFrameMsg > LidarFrameMsgPtr;
typedef boost::shared_ptr< ::perception_ros_msg::LidarFrameMsg const> LidarFrameMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator1> & lhs, const ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator2> & rhs)
{
  return lhs.frame_id == rhs.frame_id &&
    lhs.timestamp == rhs.timestamp &&
    lhs.global_pose == rhs.global_pose &&
    lhs.gps_origin == rhs.gps_origin &&
    lhs.status_pose_map == rhs.status_pose_map &&
    lhs.status == rhs.status &&
    lhs.valid_indices == rhs.valid_indices &&
    lhs.objects == rhs.objects &&
    lhs.has_pointcloud == rhs.has_pointcloud &&
    lhs.scan_pointcloud == rhs.scan_pointcloud &&
    lhs.has_attention_objects == rhs.has_attention_objects &&
    lhs.attention_objects == rhs.attention_objects &&
    lhs.has_freespace == rhs.has_freespace &&
    lhs.freespace_infos == rhs.freespace_infos &&
    lhs.has_lanes == rhs.has_lanes &&
    lhs.lanes == rhs.lanes &&
    lhs.has_roadedges == rhs.has_roadedges &&
    lhs.roadedges == rhs.roadedges &&
    lhs.has_sematice_indices == rhs.has_sematice_indices &&
    lhs.non_ground_indices == rhs.non_ground_indices &&
    lhs.ground_indices == rhs.ground_indices &&
    lhs.background_indices == rhs.background_indices;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator1> & lhs, const ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace perception_ros_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "caa4cd6acb7dc86f4a76180c583c5946";
  }

  static const char* value(const ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcaa4cd6acb7dc86fULL;
  static const uint64_t static_value2 = 0x4a76180c583c5946ULL;
};

template<class ContainerAllocator>
struct DataType< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "perception_ros_msg/LidarFrameMsg";
  }

  static const char* value(const ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
"std_msgs/String frame_id\n"
"std_msgs/Float64 timestamp \n"
"Pose             global_pose \n"
"Point3d          gps_origin \n"
"PoseMap          status_pose_map \n"
"std_msgs/Int32   status \n"
"Indices          valid_indices \n"
"Objects          objects \n"
"\n"
"std_msgs/Bool  has_pointcloud \n"
"Point4f[]      scan_pointcloud \n"
"\n"
"std_msgs/Bool  has_attention_objects \n"
"Objects        attention_objects \n"
"\n"
"std_msgs/Bool  has_freespace \n"
"FreeSpaceInfos freespace_infos \n"
"\n"
"std_msgs/Bool   has_lanes \n"
"Lanes           lanes \n"
"\n"
"std_msgs/Bool   has_roadedges \n"
"RoadEdges       roadedges \n"
"\n"
"std_msgs/Bool   has_sematice_indices \n"
"Indices         non_ground_indices\n"
"Indices         ground_indices \n"
"Indices         background_indices \n"
"\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Float64\n"
"float64 data\n"
"================================================================================\n"
"MSG: perception_ros_msg/Pose\n"
"std_msgs/Float32 x \n"
"std_msgs/Float32 y \n"
"std_msgs/Float32 z \n"
"\n"
"std_msgs/Float32 roll \n"
"std_msgs/Float32 pitch\n"
"std_msgs/Float32 yaw\n"
"\n"
"std_msgs/Int32 status \n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Float32\n"
"float32 data\n"
"================================================================================\n"
"MSG: std_msgs/Int32\n"
"int32 data\n"
"================================================================================\n"
"MSG: perception_ros_msg/Point3d\n"
"std_msgs/Float64 x \n"
"std_msgs/Float64 y \n"
"std_msgs/Float64 z \n"
"\n"
"================================================================================\n"
"MSG: perception_ros_msg/PoseMap\n"
"AxisStatusPose[] status_poses \n"
"\n"
"================================================================================\n"
"MSG: perception_ros_msg/AxisStatusPose\n"
"std_msgs/Int32 status \n"
"Pose           pose \n"
"================================================================================\n"
"MSG: perception_ros_msg/Indices\n"
"std_msgs/Int32[]    indices\n"
"================================================================================\n"
"MSG: perception_ros_msg/Objects\n"
"Object[]         objects\n"
"================================================================================\n"
"MSG: perception_ros_msg/Object\n"
"CoreInfo coreinfo\n"
"std_msgs/Bool  hassupplmentinfo \n"
"SupplementInfo supplementinfo \n"
"\n"
"================================================================================\n"
"MSG: perception_ros_msg/CoreInfo\n"
"std_msgs/Float64 timestamp\n"
"\n"
"std_msgs/Int32 priority_id\n"
"std_msgs/Float32 exist_confidence \n"
"Point3f          center \n"
"Point3f          center_cov \n"
"Point3f          size \n"
"Point3f          size_cov \n"
"Point3f          direction \n"
"Point3f          direction_cov         \n"
"std_msgs/Int32   type \n"
"std_msgs/Float32 type_confidence \n"
"std_msgs/Int32   attention_type \n"
"std_msgs/Int32   motion_state\n"
"std_msgs/Int32   lane_pos\n"
"std_msgs/Int32   trakcer_id \n"
"std_msgs/Float64 age \n"
"Point3f          velocity \n"
"Point3f          velocity_cov \n"
"Point3f          acceleration \n"
"Point3f          acceleration_cov \n"
"std_msgs/Float32 angle_velocity \n"
"std_msgs/Float32 angle_velocity_cov \n"
"std_msgs/Float32 angle_acceleration \n"
"std_msgs/Float32 angle_acceleration_cov  \n"
"Point3f          anchor \n"
"Point3f          nearest_point \n"
"\n"
"\n"
"================================================================================\n"
"MSG: perception_ros_msg/Point3f\n"
"std_msgs/Float32 x \n"
"std_msgs/Float32 y \n"
"std_msgs/Float32 z \n"
"================================================================================\n"
"MSG: std_msgs/Bool\n"
"bool data\n"
"================================================================================\n"
"MSG: perception_ros_msg/SupplementInfo\n"
"std_msgs/UInt32     unique_id\n"
"\n"
"Point3f[]           polygon\n"
"std_msgs/Int32      left_point_index \n"
"std_msgs/Int32      right_point_index \n"
"\n"
"std_msgs/Int32[]    cloud_indices\n"
"\n"
"std_msgs/Float32[]  latent_types\n"
"std_msgs/Int32      size_type \n"
"std_msgs/Int32      mode \n"
"std_msgs/Bool       in_roi \n"
"std_msgs/Int32      tracking_state\n"
"Point3f             geo_center \n"
"Point3f             geo_size \n"
"\n"
"\n"
"\n"
"Point3f[]           trajectory \n"
"Point3f[]           history_velocity\n"
"std_msgs/Int32[]    history_type \n"
"std_msgs/Int32      gps_mode \n"
"Point3d             gps_info \n"
"\n"
"================================================================================\n"
"MSG: std_msgs/UInt32\n"
"uint32 data\n"
"================================================================================\n"
"MSG: perception_ros_msg/Point4f\n"
"std_msgs/Float32 x \n"
"std_msgs/Float32 y \n"
"std_msgs/Float32 z \n"
"std_msgs/Float32 i \n"
"\n"
"================================================================================\n"
"MSG: perception_ros_msg/FreeSpaceInfos\n"
"Point3f[]        fs_pts\n"
"std_msgs/Float32[] fs_confidence \n"
"\n"
"================================================================================\n"
"MSG: perception_ros_msg/Lanes\n"
"Lane[] lanes\n"
"================================================================================\n"
"MSG: perception_ros_msg/Lane\n"
"std_msgs/Int32   lane_id \n"
"Curve            curve \n"
"EndPoints        end_points \n"
"std_msgs/Int32   measure_status  \n"
"std_msgs/Float32 confidence \n"
"\n"
"================================================================================\n"
"MSG: perception_ros_msg/Curve\n"
"std_msgs/Float32 x_start \n"
"std_msgs/Float32 x_end \n"
"std_msgs/Float32 a \n"
"std_msgs/Float32 b \n"
"std_msgs/Float32 c \n"
"std_msgs/Float32 d \n"
"================================================================================\n"
"MSG: perception_ros_msg/EndPoints\n"
"Point2f start \n"
"Point2f end \n"
"\n"
"================================================================================\n"
"MSG: perception_ros_msg/Point2f\n"
"std_msgs/Float32 x \n"
"std_msgs/Float32 y \n"
"\n"
"================================================================================\n"
"MSG: perception_ros_msg/RoadEdges\n"
"RoadEdge[] curbs\n"
"\n"
"================================================================================\n"
"MSG: perception_ros_msg/RoadEdge\n"
"std_msgs/Int32   roadedge_id  \n"
"Curve            curve \n"
"EndPoints        end_points \n"
"std_msgs/Int32   measure_status  \n"
"std_msgs/Float32 confidence \n"
;
  }

  static const char* value(const ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.frame_id);
      stream.next(m.timestamp);
      stream.next(m.global_pose);
      stream.next(m.gps_origin);
      stream.next(m.status_pose_map);
      stream.next(m.status);
      stream.next(m.valid_indices);
      stream.next(m.objects);
      stream.next(m.has_pointcloud);
      stream.next(m.scan_pointcloud);
      stream.next(m.has_attention_objects);
      stream.next(m.attention_objects);
      stream.next(m.has_freespace);
      stream.next(m.freespace_infos);
      stream.next(m.has_lanes);
      stream.next(m.lanes);
      stream.next(m.has_roadedges);
      stream.next(m.roadedges);
      stream.next(m.has_sematice_indices);
      stream.next(m.non_ground_indices);
      stream.next(m.ground_indices);
      stream.next(m.background_indices);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LidarFrameMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::perception_ros_msg::LidarFrameMsg_<ContainerAllocator>& v)
  {
    s << indent << "frame_id: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.frame_id);
    s << indent << "timestamp: ";
    s << std::endl;
    Printer< ::std_msgs::Float64_<ContainerAllocator> >::stream(s, indent + "  ", v.timestamp);
    s << indent << "global_pose: ";
    s << std::endl;
    Printer< ::perception_ros_msg::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.global_pose);
    s << indent << "gps_origin: ";
    s << std::endl;
    Printer< ::perception_ros_msg::Point3d_<ContainerAllocator> >::stream(s, indent + "  ", v.gps_origin);
    s << indent << "status_pose_map: ";
    s << std::endl;
    Printer< ::perception_ros_msg::PoseMap_<ContainerAllocator> >::stream(s, indent + "  ", v.status_pose_map);
    s << indent << "status: ";
    s << std::endl;
    Printer< ::std_msgs::Int32_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
    s << indent << "valid_indices: ";
    s << std::endl;
    Printer< ::perception_ros_msg::Indices_<ContainerAllocator> >::stream(s, indent + "  ", v.valid_indices);
    s << indent << "objects: ";
    s << std::endl;
    Printer< ::perception_ros_msg::Objects_<ContainerAllocator> >::stream(s, indent + "  ", v.objects);
    s << indent << "has_pointcloud: ";
    s << std::endl;
    Printer< ::std_msgs::Bool_<ContainerAllocator> >::stream(s, indent + "  ", v.has_pointcloud);
    s << indent << "scan_pointcloud[]" << std::endl;
    for (size_t i = 0; i < v.scan_pointcloud.size(); ++i)
    {
      s << indent << "  scan_pointcloud[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::perception_ros_msg::Point4f_<ContainerAllocator> >::stream(s, indent + "    ", v.scan_pointcloud[i]);
    }
    s << indent << "has_attention_objects: ";
    s << std::endl;
    Printer< ::std_msgs::Bool_<ContainerAllocator> >::stream(s, indent + "  ", v.has_attention_objects);
    s << indent << "attention_objects: ";
    s << std::endl;
    Printer< ::perception_ros_msg::Objects_<ContainerAllocator> >::stream(s, indent + "  ", v.attention_objects);
    s << indent << "has_freespace: ";
    s << std::endl;
    Printer< ::std_msgs::Bool_<ContainerAllocator> >::stream(s, indent + "  ", v.has_freespace);
    s << indent << "freespace_infos: ";
    s << std::endl;
    Printer< ::perception_ros_msg::FreeSpaceInfos_<ContainerAllocator> >::stream(s, indent + "  ", v.freespace_infos);
    s << indent << "has_lanes: ";
    s << std::endl;
    Printer< ::std_msgs::Bool_<ContainerAllocator> >::stream(s, indent + "  ", v.has_lanes);
    s << indent << "lanes: ";
    s << std::endl;
    Printer< ::perception_ros_msg::Lanes_<ContainerAllocator> >::stream(s, indent + "  ", v.lanes);
    s << indent << "has_roadedges: ";
    s << std::endl;
    Printer< ::std_msgs::Bool_<ContainerAllocator> >::stream(s, indent + "  ", v.has_roadedges);
    s << indent << "roadedges: ";
    s << std::endl;
    Printer< ::perception_ros_msg::RoadEdges_<ContainerAllocator> >::stream(s, indent + "  ", v.roadedges);
    s << indent << "has_sematice_indices: ";
    s << std::endl;
    Printer< ::std_msgs::Bool_<ContainerAllocator> >::stream(s, indent + "  ", v.has_sematice_indices);
    s << indent << "non_ground_indices: ";
    s << std::endl;
    Printer< ::perception_ros_msg::Indices_<ContainerAllocator> >::stream(s, indent + "  ", v.non_ground_indices);
    s << indent << "ground_indices: ";
    s << std::endl;
    Printer< ::perception_ros_msg::Indices_<ContainerAllocator> >::stream(s, indent + "  ", v.ground_indices);
    s << indent << "background_indices: ";
    s << std::endl;
    Printer< ::perception_ros_msg::Indices_<ContainerAllocator> >::stream(s, indent + "  ", v.background_indices);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PERCEPTION_ROS_MSG_MESSAGE_LIDARFRAMEMSG_H
