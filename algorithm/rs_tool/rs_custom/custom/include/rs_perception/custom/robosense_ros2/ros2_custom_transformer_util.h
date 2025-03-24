/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS2_ROS2_CUSTOM_TRANSFORMER_UTIL_H
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS2_ROS2_CUSTOM_TRANSFORMER_UTIL_H

#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS2_FOUND
#include "perception_ros2_msg/msg/rs_perception_msg.hpp"
#endif  // RS_ROS2_FOUND

namespace robosense {
namespace perception {

#ifdef RS_ROS2_FOUND
namespace Ros2 {

class Ros2TransformUtil {
public:
    // Objects
    static void transform(const VecObjectPtr& robo_objcets, perception_ros2_msg::msg::Objects& ros2_objects);

    static void transform(const perception_ros2_msg::msg::Objects& ros2_objects, VecObjectPtr& robo_objects);

    // Freespace
    static void transform(const RsFreeSpace::Ptr& robo_freespaces,
                          perception_ros2_msg::msg::FreeSpaceInfos& ros2_freespaces);

    static void transform(const perception_ros2_msg::msg::FreeSpaceInfos& ros2_freespaces,
                          RsFreeSpace::Ptr& robo_freespaces);

    // Lanes
    static void transform(const std::vector<Lane::Ptr>& robo_lanes, perception_ros2_msg::msg::Lanes& ros2_lanes);

    static void transform(const perception_ros2_msg::msg::Lanes& ros2_lanes, std::vector<Lane::Ptr>& robo_lanes);

    // Road edges
    static void transform(const std::vector<Roadedge::Ptr>& robo_curbs,
                          perception_ros2_msg::msg::RoadEdges& ros2_curbs);

    static void transform(const perception_ros2_msg::msg::RoadEdges& ros2_curbs,
                          std::vector<Roadedge::Ptr>& robo_curbs);

    // Point cloud
    static void transform(const RsPointCloudGPT::Ptr& robo_pc,
                          std::vector<perception_ros2_msg::msg::Point4f>& ros2_pc);

    static void transform(const std::vector<perception_ros2_msg::msg::Point4f>& ros2_pc,
                          RsPointCloudGPT::Ptr& robo_pc);

    // RsPose
    static void transform(const RsPose::Ptr& robo_pose, perception_ros2_msg::msg::Pose& ros2_pose);

    static void transform(const perception_ros2_msg::msg::Pose& ros2_pose, RsPose::Ptr& robo_pose);

    // RsPose Map
    static void transform(const std::map<AxisStatus, RsPose::Ptr>& robo_pose_map,
                          perception_ros2_msg::msg::PoseMap& ros2_pose_map);

    static void transform(const perception_ros2_msg::msg::PoseMap& ros2_pose_map,
                          std::map<AxisStatus, RsPose::Ptr>& robo_pose_map);

    // Status & RsPose Pair
    static void transform(const AxisStatus status, const RsPose::Ptr& robo_pose,
                          perception_ros2_msg::msg::AxisStatusPose& ros2_pose);

    static void transform(const perception_ros2_msg::msg::AxisStatusPose& ros2_pose,
                          AxisStatus& status, RsPose::Ptr& robo_pose);

    // Indices
    static void transform(const VecInt& robo_indices, perception_ros2_msg::msg::Indices& ros2_indices);

    static void transform(const perception_ros2_msg::msg::Indices& ros2_indices, VecInt& robo_indices);

    // GPS ORIGIN
    static void transform(const RsVector3d& robo_gps_origin, perception_ros2_msg::msg::Point3d& ros2_gps_origin) {
        ros2_gps_origin.x.data = robo_gps_origin.x;
        ros2_gps_origin.y.data = robo_gps_origin.y;
        ros2_gps_origin.z.data = robo_gps_origin.z;
    }

    static void transform(const perception_ros2_msg::msg::Point3d& ros2_gps_origin, RsVector3d& robo_gps_origin) {
        robo_gps_origin.x = ros2_gps_origin.x.data;
        robo_gps_origin.y = ros2_gps_origin.y.data;
        robo_gps_origin.z = ros2_gps_origin.z.data;
    }

private:
    // RsVector2f
    static void fromRoboToRos2(const RsVector2f &robo_point, perception_ros2_msg::msg::Point2f &ros2_point) {
        ros2_point.x.data = robo_point.x;
        ros2_point.y.data = robo_point.y;
    }

    static void fromRos2ToRobo(const perception_ros2_msg::msg::Point2f &ros2_point, RsVector2f &robo_point) {
        robo_point.x = ros2_point.x.data;
        robo_point.y = ros2_point.y.data;
    }

    // RsVector3f
    static void fromRoboToRos2(const RsVector3f &robo_point, perception_ros2_msg::msg::Point3f &ros2_point) {
        ros2_point.x.data = robo_point.x;
        ros2_point.y.data = robo_point.y;
        ros2_point.z.data = robo_point.z;
    }

    static void fromRos2ToRobo(const perception_ros2_msg::msg::Point3f &ros2_point,
                              RsVector3f &robo_point) {
        robo_point.x = ros2_point.x.data;
        robo_point.y = ros2_point.y.data;
        robo_point.z = ros2_point.z.data;
    }
    
    // Lane
    static void fromRoboToRos2(const Lane::Ptr &robo_lane, perception_ros2_msg::msg::Lane &ros2_lane);

    static void fromRos2ToRobo(const perception_ros2_msg::msg::Lane &ros2_lane, Lane::Ptr &robo_lane);

    // Curb
    static void fromRoboToRos2(const Roadedge::Ptr &robo_curb, perception_ros2_msg::msg::RoadEdge &ros2_curb);

    static void fromRos2ToRobo(const perception_ros2_msg::msg::RoadEdge &ros2_curb, Roadedge::Ptr &robo_curb);

    // curve
    static void fromRoboToRos2(const Curve &robo_curve, perception_ros2_msg::msg::Curve &ros2_curve);

    static void fromRos2ToRobo(const perception_ros2_msg::msg::Curve &ros2_curve, Curve &robo_curve);

    // Endpoint
    static void fromRoboToRos2(const EndPoints &robo_endpoints, perception_ros2_msg::msg::EndPoints &ros2_endpoints);

    static void fromRos2ToRobo(const perception_ros2_msg::msg::EndPoints &ros2_endpoints, EndPoints &robo_endpoints);
};



}  // namespace Ros2
#endif  // RS_ROS2_FOUND

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS2_ROS2_CUSTOM_TRANSFORMER_UTIL_H
