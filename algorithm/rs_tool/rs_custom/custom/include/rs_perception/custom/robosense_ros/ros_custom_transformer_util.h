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
#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS_ROS_CUSTOM_TRANSFORMER_UTIL_H
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS_ROS_CUSTOM_TRANSFORMER_UTIL_H

#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_dependence/rs_dependence_manager.h"
#ifdef RS_ROS_FOUND
#include "perception_ros_msg/RsPerceptionMsg.h"
#endif  // RS_ROS_FOUND
namespace robosense {
namespace perception {
#ifdef RS_ROS_FOUND
namespace Ros {

class RosTransformUtil {
public:
    // Objects
    static void transform(const std::vector<Object::Ptr> &robo_objects,
                   perception_ros_msg::Objects &ros_objects);

    static void transform(const perception_ros_msg::Objects &ros_objects,
                   std::vector<Object::Ptr> &robo_objects);

    // Freespace
    static void transform(const RsFreeSpace::Ptr &robo_freespaces,
                   perception_ros_msg::FreeSpaceInfos &ros_freespaces);

    static void transform(
    const perception_ros_msg::FreeSpaceInfos &ros_freespaces,
    RsFreeSpace::Ptr &robo_freespaces);

    // Lanes
    static void transform(const std::vector<Lane::Ptr> &robo_lanes,
                   perception_ros_msg::Lanes &ros_lanes);

    static void transform(const perception_ros_msg::Lanes &ros_lanes,
                   std::vector<Lane::Ptr> &robo_lanes);

    // Curbs
    static void transform(const std::vector<Roadedge::Ptr> &robo_curbs,
                   perception_ros_msg::RoadEdges &ros_curbs);

    static void transform(const perception_ros_msg::RoadEdges &ros_curbs,
                   std::vector<Roadedge::Ptr> &robo_curbs);

    // Pointcloud
    static void transform(const RsPointCloudGPT::Ptr &robo_pc,
                   std::vector<perception_ros_msg::Point4f> &ros_pc);

    static void transform(const std::vector<perception_ros_msg::Point4f> &ros_pc,
                   RsPointCloudGPT::Ptr &robo_pc);

    // RsPose
    static void transform(const RsPose::Ptr &robo_pose,
                   perception_ros_msg::Pose &ros_pose);

    static void transform(const perception_ros_msg::Pose &ros_pose,
                   RsPose::Ptr &robo_pose);

    // RsPose Map
    static void transform(const std::map<AxisStatus, RsPose::Ptr> &robo_pose_map,
                   perception_ros_msg::PoseMap &ros_pose_map);

    static void transform(const perception_ros_msg::PoseMap &ros_pose_map,
                   std::map<AxisStatus, RsPose::Ptr> &robo_pose_map);

// Status & RsPose Pair
    static void transform(const AxisStatus status, const RsPose::Ptr &robo_pose,
                   perception_ros_msg::AxisStatusPose &ros_pose);

    static void transform(const perception_ros_msg::AxisStatusPose &ros_pose,
                   AxisStatus &status, RsPose::Ptr &robo_pose);

// Indices
    static void transform(const std::vector<int> &robo_indices,
                   perception_ros_msg::Indices &ros_indices);

    static void transform(const perception_ros_msg::Indices &ros_indices,
                   std::vector<int> &robo_indices);

    // RsVector3d
    static void transform(const RsVector3d &eig, perception_ros_msg::Point3d &point3d) {
        fromRoboToRos(eig, point3d);
    }

    static void transform(const perception_ros_msg::Point3d &point3d, RsVector3d &eig) {
        fromRosToRobo(point3d, eig);
    }

private:
    // RsVector2f
    static void fromRoboToRos(const RsVector2f &robo_point, perception_ros_msg::Point2f &ros_point) {
        ros_point.x.data = robo_point.x;
        ros_point.y.data = robo_point.y;
    }

    static void fromRosToRobo(const perception_ros_msg::Point2f &ros_point, RsVector2f &robo_point) {
        robo_point.x = ros_point.x.data;
        robo_point.y = ros_point.y.data;
    }

    // RsVector3f
    static void fromRoboToRos(const RsVector3f &robo_point, perception_ros_msg::Point3f &ros_point) {
        ros_point.x.data = robo_point.x;
        ros_point.y.data = robo_point.y;
        ros_point.z.data = robo_point.z;
    }

    static void fromRosToRobo(const perception_ros_msg::Point3f &ros_point,
                              RsVector3f &robo_point) {
        robo_point.x = ros_point.x.data;
        robo_point.y = ros_point.y.data;
        robo_point.z = ros_point.z.data;
    }

    // RsVector3d
    static void fromRoboToRos(const RsVector3d &robo_point,
                              perception_ros_msg::Point3d &ros_point) {
        ros_point.x.data = robo_point.x;
        ros_point.y.data = robo_point.y;
        ros_point.z.data = robo_point.z;
    }

    static void fromRosToRobo(const perception_ros_msg::Point3d &ros_point,
                              RsVector3d &robo_point) {
        robo_point.x = ros_point.x.data;
        robo_point.y = ros_point.y.data;
        robo_point.z = ros_point.z.data;
    }

// Lane
    static void fromRoboToRos(const Lane::Ptr &robo_lane,
                              perception_ros_msg::Lane &ros_lane);

    static void fromRosToRobo(const perception_ros_msg::Lane &ros_lane,
                              Lane::Ptr &robo_lane);

// Curb
    static void fromRoboToRos(const Roadedge::Ptr &robo_curb,
                              perception_ros_msg::RoadEdge &ros_curb);

    static void fromRosToRobo(const perception_ros_msg::RoadEdge ros_curb,
                              Roadedge::Ptr &robo_curb);

// curve
    static void fromRoboToRos(const Curve &robo_curve,
                              perception_ros_msg::Curve &ros_curve);

    static void fromRosToRobo(const perception_ros_msg::Curve &ros_curve,
                              Curve &robo_curve);

// Endpoint
    static void fromRoboToRos(const EndPoints &robo_endpoints,
                              perception_ros_msg::EndPoints &ros_endpoints);

    static void fromRosToRobo(const perception_ros_msg::EndPoints &ros_endpoints,
                              EndPoints &robo_endpoints);
};

}   // namespace Ros
#endif  // RS_ROS_FOUND
}   // namespace perception
}   // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS_ROS_CUSTOM_TRANSFORMER_UTIL_H
