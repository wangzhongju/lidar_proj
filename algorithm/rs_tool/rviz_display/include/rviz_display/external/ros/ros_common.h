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

#ifndef RS_PERCEPTION_RVIZ_DISPLAY_EXTENAL_COMMON_ROS_COMMON_H_
#define RS_PERCEPTION_RVIZ_DISPLAY_EXTENAL_COMMON_ROS_COMMON_H_

#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND

#include "rviz_display/external/common/base_marker_pub.h"

namespace robosense {
namespace perception {

using ROS_MARKER = visualization_msgs::Marker;
using ROS_MARKER_ARRAY = visualization_msgs::MarkerArray;

inline void transColor(const Marker &in_marker,
                       ROS_MARKER &marker) {
    marker.color.r = in_marker.color_type.r;
    marker.color.g = in_marker.color_type.g;
    marker.color.b = in_marker.color_type.b;
    marker.color.a = in_marker.color_type.a;
}

inline void transScale(const Marker &in_marker,
                       ROS_MARKER &marker) {
    marker.scale.x = in_marker.scale_type.x;
    marker.scale.y = in_marker.scale_type.y;
    marker.scale.z = in_marker.scale_type.z;
}

inline void tranMarkerType(const Marker &in_marker,
                           ROS_MARKER &marker) {
    switch (in_marker.type) {
        case MarkerType::ARROW:
            marker.type = ROS_MARKER::ARROW;
            break;
        case MarkerType::CUBE:
            marker.type = ROS_MARKER::CUBE;
            break;
        case MarkerType::SPHERE:
            marker.type = ROS_MARKER::SPHERE;
            break;
        case MarkerType::CYLINDER:
            marker.type = ROS_MARKER::CYLINDER;
            break;
        case MarkerType::LINE_STRIP:
            marker.type = ROS_MARKER::LINE_STRIP;
            break;
        case MarkerType::LINE_LIST:
            marker.type = ROS_MARKER::LINE_LIST;
            break;
        case MarkerType::CUBE_LIST:
            marker.type = ROS_MARKER::CUBE_LIST;
            break;
        case MarkerType::SPHERE_LIST:
            marker.type = ROS_MARKER::SPHERE_LIST;
            break;
        case MarkerType::POINTS:
            marker.type = ROS_MARKER::POINTS;
            break;
        case MarkerType::TEXT_VIEW_FACING:
            marker.type = ROS_MARKER::TEXT_VIEW_FACING;
            break;
        case MarkerType::MESH_RESOURCE:
            marker.type = ROS_MARKER::MESH_RESOURCE;
            break;
        case MarkerType::TRIANGLE_LIST:
            marker.type = ROS_MARKER::TRIANGLE_LIST;
            break;
    }
}

inline void tranMarkerAction(const Marker &in_marker,
                             ROS_MARKER &marker) {
    switch (in_marker.action) {
        case ActionType::ADD:
            marker.action = ROS_MARKER::ADD;
            break;
        case ActionType::MODIFY:
            marker.action = ROS_MARKER::MODIFY;
            break;
        case ActionType::DELETE:
            marker.action = ROS_MARKER::DELETE;
            break;
        case ActionType::DELETEALL:
            marker.action = ROS_MARKER::DELETEALL;
            break;
    }
}

inline void tranMarkerPosition(const Marker &in_marker,
                               ROS_MARKER &marker) {
    marker.pose.position.x = in_marker.position.x;
    marker.pose.position.y = in_marker.position.y;
    marker.pose.position.z = in_marker.position.z;
}

inline void tranMarkerOrientation(const Marker &in_marker,
                                  ROS_MARKER &marker) {
    marker.pose.orientation.x = in_marker.orientation.x;
    marker.pose.orientation.y = in_marker.orientation.y;
    marker.pose.orientation.z = in_marker.orientation.z;
    marker.pose.orientation.w = in_marker.orientation.w;
}

inline void tranMarkerPoints(const Marker &in_marker,
                             ROS_MARKER &marker) {
    marker.points.resize(in_marker.points.size());
    for (size_t i = 0; i < in_marker.points.size(); ++i) {
        marker.points[i].x = in_marker.points[i].x;
        marker.points[i].y = in_marker.points[i].y;
        marker.points[i].z = in_marker.points[i].z;
    }
    marker.colors.resize(in_marker.colors.size());
    for (size_t i = 0; i < in_marker.colors.size(); ++i) {
        marker.colors[i].r = in_marker.colors[i].r;
        marker.colors[i].g = in_marker.colors[i].g;
        marker.colors[i].b = in_marker.colors[i].b;
        marker.colors[i].a = in_marker.colors[i].a;
    }
}

inline void transMarker(const Marker &in_marker, ROS_MARKER &out_marker) {
    transColor(in_marker, out_marker);
    transScale(in_marker, out_marker);
    tranMarkerType(in_marker, out_marker);
    tranMarkerAction(in_marker, out_marker);
    tranMarkerPosition(in_marker, out_marker);
    tranMarkerOrientation(in_marker, out_marker);
    tranMarkerPoints(in_marker, out_marker);

    out_marker.ns = in_marker.ns;
    out_marker.id = in_marker.id;
    out_marker.header.frame_id = in_marker.frame_id;
    out_marker.text = in_marker.text;
}

}  // namespace perception
}  // namespace robosense

#endif  // RS_ROS_FOUND

#endif  // RS_PERCEPTION_RVIZ_DISPLAY_EXTENAL_COMMON_ROS_COMMON_H_
