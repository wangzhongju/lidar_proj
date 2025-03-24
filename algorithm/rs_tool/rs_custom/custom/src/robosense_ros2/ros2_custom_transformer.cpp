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

#include "rs_perception/custom/robosense_ros2/ros2_custom_transformer.h"
#include "rs_dependence/rs_dependence_manager.h"

namespace robosense {
namespace perception {
#ifdef RS_ROS2_FOUND
namespace Ros2 {

// RsPerceptionMsg -> ros2 msg
void transform(const RsPerceptionMsg::Ptr& rs_msg, perception_ros2_msg::msg::RsPerceptionMsg::SharedPtr& ros2_msg,
               const RsCommonCustomMsgParams& custom_params) {
    ros2_msg.reset(new perception_ros2_msg::msg::RsPerceptionMsg());

    // device_id
    ros2_msg->device_id.data = custom_params.device_id;

    const auto& rs_result_msg = rs_msg->rs_lidar_result_ptr;
    auto& ros2_result_msg = ros2_msg->lidarframe;

    // frame_id
    ros2_result_msg.frame_id.data = rs_result_msg->frame_id;

    // timestamp
    ros2_result_msg.timestamp.data = rs_result_msg->timestamp;

    // status
    ros2_result_msg.status.data = static_cast<int>(rs_result_msg->status);

    // global_pose_ptr
    Ros2TransformUtil::transform(rs_result_msg->global_pose_ptr, ros2_result_msg.global_pose);

    // gps_origin
    Ros2TransformUtil::transform(rs_result_msg->gps_origin, ros2_result_msg.gps_origin);

    // valid indices
    Ros2TransformUtil::transform(rs_result_msg->valid_indices, ros2_result_msg.valid_indices);

    // objects
    Ros2TransformUtil::transform(rs_result_msg->objects, ros2_result_msg.objects);

    //optional
    if (custom_params.send_point_cloud) {
        Ros2TransformUtil::transform(rs_result_msg->scan_ptr,ros2_result_msg.scan_pointcloud);
        ros2_result_msg.has_pointcloud.data = true;
    }
    else {
        ros2_result_msg.has_pointcloud.data = false;
    }

    if (custom_params.send_attention_objects) {
        Ros2TransformUtil::transform(rs_result_msg->attention_objects,ros2_result_msg.attention_objects);
        ros2_result_msg.has_attention_objects.data = true;
    }
    else {
        ros2_result_msg.has_attention_objects.data = false;
    }

    if (custom_params.send_freespace) {
        Ros2TransformUtil::transform(rs_result_msg->freespace_ptr,ros2_result_msg.freespace_infos);
        ros2_result_msg.has_freespace.data = true;
    }
    else {
        ros2_result_msg.has_freespace.data = false;
    }

    if (custom_params.send_lane) {
        Ros2TransformUtil::transform(rs_result_msg->lanes,ros2_result_msg.lanes);
        ros2_result_msg.has_lanes.data = true;
    }
    else {
        ros2_result_msg.has_lanes.data = false;
    }

    if (custom_params.send_roadedge) {
        Ros2TransformUtil::transform(rs_result_msg->roadedges,ros2_result_msg.roadedges);
        ros2_result_msg.has_roadedges.data = true;
    }
    else {
        ros2_result_msg.has_roadedges.data = false;
    }

    if (custom_params.send_sematic) {
        Ros2TransformUtil::transform(rs_result_msg->ground_indices,ros2_result_msg.ground_indices);
        Ros2TransformUtil::transform(rs_result_msg->background_indices,ros2_result_msg.background_indices);
        Ros2TransformUtil::transform(rs_result_msg->non_ground_indices,ros2_result_msg.non_ground_indices);
        ros2_result_msg.has_sematice_indices.data = true;
    }
    else {
        ros2_result_msg.has_sematice_indices.data = false;
    }
}

// ros2 msg -> RsPerceptionMsg
void transform(const perception_ros2_msg::msg::RsPerceptionMsg::ConstSharedPtr& ros2_msg, RsPerceptionMsg::Ptr& rs_msg,
               const RsCommonCustomMsgParams& custom_params) {
    rs_msg.reset(new RsPerceptionMsg());

    const auto& ros2_result_msg = ros2_msg->lidarframe;
    auto& rs_result_msg = rs_msg->rs_lidar_result_ptr;

    // frame_id
    rs_result_msg->frame_id = ros2_result_msg.frame_id.data;
    
    // timestamp
    rs_result_msg->timestamp = ros2_result_msg.timestamp.data;

    // status
    rs_result_msg->status = static_cast<AxisStatus>(ros2_result_msg.status.data);

    // gps origin
    Ros2TransformUtil::transform(ros2_result_msg.gps_origin, rs_result_msg->gps_origin);

    // global_pose_ptr
    Ros2TransformUtil::transform(ros2_result_msg.global_pose, rs_result_msg->global_pose_ptr);

    // valid indices
    Ros2TransformUtil::transform(ros2_result_msg.valid_indices, rs_result_msg->valid_indices);

    // objects
    Ros2TransformUtil::transform(ros2_result_msg.objects, rs_result_msg->objects);

    // optional
    if (custom_params.send_point_cloud) {
        Ros2TransformUtil::transform(ros2_result_msg.scan_pointcloud, rs_result_msg->scan_ptr);
    }

    if (custom_params.send_attention_objects) {
        Ros2TransformUtil::transform(ros2_result_msg.attention_objects, rs_result_msg->attention_objects);
    }

    if (custom_params.send_freespace) {
        Ros2TransformUtil::transform(ros2_result_msg.freespace_infos, rs_result_msg->freespace_ptr);
    }

    if (custom_params.send_lane) {
        Ros2TransformUtil::transform(ros2_result_msg.lanes, rs_result_msg->lanes);
    }

    if (custom_params.send_roadedge) {
        Ros2TransformUtil::transform(ros2_result_msg.roadedges, rs_result_msg->roadedges);
    }

    if (custom_params.send_sematic) {
        Ros2TransformUtil::transform(ros2_result_msg.ground_indices,rs_result_msg->ground_indices);
        Ros2TransformUtil::transform(ros2_result_msg.background_indices,rs_result_msg->background_indices);
        Ros2TransformUtil::transform(ros2_result_msg.non_ground_indices,rs_result_msg->non_ground_indices);
    }
}

}  // namespace Ros2
#endif  // RS_ROS2_FOUND
}  // namespace perception
}  // namespace robosense