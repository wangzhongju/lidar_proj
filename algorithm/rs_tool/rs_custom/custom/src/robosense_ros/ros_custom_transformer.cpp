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

#include "rs_perception/custom/robosense_ros/ros_custom_transformer.h"
#include "rs_dependence/rs_dependence_manager.h"

namespace robosense {
namespace perception {
#ifdef RS_ROS_FOUND
namespace Ros {
// ros msg to perception msg
void transform(const perception_ros_msg::RsPerceptionMsgConstPtr& ros_msg_, RsPerceptionMsg::Ptr& rs_msg,
               const RsCommonCustomMsgParams& params) {
    rs_msg.reset(new RsPerceptionMsg);
    const auto &perceptRosMsgPtr = ros_msg_;
    auto &msg_ = rs_msg;

    msg_->rs_lidar_result_ptr->frame_id = perceptRosMsgPtr->lidarframe.frame_id.data;

    msg_->rs_lidar_result_ptr->timestamp = perceptRosMsgPtr->lidarframe.timestamp.data;

    msg_->rs_lidar_result_ptr->status = static_cast<AxisStatus>(perceptRosMsgPtr->lidarframe.status.data);

    RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.gps_origin, msg_->rs_lidar_result_ptr->gps_origin);

    RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.global_pose, msg_->rs_lidar_result_ptr->global_pose_ptr);

    RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.valid_indices, msg_->rs_lidar_result_ptr->valid_indices);

    RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.objects, msg_->rs_lidar_result_ptr->objects);

    if (params.send_point_cloud) {
        RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.scan_pointcloud, msg_->rs_lidar_result_ptr->scan_ptr);
    }

    if (params.send_attention_objects) {
        RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.attention_objects,
                                    msg_->rs_lidar_result_ptr->attention_objects);
    }

    if (params.send_freespace) {
        RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.freespace_infos,
                                    msg_->rs_lidar_result_ptr->freespace_ptr);
    }

    if (params.send_lane) {
        RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.lanes, msg_->rs_lidar_result_ptr->lanes);
    }

    if (params.send_roadedge) {
        RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.roadedges, msg_->rs_lidar_result_ptr->roadedges);
    }

    if (params.send_sematic) {
        RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.ground_indices,
                                    msg_->rs_lidar_result_ptr->ground_indices);
        RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.background_indices,
                                    msg_->rs_lidar_result_ptr->background_indices);
        RosTransformUtil::transform(perceptRosMsgPtr->lidarframe.non_ground_indices,
                                    msg_->rs_lidar_result_ptr->non_ground_indices);
    }
}
// perception msg to ros msg
void transform(const RsPerceptionMsg::Ptr & rs_msg, perception_ros_msg::RsPerceptionMsgPtr& ros_msg,
               const RsCommonCustomMsgParams& params) {
    ros_msg.reset(new perception_ros_msg::RsPerceptionMsg());
    auto& perceptRosMsgPtr = ros_msg;
    const auto& msg_ = rs_msg;

    perceptRosMsgPtr->lidarframe.frame_id.data = msg_->rs_lidar_result_ptr->frame_id;

    perceptRosMsgPtr->lidarframe.timestamp.data = msg_->rs_lidar_result_ptr->timestamp;

    perceptRosMsgPtr->lidarframe.status.data = static_cast<int>(msg_->rs_lidar_result_ptr->status);

    RosTransformUtil::transform(msg_->rs_lidar_result_ptr->gps_origin, perceptRosMsgPtr->lidarframe.gps_origin);

    RosTransformUtil::transform(msg_->rs_lidar_result_ptr->global_pose_ptr, perceptRosMsgPtr->lidarframe.global_pose);

    RosTransformUtil::transform(msg_->rs_lidar_result_ptr->valid_indices,
                                perceptRosMsgPtr->lidarframe.valid_indices);

    RosTransformUtil::transform(msg_->rs_lidar_result_ptr->objects,
                                perceptRosMsgPtr->lidarframe.objects);

    // send point cloud
    if (params.send_point_cloud) {
        RosTransformUtil::transform(msg_->rs_lidar_result_ptr->scan_ptr,
                                    perceptRosMsgPtr->lidarframe.scan_pointcloud);
        perceptRosMsgPtr->lidarframe.has_pointcloud.data = true;
    }
    else {
        perceptRosMsgPtr->lidarframe.has_pointcloud.data = false;
    }

    // send attention objects
    if (params.send_attention_objects) {
        RosTransformUtil::transform(msg_->rs_lidar_result_ptr->attention_objects,
                                    perceptRosMsgPtr->lidarframe.attention_objects);
        perceptRosMsgPtr->lidarframe.has_attention_objects.data = true;
    }
    else {
        perceptRosMsgPtr->lidarframe.has_attention_objects.data = false;
    }

    // send freespace
    if (params.send_freespace) {
        RosTransformUtil::transform(msg_->rs_lidar_result_ptr->freespace_ptr,
                                    perceptRosMsgPtr->lidarframe.freespace_infos);
        perceptRosMsgPtr->lidarframe.has_freespace.data = true;
    }
    else {
        perceptRosMsgPtr->lidarframe.has_freespace.data = false;
    }

    // send lane
    if (params.send_lane) {
        RosTransformUtil::transform(msg_->rs_lidar_result_ptr->lanes,
                                    perceptRosMsgPtr->lidarframe.lanes);
        perceptRosMsgPtr->lidarframe.has_lanes.data = true;
    }
    else {
        perceptRosMsgPtr->lidarframe.has_lanes.data = false;
    }

    // send roadedge
    if (params.send_roadedge) {
        RosTransformUtil::transform(msg_->rs_lidar_result_ptr->roadedges,
                                    perceptRosMsgPtr->lidarframe.roadedges);
        perceptRosMsgPtr->lidarframe.has_roadedges.data = true;
    }
    else {
        perceptRosMsgPtr->lidarframe.has_roadedges.data = false;
    }

    // send sematic cloud indices
    if (params.send_sematic) {
        RosTransformUtil::transform(msg_->rs_lidar_result_ptr->ground_indices,
                                    perceptRosMsgPtr->lidarframe.ground_indices);
        RosTransformUtil::transform(msg_->rs_lidar_result_ptr->background_indices,
                                    perceptRosMsgPtr->lidarframe.background_indices);
        RosTransformUtil::transform(msg_->rs_lidar_result_ptr->non_ground_indices,
                                    perceptRosMsgPtr->lidarframe.non_ground_indices);
        perceptRosMsgPtr->lidarframe.has_sematice_indices.data = true;
    }
    else {
        perceptRosMsgPtr->lidarframe.has_sematice_indices.data = false;
    }
}

}   // namespace Ros
#endif  // RS_ROS_FOUND
}   // namespace perception
}   // namespace robosense

