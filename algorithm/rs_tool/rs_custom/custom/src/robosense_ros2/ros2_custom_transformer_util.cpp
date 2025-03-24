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

#include "rs_perception/custom/robosense_ros2/ros2_custom_transformer_util.h"
#include "rs_dependence/rs_dependence_manager.h"

namespace robosense {
namespace perception {
#ifdef  RS_ROS2_FOUND
namespace Ros2 {

// Objects
void Ros2TransformUtil::transform(const VecObjectPtr &robo_objcets, perception_ros2_msg::msg::Objects &ros2_objects) {
    int obj_nums = static_cast<int>(robo_objcets.size());

    if (obj_nums > 0) {
        ros2_objects.objects.resize(obj_nums);

        for (int i = 0; i < obj_nums; i++) {
            const auto& robo_obj = robo_objcets[i];
            auto& ros2_obj = ros2_objects.objects[i];

            // CoreInfo
            const auto& robo_coreInfo = robo_obj->core_infos_;
            auto& ros2_coreInfo = ros2_obj.coreinfo;

            // timestamp
            ros2_coreInfo.timestamp.data = robo_coreInfo.timestamp;

            // priority_id
            ros2_coreInfo.priority_id.data = robo_coreInfo.priority_id;

            // exist_confidence
            ros2_coreInfo.exist_confidence.data = robo_coreInfo.exist_confidence;

            // center
            fromRoboToRos2(robo_coreInfo.center, ros2_coreInfo.center);

            // center_cov
            fromRoboToRos2(robo_coreInfo.center_cov, ros2_coreInfo.center_cov);

            // size
            fromRoboToRos2(robo_coreInfo.size, ros2_coreInfo.size);

            // size_cov
            fromRoboToRos2(robo_coreInfo.size_cov, ros2_coreInfo.size_cov);

            // direction
            fromRoboToRos2(robo_coreInfo.direction, ros2_coreInfo.direction);

            // direction
            fromRoboToRos2(robo_coreInfo.direction_cov, ros2_coreInfo.direction_cov);

            // type
            ros2_coreInfo.type.data = static_cast<int>(robo_coreInfo.type);

            // type_confidence
            ros2_coreInfo.type_confidence.data = robo_coreInfo.type_confidence;

            // attention_type
            ros2_coreInfo.attention_type.data =
            static_cast<int>(robo_coreInfo.attention_type);

            // motion_state
            ros2_coreInfo.motion_state.data =
            static_cast<int>(robo_coreInfo.motion_state);

            // lane_pos
            ros2_coreInfo.lane_pos.data = static_cast<int>(robo_coreInfo.lane_pos);

            // tracker_id
            ros2_coreInfo.tracker_id.data = robo_coreInfo.tracker_id;

            // age
            ros2_coreInfo.age.data = robo_coreInfo.age;

            // velocity
            fromRoboToRos2(robo_coreInfo.velocity, ros2_coreInfo.velocity);

            // relative_velocity
            fromRoboToRos2(robo_coreInfo.relative_velocity, ros2_coreInfo.relative_velocity);

            // velocity_cov
            fromRoboToRos2(robo_coreInfo.velocity_cov, ros2_coreInfo.velocity_cov);

            // relative_velocity_cov
            fromRoboToRos2(robo_coreInfo.relative_velocity_cov, ros2_coreInfo.relative_velocity_cov);

            // acceleration
            fromRoboToRos2(robo_coreInfo.acceleration, ros2_coreInfo.acceleration);

            // accelearation_cov
            fromRoboToRos2(robo_coreInfo.acceleration_cov,ros2_coreInfo.acceleration_cov);

            // angle_velocity
            ros2_coreInfo.angle_velocity.data = robo_coreInfo.angle_velocity;

            // angle_velocity_cov
            ros2_coreInfo.angle_velocity_cov.data = robo_coreInfo.angle_velocity_cov;

            // angle_acceleration
            ros2_coreInfo.angle_acceleration.data = robo_coreInfo.angle_acceleration;

            // angle_acceleration_cov
            ros2_coreInfo.angle_acceleration_cov.data =
            robo_coreInfo.angle_acceleration_cov;

            // anchor
            fromRoboToRos2(robo_coreInfo.anchor, ros2_coreInfo.anchor);

            // nearest_point
            fromRoboToRos2(robo_coreInfo.nearest_point, ros2_coreInfo.nearest_point);

            // RINFO << "isSupplement = " << isSupplement;
            // SuplementInfo

            ros2_obj.hassupplmentinfo.data = true;

            const auto &robo_supInfo = robo_obj->supplement_infos_;
            auto &ros2_supInfo = ros2_obj.supplementinfo;

            // unique_id
            ros2_supInfo.unique_id.data = robo_supInfo.unique_id;

            // polygon
            const std::vector<RsVector3f> &robo_polygon = robo_supInfo.polygon;
            int polygon_nums = static_cast<int>(robo_polygon.size());
            if (polygon_nums > 0) {
                ros2_supInfo.polygon.resize(polygon_nums);
                for (int i = 0; i < polygon_nums; ++i) {
                    perception_ros2_msg::msg::Point3f ros2_point3f;
                    fromRoboToRos2(robo_polygon[i], ros2_point3f);
                    ros2_supInfo.polygon[i] = ros2_point3f;
                }
            }
            // RINFO << "polygon size = " << polygonCnt;

            // left_point_index
            ros2_supInfo.left_point_index.data = robo_supInfo.left_point_index;

            // right_point_index
            ros2_supInfo.right_point_index.data = robo_supInfo.right_point_index;

            // cloud_indice
            const std::vector<int> &robo_cloud_indices = robo_supInfo.cloud_indices;
            int cloud_indices_nums = static_cast<int>(robo_cloud_indices.size());
            if (cloud_indices_nums > 0) {
                ros2_supInfo.cloud_indices.resize(cloud_indices_nums);
                for (int i = 0; i < cloud_indices_nums; ++i) {
                    ros2_supInfo.cloud_indices[i].data = robo_cloud_indices[i];
                }
            }

            // latent_types
            const std::vector<float> &robo_latent_types = robo_supInfo.latent_types;
            int latent_types_nums = static_cast<int>(robo_latent_types.size());
            if (latent_types_nums > 0) {
                ros2_supInfo.latent_types.resize(latent_types_nums);
                for (int i = 0; i < latent_types_nums; ++i) {
                    ros2_supInfo.latent_types[i].data = robo_latent_types[i];
                }
            }

            // size_type
            ros2_supInfo.size_type.data = static_cast<int>(robo_supInfo.size_type);

            // mode
            ros2_supInfo.mode.data = static_cast<int>(robo_supInfo.mode);

            // in_roi
            ros2_supInfo.in_roi.data = robo_supInfo.in_roi;

            // tarcking_state
            ros2_supInfo.tracking_state.data =
            static_cast<int>(robo_supInfo.tracking_state);

            // geo_center
            fromRoboToRos2(robo_supInfo.geo_center, ros2_supInfo.geo_center);

            // geo_size
            fromRoboToRos2(robo_supInfo.geo_size, ros2_supInfo.geo_size);

            // trajectory
            int trajectorySize = static_cast<int>(robo_supInfo.trajectory.size());
            if (trajectorySize > 0) {
                ros2_supInfo.trajectory.resize(trajectorySize);
                for (int i = 0; i < trajectorySize; ++i) {
                    fromRoboToRos2(robo_supInfo.trajectory[i], ros2_supInfo.trajectory[i]);
                }
            }

            // history_velocity
            int history_velocitySize = static_cast<int>(robo_supInfo.history_velocity.size());
            if (history_velocitySize > 0) {
                ros2_supInfo.history_velocity.resize(history_velocitySize);
                for (int i = 0; i < history_velocitySize; ++i) {
                    fromRoboToRos2(robo_supInfo.history_velocity[i],
                                  ros2_supInfo.history_velocity[i]);
                }
            }

            // history_type
            int history_typeSize = static_cast<int>(robo_supInfo.history_type.size());
            if (history_typeSize > 0) {
                ros2_supInfo.history_type.resize(history_typeSize);
                for (int i = 0; i < history_typeSize; ++i) {
                    ros2_supInfo.history_type[i].data =
                    static_cast<int>(robo_supInfo.history_type[i]);
                }
            }

            // gps_mode
            ros2_supInfo.gps_mode.data = static_cast<int>(robo_supInfo.gps_mode);

            // gps_info
            ros2_supInfo.gps_info.x.data = robo_supInfo.gps_longtitude;
            ros2_supInfo.gps_info.y.data = robo_supInfo.gps_latitude;
            ros2_supInfo.gps_info.z.data = robo_supInfo.gps_altitude;

        }
    }
}

void Ros2TransformUtil::transform(const perception_ros2_msg::msg::Objects &ros2_objects, VecObjectPtr &robo_objects) {
    int object_nums = static_cast<int>(ros2_objects.objects.size());
    if (object_nums > 0) {
        robo_objects.resize(object_nums);
        for (int i = 0; i < object_nums; ++i) {
            robo_objects[i].reset(new Object);

            const auto &ros2_coreInfo = ros2_objects.objects[i].coreinfo;
            auto &robo_coreInfo = robo_objects[i]->core_infos_;

            // timestamp
            robo_coreInfo.timestamp = ros2_coreInfo.timestamp.data;

            // priority_id
            robo_coreInfo.priority_id = ros2_coreInfo.priority_id.data;

            // exist_confidence
            robo_coreInfo.exist_confidence = ros2_coreInfo.exist_confidence.data;

            // center
            fromRos2ToRobo(ros2_coreInfo.center, robo_coreInfo.center);

            // center_cov
            fromRos2ToRobo(ros2_coreInfo.center_cov, robo_coreInfo.center_cov);

            // size
            fromRos2ToRobo(ros2_coreInfo.size, robo_coreInfo.size);

            // size_cov
            fromRos2ToRobo(ros2_coreInfo.size_cov, robo_coreInfo.size_cov);

            // direction
            fromRos2ToRobo(ros2_coreInfo.direction, robo_coreInfo.direction);

            // direction_cov
            fromRos2ToRobo(ros2_coreInfo.direction_cov, robo_coreInfo.direction_cov);

            // type
            robo_coreInfo.type = static_cast<ObjectType>(ros2_coreInfo.type.data);

            // type_confidence
            robo_coreInfo.type_confidence = ros2_coreInfo.type_confidence.data;

            // attention_type
            robo_coreInfo.attention_type =
            static_cast<AttentionType>(ros2_coreInfo.attention_type.data);

            // motion_state
            robo_coreInfo.motion_state =
            static_cast<MotionType>(ros2_coreInfo.motion_state.data);

            // lane_pos
            robo_coreInfo.lane_pos =
            static_cast<LanePosition>(ros2_coreInfo.lane_pos.data);

            // tracker_id
            robo_coreInfo.tracker_id = ros2_coreInfo.tracker_id.data;

            // age
            robo_coreInfo.age = ros2_coreInfo.age.data;

            // velocity
            fromRos2ToRobo(ros2_coreInfo.velocity, robo_coreInfo.velocity);

            // relative_velocity
            fromRos2ToRobo(ros2_coreInfo.relative_velocity, robo_coreInfo.relative_velocity);

            // velocity_cov
            fromRos2ToRobo(ros2_coreInfo.velocity_cov, robo_coreInfo.velocity_cov);

            // relative_velocity_cov
            fromRos2ToRobo(ros2_coreInfo.relative_velocity_cov,robo_coreInfo.relative_velocity_cov);

            // acceleration
            fromRos2ToRobo(ros2_coreInfo.acceleration, robo_coreInfo.acceleration);

            // acceleration_cov
            fromRos2ToRobo(ros2_coreInfo.acceleration_cov,robo_coreInfo.acceleration_cov);

            // angle_velocity
            robo_coreInfo.angle_velocity = ros2_coreInfo.angle_velocity.data;

            // angle_velocity_cov
            robo_coreInfo.angle_velocity_cov = ros2_coreInfo.angle_velocity_cov.data;

            // angle_acceleration
            robo_coreInfo.angle_acceleration = ros2_coreInfo.angle_acceleration.data;

            // angle_acceleration_cov
            robo_coreInfo.angle_acceleration_cov = ros2_coreInfo.angle_acceleration_cov.data;

            // anchor
            fromRos2ToRobo(ros2_coreInfo.anchor, robo_coreInfo.anchor);

            // nearest_point
            fromRos2ToRobo(ros2_coreInfo.nearest_point, robo_coreInfo.nearest_point);

            auto &robo_supInfo = robo_objects[i]->supplement_infos_;
            const auto &ros2_supInfo = ros2_objects.objects[i].supplementinfo;

            // unique_id
            robo_supInfo.unique_id = ros2_supInfo.unique_id.data;

            // polygon
            const std::vector<perception_ros2_msg::msg::Point3f> &ros2_polygon =
            ros2_supInfo.polygon;
            auto& robo_polygon = robo_supInfo.polygon;
            int polygonSize = static_cast<int>(ros2_polygon.size());
            if (polygonSize > 0) {
                robo_polygon.resize(polygonSize);
                for (int i = 0; i < polygonSize; ++i) {
                    fromRos2ToRobo(ros2_polygon[i], robo_polygon[i]);
                }
            }
            // RINFO << "polygon size = " << polygonSize;
            // left_point_index
            robo_supInfo.left_point_index = ros2_supInfo.left_point_index.data;

            // right_point_index
            robo_supInfo.right_point_index = ros2_supInfo.right_point_index.data;

            // cloud_indices
            const auto &ros2_cloud_indices = ros2_supInfo.cloud_indices;
            auto& robo_cloud_indices = robo_supInfo.cloud_indices;
            int cloudIndicesSize = static_cast<int>(ros2_cloud_indices.size());
            if (cloudIndicesSize > 0) {
                robo_cloud_indices.resize(cloudIndicesSize);
                for (int i = 0; i < cloudIndicesSize; ++i) {
                    robo_cloud_indices[i] = ros2_cloud_indices[i].data;
                }
            }

            // latent_types
            const auto &ros2_latent_types = ros2_supInfo.latent_types;
            auto& robo_latent_types = robo_supInfo.latent_types;
            int latentTypeSize = static_cast<int>(ros2_latent_types.size());
            if (latentTypeSize > 0) {
                robo_latent_types.resize(latentTypeSize);
                for (int i = 0; i < latentTypeSize; ++i) {
                    robo_latent_types[i] = ros2_latent_types[i].data;
                }
            }
            else {
                robo_supInfo.latent_types.clear();
            }

            // size_type
            robo_supInfo.size_type = static_cast<SizeType>(ros2_supInfo.size_type.data);

            // in_roi
            robo_supInfo.in_roi = ros2_supInfo.in_roi.data;

            // tracking_state
            robo_supInfo.tracking_state = static_cast<TrackingState >(ros2_supInfo.tracking_state.data);

            // geo_center
            fromRos2ToRobo(ros2_supInfo.geo_center, robo_supInfo.geo_center);

            // geo_size
            fromRos2ToRobo(ros2_supInfo.geo_size, robo_supInfo.geo_size);

            // trajectory
            const auto &ros2_trajectory = ros2_supInfo.trajectory;
            auto &robo_trajectory = robo_supInfo.trajectory;
            int trajectorySize = static_cast<int>(ros2_trajectory.size());
            if (trajectorySize > 0) {
                RsVector3f eig;
                for (int i = 0; i < trajectorySize; ++i) {
                    fromRos2ToRobo(ros2_trajectory[i], eig);
                    robo_trajectory.push_back(eig);
                }
            }

            // history_velocity
            const auto &ros2_history_velocity = ros2_supInfo.history_velocity;
            auto &robo_history_velocity = robo_supInfo.history_velocity;
            int historyVelocitySize = static_cast<int>(ros2_history_velocity.size());
            if (historyVelocitySize > 0) {
                RsVector3f eig;
                for (int i = 0; i < historyVelocitySize; ++i) {
                    fromRos2ToRobo(ros2_history_velocity[i], eig);
                    robo_history_velocity.push_back(eig);
                }
            }

            // history_type
            const auto &ros2_history_type = ros2_supInfo.history_type;
            auto &robo_history_type = robo_supInfo.history_type;
            int historyTypeSize = static_cast<int>(ros2_history_type.size());
            if (historyTypeSize > 0) {
                for (int i = 0; i < historyTypeSize; ++i) {
                    robo_history_type.push_back(
                    static_cast<ObjectType>(ros2_history_type[i].data));
                }
            }

            // gps_mode
            robo_supInfo.gps_mode = static_cast<GpsType>(ros2_supInfo.gps_mode.data);

            // gps_infos
            robo_supInfo.gps_longtitude = ros2_supInfo.gps_info.x.data;
            robo_supInfo.gps_latitude = ros2_supInfo.gps_info.y.data;
            robo_supInfo.gps_altitude = ros2_supInfo.gps_info.z.data;
        }
    }
}

// Freespaces
void Ros2TransformUtil::transform(const RsFreeSpace::Ptr &robo_freespaces,
                                  perception_ros2_msg::msg::FreeSpaceInfos &ros2_freespaces) {
    if (robo_freespaces == nullptr || robo_freespaces->fs_pts.size() != robo_freespaces->fs_confidence.size()) {
        return;
    }

    size_t freespace_points_nums = robo_freespaces->fs_pts.size();

    if (freespace_points_nums > 0) {
        ros2_freespaces.fs_pts.resize(freespace_points_nums);
        ros2_freespaces.fs_confidence.resize(freespace_points_nums);
        for (size_t i = 0; i < freespace_points_nums; i++) {
            fromRoboToRos2(robo_freespaces->fs_pts[i], ros2_freespaces.fs_pts[i]);
            ros2_freespaces.fs_confidence[i].data = robo_freespaces->fs_confidence[i];
        }
    }
}

void Ros2TransformUtil::transform(const perception_ros2_msg::msg::FreeSpaceInfos &ros2_freespaces,
                                  RsFreeSpace::Ptr &robo_freespaces) {
    if (robo_freespaces == nullptr) {
        robo_freespaces.reset(new RsFreeSpace());
    }

    size_t freespace_points_nums = ros2_freespaces.fs_pts.size();

    if (freespace_points_nums > 0) {
        robo_freespaces->fs_pts.resize(freespace_points_nums);
        robo_freespaces->fs_confidence.resize(freespace_points_nums);

        for (size_t i = 0; i < freespace_points_nums; i++) {
            fromRos2ToRobo(ros2_freespaces.fs_pts[i], robo_freespaces->fs_pts[i]);
            robo_freespaces->fs_confidence[i] = ros2_freespaces.fs_confidence[i].data;
        }
    }
}

// Lanes
void Ros2TransformUtil::transform(const std::vector<Lane::Ptr> &robo_lanes,
                                  perception_ros2_msg::msg::Lanes &ros2_lanes) {
    size_t lane_points_nums = robo_lanes.size();

    if (lane_points_nums > 0) {
        ros2_lanes.lanes.resize(lane_points_nums);
        for (size_t i = 0; i < lane_points_nums; i++) {
            fromRoboToRos2(robo_lanes[i], ros2_lanes.lanes[i]);
        }
    }
}

void Ros2TransformUtil::transform(const perception_ros2_msg::msg::Lanes &ros2_lanes,
                                  std::vector<Lane::Ptr> &robo_lanes) {
    size_t lane_points_nums = ros2_lanes.lanes.size();

    if (lane_points_nums > 0) {
        robo_lanes.resize(lane_points_nums);
        for (size_t i = 0; i < lane_points_nums; i++) {
            fromRos2ToRobo(ros2_lanes.lanes[i], robo_lanes[i]);
        }
    }
}

// Road edges
void Ros2TransformUtil::transform(const std::vector<Roadedge::Ptr> &robo_curbs,
                                  perception_ros2_msg::msg::RoadEdges &ros2_curbs) {
    size_t curb_points_nums = robo_curbs.size();

    if (curb_points_nums > 0) {
        ros2_curbs.curbs.resize(curb_points_nums);
        for (size_t i = 0; i < curb_points_nums; i++) {
            fromRoboToRos2(robo_curbs[i], ros2_curbs.curbs[i]);
        }
    }
}

void Ros2TransformUtil::transform(const perception_ros2_msg::msg::RoadEdges &ros2_curbs,
                                  std::vector<Roadedge::Ptr> &robo_curbs) {
    size_t curb_points_nums = ros2_curbs.curbs.size();

    if (curb_points_nums > 0) {
        robo_curbs.resize(curb_points_nums);
        for (size_t i = 0; i < curb_points_nums; i++) {
            fromRos2ToRobo(ros2_curbs.curbs[i], robo_curbs[i]);
        }
    }
}

// Point cloud
void Ros2TransformUtil::transform(const RsPointCloudGPT::Ptr &robo_pc,
                                  std::vector<perception_ros2_msg::msg::Point4f> &ros2_pc) {
    if (robo_pc == nullptr) {
        return;
    }

    size_t pc_size = robo_pc->size();
    ros2_pc.resize(pc_size);

    for (size_t i = 0; i < pc_size; i++) {
        const auto& point = robo_pc->points[i];
        auto& ros2_point = ros2_pc[i];

        ros2_point.x.data = point.x;
        ros2_point.y.data = point.y;
        ros2_point.z.data = point.z;
        ros2_point.i.data = point.intensity;
    }
}

void Ros2TransformUtil::transform(const std::vector<perception_ros2_msg::msg::Point4f> &ros2_pc,
                                  RsPointCloudGPT::Ptr &robo_pc) {
    if (robo_pc == nullptr) {
        robo_pc.reset(new RsPointCloudGPT());
    }

    size_t pc_size = ros2_pc.size();
    robo_pc->resize(pc_size);

    for (size_t i = 0; i < pc_size; i++) {
        const auto& ros2_point = ros2_pc[i];
        auto& point = robo_pc->points[i];
        point.x = ros2_point.x.data;
        point.y = ros2_point.y.data;
        point.z = ros2_point.z.data;
        point.intensity = ros2_point.i.data;
    }
}

// RsPose
void Ros2TransformUtil::transform(const RsPose::Ptr &robo_pose, perception_ros2_msg::msg::Pose &ros2_pose) {
    if (robo_pose == nullptr) {
        return;
    }
    ros2_pose.x.data = robo_pose->x;
    ros2_pose.y.data = robo_pose->y;
    ros2_pose.z.data = robo_pose->z;
    ros2_pose.roll.data = robo_pose->roll;
    ros2_pose.pitch.data = robo_pose->pitch;
    ros2_pose.yaw.data = robo_pose->yaw;
}

void Ros2TransformUtil::transform(const perception_ros2_msg::msg::Pose &ros2_pose, RsPose::Ptr &robo_pose) {
    if (robo_pose == nullptr) {
        robo_pose.reset(new RsPose);
    }

    robo_pose->x = ros2_pose.x.data;
    robo_pose->y = ros2_pose.y.data;
    robo_pose->z = ros2_pose.z.data;
    robo_pose->roll = ros2_pose.roll.data;
    robo_pose->pitch = ros2_pose.pitch.data;
    robo_pose->yaw = ros2_pose.yaw.data;
}

// RsPose Map
void Ros2TransformUtil::transform(const std::map<AxisStatus, RsPose::Ptr> &robo_pose_map,
                                  perception_ros2_msg::msg::PoseMap &ros2_pose_map) {
    if (robo_pose_map.empty()) {
        return;
    }

    for (const auto& item: robo_pose_map) {
        const auto& status = item.first;
        const auto& robo_pose = item.second;

        perception_ros2_msg::msg::AxisStatusPose ros2_pose;
        transform(status, robo_pose, ros2_pose);

        ros2_pose_map.status_poses.push_back(ros2_pose);
    }
}

void Ros2TransformUtil::transform(const perception_ros2_msg::msg::PoseMap &ros2_pose_map,
                                  std::map<AxisStatus, RsPose::Ptr> &robo_pose_map) {
    size_t pose_nums = ros2_pose_map.status_poses.size();
    for (size_t i = 0; i < pose_nums; i++) {
        AxisStatus status;
        RsPose::Ptr robo_pose;
        transform(ros2_pose_map.status_poses[i], status, robo_pose);
        robo_pose_map[status] = robo_pose;
    }
}

// Status & RsPose Pair
void Ros2TransformUtil::transform(const AxisStatus status, const RsPose::Ptr &robo_pose,
                                  perception_ros2_msg::msg::AxisStatusPose &ros2_pose) {
    transform(robo_pose, ros2_pose.pose);
    ros2_pose.status.data = static_cast<int>(status);
}

void Ros2TransformUtil::transform(const perception_ros2_msg::msg::AxisStatusPose &ros2_pose, AxisStatus &status,
                                  RsPose::Ptr &robo_pose) {
    if (robo_pose == nullptr) {
        robo_pose.reset(new RsPose());
    }

    transform(ros2_pose.pose, robo_pose);
    status = static_cast<AxisStatus>(ros2_pose.status.data);
}

// Indices
void Ros2TransformUtil::transform(const VecInt &robo_indices, perception_ros2_msg::msg::Indices &ros2_indices) {
    size_t indices_nums = robo_indices.size();
    ros2_indices.indices.resize(indices_nums);
    for (size_t i = 0; i < indices_nums; i++) {
        ros2_indices.indices[i].data = robo_indices[i];
    }
    
}

void Ros2TransformUtil::transform(const perception_ros2_msg::msg::Indices &ros2_indices, VecInt &robo_indices) {
    size_t indices_nums = ros2_indices.indices.size();
    robo_indices.resize(indices_nums);
    for (size_t i = 0; i < indices_nums; i++) {
        robo_indices[i] = ros2_indices.indices[i].data;
    }
}

// Lane
void Ros2TransformUtil::fromRoboToRos2(const Lane::Ptr &robo_lane, perception_ros2_msg::msg::Lane &ros2_lane) {
    if (robo_lane == nullptr) {
        return;
    }
    
    ros2_lane.lane_id.data = static_cast<int>(robo_lane->lane_id);

    fromRoboToRos2(robo_lane->curve, ros2_lane.curve);

    fromRoboToRos2(robo_lane->end_point, ros2_lane.end_points);

    ros2_lane.measure_status.data = static_cast<int>(robo_lane->measure_status);

    ros2_lane.confidence.data = robo_lane->confidence;
}

void Ros2TransformUtil::fromRos2ToRobo(const perception_ros2_msg::msg::Lane &ros2_lane, Lane::Ptr &robo_lane) {
    if (robo_lane == nullptr) {
        robo_lane.reset(new Lane);
    }

    robo_lane->lane_id = static_cast<LanePosition>(ros2_lane.lane_id.data);

    fromRos2ToRobo(ros2_lane.curve, robo_lane->curve);

    fromRos2ToRobo(ros2_lane.end_points, robo_lane->end_point);

    robo_lane->measure_status = static_cast<MeasureStatus>(ros2_lane.measure_status.data);

    robo_lane->confidence = ros2_lane.confidence.data;
}

// Curb
void Ros2TransformUtil::fromRoboToRos2(const Roadedge::Ptr &robo_curb, perception_ros2_msg::msg::RoadEdge &ros2_curb) {
    if (robo_curb == nullptr) {
        return;
    }
    
    ros2_curb.roadedge_id.data = static_cast<int>(robo_curb->roadedge_id);

    fromRoboToRos2(robo_curb->curve, ros2_curb.curve);

    fromRoboToRos2(robo_curb->end_point, ros2_curb.end_points);

    ros2_curb.measure_status.data = static_cast<int>(robo_curb->measure_status);

    ros2_curb.confidence.data = robo_curb->confidence;
}

void Ros2TransformUtil::fromRos2ToRobo(const perception_ros2_msg::msg::RoadEdge &ros2_curb, Roadedge::Ptr &robo_curb) {
    if (robo_curb == nullptr) {
        robo_curb.reset(new Roadedge);
    }

    robo_curb->roadedge_id = static_cast<RoadedgePosition>(ros2_curb.roadedge_id.data);

    fromRos2ToRobo(ros2_curb.curve, robo_curb->curve);

    fromRos2ToRobo(ros2_curb.end_points, robo_curb->end_point);

    robo_curb->measure_status = static_cast<MeasureStatus>(ros2_curb.measure_status.data);

    robo_curb->confidence = ros2_curb.confidence.data;
}

// curve
void Ros2TransformUtil::fromRoboToRos2(const Curve &robo_curve, perception_ros2_msg::msg::Curve &ros2_curve) {
    ros2_curve.x_start.data = robo_curve.x_start;
    ros2_curve.x_end.data = robo_curve.x_end;
    ros2_curve.a.data = robo_curve.a;
    ros2_curve.b.data = robo_curve.b;
    ros2_curve.c.data = robo_curve.c;
    ros2_curve.d.data = robo_curve.d;
}

void Ros2TransformUtil::fromRos2ToRobo(const perception_ros2_msg::msg::Curve &ros2_curve, Curve &robo_curve) {
    robo_curve.x_start = ros2_curve.x_start.data;
    robo_curve.x_end = ros2_curve.x_end.data;
    robo_curve.a = ros2_curve.a.data;
    robo_curve.b = ros2_curve.b.data;
    robo_curve.c = ros2_curve.c.data;
    robo_curve.d = ros2_curve.d.data;
}

// Endpoint
void Ros2TransformUtil::fromRoboToRos2(const EndPoints &robo_endpoints,
                                       perception_ros2_msg::msg::EndPoints &ros2_endpoints) {
    fromRoboToRos2(robo_endpoints.start, ros2_endpoints.start);
    fromRoboToRos2(robo_endpoints.end, ros2_endpoints.end);
}

void Ros2TransformUtil::fromRos2ToRobo(const perception_ros2_msg::msg::EndPoints &ros2_endpoints,
                                       EndPoints &robo_endpoints) {
    fromRos2ToRobo(ros2_endpoints.start, robo_endpoints.start);
    fromRos2ToRobo(ros2_endpoints.end, robo_endpoints.end);
}

}  // namespace Ros2
#endif  // RS_ROS2_FOUND
}  // namespace perception
}  // namespace robosense
