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

#include "rs_perception/custom/robosense_ros/ros_custom_transformer_util.h"
#include "rs_dependence/rs_dependence_manager.h"

namespace robosense {
namespace perception {
#ifdef RS_ROS_FOUND
namespace Ros {
// Objects
void RosTransformUtil::transform(const std::vector<Object::Ptr> &robo_objects,
                                 perception_ros_msg::Objects &ros_objects) {
    int objectCnt = robo_objects.size();

    if (objectCnt > 0) {
        ros_objects.objects.resize(objectCnt);

        for (int i = 0; i < objectCnt; ++i) {
            //
            const Object::Ptr robo_object = robo_objects[i];
            perception_ros_msg::Object &ros_object = ros_objects.objects[i];

            // CoreInfo
            perception_ros_msg::CoreInfo &ros_coreInfo = ros_object.coreinfo;
            const CoreInfos robo_coreInfo = robo_object->core_infos_;

            // timestamp
            ros_coreInfo.timestamp.data = robo_coreInfo.timestamp;

            // priority_id
            ros_coreInfo.priority_id.data = robo_coreInfo.priority_id;

            // exist_confidence
            ros_coreInfo.exist_confidence.data = robo_coreInfo.exist_confidence;

            // center
            fromRoboToRos(robo_coreInfo.center, ros_coreInfo.center);

            // center_cov
            fromRoboToRos(robo_coreInfo.center_cov, ros_coreInfo.center_cov);

            // size
            fromRoboToRos(robo_coreInfo.size, ros_coreInfo.size);

            // size_cov
            fromRoboToRos(robo_coreInfo.size_cov, ros_coreInfo.size_cov);

            // direction
            fromRoboToRos(robo_coreInfo.direction, ros_coreInfo.direction);

            // direction cov
            fromRoboToRos(robo_coreInfo.direction_cov, ros_coreInfo.direction_cov);

            // type
            ros_coreInfo.type.data = static_cast<int>(robo_coreInfo.type);

            // type_confidence
            ros_coreInfo.type_confidence.data = robo_coreInfo.type_confidence;

            // attention_type
            ros_coreInfo.attention_type.data = static_cast<int>(robo_coreInfo.attention_type);

            // motion_state
            ros_coreInfo.motion_state.data = static_cast<int>(robo_coreInfo.motion_state);

            // lane_pos
            ros_coreInfo.lane_pos.data = static_cast<int>(robo_coreInfo.lane_pos);

            // tracker_id
            ros_coreInfo.trakcer_id.data = robo_coreInfo.tracker_id;

            // age
            ros_coreInfo.age.data = robo_coreInfo.age;

            // velocity
            fromRoboToRos(robo_coreInfo.velocity, ros_coreInfo.velocity);

            // velocity_cov
            fromRoboToRos(robo_coreInfo.velocity_cov, ros_coreInfo.velocity_cov);

            // acceleration
            fromRoboToRos(robo_coreInfo.acceleration, ros_coreInfo.acceleration);

            // acceleration_cov
            fromRoboToRos(robo_coreInfo.acceleration_cov,ros_coreInfo.acceleration_cov);

            // angle_velocity
            ros_coreInfo.angle_velocity.data = robo_coreInfo.angle_velocity;

            // angle_velocity_cov
            ros_coreInfo.angle_velocity_cov.data = robo_coreInfo.angle_velocity_cov;

            // angle_acceleration
            ros_coreInfo.angle_acceleration.data = robo_coreInfo.angle_acceleration;

            // angle_acceleration_cov
            ros_coreInfo.angle_acceleration_cov.data = robo_coreInfo.angle_acceleration_cov;

            // anchor
            fromRoboToRos(robo_coreInfo.anchor, ros_coreInfo.anchor);

            // nearest_point
            fromRoboToRos(robo_coreInfo.nearest_point, ros_coreInfo.nearest_point);

            // RINFO << "isSupplement = " << isSupplement;
            // SuplementInfo

            ros_object.hassupplmentinfo.data = true;

            const SupplementInfos &robo_supInfo = robo_object->supplement_infos_;
            perception_ros_msg::SupplementInfo &ros_supInfo = ros_object.supplementinfo;

            // unique_id
            ros_supInfo.unique_id.data = robo_supInfo.unique_id;

            // polygon
            const std::vector<RsVector3f> &robo_polygon = robo_supInfo.polygon;
            int polygonCnt = robo_polygon.size();
            if (polygonCnt > 0) {
                ros_supInfo.polygon.resize(polygonCnt);
                for (int i = 0; i < polygonCnt; ++i) {
                    perception_ros_msg::Point3f ros_point3f;
                    fromRoboToRos(robo_polygon[i], ros_point3f);
                    ros_supInfo.polygon[i] = ros_point3f;
                }
            }
            // RINFO << "polygon size = " << polygonCnt;

            // left_point_index
            ros_supInfo.left_point_index.data = robo_supInfo.left_point_index;

            // right_point_index
            ros_supInfo.right_point_index.data = robo_supInfo.right_point_index;

            // cloud_indice
            const std::vector<int> &robo_cloud_indices = robo_supInfo.cloud_indices;
            int cloud_indicesCnt = robo_cloud_indices.size();
            if (cloud_indicesCnt > 0) {
                ros_supInfo.cloud_indices.resize(cloud_indicesCnt);
                for (int i = 0; i < cloud_indicesCnt; ++i) {
                    ros_supInfo.cloud_indices[i].data = robo_cloud_indices[i];
                }
            }

            // latent_types
            const std::vector<float> &robo_latent_types = robo_supInfo.latent_types;
            int latent_typesCnt = robo_latent_types.size();
            if (latent_typesCnt > 0) {
                ros_supInfo.latent_types.resize(latent_typesCnt);
                for (int i = 0; i < latent_typesCnt; ++i) {
                    ros_supInfo.latent_types[i].data = robo_latent_types[i];
                }
            }

            // size_type
            ros_supInfo.size_type.data = static_cast<int>(robo_supInfo.size_type);

            // mode
            ros_supInfo.mode.data = static_cast<int>(robo_supInfo.mode);

            // in_roi
            ros_supInfo.in_roi.data = robo_supInfo.in_roi;

            // tracking_state
            ros_supInfo.tracking_state.data = static_cast<int>(robo_supInfo.tracking_state);

            // geo_center
            fromRoboToRos(robo_supInfo.geo_center, ros_supInfo.geo_center);

            // geo_size
            fromRoboToRos(robo_supInfo.geo_size, ros_supInfo.geo_size);

            // trajectory
            int trajectorySize = robo_supInfo.trajectory.size();
            if (trajectorySize > 0) {
                ros_supInfo.trajectory.resize(trajectorySize);
                for (int i = 0; i < trajectorySize; ++i) {
                    fromRoboToRos(robo_supInfo.trajectory[i], ros_supInfo.trajectory[i]);
                }
            }

            // history_velocity
            int history_velocitySize = robo_supInfo.history_velocity.size();
            if (history_velocitySize > 0) {
                ros_supInfo.history_velocity.resize(history_velocitySize);
                for (int i = 0; i < history_velocitySize; ++i) {
                    fromRoboToRos(robo_supInfo.history_velocity[i],
                                  ros_supInfo.history_velocity[i]);
                }
            }

            // history_type
            int history_typeSize = robo_supInfo.history_type.size();
            if (history_typeSize > 0) {
                ros_supInfo.history_type.resize(history_typeSize);
                for (int i = 0; i < history_typeSize; ++i) {
                    ros_supInfo.history_type[i].data =
                    static_cast<int>(robo_supInfo.history_type[i]);
                }
            }

            // gps_mode
            ros_supInfo.gps_mode.data = static_cast<int>(robo_supInfo.gps_mode);

            // gps_info
            ros_supInfo.gps_info.x.data = robo_supInfo.gps_longtitude;
            ros_supInfo.gps_info.y.data = robo_supInfo.gps_latitude;
            ros_supInfo.gps_info.z.data = robo_supInfo.gps_altitude;
        }
    }
}
void RosTransformUtil::transform(const perception_ros_msg::Objects &ros_objects,
                                 std::vector<Object::Ptr> &robo_objects) {
    int objectSize = ros_objects.objects.size();
    if (objectSize > 0) {
        robo_objects.resize(objectSize);
        for (int i = 0; i < objectSize; ++i) {
            robo_objects[i].reset(new Object);

            const perception_ros_msg::CoreInfo &ros_coreInfo =
            ros_objects.objects[i].coreinfo;
            auto &robo_coreInfo = robo_objects[i]->core_infos_;

            // timestamp
            robo_coreInfo.timestamp = ros_coreInfo.timestamp.data;

            // priority_id
            robo_coreInfo.priority_id = ros_coreInfo.priority_id.data;

            // exist_confidence
            robo_coreInfo.exist_confidence = ros_coreInfo.exist_confidence.data;

            // center
            fromRosToRobo(ros_coreInfo.center, robo_coreInfo.center);

            // center_cov
            fromRosToRobo(ros_coreInfo.center_cov, robo_coreInfo.center_cov);

            // size
            fromRosToRobo(ros_coreInfo.size, robo_coreInfo.size);

            // size_cov
            fromRosToRobo(ros_coreInfo.size_cov, robo_coreInfo.size_cov);

            // direction
            fromRosToRobo(ros_coreInfo.direction, robo_coreInfo.direction);

            // direction_cov
            fromRosToRobo(ros_coreInfo.direction_cov, robo_coreInfo.direction_cov);

            // type
            robo_coreInfo.type = static_cast<ObjectType>(ros_coreInfo.type.data);

            // type_confidence
            robo_coreInfo.type_confidence = ros_coreInfo.type_confidence.data;

            // attention_type
            robo_coreInfo.attention_type = static_cast<AttentionType>(ros_coreInfo.attention_type.data);

            // motion_state
            robo_coreInfo.motion_state = static_cast<MotionType>(ros_coreInfo.motion_state.data);

            // lane_pos
            robo_coreInfo.lane_pos = static_cast<LanePosition>(ros_coreInfo.lane_pos.data);

            // tracker_id
            robo_coreInfo.tracker_id = ros_coreInfo.trakcer_id.data;

            // age
            robo_coreInfo.age = ros_coreInfo.age.data;

            // velocity
            fromRosToRobo(ros_coreInfo.velocity, robo_coreInfo.velocity);

            // velocity_cov
            fromRosToRobo(ros_coreInfo.velocity_cov, robo_coreInfo.velocity_cov);

            // acceleration
            fromRosToRobo(ros_coreInfo.acceleration, robo_coreInfo.acceleration);

            // acceleration_cov
            fromRosToRobo(ros_coreInfo.acceleration_cov, robo_coreInfo.acceleration_cov);

            // angle_velocity
            robo_coreInfo.angle_velocity = ros_coreInfo.angle_velocity.data;

            // angle_velocity_cov
            robo_coreInfo.angle_velocity_cov = ros_coreInfo.angle_velocity_cov.data;

            // angle_acceleration
            robo_coreInfo.angle_acceleration = ros_coreInfo.angle_acceleration.data;

            // angle_acceleration_cov
            robo_coreInfo.angle_acceleration_cov = ros_coreInfo.angle_acceleration_cov.data;

            // anchor
            fromRosToRobo(ros_coreInfo.anchor, robo_coreInfo.anchor);

            // nearest_point
            fromRosToRobo(ros_coreInfo.nearest_point, robo_coreInfo.nearest_point);

            SupplementInfos &robo_supInfo = robo_objects[i]->supplement_infos_;
            const perception_ros_msg::SupplementInfo &ros_supInfo =
            ros_objects.objects[i].supplementinfo;

            // unique_id
            robo_supInfo.unique_id = ros_supInfo.unique_id.data;

            // polygon
            const std::vector<perception_ros_msg::Point3f> &ros_polygon =
            ros_supInfo.polygon;
            auto& robo_polygon = robo_supInfo.polygon;
            int polygonSize = ros_polygon.size();
            if (polygonSize > 0) {
                robo_polygon.resize(polygonSize);
                for (int i = 0; i < polygonSize; ++i) {
                    fromRosToRobo(ros_polygon[i], robo_polygon[i]);
                }
            }
            // RINFO << "polygon size = " << polygonSize;
            // left_point_index
            robo_supInfo.left_point_index = ros_supInfo.left_point_index.data;

            // right_point_index
            robo_supInfo.right_point_index = ros_supInfo.right_point_index.data;

            // cloud_indices
            const std::vector<std_msgs::Int32> &ros_cloud_indices = ros_supInfo.cloud_indices;
            auto& robo_cloud_indices = robo_supInfo.cloud_indices;
            int cloudIndicesSize = ros_cloud_indices.size();
            if (cloudIndicesSize > 0) {
                robo_cloud_indices.resize(cloudIndicesSize);
                for (int i = 0; i < cloudIndicesSize; ++i) {
                    robo_cloud_indices[i] = ros_cloud_indices[i].data;
                }
            }

            // latent_types
            const std::vector<std_msgs::Float32> &ros_latent_types =
            ros_supInfo.latent_types;
            auto& robo_latent_types = robo_supInfo.latent_types;
            int latentTypeSize = ros_latent_types.size();
            if (latentTypeSize > 0) {
                robo_latent_types.resize(latentTypeSize);
                for (int i = 0; i < latentTypeSize; ++i) {
                    robo_latent_types[i] = ros_latent_types[i].data;
                }
            } else {
                robo_supInfo.latent_types.clear();
            }

            // size_type
            robo_supInfo.size_type = static_cast<SizeType>(ros_supInfo.size_type.data);

            // mode
            robo_supInfo.mode = static_cast<ModeType>(ros_supInfo.mode.data);

            // in_roi
            robo_supInfo.in_roi = ros_supInfo.in_roi.data;

            // tracking_state
            robo_supInfo.tracking_state = static_cast<TrackingState >(ros_supInfo.tracking_state.data);

            // geo_center
            fromRosToRobo(ros_supInfo.geo_center, robo_supInfo.geo_center);

            // geo_size
            fromRosToRobo(ros_supInfo.geo_size, robo_supInfo.geo_size);

            // trajectory
            const std::vector<perception_ros_msg::Point3f> &ros_trajectory = ros_supInfo.trajectory;
            auto &robo_trajectory = robo_supInfo.trajectory;
            int trajectorySize = ros_trajectory.size();
            if (trajectorySize > 0) {
                RsVector3f eig;
                for (int i = 0; i < trajectorySize; ++i) {
                    fromRosToRobo(ros_trajectory[i], eig);
                    robo_trajectory.push_back(eig);
                }
            }

            // history_velocity
            const std::vector<perception_ros_msg::Point3f> &ros_history_velocity = ros_supInfo.history_velocity;
            auto &robo_history_velocity = robo_supInfo.history_velocity;
            int historyVelocitySize = ros_history_velocity.size();
            if (historyVelocitySize > 0) {
                RsVector3f eig;
                for (int i = 0; i < historyVelocitySize; ++i) {
                    fromRosToRobo(ros_history_velocity[i], eig);
                    robo_history_velocity.push_back(eig);
                }
            }

            // history_type
            const std::vector<std_msgs::Int32> &ros_history_type = ros_supInfo.history_type;
            auto &robo_history_type = robo_supInfo.history_type;
            int historyTypeSize = ros_history_type.size();
            if (historyTypeSize > 0) {
                for (int i = 0; i < historyTypeSize; ++i) {
                    robo_history_type.push_back(
                    static_cast<ObjectType>(ros_history_type[i].data));
                }
            }

            // gps_mode
            robo_supInfo.gps_mode = static_cast<GpsType>(ros_supInfo.gps_mode.data);

            // gps_infos
            robo_supInfo.gps_longtitude = ros_supInfo.gps_info.x.data;
            robo_supInfo.gps_latitude = ros_supInfo.gps_info.y.data;
            robo_supInfo.gps_altitude = ros_supInfo.gps_info.z.data;
        }
    }
}

// Freespace
void RosTransformUtil::transform(const RsFreeSpace::Ptr &robo_freespaces_ptr,
perception_ros_msg::FreeSpaceInfos &ros_freespaces) {
    if (robo_freespaces_ptr == nullptr) {
        return;
    }

    int freespaceCnt = robo_freespaces_ptr->fs_pts.size();

    if (freespaceCnt > 0) {
        ros_freespaces.fs_pts.resize(freespaceCnt);
        ros_freespaces.fs_confidence.resize(freespaceCnt);
        for (int i = 0; i < freespaceCnt; ++i) {
            fromRoboToRos(robo_freespaces_ptr->fs_pts[i], ros_freespaces.fs_pts[i]);
            ros_freespaces.fs_confidence[i].data = robo_freespaces_ptr->fs_confidence[i];
        }
    }
}

void RosTransformUtil::transform(const perception_ros_msg::FreeSpaceInfos &ros_freespaces,
RsFreeSpace::Ptr &robo_freespaces_ptr) {
    if (robo_freespaces_ptr == nullptr) {
        robo_freespaces_ptr.reset(new RsFreeSpace());
    }

    int freespaceSize = ros_freespaces.fs_pts.size();
    if (freespaceSize > 0) {
        robo_freespaces_ptr->fs_pts.resize(freespaceSize);
        robo_freespaces_ptr->fs_confidence.resize(freespaceSize);
        for (int i = 0; i < freespaceSize; ++i) {
            fromRosToRobo(ros_freespaces.fs_pts[i], robo_freespaces_ptr->fs_pts[i]);
            robo_freespaces_ptr->fs_confidence[i] = ros_freespaces.fs_confidence[i].data;
        }
    }
}

// Lanes
void RosTransformUtil::transform(const std::vector<Lane::Ptr> &robo_lanes,
                                  perception_ros_msg::Lanes &ros_lanes) {
    int laneCnt = robo_lanes.size();
    if (laneCnt > 0) {
        ros_lanes.lanes.resize(laneCnt);
        for (int i = 0; i < laneCnt; ++i) {
            fromRoboToRos(robo_lanes[i], ros_lanes.lanes[i]);
        }
    }
}

void RosTransformUtil::transform(const perception_ros_msg::Lanes &ros_lanes,
                                    std::vector<Lane::Ptr> &robo_lanes) {
    int laneSize = ros_lanes.lanes.size();
    if (laneSize > 0) {
        robo_lanes.resize(laneSize);
        for (int i = 0; i < laneSize; ++i) {
            fromRosToRobo(ros_lanes.lanes[i], robo_lanes[i]);
        }
    }
}

// RoadEdges
void RosTransformUtil::transform(const std::vector<Roadedge::Ptr> &robo_curbs,
                                  perception_ros_msg::RoadEdges &ros_curbs) {
    int curbCnt = robo_curbs.size();

    if (curbCnt > 0) {
        ros_curbs.curbs.resize(curbCnt);
        for (int i = 0; i < curbCnt; ++i) {
            fromRoboToRos(robo_curbs[i], ros_curbs.curbs[i]);
        }
    }
}

void RosTransformUtil::transform(const perception_ros_msg::RoadEdges &ros_curbs,
std::vector<Roadedge::Ptr> &robo_curbs) {
    int curbSize = ros_curbs.curbs.size();
    if (curbSize > 0) {
        robo_curbs.resize(curbSize);
        for (int i = 0; i < curbSize; ++i) {
            fromRosToRobo(ros_curbs.curbs[i], robo_curbs[i]);
        }
    }
}

// Pose
void RosTransformUtil::transform(const RsPose::Ptr &robo_pose,
                                  perception_ros_msg::Pose &ros_pose) {
    ros_pose.x.data = robo_pose->x;
    ros_pose.y.data = robo_pose->y;
    ros_pose.z.data = robo_pose->z;
    ros_pose.roll.data = robo_pose->roll;
    ros_pose.pitch.data = robo_pose->pitch;
    ros_pose.yaw.data = robo_pose->yaw;
}

void RosTransformUtil::transform(const perception_ros_msg::Pose &ros_pose,
                                    RsPose::Ptr &robo_pose) {
    if (robo_pose == nullptr) {
        robo_pose.reset(new RsPose);
    }

    robo_pose->x = ros_pose.x.data;
    robo_pose->y = ros_pose.y.data;
    robo_pose->z = ros_pose.z.data;
    robo_pose->roll = ros_pose.roll.data;
    robo_pose->pitch = ros_pose.pitch.data;
    robo_pose->yaw = ros_pose.yaw.data;
}

// Pose Map
void RosTransformUtil::transform(const std::map<AxisStatus, RsPose::Ptr> &robo_poses,
perception_ros_msg::PoseMap &ros_poses) {
    for (auto iterMap = robo_poses.begin(); iterMap != robo_poses.end();
         ++iterMap) {
        const AxisStatus status = iterMap->first;
        const RsPose::Ptr &robo_pose = iterMap->second;

        perception_ros_msg::AxisStatusPose ros_pose;
        transform(status, robo_pose, ros_pose);

        ros_poses.status_poses.push_back(ros_pose);
    }
}

void RosTransformUtil::transform(const perception_ros_msg::PoseMap &ros_poses,
std::map<AxisStatus, RsPose::Ptr> &robo_poses) {
    int poseCnt = ros_poses.status_poses.size();
    for (int i = 0; i < poseCnt; ++i) {
        AxisStatus status;
        RsPose::Ptr robo_pose;
        transform(ros_poses.status_poses[i], status, robo_pose);
        robo_poses[status] = robo_pose;
    }
}

// Status & Pose Pair
void RosTransformUtil::transform(const AxisStatus status, const RsPose::Ptr &robo_pose,
perception_ros_msg::AxisStatusPose &ros_pose) {
    transform(robo_pose, ros_pose.pose);
    ros_pose.status.data = static_cast<int>(status);
}

void RosTransformUtil::transform(const perception_ros_msg::AxisStatusPose &ros_pose, AxisStatus &status,
RsPose::Ptr &robo_pose) {
    if (robo_pose == nullptr) {
        robo_pose.reset(new RsPose());
    }

    transform(ros_pose.pose, robo_pose);
    status = static_cast<AxisStatus>(ros_pose.status.data);
}

// PointCloud
void RosTransformUtil::transform(const RsPointCloudGPT::Ptr &robo_pc,
std::vector<perception_ros_msg::Point4f> &ros_pc) {
    if (robo_pc == nullptr) {
        return;
    }

    int pcSize = robo_pc->size();
    ros_pc.resize(pcSize);
    for (int i = 0; i < pcSize; ++i) {
        const auto &point = robo_pc->points[i];
        perception_ros_msg::Point4f &ros_point = ros_pc[i];
        ros_point.x.data = point.x;
        ros_point.y.data = point.y;
        ros_point.z.data = point.z;
        ros_point.i.data = point.intensity;
    }
}

void RosTransformUtil::transform(const std::vector<perception_ros_msg::Point4f> &ros_pc,
RsPointCloudGPT::Ptr &robo_pc) {
    if (robo_pc == nullptr) {
        robo_pc.reset(new RsPointCloudGPT());
    }
    int pcSize = ros_pc.size();
    robo_pc->resize(pcSize);
    for (int i = 0; i < pcSize; ++i) {
        const perception_ros_msg::Point4f &ros_point = ros_pc[i];
        auto &point = robo_pc->points[i];
        point.x = ros_point.x.data;
        point.y = ros_point.y.data;
        point.z = ros_point.z.data;
        point.intensity = ros_point.i.data;
    }
}

// Indices
void RosTransformUtil::transform(const std::vector<int> &robo_indices,
                                  perception_ros_msg::Indices &ros_indices) {
    int indices_size = robo_indices.size();
    ros_indices.indices.resize(indices_size);
    for (int i = 0; i < indices_size; ++i) {
        ros_indices.indices[i].data = robo_indices[i];
    }
}

void RosTransformUtil::transform(const perception_ros_msg::Indices &ros_indices,
std::vector<int> &robo_indices) {
    int indices_size = ros_indices.indices.size();
    robo_indices.resize(indices_size);
    for (int i = 0; i < indices_size; ++i) {
        robo_indices[i] = ros_indices.indices[i].data;
    }
}



// Lane
void RosTransformUtil::fromRoboToRos(const Lane::Ptr &robo_lane,
                                       perception_ros_msg::Lane &ros_lane) {
    ros_lane.lane_id.data = static_cast<int>(robo_lane->lane_id);

    fromRoboToRos(robo_lane->curve, ros_lane.curve);

    fromRoboToRos(robo_lane->end_point, ros_lane.end_points);

    ros_lane.measure_status.data = static_cast<int>(robo_lane->measure_status);

    ros_lane.confidence.data = robo_lane->confidence;
}

void RosTransformUtil::fromRosToRobo(const perception_ros_msg::Lane &ros_lane,
                                       Lane::Ptr &robo_lane) {
    if (robo_lane == nullptr) {
        robo_lane.reset(new Lane);
    }

    robo_lane->lane_id = static_cast<LanePosition>(ros_lane.lane_id.data);

    fromRosToRobo(ros_lane.curve, robo_lane->curve);

    fromRosToRobo(ros_lane.end_points, robo_lane->end_point);

    robo_lane->measure_status =
    static_cast<MeasureStatus>(ros_lane.measure_status.data);

    robo_lane->confidence = ros_lane.confidence.data;
}

// RoadEdge
void RosTransformUtil::fromRoboToRos(const Roadedge::Ptr &robo_curb,
                                       perception_ros_msg::RoadEdge &ros_curb) {
    ros_curb.roadedge_id.data = static_cast<int>(robo_curb->roadedge_id);

    fromRoboToRos(robo_curb->curve, ros_curb.curve);

    fromRoboToRos(robo_curb->end_point, ros_curb.end_points);

    ros_curb.measure_status.data = static_cast<int>(robo_curb->measure_status);

    ros_curb.confidence.data = robo_curb->confidence;
}

void RosTransformUtil::fromRosToRobo(const perception_ros_msg::RoadEdge ros_curb, Roadedge::Ptr &robo_curb) {
    if (robo_curb == nullptr) {
        robo_curb.reset(new Roadedge);
    }

    robo_curb->roadedge_id =
    static_cast<RoadedgePosition>(ros_curb.roadedge_id.data);

    fromRosToRobo(ros_curb.curve, robo_curb->curve);

    fromRosToRobo(ros_curb.end_points, robo_curb->end_point);

    robo_curb->measure_status =
    static_cast<MeasureStatus>(ros_curb.measure_status.data);

    robo_curb->confidence = ros_curb.confidence.data;
}

// Curve
void RosTransformUtil::fromRoboToRos(const Curve &robo_curve, perception_ros_msg::Curve &ros_curve) {
    ros_curve.x_start.data = robo_curve.x_start;
    ros_curve.x_end.data = robo_curve.x_end;
    ros_curve.a.data = robo_curve.a;
    ros_curve.b.data = robo_curve.b;
    ros_curve.c.data = robo_curve.c;
    ros_curve.d.data = robo_curve.d;
}

void RosTransformUtil::fromRosToRobo(const perception_ros_msg::Curve &ros_curve, Curve &robo_curve) {
    robo_curve.x_start = ros_curve.x_start.data;
    robo_curve.x_end = ros_curve.x_end.data;
    robo_curve.a = ros_curve.a.data;
    robo_curve.b = ros_curve.b.data;
    robo_curve.c = ros_curve.c.data;
    robo_curve.d = ros_curve.d.data;
}

// Endpoint
void RosTransformUtil::fromRoboToRos(const EndPoints &robo_endpoints,
perception_ros_msg::EndPoints &ros_endpoints) {
    fromRoboToRos(robo_endpoints.start, ros_endpoints.start);
    fromRoboToRos(robo_endpoints.end, ros_endpoints.end);
}

void RosTransformUtil::fromRosToRobo(const perception_ros_msg::EndPoints &ros_endpoints,
EndPoints &robo_endpoints) {
    fromRosToRobo(ros_endpoints.start, robo_endpoints.start);
    fromRosToRobo(ros_endpoints.end, robo_endpoints.end);
}

}   // namespace Ros
#endif  // RS_ROS_FOUND
}   // namespace perception
}   // namespace robosense

