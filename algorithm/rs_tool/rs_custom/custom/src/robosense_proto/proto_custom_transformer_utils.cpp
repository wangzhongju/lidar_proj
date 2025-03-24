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
#include "rs_perception/custom/robosense_proto/proto_custom_transformer_utils.h"

#ifdef RS_PROTO_FOUND

namespace robosense {
namespace perception {
namespace native_sdk_3_1 {

// Object
int RsProtoSerializeUtils::serialize(const Object::Ptr& object, const int& device_id, bool isSupplement,
                                     Proto_msg::Object& proto_object) {
    const auto& robo_core = object->core_infos_;

    // timestamps
    proto_object.set_timestamp(robo_core.timestamp);

    // ==================== Protobuf Serialize CoreInfo ====================//
    Proto_msg::CoreInfos *proto_core_ptr = proto_object.mutable_core_infos();
    // timestamp
    proto_core_ptr->set_timestamp(robo_core.timestamp);

    // priority_id
    proto_core_ptr->set_priority_id(robo_core.priority_id);

    // existence_confidence
    proto_core_ptr->set_existence_confidence(robo_core.exist_confidence);

    // center
    Proto_msg::Point3f *proto_center_ptr = proto_core_ptr->mutable_center();
    fromEigenToProto(robo_core.center, proto_center_ptr);

    //center_cov
    Proto_msg::Point3f *proto_center_cov_ptr = proto_core_ptr->mutable_center_cov();
    fromEigenToProto(robo_core.center_cov, proto_center_cov_ptr);

    // size
    Proto_msg::Point3f *proto_size_ptr = proto_core_ptr->mutable_size();
    fromEigenToProto(robo_core.size, proto_size_ptr);

    // size_cov
    Proto_msg::Point3f *proto_size_cov_ptr = proto_core_ptr->mutable_size_cov();
    fromEigenToProto(robo_core.size_cov, proto_size_cov_ptr);

    // direction
    Proto_msg::Point3f *proto_direction_ptr = proto_core_ptr->mutable_direction();
    fromEigenToProto(robo_core.direction, proto_direction_ptr);

    // direction_cov
    Proto_msg::Point3f *proto_direction_cov_ptr = proto_core_ptr->mutable_direction_cov();
    fromEigenToProto(robo_core.direction_cov, proto_direction_cov_ptr);

    // type
    proto_core_ptr->set_type(static_cast<Proto_msg::ObjectType>(robo_core.type));

    // type_confidence
    proto_core_ptr->set_type_confidence(robo_core.type_confidence);

    // attention_type
    proto_core_ptr->set_attention_type(static_cast<Proto_msg::AttentionType>(robo_core.attention_type));

    // motion_state
    proto_core_ptr->set_motion_state(static_cast<Proto_msg::MotionType>(robo_core.motion_state));

    // lane_pos
    proto_core_ptr->set_lane_pos(static_cast<Proto_msg::LanePosition>(robo_core.lane_pos));

    // tracker_id
    proto_core_ptr->set_tracker_id(robo_core.tracker_id);

    // age
    proto_core_ptr->set_age(robo_core.age);

    // velocity
    Proto_msg::Point3f *proto_velocity_ptr = proto_core_ptr->mutable_velocity();
    fromEigenToProto(robo_core.velocity, proto_velocity_ptr);

    // velocity_cov
    Proto_msg::Point3f *proto_velocity_cov_ptr = proto_core_ptr->mutable_velocity_cov();
    fromEigenToProto(robo_core.velocity_cov, proto_velocity_cov_ptr);

    // acceleration
    Proto_msg::Point3f *proto_acc_ptr = proto_core_ptr->mutable_acceleration();
    fromEigenToProto(robo_core.acceleration, proto_acc_ptr);

    // acceleration_cov
    Proto_msg::Point3f *proto_acc_cov_ptr = proto_core_ptr->mutable_acceleration_cov();
    fromEigenToProto(robo_core.acceleration_cov, proto_acc_cov_ptr);

    // angle_velocity
    proto_core_ptr->set_angle_velocity(robo_core.angle_velocity);

    // angle_velocity_cov
    proto_core_ptr->set_angle_velocity_cov(robo_core.angle_velocity_cov);

    // angle_acceleration
    proto_core_ptr->set_angle_acceleration(robo_core.angle_acceleration);

    // angle_acceleration_cov
    proto_core_ptr->set_angle_acceleration_cov(robo_core.angle_acceleration_cov);

    // anchor
    Proto_msg::Point3f *proto_anchor_ptr = proto_core_ptr->mutable_anchor();
    fromEigenToProto(robo_core.anchor, proto_anchor_ptr);

    // nearest_point
    Proto_msg::Point3f *proto_nearest_ptr = proto_core_ptr->mutable_nearest_point();
    fromEigenToProto(robo_core.nearest_point, proto_nearest_ptr);

    if (isSupplement){
        const auto& robo_supplement = object->supplement_infos_;

        Proto_msg::SupplementInfos *proto_supplement_ptr = proto_object.mutable_supplement_infos();

        // unique_id
        proto_supplement_ptr->set_unique_id(static_cast<int>(robo_supplement.unique_id));

        // polygon
        Proto_msg::Vector3f *proto_polygon_ptr = proto_supplement_ptr->mutable_polygon();
        size_t polygonSize = robo_supplement.polygon.size();
        for (size_t idx = 0; idx < polygonSize; ++idx)
        {
            const RsVector3f& robo_point = robo_supplement.polygon[idx];
            Proto_msg::Point3f *proto_point = proto_polygon_ptr->add_data();
            fromEigenToProto(robo_point, proto_point);
        }

        // left_point_index
        proto_supplement_ptr->set_left_point_index(robo_supplement.left_point_index);

        // right_point_index
        proto_supplement_ptr->set_right_point_index(robo_supplement.right_point_index);

        // latent_types
        Proto_msg::Vector1f *proto_latent_ptr = proto_supplement_ptr->mutable_latent_types();
        size_t latentSize = robo_supplement.latent_types.size();
        for (size_t idx = 0; idx < latentSize; ++idx)
        {
            proto_latent_ptr->add_data(robo_supplement.latent_types[idx]);
        }

        // size_type
        proto_supplement_ptr->set_size_type(static_cast<Proto_msg::SizeType>(robo_supplement.size_type));

        // mode
        proto_supplement_ptr->set_mode(static_cast<Proto_msg::ModeType>(robo_supplement.mode));

        // in_roi
        proto_supplement_ptr->set_in_roi(robo_supplement.in_roi);

        // tracking_state
        proto_supplement_ptr->set_tracking_state(static_cast<Proto_msg::TrackingState>(robo_supplement.tracking_state));

        // geo_center
        Proto_msg::Point3f *proto_geo_center = proto_supplement_ptr->mutable_geo_center();
        fromEigenToProto(robo_supplement.geo_center, proto_geo_center);

        // geo_size
        Proto_msg::Point3f *proto_geo_size = proto_supplement_ptr->mutable_geo_size();
        fromEigenToProto(robo_supplement.geo_size, proto_geo_size);

        // trajectory
        Proto_msg::Vector3f *proto_trajectory_ptr = proto_supplement_ptr->mutable_trajectory();
        size_t trajectorySize = robo_supplement.trajectory.size();
        for (size_t i = 0; i < trajectorySize; ++i) {
            const RsVector3f& robo_point = robo_supplement.trajectory[i];
            Proto_msg::Point3f *proto_point = proto_trajectory_ptr->add_data();
            fromEigenToProto(robo_point, proto_point);
        }

        // history_velocity
        Proto_msg::Vector3f *proto_history_velocity_ptr = proto_supplement_ptr->mutable_history_velocity();
        size_t historyVelocitySize = robo_supplement.history_velocity.size();
        for (size_t i = 0; i < historyVelocitySize; ++i) {
            const RsVector3f& robo_point = robo_supplement.history_velocity[i];
            Proto_msg::Point3f *proto_point = proto_history_velocity_ptr->add_data();
            fromEigenToProto(robo_point, proto_point);
        }

        // history_type
        Proto_msg::VectorObjType *proto_history_type_ptr = proto_supplement_ptr->mutable_history_type();
        size_t historyTypeSize = robo_supplement.history_type.size();
        for (size_t i = 0; i < historyTypeSize; ++i) {
            proto_history_type_ptr->add_data(static_cast<Proto_msg::ObjectType>(robo_supplement.history_type[i]));
        }

        // gps_mode
        proto_supplement_ptr->set_gps_mode(static_cast<Proto_msg::GpsType>(robo_supplement.gps_mode));

        // gps_info
        Proto_msg::Point3d *proto_gps_info_ptr = proto_supplement_ptr->mutable_gps_info();
        proto_gps_info_ptr->add_data(robo_supplement.gps_longtitude);
        proto_gps_info_ptr->add_data(robo_supplement.gps_latitude);
        proto_gps_info_ptr->add_data(robo_supplement.gps_altitude);
    }
    else {
        proto_object.clear_supplement_infos();
    }

    return 0;
}

int RsProtoSerializeUtils::deserialize(const Proto_msg::Object& proto_object, Object::Ptr& object, int& device_id) {
    object.reset(new Object());

    // CoreInfos
    auto& robo_core = object->core_infos_;

    // Protobuf CoreInfos
    const Proto_msg::CoreInfos& proto_core = proto_object.core_infos();

    // center
    const Proto_msg::Point3f& proto_center = proto_core.center();
    fromProtoToEigen(proto_center, robo_core.center);

    // center_cov
    const Proto_msg::Point3f& proto_center_cov = proto_core.center_cov();
    fromProtoToEigen(proto_center_cov, robo_core.center_cov);

    // size
    const Proto_msg::Point3f& proto_size = proto_core.size();
    fromProtoToEigen(proto_size, robo_core.size);

    // size_cov
    const Proto_msg::Point3f& proto_size_cov = proto_core.size_cov();
    fromProtoToEigen(proto_size_cov, robo_core.size_cov);

    // direction
    const Proto_msg::Point3f& proto_direction = proto_core.direction();
    fromProtoToEigen(proto_direction, robo_core.direction);

    // direction_cov
    const Proto_msg::Point3f& proto_direction_cov = proto_core.direction_cov();
    fromProtoToEigen(proto_direction_cov, robo_core.direction_cov);

    // type
    robo_core.type = static_cast<ObjectType>(proto_core.type());

    // type_confidence
    robo_core.type_confidence = proto_core.type_confidence();

    // attention_type
    robo_core.attention_type = static_cast<AttentionType>(proto_core.attention_type());

    // motion_state
    robo_core.motion_state = static_cast<MotionType>(proto_core.motion_state());

    // lane_pos
    // RINFO << "proto_core lane_pose = " << proto_core.has_lane_pos();
    robo_core.lane_pos = static_cast<LanePosition>(proto_core.lane_pos());

    // trakcer_id
    robo_core.tracker_id = proto_core.tracker_id();

    // age
    robo_core.age = proto_core.age();

    // velocity
    const Proto_msg::Point3f& proto_velocity = proto_core.velocity();
    fromProtoToEigen(proto_velocity, robo_core.velocity);

    // velocity_cov
    const Proto_msg::Point3f& proto_velocity_cov = proto_core.velocity_cov();
    fromProtoToEigen(proto_velocity_cov, robo_core.velocity_cov);

    // acceleration
    const Proto_msg::Point3f& proto_acc = proto_core.acceleration();
    fromProtoToEigen(proto_acc, robo_core.acceleration);

    // acceleration_cov
    const Proto_msg::Point3f& proto_acc_cov = proto_core.acceleration_cov();
    fromProtoToEigen(proto_acc_cov, robo_core.acceleration_cov);

    // angle_velocity
    robo_core.angle_velocity = proto_core.angle_velocity();

    // angle_velocity_cov
    robo_core.angle_velocity_cov = proto_core.angle_velocity_cov();

    // angle_acceleration
    robo_core.angle_acceleration = proto_core.angle_acceleration();

    // angle_acceleration_cov
    robo_core.angle_acceleration_cov = proto_core.angle_acceleration_cov();

    // anchor
    const Proto_msg::Point3f& proto_anchor = proto_core.anchor();
    fromProtoToEigen(proto_anchor, robo_core.anchor);

    // nearest_point
    const Proto_msg::Point3f& proto_nearest = proto_core.nearest_point();
    fromProtoToEigen(proto_nearest, robo_core.nearest_point);

    // Supplement Info
    if (proto_object.has_supplement_infos()) {
        const Proto_msg::SupplementInfos& proto_supplement = proto_object.supplement_infos();
        auto& robo_supplement = object->supplement_infos_;

        // unique_id
        robo_supplement.unique_id = proto_supplement.unique_id();

        // polygon
        if (proto_supplement.has_polygon()) {
            const Proto_msg::Vector3f& proto_polygon = proto_supplement.polygon();
            int polygonSize = proto_polygon.data_size();
            if (polygonSize > 0) {
                robo_supplement.polygon.resize(polygonSize);
                for (int idx = 0; idx < polygonSize; ++idx) {
                    const Proto_msg::Point3f& proto_point = proto_polygon.data(idx);
                    fromProtoToEigen(proto_point, robo_supplement.polygon[idx]);
                }
            }
            else {
                robo_supplement.polygon.clear();
            }
        }

        // left_point_index
        robo_supplement.left_point_index = proto_supplement.left_point_index();

        // right_point_index
        robo_supplement.right_point_index = proto_supplement.right_point_index();

        // latent_type
        const Proto_msg::Vector1f& proto_latent_type = proto_supplement.latent_types();
        int latentTypeSize = proto_latent_type.data_size();
        if (latentTypeSize > 0) {
            robo_supplement.latent_types.resize(latentTypeSize);
            for (int idx = 0; idx < latentTypeSize; ++idx) {
                robo_supplement.latent_types[idx] = proto_latent_type.data(idx);
            }
        }
        else {
            robo_supplement.latent_types.clear();
        }

        // size_type
        robo_supplement.size_type = static_cast<SizeType>(proto_supplement.size_type());

        // mode
        robo_supplement.mode = static_cast<ModeType>(proto_supplement.mode());

        // in_roi
        robo_supplement.in_roi = proto_supplement.in_roi();

        // tracking state
        robo_supplement.tracking_state = static_cast<TrackingState>(proto_supplement.tracking_state());

        // geo_center
        const Proto_msg::Point3f& proto_geo_center = proto_supplement.geo_center();
        fromProtoToEigen(proto_geo_center, robo_supplement.geo_center);

        // geo_size
        const Proto_msg::Point3f& proto_geo_size = proto_supplement.geo_size();
        fromProtoToEigen(proto_geo_size, robo_supplement.geo_size);
        
        // trajectory
        const Proto_msg::Vector3f& proto_trajectory = proto_supplement.trajectory();
        int trajectorySize = proto_trajectory.data_size();
        if (trajectorySize > 0) {
            RsVector3f robo_point;
            for (int idx = 0; idx < trajectorySize; ++idx) {
                const Proto_msg::Point3f& proto_point = proto_trajectory.data(idx);
                fromProtoToEigen(proto_point, robo_point);
                robo_supplement.trajectory.push_back(robo_point);
            }
        }

        // history_velocity
        const Proto_msg::Vector3f& proto_history_velocity = proto_supplement.history_velocity();
        int historyVelocitySize = proto_history_velocity.data_size();
        if (historyVelocitySize > 0) {
            RsVector3f robo_point;
            for (int idx = 0; idx < historyVelocitySize; ++idx) {
                const Proto_msg::Point3f& proto_point = proto_history_velocity.data(idx);
                fromProtoToEigen(proto_point, robo_point);
                robo_supplement.history_velocity.push_back(robo_point);
            }
        }

        // history_type
        const Proto_msg::VectorObjType& proto_object_type = proto_supplement.history_type();
        int historyTypeSize = proto_object_type.data_size();
        if (historyTypeSize > 0) {
            for (int idx = 0; idx < historyVelocitySize; ++idx) {
                robo_supplement.history_type.push_back(static_cast<ObjectType>(proto_object_type.data(idx)));
            }
        }

        // gps_info
        robo_supplement.gps_mode = static_cast<GpsType>(proto_supplement.gps_mode());

        const Proto_msg::Point3d& proto_gps_info = proto_supplement.gps_info();
        robo_supplement.gps_longtitude = proto_gps_info.data(0);
        robo_supplement.gps_latitude = proto_gps_info.data(1);
        robo_supplement.gps_altitude = proto_gps_info.data(2);
    }
    return 0;
}

// VecObjectPtr
int RsProtoSerializeUtils::serialize(const VecObjectPtr& objects, const int& device_ids, bool isSupplement,
                                     std::vector<Proto_msg::Object>& proto_objects) {
    proto_objects.clear();
    size_t obj_nums = objects.size();
    if (obj_nums == 0) {
        return -1;
    }

    proto_objects.resize(obj_nums);

    for (size_t i = 0; i < obj_nums; i++) {
        serialize(objects[i], device_ids, isSupplement, proto_objects[i]);
    }

    return 0;
}

int RsProtoSerializeUtils::deserialize(const std::vector<Proto_msg::Object>& proto_objects, VecObjectPtr& objects,
                                       int& device_id) {
    objects.clear();
    size_t proto_obj_nums = proto_objects.size();
    if (proto_obj_nums == 0) {
        return -1;
    }

    objects.resize(proto_obj_nums);
    for (size_t i = 0; i < proto_obj_nums; i++) {
        deserialize(proto_objects[i], objects[i], device_id);
    }
    return 0;
}

// Freespace
int RsProtoSerializeUtils::serialize(const RsFreeSpace::Ptr& freespaces, const double& timestamp, const int& device_id,
                                     Proto_msg::FreeSpaces& proto_freespaces) {
    if (freespaces == nullptr) {
        return -1;
    }

    size_t freespace_point_nums = freespaces->fs_pts.size();
    if (freespace_point_nums == 0) {
        return -2;
    }

    // timestamp
    proto_freespaces.set_timestamp(timestamp);

    // fs_pts
    Proto_msg::Vector3f *proto_freespace_points_ptr = proto_freespaces.mutable_fs_pts();
    for (size_t i = 0; i < freespace_point_nums; i++) {
        const RsVector3f& robo_fs_pts = freespaces->fs_pts[i];
        Proto_msg::Point3f *proto_freespace_point_ptr = proto_freespace_points_ptr->add_data();
        fromEigenToProto(robo_fs_pts, proto_freespace_point_ptr);
    }

    // fs_confidence
    Proto_msg::Vector1f *proto_fs_confidence_ptr = proto_freespaces.mutable_fs_confidence();
    size_t fs_confidence_nums = freespaces->fs_confidence.size();
    for (size_t j = 0; j < fs_confidence_nums; j++) {
        proto_fs_confidence_ptr->add_data(freespaces->fs_confidence[j]);
    }

    // fs_type
    Proto_msg::VectorFreeSpaceType *proto_fs_types_ptr = proto_freespaces.mutable_fs_types();
    for (const auto& fs_type: freespaces->fs_types) {
        proto_fs_types_ptr->add_data(static_cast<Proto_msg::FreeSpaceType>(fs_type));
    }

    return 0;
}

int RsProtoSerializeUtils::deserialize(const Proto_msg::FreeSpaces& proto_freespaces, RsFreeSpace::Ptr& freespaces,
                                       double& timestamp, int& device_id) {
    if (freespaces == nullptr) {
        freespaces.reset(new RsFreeSpace);
    }
    freespaces->fs_pts.clear();
    freespaces->fs_confidence.clear();
    freespaces->fs_types.clear();

    // timestamps
    timestamp = proto_freespaces.timestamp();

    // fs_pts
    if (proto_freespaces.has_fs_pts()) {
        const Proto_msg::Vector3f& proto_fs_pts = proto_freespaces.fs_pts();
        int fs_pts_nums = proto_fs_pts.data_size();
        if (fs_pts_nums > 0) {
            freespaces->fs_pts.resize(fs_pts_nums);
            for (int idx = 0; idx < fs_pts_nums; idx++) {
                const Proto_msg::Point3f& proto_fs_pt = proto_fs_pts.data(idx);
                fromProtoToEigen(proto_fs_pt, freespaces->fs_pts[idx]);
            }
        }
        else {
            freespaces->fs_pts.clear();
        }
    }

    // fs_confidence
    const Proto_msg::Vector1f& proto_fs_confidence = proto_freespaces.fs_confidence();
    int fs_confidence_nums = proto_fs_confidence.data_size();
    if (fs_confidence_nums > 0) {
        freespaces->fs_confidence.resize(fs_confidence_nums);
        for (int i = 0; i < fs_confidence_nums; i++) {
            freespaces->fs_confidence[i] = proto_fs_confidence.data(i);
        }
    }
    else {
        freespaces->fs_confidence.clear();
    }

    // fs_type
    const Proto_msg::VectorFreeSpaceType& proto_fs_types = proto_freespaces.fs_types();
    int fs_types_nums = proto_fs_types.data_size();
    if (fs_types_nums > 0) {
        for (int j = 0; j < fs_types_nums; j++) {
            freespaces->fs_types.push_back(static_cast<RsFreespaceType>(proto_fs_types.data(j)));
        }
    }
    return 0;
}

// Curb
int RsProtoSerializeUtils::serialize(const std::vector<Roadedge::Ptr>& curbs, const double& timestamp,
                                     const int& device_id, Proto_msg::RoadEdges& proto_curbs) {
    size_t curb_nums = curbs.size();
    if (curb_nums == 0) {
        return -1;
    }

    // timestamp
    proto_curbs.set_timestamp(timestamp);

    // curbs
    for (const auto& curb: curbs) {
        // curb
        Proto_msg::RoadEdge *proto_curb_ptr = proto_curbs.add_roadedges();

        // EndPoints
        const auto& robo_endpoint = curb->end_point;
        Proto_msg::EndPoints *proto_endpoint_ptr = proto_curb_ptr->mutable_end_points();
        Proto_msg::Point2f *proto_start_ptr = proto_endpoint_ptr->mutable_start();
        Proto_msg::Point2f *proto_end_ptr = proto_endpoint_ptr->mutable_end();

        fromEigenToProto(robo_endpoint.start, proto_start_ptr);
        fromEigenToProto(robo_endpoint.end, proto_end_ptr);

        // MeasureStatus
        proto_curb_ptr->set_measure_status(static_cast<Proto_msg::MeasureStatus>(curb->measure_status));

        // RoadedgePosition
        proto_curb_ptr->set_roadedge_id(static_cast<Proto_msg::RoadedgePosition>(curb->roadedge_id));

        // road_type
//        proto_curb_ptr->set_road_type(curb->road_type);

        // Curve
        const auto& robo_curve = curb->curve;
        Proto_msg::Curve *proto_curve_ptr = proto_curb_ptr->mutable_curve();
        fromEigenToProto(robo_curve, proto_curve_ptr);

        // confidence
        proto_curb_ptr->set_confidence(curb->confidence);

        // road_start
//        const auto& road_start = curb->road_start;
//        Proto_msg::Point2f *proto_road_start_ptr = proto_curb_ptr->mutable_road_start();
//        fromEigenToProto(road_start, proto_road_start_ptr);

        // road_end
//        const auto& road_end = curb->road_end;
//        Proto_msg::Point2f *proto_road_end_ptr = proto_curb_ptr->mutable_road_end();
//        fromEigenToProto(road_end, proto_road_end_ptr);

        // predict_road_start
//        const auto& predict_road_start = curb->predict_road_start;
//        Proto_msg::Point2f *proto_predict_road_start_ptr = proto_curb_ptr->mutable_predict_road_start();
//        fromEigenToProto(predict_road_start, proto_predict_road_start_ptr);

        // predict_road_end
//        const auto& predict_road_end = curb->predict_road_end;
//        Proto_msg::Point2f *proto_predict_road_end_ptr = proto_curb_ptr->mutable_predict_road_end();
//        fromEigenToProto(predict_road_end, proto_predict_road_end_ptr);

        // control_points
//        Proto_msg::Vector3f *proto_control_points_ptr = proto_curb_ptr->mutable_control_points();
//        for (const auto& robo_point: curb->control_points) {
//            Proto_msg::Point3f *proto_control_point = proto_control_points_ptr->add_data();
//            fromEigenToProto(robo_point, proto_control_point);
//        }
    }

    return 0;
}

int RsProtoSerializeUtils::deserialize(const Proto_msg::RoadEdges& proto_curbs, std::vector<Roadedge::Ptr>& curbs,
                                       double& timestamp, int& device_id) {
    const google::protobuf::RepeatedPtrField<Proto_msg::RoadEdge>& proto_vec_curbs = proto_curbs.roadedges();

    int curb_nums = proto_vec_curbs.size();

    if (curb_nums == 0) {
        return -1;
    }

    curbs.resize(curb_nums);

    // timestamp
    timestamp = proto_curbs.timestamp();

    // curbs
    for (int i = 0; i < curb_nums; i++) {
        Roadedge::Ptr &robo_curb_ptr = curbs[i];

        robo_curb_ptr.reset(new Roadedge());

        const Proto_msg::RoadEdge &proto_curb = proto_curbs.roadedges(i);

        // end_points
        const Proto_msg::EndPoints &proto_endpoint = proto_curb.end_points();
        EndPoints &robo_endpoint = robo_curb_ptr->end_point;

        fromProtoToEigen(proto_endpoint.start(), robo_endpoint.start);
        fromProtoToEigen(proto_endpoint.end(), robo_endpoint.end);

        // measure_status
        robo_curb_ptr->measure_status = static_cast<MeasureStatus>(proto_curb.measure_status());

        // roadedge_id
        robo_curb_ptr->roadedge_id = static_cast<RoadedgePosition>(proto_curb.roadedge_id());

        // road_type
//        robo_curb_ptr->road_type = proto_curb.road_type();

        // curve
        const Proto_msg::Curve& proto_curve = proto_curb.curve();
        Curve& robo_curve = robo_curb_ptr->curve;
        fromProtoToEigen(proto_curve, robo_curve);

        // confidence
        robo_curb_ptr->confidence = proto_curb.confidence();

        // road_start
//        const Proto_msg::Point2f& proto_road_start = proto_curb.road_start();
//        fromProtoToEigen(proto_road_start, robo_curb_ptr->road_start);

        // road_end
//        const Proto_msg::Point2f& proto_road_end = proto_curb.road_end();
//        fromProtoToEigen(proto_road_end, robo_curb_ptr->road_end);

        // predict_road_start
//        const Proto_msg::Point2f& proto_predict_road_start = proto_curb.predict_road_start();
//        fromProtoToEigen(proto_predict_road_start, robo_curb_ptr->predict_road_start);

        // predict_road_end
//        const Proto_msg::Point2f& proto_predict_road_end = proto_curb.predict_road_end();
//        fromProtoToEigen(proto_predict_road_end, robo_curb_ptr->predict_road_end);

//        // control_point
//        if (proto_curb.has_control_points()) {
//            const Proto_msg::Vector3f& proto_control_point = proto_curb.control_points();
//            int control_point_nums = proto_control_point.data_size();
//            if (control_point_nums > 0) {
//                robo_curb_ptr->control_points.resize(control_point_nums);
//                for (int idx = 0; idx < control_point_nums; idx++) {
//                    const Proto_msg::Point3f& proto_point = proto_control_point.data(idx);
//                    fromProtoToEigen(proto_point, robo_curb_ptr->control_points[idx]);
//                }
//            }
//            else {
//                robo_curb_ptr->control_points.clear();
//            }
//        }
    }
    return 0;
}

// Lane
int RsProtoSerializeUtils::serialize(const std::vector<Lane::Ptr>& lanes, const double& timestamp, const int& device_id,
                                     Proto_msg::Lanes& proto_lanes) {
    size_t lane_nums = lanes.size();
    if (lane_nums == 0) {
        return -1;
    }

    // timestamp
    proto_lanes.set_timestamp(timestamp);

    for (const auto& lane: lanes) {
        // lane
        Proto_msg::Lane *proto_lane_ptr = proto_lanes.add_lanes();

        // EndPoints
        const auto& robo_endpoint = lane->end_point;
        Proto_msg::EndPoints *proto_endpoint_ptr = proto_lane_ptr->mutable_end_point();
        Proto_msg::Point2f *proto_start_ptr = proto_endpoint_ptr->mutable_start();
        Proto_msg::Point2f *proto_end_ptr = proto_endpoint_ptr->mutable_end();

        fromEigenToProto(robo_endpoint.start, proto_start_ptr);
        fromEigenToProto(robo_endpoint.end, proto_end_ptr);

        // MeasureStatus
        proto_lane_ptr->set_measure_status(static_cast<Proto_msg::MeasureStatus>(lane->measure_status));

        // pose_type
        proto_lane_ptr->set_lane_id(static_cast<Proto_msg::LanePosition>(lane->lane_id));

        // type
//        proto_lane_ptr->set_type(lane->type);

        // Curve
        const auto& robo_curve = lane->curve;
        Proto_msg::Curve *proto_curve_ptr = proto_lane_ptr->mutable_curve();
        fromEigenToProto(robo_curve, proto_curve_ptr);

        // confidence
        proto_lane_ptr->set_confidence(lane->confidence);

        // ego_lane_width
//        proto_lane_ptr->set_ego_lane_width(lane->ego_lane_width);

        // dotted_line_gap_len
//        proto_lane_ptr->set_dotted_line_gap_len(lane->dotted_line_gap_len);

        // lane_line_width
//        proto_lane_ptr->set_lane_line_width(lane->lane_line_width);

        // lane_start
//        const auto& lane_start = lane->lane_start;
//        Proto_msg::Point2f *proto_lane_start_ptr = proto_lane_ptr->mutable_lane_start();
//        fromEigenToProto(lane_start, proto_lane_start_ptr);

        // lane_end
//        const auto& lane_end = lane->lane_end;
//        Proto_msg::Point2f *proto_lane_end_ptr = proto_lane_ptr->mutable_lane_end();
//        fromEigenToProto(lane_end, proto_lane_end_ptr);

        // predict_lane_start
//        const auto& predict_lane_start = lane->predict_lane_start;
//        Proto_msg::Point2f *proto_predict_lane_start_ptr = proto_lane_ptr->mutable_predict_lane_start();
//        fromEigenToProto(predict_lane_start, proto_predict_lane_start_ptr);

        // predict_lane_end
//        const auto& predict_lane_end = lane->predict_lane_end;
//        Proto_msg::Point2f *proto_predict_lane_end_ptr = proto_lane_ptr->mutable_predict_lane_end();
//        fromEigenToProto(predict_lane_end, proto_predict_lane_end_ptr);

        // control_points
//        Proto_msg::Vector3f *proto_control_points_ptr = proto_lane_ptr->mutable_control_points();
//        for (const auto& robo_point: lane->control_points) {
//            Proto_msg::Point3f *proto_control_point = proto_control_points_ptr->add_data();
//            fromEigenToProto(robo_point, proto_control_point);
//        }
    }

    return 0;
}

int RsProtoSerializeUtils::deserialize(const Proto_msg::Lanes& proto_lanes, std::vector<Lane::Ptr>& lanes,
                                       double& timestamp, int& device_id) {
    const google::protobuf::RepeatedPtrField<Proto_msg::Lane> &proto_vec_lanes = proto_lanes.lanes();

    int lane_nums = proto_vec_lanes.size();

    if (lane_nums == 0) {
        return -1;
    }

    lanes.resize(lane_nums);

    // timestamp
    timestamp = proto_lanes.timestamp();

    for (int i = 0; i < lane_nums; i++) {
        Lane::Ptr &robo_lane_ptr = lanes[i];

        robo_lane_ptr.reset(new Lane());

        const Proto_msg::Lane &proto_lane = proto_lanes.lanes(i);

        // end_points
        const Proto_msg::EndPoints &proto_endpoint = proto_lane.end_point();
        EndPoints &robo_endpoint = robo_lane_ptr->end_point;

        fromProtoToEigen(proto_endpoint.start(), robo_endpoint.start);
        fromProtoToEigen(proto_endpoint.end(), robo_endpoint.end);

        // measure_status
        robo_lane_ptr->measure_status = static_cast<MeasureStatus>(proto_lane.measure_status());

        // lane_id
        robo_lane_ptr->lane_id = static_cast<LanePosition>(proto_lane.lane_id());

        // type
//        robo_lane_ptr->type = proto_lane.type();

        // curve
        const Proto_msg::Curve& proto_curve = proto_lane.curve();
        Curve& robo_curve = robo_lane_ptr->curve;
        fromProtoToEigen(proto_curve, robo_curve);

        // confidence
        robo_lane_ptr->confidence = proto_lane.confidence();

        // ego_lane_width
//        robo_lane_ptr->ego_lane_width = proto_lane.ego_lane_width();

        // dotted_line_gap_len
//        robo_lane_ptr->dotted_line_gap_len = proto_lane.dotted_line_gap_len();

        // lane_line_width
//        robo_lane_ptr->lane_line_width = proto_lane.lane_line_width();

        // lane_start
//        const Proto_msg::Point2f& proto_lane_start = proto_lane.lane_start();
//        fromProtoToEigen(proto_lane_start, robo_lane_ptr->lane_start);

        // lane_end
//        const Proto_msg::Point2f& proto_lane_end = proto_lane.lane_end();
//        fromProtoToEigen(proto_lane_end, robo_lane_ptr->lane_end);

        // predict_lane_start
//        const Proto_msg::Point2f& proto_predict_lane_start = proto_lane.predict_lane_start();
//        fromProtoToEigen(proto_predict_lane_start, robo_lane_ptr->predict_lane_start);

        // predict_lane_end
//        const Proto_msg::Point2f& proto_predict_lane_end = proto_lane.predict_lane_end();
//        fromProtoToEigen(proto_predict_lane_end, robo_lane_ptr->predict_lane_end);

        // control_point
//        if (proto_lane.has_control_points()) {
//            const Proto_msg::Vector3f& proto_control_point = proto_lane.control_points();
//            int control_point_nums = proto_control_point.data_size();
//            if (control_point_nums > 0) {
//                robo_lane_ptr->control_points.resize(control_point_nums);
//                for (int idx = 0; idx < control_point_nums; idx++) {
//                    const Proto_msg::Point3f& proto_point = proto_control_point.data(idx);
//                    fromProtoToEigen(proto_point, robo_lane_ptr->control_points[idx]);
//                }
//            }
//            else {
//                robo_lane_ptr->control_points.clear();
//            }
//        }

    }
    return 0;
}

// Global Pose
int RsProtoSerializeUtils::serialize(const RsPose::Ptr& pose, const double& timestamp, const int& device_id,
                                     Proto_msg::PoseInfo& proto_pose) {
    // timestamp
    proto_pose.set_timestamp(timestamp);

    // Pose
    Proto_msg::Pose *proto_pose_ptr = proto_pose.mutable_pose();

    proto_pose_ptr->set_x(pose->x);
    proto_pose_ptr->set_y(pose->y);
    proto_pose_ptr->set_z(pose->z);
    proto_pose_ptr->set_roll(pose->roll);
    proto_pose_ptr->set_pitch(pose->pitch);
    proto_pose_ptr->set_yaw(pose->yaw);

    return 0;
}

int RsProtoSerializeUtils::deserialize(const Proto_msg::PoseInfo& proto_pose, RsPose::Ptr& robo_pose, double& timestamp,
                                       int& device_id) {
    // timestamp
    timestamp = proto_pose.timestamp();

    if (robo_pose == nullptr) {
        robo_pose.reset(new RsPose);
    }

    const Proto_msg::Pose &proto_pose_ptr = proto_pose.pose();
    // x, y, z, roll, pitch, yaw
    robo_pose->x = proto_pose_ptr.x();
    robo_pose->y = proto_pose_ptr.y();
    robo_pose->z = proto_pose_ptr.z();
    robo_pose->roll = proto_pose_ptr.roll();
    robo_pose->pitch = proto_pose_ptr.pitch();
    robo_pose->yaw = proto_pose_ptr.yaw();

    return 0;
}

// Point Cloud with sematic indices and object cloud indices
int RsProtoSerializeUtils::serialize(const RsPointCloudGPT::Ptr &seg_pointCloud, const VecInt &point_label,
                                     const double &timestamp, Proto_msg::PointCloud &proto_pointCloud) {
    size_t points_nums = seg_pointCloud->points.size();

    if (points_nums == 0) {
        return -1;
    }

    if (points_nums != point_label.size()) {
        return -2;
    }

    // timestamp
    proto_pointCloud.set_timestamp(timestamp);

    // point
    for (size_t i = 0; i < points_nums; i++) {
        const auto& point = seg_pointCloud->points[i];
        const auto& label = point_label[i];
        Proto_msg::PointXYZITL *proto_point_ptr = proto_pointCloud.add_point();

        proto_point_ptr->set_x(point.x);
        proto_point_ptr->set_y(point.y);
        proto_point_ptr->set_z(point.z);
        proto_point_ptr->set_intensity(point.intensity);
        proto_point_ptr->set_timestamp(point.timestamp);
        proto_point_ptr->set_label(label);
    }

    return 0;
}

int RsProtoSerializeUtils::deserialize(const Proto_msg::PointCloud &proto_pointCloud,
                                       RsPointCloudGPT::Ptr &robo_pointCloud, VecInt &point_label, double &timestamp) {
    const auto& proto_point_cloud = proto_pointCloud.point();

    int point_nums = proto_point_cloud.size();

    if (point_nums == 0) {
        return -1;
    }

    if (robo_pointCloud == nullptr) {
        robo_pointCloud.reset(new RsPointCloudGPT);
    }

    robo_pointCloud->points.clear();
    point_label.clear();

    robo_pointCloud->points.resize(point_nums);
    point_label.resize(point_nums);

    // timestamp
    timestamp = proto_pointCloud.timestamp();

    // points
    for (int i = 0; i < point_nums; i++) {
        auto& point = robo_pointCloud->points[i];
        auto& label = point_label[i];
        const Proto_msg::PointXYZITL& proto_point = proto_pointCloud.point(i);

        // point
        point.x = proto_point.x();
        point.y = proto_point.y();
        point.z = proto_point.z();
        point.intensity = proto_point.intensity();
        point.timestamp = proto_point.timestamp();

        // label
        label = proto_point.label();
    }

    return 0;
}

}  // namespace native_sdk_3_1
}  // namespace perception
}  // namespace robosense

#endif  // RS_PROTO_FOUND