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
#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_PROTO_CUSTOM_TRANSFORMER_H
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_PROTO_CUSTOM_TRANSFORMER_H

#include "rs_perception/custom/robosense_proto/proto_custom_transformer_utils.h"

#ifdef RS_PROTO_FOUND

namespace robosense {
namespace perception {
namespace Proto {

class RsProtoSerialize {
public:
    using Ptr = std::shared_ptr<RsProtoSerialize>;
    using ConstPtr = std::shared_ptr<const RsProtoSerialize>;

    explicit RsProtoSerialize(const RsProtoCustomMsgParams::Ptr & customParams) {
        customParams_ = customParams;
//        defaultTrackerId_ = static_cast<int>(native_sdk_3_1::RS_INDEX_TYPE::RS_INDEX_ALG_UNTRACKER);
        currentTrackerId_ = static_cast<int>(native_sdk_3_1::RS_INDEX_TYPE::RS_INDEX_ALG_UNTRACKER);
    }

    // perception message -> protobuf message
    int serialize(const RsPerceptionMsg::Ptr& msg_ptr, const native_sdk_3_1::ROBO_PROTO_DATA_TYPE msgType,
                  native_sdk_3_1::RSProtoSerializeBuffer& en_msg) {
        int offset = 0;
        if (!en_msg.offsets.empty()) {
            offset = *(en_msg.offsets.rbegin()) + *(en_msg.lengths.rbegin());
        }

        // 分配空间，初始化label的状态。这个过程每一帧只做一次。
        int total_point_nums = static_cast<int>(msg_ptr->rs_lidar_result_ptr->scan_ptr->size());
        if (total_point_nums > 0 && customParams_->send_point_cloud
            && static_cast<int>(pointCloudLabels_.size()) != total_point_nums) {
            pointCloudLabels_.resize(total_point_nums);
            for (int i = 0; i <total_point_nums; i++) {
                pointCloudLabels_[i] = -1;
            }
        }

        if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT) {
            auto& objects = msg_ptr->rs_lidar_result_ptr->objects;
            size_t object_nums = objects.size();
            if (object_nums > 0) {
                std::vector<Proto_msg::Object> proto_objects;

                if (customParams_->send_point_cloud && customParams_->send_object_supplement) {
                    for (const auto& obj: objects) {
                        const auto& cloud_indices = obj->supplement_infos_.cloud_indices;
                        auto tracker_id = obj->core_infos_.tracker_id;
                        if (tracker_id < 0) {
                            obj->core_infos_.tracker_id = currentTrackerId_;
                            tracker_id = currentTrackerId_;
                            currentTrackerId_--;
                        }

                        for (int index: cloud_indices) {
                            pointCloudLabels_[index] = tracker_id;
                        }
                    }
                }
                native_sdk_3_1::RsProtoSerializeUtils::serialize(objects,customParams_->device_id,
                                                                 customParams_->send_object_supplement,
                                                                 proto_objects);
                for (size_t i = 0; i < object_nums; i++) {
                    en_msg.offsets.push_back(offset);
                    int object_size = proto_objects[i].ByteSize();
                    proto_objects[i].SerializeToArray(en_msg.buffers.data() + offset, object_size);

                    en_msg.lengths.push_back(object_size);
                    en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT);
                    offset += object_size;
                }

            }
            else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap[msgType] = static_cast<int>(object_nums);
        }
        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT) {
            auto& att_objects = msg_ptr->rs_lidar_result_ptr->attention_objects;
            size_t att_objects_nums = att_objects.size();
            if (att_objects_nums > 0) {
                std::vector<Proto_msg::Object> proto_objects;
                
                // 由于一度目标的定义关系，被标为attention object的物体可能也出现在了object中。
                // 因此，在开启了序列化supplement info的时候，可能会出现同样的cloud indices又被序列化了一次。这个太占地方了，没必要。
                // 所以要在这个地方判定一下每一个attention object是不是同时也是object。只为那些不是的attention object进行序列化cloud indices。
                if (customParams_->send_point_cloud && customParams_->send_object_supplement) {
                    for (const auto& att_obj: att_objects) {
                        bool isObject = false;
                        // 防止关了object只开attention object的骚操作带来的风险。
                        if (customParams_->send_objects) {
                            for (const auto& obj: msg_ptr->rs_lidar_result_ptr->objects) {
                                if (att_obj == obj) {
                                    isObject = true;
                                    break;
                                }
                            }
                        }
                        if (isObject){
                            continue;
                        }

                        // 只有没有双重身份的attention object才配分配ID和给indices上标签
                        if (att_obj->core_infos_.tracker_id < 0) {
                            att_obj->core_infos_.tracker_id = currentTrackerId_;
                            currentTrackerId_--;
                        }
                        const auto& cloud_indices = att_obj->supplement_infos_.cloud_indices;
                        for (const auto& index: cloud_indices) {
                            pointCloudLabels_[index] = att_obj->core_infos_.tracker_id;
                        }
                    }
                }
                
                native_sdk_3_1::RsProtoSerializeUtils::serialize(att_objects, customParams_->device_id,
                                                                 customParams_->send_object_supplement,
                                                                 proto_objects);
                for (size_t i = 0; i < att_objects_nums; i++) {
                    en_msg.offsets.push_back(offset);
                    int object_size = proto_objects[i].ByteSize();
                    proto_objects[i].SerializeToArray(en_msg.buffers.data() + offset, object_size);

                    en_msg.lengths.push_back(object_size);
                    en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT);
                    offset += object_size;
                }
            }
            else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap[msgType] = static_cast<int>(att_objects_nums);
        }
        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE) {
            en_msg.offsets.push_back(offset);
            const auto& freespaces = msg_ptr->rs_lidar_result_ptr->freespace_ptr;
            size_t freesapce_point_nums = freespaces->fs_pts.size();
            if (freesapce_point_nums > 0) {
                Proto_msg::FreeSpaces proto_freespaces;
                native_sdk_3_1::RsProtoSerializeUtils::serialize(freespaces, msg_ptr->rs_lidar_result_ptr->timestamp,
                                                                 customParams_->device_id, proto_freespaces);
                int freespaces_size = proto_freespaces.ByteSize();
                proto_freespaces.SerializeToArray(en_msg.buffers.data() + offset, freespaces_size);

                en_msg.lengths.push_back(freespaces_size);
                en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE);
                offset += freespaces_size;
            }
            else {
                en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap[msgType] = static_cast<int>(freesapce_point_nums);
        }
        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_GLOBALCAR_POSE) {
            en_msg.offsets.push_back(offset);
            const auto& pose_info = msg_ptr->rs_lidar_result_ptr->global_pose_ptr;

            Proto_msg::PoseInfo proto_pose_info;
            native_sdk_3_1::RsProtoSerializeUtils::serialize(pose_info, msg_ptr->rs_lidar_result_ptr->timestamp,
                                                             customParams_->device_id, proto_pose_info);
            int poseinfo_size = proto_pose_info.ByteSize();
            proto_pose_info.SerializeToArray(en_msg.buffers.data() + offset, poseinfo_size);

            en_msg.lengths.push_back(poseinfo_size);
            en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_GLOBALCAR_POSE);

            en_msg.msgCntMap[msgType] = 1;
        }
        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE) {
            en_msg.offsets.push_back(offset);
            const auto& lanes = msg_ptr->rs_lidar_result_ptr->lanes;
            size_t lane_point_num = lanes.size();
            if (lane_point_num > 0) {
                Proto_msg::Lanes proto_lanes;
                native_sdk_3_1::RsProtoSerializeUtils::serialize(lanes, msg_ptr->rs_lidar_result_ptr->timestamp,
                                                                 customParams_->device_id, proto_lanes);
                int lanes_size = proto_lanes.ByteSize();
                proto_lanes.SerializeToArray(en_msg.buffers.data() + offset, lanes_size);

                en_msg.lengths.push_back(lanes_size);
                en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE);
                offset += lanes_size;
            }
            else {
                en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap[msgType] = static_cast<int>(lane_point_num);
        }
        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB) {
            en_msg.offsets.push_back(offset);
            const auto& roadedges = msg_ptr->rs_lidar_result_ptr->roadedges;
            size_t road_point_num = roadedges.size();
            if (road_point_num > 0) {
                Proto_msg::RoadEdges proto_curbs;
                native_sdk_3_1::RsProtoSerializeUtils::serialize(roadedges, msg_ptr->rs_lidar_result_ptr->timestamp,
                                                                 customParams_->device_id, proto_curbs);
                int curb_size = proto_curbs.ByteSize();
                proto_curbs.SerializeToArray(en_msg.buffers.data() + offset, curb_size);

                en_msg.lengths.push_back(curb_size);
                en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB);
                offset += curb_size;
            }
            else {
                en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap[msgType] = static_cast<int>(road_point_num);
        }
        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT) {
            if (total_point_nums > 0) {
                // 获取background indices
                const auto& background_indices = msg_ptr->rs_lidar_result_ptr->background_indices;
                const int bg_label = static_cast<int>(native_sdk_3_1::RS_INDEX_TYPE::RS_INDEX_BG_INDEX);
                for (const auto& bg_index: background_indices) {
                    if (bg_index >= 0 && bg_index < total_point_nums && pointCloudLabels_[bg_index] == -1) {
                        pointCloudLabels_[bg_index] = bg_label;
                    }
                }

                // 获取non_ground indices
                // 说明：本质上 object+background = non_ground。但是这里为了防止没开object，所以还是要处理一下non_ground
                const auto& non_ground_indices = msg_ptr->rs_lidar_result_ptr->non_ground_indices;
                const int ng_label = static_cast<int>(native_sdk_3_1::RS_INDEX_TYPE::RS_INDEX_NGD_INDEX);
                for (const auto& ng_index: non_ground_indices) {
                    if (ng_index >= 0 && ng_index < total_point_nums && pointCloudLabels_[ng_index] == -1) {
                        pointCloudLabels_[ng_index] = ng_label;
                    }
                }

                // 获取ground indices
                // 说明：ground + non_ground = total point cloud indices
                const auto& ground_indices = msg_ptr->rs_lidar_result_ptr->ground_indices;
                const int g_label = static_cast<int>(native_sdk_3_1::RS_INDEX_TYPE::RS_INDEX_GD_INDEX);
                for (const auto& g_index: ground_indices) {
                    if (g_index >= 0 && g_index < total_point_nums && pointCloudLabels_[g_index] == -1) {
                        pointCloudLabels_[g_index] = g_label;
                    }
                }

                // 数据类型大小
                int double_size = sizeof(double);
                int float_size = sizeof(float);
                int int32_size = sizeof(int32_t);
                int uint32_size = sizeof(uint32_t);

                int xyz_size = float_size * 3;
                int xyzi_size = xyz_size + uint32_size;
                int xyzit_size = xyzi_size + double_size;
                int xyzitl_size = xyzit_size + int32_size;

                // 计算最多传输的点的个数
                // int points_per_group
                // = static_cast<int>((customParams_->max_msg_size - sizeof(native_sdk_3_1::st_RoboMsgHeader) - double_size) / xyzitl_size);
                int points_per_group=800;
                // proto序列化点云
                // 说明：由于点云很大，所以要拆成一块块来进行序列化。
                RsPointCloudGPT::Ptr cache_cloud;
                cache_cloud.reset(new RsPointCloudGPT);
                VecInt cache_label;
                const auto& cloud_ptr = msg_ptr->rs_lidar_result_ptr->scan_ptr;
                for (int i = 0; i < total_point_nums; i += points_per_group) {
                    int start = i;
                    int end = std::min(i + points_per_group, total_point_nums);
                    en_msg.offsets.push_back(offset);

                    // 构建一片临时点云和标签索引
                    cache_cloud->points.clear();
                    cache_label.clear();
                    for (int j = start; j < end; j++) {
                        cache_cloud->points.push_back(cloud_ptr->points[j]);
                        cache_label.push_back(pointCloudLabels_[j]);
                    }
                    // 序列化这部分点云
                    Proto_msg::PointCloud proto_point_cloud;
                    native_sdk_3_1::RsProtoSerializeUtils::serialize(cache_cloud, cache_label,
                                                                     msg_ptr->rs_lidar_result_ptr->timestamp,
                                                                     proto_point_cloud);
                    int point_cloud_size = proto_point_cloud.ByteSize();
                    proto_point_cloud.SerializeToArray(en_msg.buffers.data() + offset, point_cloud_size);

                    en_msg.lengths.push_back(point_cloud_size);
                    en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT);
                    en_msg.msgPointCntMap[static_cast<int>(en_msg.types.size()-1)] = end-start;
                    offset += point_cloud_size;
                }
            }
            else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT_EMPTY);
                en_msg.lengths.push_back(0);
                en_msg.msgPointCntMap[static_cast<int>(en_msg.types.size()-1)] = 0;
            }
            en_msg.msgCntMap[msgType] = static_cast<int>(total_point_nums);
            currentTrackerId_ = static_cast<int>(native_sdk_3_1::RS_INDEX_TYPE::RS_INDEX_ALG_UNTRACKER);
        }

        return 0;
    }

    int deserialize(const char *data, const int length, const native_sdk_3_1::ROBO_PROTO_DATA_TYPE msgType,
                    native_sdk_3_1::st_RoboProtoRecvMessage& recvMsg) {


              
        if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT) {
            Proto_msg::Object proto_objects;
        //    std::vector<Proto_msg::Object> objlist;
            Object::Ptr robo_object;
            bool objture = proto_objects.ParseFromArray(data,length);
            native_sdk_3_1::RsProtoSerializeUtils::deserialize(proto_objects, robo_object, recvMsg.device_id);
            recvMsg.msg->rs_lidar_result_ptr->objects.push_back(robo_object);
        }
        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT) {
            Proto_msg::Object proto_objects;
            Object::Ptr robo_object;
            bool objture = proto_objects.ParseFromArray(data,length);
            native_sdk_3_1::RsProtoSerializeUtils::deserialize(proto_objects, robo_object, recvMsg.device_id);
            recvMsg.msg->rs_lidar_result_ptr->attention_objects.push_back(robo_object);
            
        }
        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE) {
            Proto_msg::FreeSpaces proto_freemsg;
            RsFreeSpace::Ptr robo_freespace;
            bool freemsg = proto_freemsg.ParseFromArray(data,length);
            native_sdk_3_1::RsProtoSerializeUtils::deserialize(
            proto_freemsg, robo_freespace,recvMsg.timestamp ,recvMsg.device_id);
            recvMsg.msg->rs_lidar_result_ptr->freespace_ptr=robo_freespace;
        }
        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE) {
            Proto_msg::Lanes proto_lanes;
            std::vector<Lane::Ptr> robo_lanes;
            bool laneture = proto_lanes.ParseFromArray(data,length);
            native_sdk_3_1::RsProtoSerializeUtils::deserialize(proto_lanes, robo_lanes, recvMsg.timestamp ,recvMsg.device_id);
            recvMsg.msg->rs_lidar_result_ptr->lanes=robo_lanes;
        }
        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB) {
            Proto_msg::RoadEdges proto_curbs;
            std::vector<Roadedge::Ptr> robo_curbs;
            bool curbmsg = proto_curbs.ParseFromArray(data,length);
            native_sdk_3_1::RsProtoSerializeUtils::deserialize(
            proto_curbs, robo_curbs,recvMsg.timestamp ,recvMsg.device_id);
            recvMsg.msg->rs_lidar_result_ptr->roadedges=robo_curbs;
        }
        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_GLOBALCAR_POSE) {
            Proto_msg::PoseInfo proto_pose;
            RsPose::Ptr robo_pose;
            bool posemsg = proto_pose.ParseFromArray(data,length);
            native_sdk_3_1::RsProtoSerializeUtils::deserialize(
            proto_pose, robo_pose,recvMsg.timestamp ,recvMsg.device_id);
            recvMsg.msg->rs_lidar_result_ptr->global_pose_ptr=robo_pose;
        }

        else if (msgType == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT) {
            Proto_msg::PointCloud proto_point;
            RsPointCloudGPT::Ptr robo_point;
            bool pointmsg = proto_point.ParseFromArray(data,length);
            native_sdk_3_1::RsProtoSerializeUtils::deserialize(
            proto_point, robo_point,pointCloudLabels_,recvMsg.timestamp );

            recvMsg.msg->rs_lidar_result_ptr->scan_ptr->points.insert(
            recvMsg.msg->rs_lidar_result_ptr->scan_ptr->points.end(),
            robo_point->points.begin(),robo_point->points.end());
        }

        return 0;
    }

private:
    RsProtoCustomMsgParams::Ptr customParams_;
    std::vector<int> pointCloudLabels_;
//    int defaultTrackerId_;
    int currentTrackerId_;
};

}  // namespace proto
}  // namespace perception
}  // namespace robosense

#endif  // RS_PROTO_FOUND
#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_PROTO_CUSTOM_TRANSFORMER_H
