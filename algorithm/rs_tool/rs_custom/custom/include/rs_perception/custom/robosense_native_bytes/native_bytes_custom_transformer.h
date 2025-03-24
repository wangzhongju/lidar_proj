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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_CUSTOM_TRANSFORMATER_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_CUSTOM_TRANSFORMATER_H_
#include "rs_perception/custom/robosense_native_bytes/native_bytes_custom_transformer_util.h"

namespace robosense {
namespace perception {
namespace native_sdk_2_x {

class RSNativeSerialize {
public:
    using Ptr = std::shared_ptr<RSNativeSerialize>;
    using ConstPtr = std::shared_ptr<const RSNativeSerialize>;

public:
    /**
     * @brief serialize
     * @param msg: Robosense Perception Message Data
     * @param en_msg: Protobuf Encode Message Data
     * @return
     */
    static int serialize(const RsPerceptionMsg::Ptr &msg,
                         const ROBO_MSG_TYPE msgType, const int device_id,
                         const int isSupplement, RSSerializeBuffer &en_msg) {
        int offset = 0;
        if (!en_msg.offsets.empty()) {
            offset = *(en_msg.offsets.rbegin()) + *(en_msg.lengths.rbegin());
        }

        if (msgType == ROBO_MSG_TYPE::ROBO_MSG_OBJECT) {
            int objectCnt = msg->rs_lidar_result_ptr->objects.size();
            if (objectCnt > 0) {
                for (int i = 0; i < objectCnt; ++i) {
                    en_msg.offsets.push_back(offset);
                    int object_size = RSNativeSerializeUtil::serialize(
                    msg->rs_lidar_result_ptr->objects[i], isSupplement,
                    en_msg.buffers.data() + offset);
                    en_msg.lengths.push_back(object_size);
                    en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_OBJECT);
                    offset += object_size;
                }
            } else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_OBJECT_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap.insert(
            std::pair<ROBO_MSG_TYPE, int>(msgType, objectCnt));
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT) {
            int objectCnt = msg->rs_lidar_result_ptr->attention_objects.size();
            if (objectCnt > 0) {
                for (int i = 0; i < objectCnt; ++i) {
                    en_msg.offsets.push_back(offset);
                    int object_size = RSNativeSerializeUtil::serialize(
                    msg->rs_lidar_result_ptr->attention_objects[i], isSupplement,
                    en_msg.buffers.data() + offset);
                    en_msg.lengths.push_back(object_size);
                    en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT);
                    offset += object_size;
                }
            } else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap.insert(
            std::pair<ROBO_MSG_TYPE, int>(msgType, objectCnt));
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_FREESPACE) {
            en_msg.offsets.push_back(offset);
            int freespaceCnt = msg->rs_lidar_result_ptr->freespace_ptr->fs_pts.size();
            if (freespaceCnt > 0) {
                int freespaces_size = RSNativeSerializeUtil::serialize(
                msg->rs_lidar_result_ptr->freespace_ptr,
                en_msg.buffers.data() + offset);
                // std::cout << "freespaces_size = " << freespaces_size << std::endl;
                en_msg.lengths.push_back(freespaces_size);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_FREESPACE);
                offset += freespaces_size;
            } else {
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_FREESPACE_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap.insert(
            std::pair<ROBO_MSG_TYPE, int>(msgType, freespaceCnt));
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_LANE) {
            en_msg.offsets.push_back(offset);
            int lanesCnt = msg->rs_lidar_result_ptr->lanes.size();
            if (lanesCnt > 0) {
                int lanes_size = RSNativeSerializeUtil::serialize(
                msg->rs_lidar_result_ptr->lanes, en_msg.buffers.data() + offset);
                en_msg.lengths.push_back(lanes_size);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_LANE);
                offset += lanes_size;
            } else {
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_LANE_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap.insert(std::pair<ROBO_MSG_TYPE, int>(msgType, lanesCnt));
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_CURB) {
            en_msg.offsets.push_back(offset);
            int curbsCnt = msg->rs_lidar_result_ptr->roadedges.size();
            if (curbsCnt > 0) {
                int curbs_size = RSNativeSerializeUtil::serialize(
                msg->rs_lidar_result_ptr->roadedges,
                en_msg.buffers.data() + offset);
                en_msg.lengths.push_back(curbs_size);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_CURB);
                offset += curbs_size;
            } else {
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_CURB_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap.insert(std::pair<ROBO_MSG_TYPE, int>(msgType, curbsCnt));
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_POSE) {
            en_msg.offsets.push_back(offset);

            int pose_size = RSNativeSerializeUtil::serialize(
            msg->rs_lidar_result_ptr->global_pose_ptr,
            en_msg.buffers.data() + offset);
            en_msg.lengths.push_back(pose_size);
            en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_POSE);
            offset += pose_size;

            en_msg.msgCntMap.insert(std::pair<ROBO_MSG_TYPE, int>(msgType, 1));
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_NON_GD_IDX ||
                   msgType == ROBO_MSG_TYPE::ROBO_MSG_GD_IDX ||
                   msgType == ROBO_MSG_TYPE::ROBO_MSG_BG_IDX) {
            // Use Binary
            if (msgType == ROBO_MSG_TYPE::ROBO_MSG_NON_GD_IDX) {
                serializeIndices(msg->rs_lidar_result_ptr->non_ground_indices, msgType,
                                 isSupplement, en_msg);
            } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_GD_IDX) {
                serializeIndices(msg->rs_lidar_result_ptr->ground_indices, msgType,
                                 isSupplement, en_msg);
            } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_BG_IDX) {
                serializeIndices(msg->rs_lidar_result_ptr->background_indices, msgType,
                                 isSupplement, en_msg);
            }
        } else if (msgType ==
                   ROBO_MSG_TYPE::ROBO_MSG_POINT) {  // isSupplement is the segment
            // include points count

            int totalPcCnt = msg->rs_lidar_result_ptr->scan_ptr->size();
            if (totalPcCnt > 0) {
                // single dgram frame include pointcloud
                int segmentCnt = isSupplement / 16;
                RSEndian<float> float32Endian;

                RS_DATA_ENDIAN_TYPE hostEndian = float32Endian.getHostEndian();

                if (hostEndian != RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
                    for (int i = 0; i < totalPcCnt; i += segmentCnt) {
                        int start = i;                                   // include
                        int end = std::min(i + segmentCnt, totalPcCnt);  // exclude
                        en_msg.offsets.push_back(offset);

                        int pc_size = 0;
                        for (int j = start; j < end; ++j) {
                            char *data = en_msg.buffers.data();
                            const RsPoint pc = msg->rs_lidar_result_ptr->scan_ptr->points[j];
                            float32Endian.toTargetEndianArray(
                            pc.x, data + offset, 4,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            float32Endian.toTargetEndianArray(
                            pc.y, data + offset + 4, 4,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            float32Endian.toTargetEndianArray(
                            pc.z, data + offset + 8, 4,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            float intensity = pc.intensity; 
                            float32Endian.toTargetEndianArray(
                            intensity, data + offset + 12, 4,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            offset += 16;
                            pc_size += 16;
                        }
                        en_msg.lengths.push_back(pc_size);
                        en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_POINT);
                    }
                } else {
                    int float_size = sizeof(float);
                    int x_size = float_size;
                    int xy_size = float_size * 2;
                    int xyz_size = float_size * 3;
                    int xyzi_size = float_size * 4;
                    for (int i = 0; i < totalPcCnt; i += segmentCnt) {
                        en_msg.offsets.push_back(offset);                //
                        int start = i;                                   // include
                        int end = std::min(i + segmentCnt, totalPcCnt);  // exclude

                        int pc_size = 0;  // (end - start) * xyzi_size;
                        char *data = en_msg.buffers.data();
                        for (int j = start; j < end; ++j) {
                            const RsPoint pc = msg->rs_lidar_result_ptr->scan_ptr->points[j];

                            // directly copy memory
                            // memcpy(data + offset, &(pc.x), xyz_size);
                            memcpy(data + offset, &(pc.x), float_size);
                            memcpy(data + offset + x_size, &(pc.y), float_size);
                            memcpy(data + offset + xy_size, &(pc.z), float_size);
                            float intensity = pc.intensity; 
                            memcpy(data + offset + xyz_size, &(intensity), float_size);

                            float x = (*reinterpret_cast<float *>(data + offset));
                            float y = (*reinterpret_cast<float *>(data + offset + float_size));
                            // if (std::abs(y) < 0.05)
                            // {
                            //     std::cout << "pc.x = " << pc.x << ", x = " << x << ", pc.y
                            //     = " << pc.y << ",y = " << y << std::endl;
                            // }
                            offset += xyzi_size;
                            pc_size += xyzi_size;
                        }
                        en_msg.lengths.push_back(pc_size);
                        en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_POINT);
                    }
                }
                en_msg.msgCntMap.insert(
                std::pair<ROBO_MSG_TYPE, int>(msgType, totalPcCnt));
            } else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_POINT_EMPTY);
                en_msg.lengths.push_back(0);
            }
        }
        return 0;
    }

    /**
     * @brief deserialize
     * @param en_msg: Protobuf Encode Message Data
     * @param msg: Robosense Perception Message Data
     * @return
     */
    static int deserialize(const char *data, const int length,
                           const ROBO_MSG_TYPE msgType,
                           st_RoboRecvMessage &recvMsg) {
        // std::cout << __FUNCTION__ << "==> length = " << length << " ==> msgType =
        // " << static_cast<int>(msgType) << std::endl;
        if (msgType == ROBO_MSG_TYPE::ROBO_MSG_OBJECT) {
            Object::Ptr robo_object;
            RSNativeSerializeUtil::deserialize(data, length, robo_object);
            recvMsg.msg->rs_lidar_result_ptr->objects.push_back(robo_object);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT) {
            Object::Ptr robo_object;
            RSNativeSerializeUtil::deserialize(data, length, robo_object);
            recvMsg.msg->rs_lidar_result_ptr->attention_objects.push_back(
            robo_object);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_FREESPACE) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->freespace_ptr);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_LANE) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->lanes);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_CURB) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->roadedges);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_POSE) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->global_pose_ptr);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_NON_GD_IDX ||
                   msgType == ROBO_MSG_TYPE::ROBO_MSG_GD_IDX ||
                   msgType == ROBO_MSG_TYPE::ROBO_MSG_BG_IDX) {
            if (msgType == ROBO_MSG_TYPE::ROBO_MSG_NON_GD_IDX) {
                deserializeIndices(
                data, length, recvMsg.msg->rs_lidar_result_ptr->non_ground_indices);
            } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_GD_IDX) {
                deserializeIndices(data, length,
                                   recvMsg.msg->rs_lidar_result_ptr->ground_indices);
            } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_BG_IDX) {
                deserializeIndices(
                data, length, recvMsg.msg->rs_lidar_result_ptr->background_indices);
            }
        } else if (msgType ==
                   ROBO_MSG_TYPE::ROBO_MSG_POINT) {  // Only Support Binary Now
            RSEndian<float> float32Endian;
            RS_DATA_ENDIAN_TYPE hostEndian = float32Endian.getHostEndian();

            if (hostEndian != RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
                for (int i = 0; i < length; i += 16) {
                    RsPoint pc;
                    float32Endian.toHostEndianValue(
                    pc.x, data + i, 4, RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    float32Endian.toHostEndianValue(
                    pc.y, data + i + 4, 4,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    float32Endian.toHostEndianValue(
                    pc.z, data + i + 8, 4,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    float intensity = 0; 
                    float32Endian.toHostEndianValue(
                    intensity, data + i + 12, 4,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    pc.intensity = static_cast<uint8_t>(intensity); 
                    recvMsg.msg->rs_lidar_result_ptr->scan_ptr->points.emplace_back(pc);
                }
            } else {
                for (int i = 0; i < length; i += 16) {
                    RsPoint pc;
                    pc.x = *reinterpret_cast<const float *>(data + i);
                    pc.y = *reinterpret_cast<const float *>(data + i + 4);
                    pc.z = *reinterpret_cast<const float *>(data + i + 8);
                    pc.intensity =static_cast<uint8_t>(*reinterpret_cast<const float *>(data + i + 12));
                    recvMsg.msg->rs_lidar_result_ptr->scan_ptr->points.emplace_back(pc);
                }
            }
        }
        return 0;
    }

private:
    static int serializeIndices(const std::vector<int> &indices,
                                const ROBO_MSG_TYPE msgType, const int maxMsgSize,
                                RSSerializeBuffer &en_msg) {
        int offset = 0;
        if (!en_msg.offsets.empty()) {
            offset = *(en_msg.offsets.rbegin()) + *(en_msg.lengths.rbegin());
        }

        int totalIndicesCnt = indices.size();

        // single dgram frame include indices count
        int segmentIndicesCnt = maxMsgSize / sizeof(int);

        if (totalIndicesCnt > 0) {
            RSEndian<int> int32Endian;
            RS_DATA_ENDIAN_TYPE hostEndian = int32Endian.getHostEndian();

            if (hostEndian != RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
                for (int i = 0; i < totalIndicesCnt; i += segmentIndicesCnt) {
                    int start = i;
                    int end = std::min(i + segmentIndicesCnt, totalIndicesCnt);

                    en_msg.offsets.push_back(offset);
                    int indices_size = 0;
                    for (int j = start; j < end; ++j) {
                        int32Endian.toTargetEndianArray(
                        indices[j], en_msg.buffers.data() + offset, 4,
                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                        offset += 4;
                        indices_size += 4;
                    }
                    en_msg.lengths.push_back(indices_size);
                    en_msg.types.push_back(msgType);
                }
            } else {
                for (int i = 0; i < totalIndicesCnt; i += segmentIndicesCnt) {
                    en_msg.offsets.push_back(offset);
                    int start = i;
                    int end = std::min(i + segmentIndicesCnt, totalIndicesCnt);

                    // direclty copy memory
                    int indices_size = (end - start) * sizeof(int);
                    memcpy(en_msg.buffers.data() + offset, indices.data() + start,
                           indices_size);
                    offset += indices_size;

                    en_msg.lengths.push_back(indices_size);
                    en_msg.types.push_back(msgType);
                }
            }
        } else {
            en_msg.offsets.push_back(offset);
            en_msg.types.push_back(
            static_cast<ROBO_MSG_TYPE>(static_cast<int>(msgType) + 1));
            en_msg.lengths.push_back(0);
        }
        en_msg.msgCntMap.insert(
        std::pair<ROBO_MSG_TYPE, int>(msgType, totalIndicesCnt));

        return 0;
    }

    static int deserializeIndices(const char *data, const int length,
                                  std::vector<int> &indices) {
        int oldSize = indices.size();
        int newSize = indices.size() + length / 4;
        indices.resize(newSize);

        RSEndian<int> int32Endian;
        RS_DATA_ENDIAN_TYPE hostEndian = int32Endian.getHostEndian();
        if (hostEndian != RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
            for (int i = 0; i < length; i += 4) {
                int j = oldSize + i / 4;
                int32Endian.toHostEndianValue(
                indices[j], data + i, 4,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            }
        } else {  // directly copy data
            memcpy(indices.data() + oldSize, data, length);
        }

        return 0;
    }
};

}  // namespace native_sdk_2_x

namespace native_sdk_3_0 {

class RSNativeSerialize {
public:
    using Ptr = std::shared_ptr<RSNativeSerialize>;
    using ConstPtr = std::shared_ptr<const RSNativeSerialize>;

public:
    explicit RSNativeSerialize(
    const RsCustomNativeBytesSDK3_0MsgParams &commConfig)
    : _defaultTrackerId(
    static_cast<int>(RS_INDEX_TYPE::RS_INDEX_ALG_UNTRACKER)),
      _currentUnTrackerId(
      static_cast<int>(RS_INDEX_TYPE::RS_INDEX_ALG_UNTRACKER)),
      _commConfig(commConfig) {
        _isTranslatePointCloud = commConfig.send_pointcloud;
        _isTranslateSupplement = commConfig.send_object_supplements;
    }

    /**
     * @brief serialize
     * @param msg: Robosense Perception Message Data
     * @param en_msg: Protobuf Encode Message Data
     * @return
     */
    int serialize(const RsPerceptionMsg::Ptr &msg, const ROBO_MSG_TYPE msgType,
                  RSSerializeBuffer &en_msg) {
        int offset = 0;
        if (!en_msg.offsets.empty()) {
            offset = *(en_msg.offsets.rbegin()) + *(en_msg.lengths.rbegin());
        }

        // 分配空间: 并恢复初始Label状态
        int totalPcCnt = static_cast<int>(msg->rs_lidar_result_ptr->scan_ptr->size());
        if (totalPcCnt > 0 && _isTranslatePointCloud == true &&
            static_cast<int>(_pointCloudLabels.size()) != totalPcCnt) {
            _pointCloudLabels.resize(totalPcCnt);
            for (int i = 0; i < totalPcCnt; ++i) {
                _pointCloudLabels[i] = _defaultTrackerId;
            }
        }

        if (msgType == ROBO_MSG_TYPE::ROBO_MSG_OBJECT) {
            int objectCnt = msg->rs_lidar_result_ptr->objects.size();
            if (objectCnt > 0) {
                if (!_isTranslatePointCloud) {
                    for (int i = 0; i < objectCnt; ++i) {
                        en_msg.offsets.push_back(offset);
                        int object_size = RSNativeSerializeUtil::serialize(
                        msg->rs_lidar_result_ptr->objects[i], _isTranslateSupplement,
                        en_msg.buffers.data() + offset);
                        en_msg.lengths.push_back(object_size);
                        en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_OBJECT);
                        offset += object_size;
                    }
                } else {
                    for (int i = 0; i < objectCnt; ++i) {
                        auto &objectPtr = msg->rs_lidar_result_ptr->objects[i];
                        auto tracker_id = objectPtr->core_infos_.tracker_id;
                        if (tracker_id < 0) {
                            objectPtr->core_infos_.tracker_id = _currentUnTrackerId;
                            tracker_id = _currentUnTrackerId;
                            --_currentUnTrackerId;
                        }

                        en_msg.offsets.push_back(offset);
                        int object_size = RSNativeSerializeUtil::serialize(objectPtr, _isTranslateSupplement,
                                                                           en_msg.buffers.data() + offset);
                        en_msg.lengths.push_back(object_size);
                        en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_OBJECT);
                        offset += object_size;

                        // 保存object中的点云索引
                        if (_isTranslateSupplement) {
                            const auto &cloud_indices =
                            objectPtr->supplement_infos_.cloud_indices;
                            int indicesCnt =
                            objectPtr->supplement_infos_.cloud_indices.size();
                            for (int j = 0; j < indicesCnt; ++j) {
                                _pointCloudLabels[cloud_indices[j]] = tracker_id;
                            }
                        }
                    }
                }
            } else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_OBJECT_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap.insert(
            std::pair<ROBO_MSG_TYPE, int>(msgType, objectCnt));
        }
        else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT) {
            int attObjectCnt = msg->rs_lidar_result_ptr->attention_objects.size();
            if (attObjectCnt > 0) {
                if (_isTranslatePointCloud == false) {
                    for (int i = 0; i < attObjectCnt; ++i) {
                        en_msg.offsets.push_back(offset);
                        int object_size = RSNativeSerializeUtil::serialize(
                        msg->rs_lidar_result_ptr->attention_objects[i],
                        _isTranslateSupplement, en_msg.buffers.data() + offset);
                        en_msg.lengths.push_back(object_size);
                        en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT);
                        offset += object_size;
                    }
                } else {
                    // 处理tracker_id
                    for (int i = 0; i < attObjectCnt; ++i) {
                        auto &attObjectPtr = msg->rs_lidar_result_ptr->attention_objects[i];
                        bool isObjectTranslate = false;
                        auto tracker_id = attObjectPtr->core_infos_.tracker_id;
                        if (_isTranslateSupplement == true) {
                            if (_commConfig.send_objects &&
                                _commConfig.send_attention_objects) {
                                int objectCnt = msg->rs_lidar_result_ptr->objects.size();
                                for (int j = 0; j < objectCnt; ++j) {
                                    const auto &objectPtr = msg->rs_lidar_result_ptr->objects[j];
                                    if (objectPtr == attObjectPtr) {
                                        isObjectTranslate = true;
                                        break;
                                    }
                                }
                            }

                            if (tracker_id < 0) {
                                attObjectPtr->core_infos_.tracker_id = _currentUnTrackerId;
                                // tracker_id = _currentUnTrackerId;
                                --_currentUnTrackerId;
                            }
                        }

                        // 序列化
                        en_msg.offsets.push_back(offset);
                        int object_size = RSNativeSerializeUtil::serialize(attObjectPtr, _isTranslateSupplement,
                                                                           en_msg.buffers.data() + offset);
                        en_msg.lengths.push_back(object_size);
                        en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_OBJECT);
                        offset += object_size;

                        if (isObjectTranslate == true) {
                            continue;
                        }

                        // 保存点云索引
                        if (_isTranslateSupplement == true) {
                            const auto &cloud_indices =
                            attObjectPtr->supplement_infos_.cloud_indices;
                            int indicesCnt =
                            attObjectPtr->supplement_infos_.cloud_indices.size();
                            for (int j = 0; j < indicesCnt; ++j) {
                                _pointCloudLabels[cloud_indices[j]] = tracker_id;
                            }
                        }
                    }
                }
            } else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap.insert(
            std::pair<ROBO_MSG_TYPE, int>(msgType, attObjectCnt));
        }
        else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_AXISSTATUS) {
            en_msg.offsets.push_back(offset);
            int lanes_size = RSNativeSerializeUtil::serialize(
            msg->rs_lidar_result_ptr->status, en_msg.buffers.data() + offset);
            en_msg.lengths.push_back(lanes_size);
            en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_AXISSTATUS);
            offset += lanes_size;

            en_msg.msgCntMap.insert(std::pair<ROBO_MSG_TYPE, int>(msgType, 1));
        }
        else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_FREESPACE) {
            en_msg.offsets.push_back(offset);
            int freespaceCnt = msg->rs_lidar_result_ptr->freespace_ptr->fs_pts.size();
            if (freespaceCnt > 0) {
                int freespaces_size = RSNativeSerializeUtil::serialize(
                msg->rs_lidar_result_ptr->freespace_ptr,
                en_msg.buffers.data() + offset);
                // std::cout << "freespaces_size = " << freespaces_size << std::endl;
                en_msg.lengths.push_back(freespaces_size);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_FREESPACE);
                offset += freespaces_size;
            } else {
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_FREESPACE_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap.insert(
            std::pair<ROBO_MSG_TYPE, int>(msgType, freespaceCnt));
        }
        else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_LANE) {
            en_msg.offsets.push_back(offset);
            int lanesCnt = msg->rs_lidar_result_ptr->lanes.size();
            if (lanesCnt > 0) {
                int lanes_size = RSNativeSerializeUtil::serialize(
                msg->rs_lidar_result_ptr->lanes, en_msg.buffers.data() + offset);
                en_msg.lengths.push_back(lanes_size);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_LANE);
                offset += lanes_size;
            } else {
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_LANE_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap.insert(std::pair<ROBO_MSG_TYPE, int>(msgType, lanesCnt));
        }
        else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_CURB) {
            en_msg.offsets.push_back(offset);
            int curbsCnt = msg->rs_lidar_result_ptr->roadedges.size();
            if (curbsCnt > 0) {
                int curbs_size = RSNativeSerializeUtil::serialize(msg->rs_lidar_result_ptr->roadedges,
                en_msg.buffers.data() + offset);
                en_msg.lengths.push_back(curbs_size);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_CURB);
                offset += curbs_size;
            } else {
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_CURB_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap.insert(std::pair<ROBO_MSG_TYPE, int>(msgType, curbsCnt));
        }
        else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_AXISLIDAR_POSE) {
            en_msg.offsets.push_back(offset);

            int pose_size = RSNativeSerializeUtil::serialize(msg->rs_lidar_result_ptr->status_pose_map,
                                                             en_msg.buffers.data() + offset);
            en_msg.lengths.push_back(pose_size);
            en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_AXISLIDAR_POSE);
            offset += pose_size;

            en_msg.msgCntMap.insert(std::pair<ROBO_MSG_TYPE, int>(msgType, 1));
        }
        else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_GLOBALCAR_POSE) {
            en_msg.offsets.push_back(offset);

            AxisStatus status = AxisStatus::GLOBAL_AXIS;
            int pose_size = RSNativeSerializeUtil::serialize(
            status, msg->rs_lidar_result_ptr->global_pose_ptr,
            en_msg.buffers.data() + offset);
            en_msg.lengths.push_back(pose_size);
            en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_GLOBALCAR_POSE);
            offset += pose_size;

            en_msg.msgCntMap.insert(std::pair<ROBO_MSG_TYPE, int>(msgType, 1));
        }
        else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_POINT) {  // isSupplement is the segment
            // include points count
            if (totalPcCnt > 0) {
                // 获取bg/ngd/gd的索引信息
                if (_isTranslatePointCloud == true) {
                    const auto &background_indices =
                    msg->rs_lidar_result_ptr->background_indices;
                    int bgIndexCnt = background_indices.size();
                    const int bgLabel =
                    static_cast<int>(RS_INDEX_TYPE::RS_INDEX_BG_INDEX);
                    for (int j = 0; j < bgIndexCnt; ++j) {
                        int index = background_indices[j];
                        if (index < 0 || index >= totalPcCnt) {
                            continue;
                        }
                        _pointCloudLabels[index] = bgLabel;
                    }

                    const auto &non_ground_indices =
                    msg->rs_lidar_result_ptr->non_ground_indices;
                    int ngIndexCnt = non_ground_indices.size();
                    const int ngLabel =
                    static_cast<int>(RS_INDEX_TYPE::RS_INDEX_NGD_INDEX);
                    for (int j = 0; j < ngIndexCnt; ++j) {
                        int index = non_ground_indices[j];
                        if (index < 0 || index >= totalPcCnt) {
                            continue;
                        }
                        _pointCloudLabels[index] = ngLabel;
                    }

                    const auto &ground_indices = msg->rs_lidar_result_ptr->ground_indices;
                    int gIndexCnt = ground_indices.size();
                    const int gLabel = static_cast<int>(RS_INDEX_TYPE::RS_INDEX_GD_INDEX);
                    for (int j = 0; j < gIndexCnt; ++j) {
                        int index = ground_indices[j];
                        if (index < 0 || index >= totalPcCnt) {
                            continue;
                        }
                        _pointCloudLabels[index] = gLabel;
                    }
                }

                // single dgram frame include pointcloud
                int float_size = sizeof(float);
                int int_size = sizeof(int);
                int x_size = float_size;
                int xy_size = float_size * 2;
                int xyz_size = float_size * 3;
                int xyzi_size = float_size * 4;
                int xyzil_size = xyzi_size + int_size;

                // 计算最多传输点的个数
                size_t segmentCnt =
                (_commConfig.max_msg_size - sizeof(st_RoboMsgHeader)) / xyzil_size;
                RSEndian<float> float32Endian;
                RSEndian<int> int32Endian;

                RS_DATA_ENDIAN_TYPE hostEndian = float32Endian.getHostEndian();

                const auto cloud_ptr = msg->rs_lidar_result_ptr->scan_ptr;
                if (hostEndian != RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
                    for (int i = 0; i < totalPcCnt; i += segmentCnt) {
                        int start = i;                                   // include
                        int end = std::min(static_cast<int>(i + segmentCnt), totalPcCnt);  // exclude
                        en_msg.offsets.push_back(offset);

                        int pc_size = 0;
                        char *data = en_msg.buffers.data();
                        for (int j = start; j < end; ++j) {
                            const auto &point = cloud_ptr->points[j];
                            int label = _pointCloudLabels[j];
                            // std::cout << "Send x, y, z, i , l = " << point.x << ", " <<
                            // point.y << ", " << point.z << ", " << point.intensity << ", "
                            // << _pointCloudLabels[j] << std::endl;
                            float32Endian.toTargetEndianArray(
                            point.x, data + offset, float_size,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            float32Endian.toTargetEndianArray(
                            point.y, data + offset + x_size, float_size,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            float32Endian.toTargetEndianArray(
                            point.z, data + offset + xy_size, float_size,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            float32Endian.toTargetEndianArray(
                            point.intensity, data + offset + xyz_size, float_size,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            int32Endian.toTargetEndianArray(
                            label, data + offset + xyzi_size, int_size,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            offset += xyzil_size;
                            pc_size += xyzil_size;
                        }
                        en_msg.lengths.push_back(pc_size);
                        en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_POINT);
                    }
                } else {
                    for (int i = 0; i < totalPcCnt; i += segmentCnt) {
                        en_msg.offsets.push_back(offset);                //
                        int start = i;                                   // include
                        int end = std::min(static_cast<int>(i + segmentCnt), totalPcCnt);  // exclude

                        int pc_size = 0;  // (end - start) * xyzi_size;
                        char *data = en_msg.buffers.data();
                        for (int j = start; j < end; ++j) {
                            const auto &point = cloud_ptr->points[j];
                            int label = _pointCloudLabels[j];

                            // std::cout << "Send x, y, z, i , l = " << point.x << ", " <<
                            // point.y << ", " << point.z << ", " << point.intensity << ", "
                            // << _pointCloudLabels[j] << std::endl;
                            memcpy(data + offset, &(point.x), xyz_size);
                            memcpy(data + offset + xyz_size, &(point.intensity), int_size);
                            memcpy(data + offset + xyzi_size, &label, int_size);

                            offset += xyzil_size;
                            pc_size += xyzil_size;
                        }
                        en_msg.lengths.push_back(pc_size);
                        en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_POINT);
                    }
                }
                en_msg.msgCntMap.insert(
                std::pair<ROBO_MSG_TYPE, int>(msgType, totalPcCnt));
            } else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(ROBO_MSG_TYPE::ROBO_MSG_POINT_EMPTY);
                en_msg.lengths.push_back(0);
            }

            // 恢复Un-Tracker Object Id
            _currentUnTrackerId =
            static_cast<int>(RS_INDEX_TYPE::RS_INDEX_ALG_UNTRACKER);
        }
        return 0;
    }

    /**
     * @brief deserialize
     * @param en_msg: Protobuf Encode Message Data
     * @param msg: Robosense Perception Message Data
     * @return
     */
    int deserialize(const char *data, const int length,
                    const ROBO_MSG_TYPE msgType, st_RoboRecvMessage &recvMsg) {
        // std::cout << __FUNCTION__ << "==> length = " << length << " ==> msgType =
        // " << static_cast<int>(msgType) << std::endl;
        if (msgType == ROBO_MSG_TYPE::ROBO_MSG_OBJECT) {
            Object::Ptr robo_object;
            RSNativeSerializeUtil::deserialize(data, length, robo_object);
            recvMsg.msg->rs_lidar_result_ptr->objects.push_back(robo_object);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT) {
            Object::Ptr robo_object;
            RSNativeSerializeUtil::deserialize(data, length, robo_object);
            recvMsg.msg->rs_lidar_result_ptr->attention_objects.push_back(
            robo_object);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_AXISSTATUS) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->status);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_FREESPACE) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->freespace_ptr);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_LANE) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->lanes);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_CURB) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->roadedges);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_AXISLIDAR_POSE) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->status_pose_map);
        } else if (msgType == ROBO_MSG_TYPE::ROBO_MSG_GLOBALCAR_POSE) {
            AxisStatus status;
            RSNativeSerializeUtil::deserialize(
            data, length, status,
            recvMsg.msg->rs_lidar_result_ptr->global_pose_ptr);
        } else if (msgType ==
                   ROBO_MSG_TYPE::ROBO_MSG_POINT) {  // Only Support Binary Now
            RSEndian<float> float32Endian;
            RSEndian<int> int32Endian;
            RS_DATA_ENDIAN_TYPE hostEndian = float32Endian.getHostEndian();

            int float_size = sizeof(float);
            int int_size = sizeof(int);
            int x_size = float_size;
            int xy_size = float_size * 2;
            int xyz_size = float_size * 3;
            int xyzi_size = float_size * 4;
            int xyzil_size = xyzi_size + int_size;

            auto cloud_ptr = recvMsg.msg->rs_lidar_result_ptr->scan_ptr;
            auto &valid_indices = recvMsg.msg->rs_lidar_result_ptr->valid_indices;
            const int offsetPointIndex = cloud_ptr->size();
            std::map<int, std::vector<int>> labelMap;

            if (hostEndian != RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
                for (int i = 0; i < length; i += xyzil_size) {
                    RSCommPoint pc;
                    float32Endian.toHostEndianValue(
                    pc.x, data + i, float_size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    float32Endian.toHostEndianValue(
                    pc.y, data + i + x_size, float_size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    float32Endian.toHostEndianValue(
                    pc.z, data + i + xy_size, float_size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN); 
                    float32Endian.toHostEndianValue(
                    pc.intensity, data + i + xyz_size, float_size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    int32Endian.toHostEndianValue(
                    pc.label, data + i + xyzi_size, int_size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

                    if (labelMap.find(pc.label) == labelMap.end()) {
                        labelMap[pc.label] =
                        std::vector<int>{offsetPointIndex + i / xyzil_size};
                    } else {
                        labelMap[pc.label].push_back(offsetPointIndex + i / xyzil_size);
                    }

                    // std::cout << "Receive x, y, z, i , l = " << pc.x << ", " << pc.y <<
                    // ", " << pc.z << ", " << pc.intensity << ", " << pc.label <<
                    // std::endl;
                    RsPoint point;
                    point.x = pc.x;
                    point.y = pc.y;
                    point.z = pc.z;
                    point.intensity = static_cast<uint8_t>(pc.intensity);
                    cloud_ptr->points.emplace_back(point);

                    if (!(std::isnan(pc.x) || std::isnan(pc.y) || std::isnan(pc.z))) {
                        valid_indices.push_back(offsetPointIndex + i / xyzil_size);
                    }
                }
            } else {
                for (int i = 0; i < length; i += xyzil_size) {
                    RSCommPoint pc;
                    pc.x = *reinterpret_cast<const float *>(data + i);
                    pc.y = *reinterpret_cast<const float *>(data + i + x_size);
                    pc.z = *reinterpret_cast<const float *>(data + i + xy_size);
                    pc.intensity = *reinterpret_cast<const float *>(data + i + xyz_size);
                    pc.label = *reinterpret_cast<const int *>(data + i + xyzi_size);

                    if (labelMap.find(pc.label) == labelMap.end()) {
                        labelMap[pc.label] =
                        std::vector<int>{offsetPointIndex + i / xyzil_size};
                    } else {
                        labelMap[pc.label].push_back(offsetPointIndex + i / xyzil_size);
                    }

                    // std::cout << "Receive x, y, z, i , l = " << pc.x << ", " << pc.y <<
                    // ", " << pc.z << ", " << pc.intensity << ", " << pc.label <<
                    // std::endl;

                    RsPoint point;
                    point.x = pc.x;
                    point.y = pc.y;
                    point.z = pc.z;
                    point.intensity = static_cast<uint8_t>(pc.intensity);
                    cloud_ptr->points.emplace_back(point);

                    // 更新valid
                    if (!(std::isnan(pc.x) || std::isnan(pc.y) || std::isnan(pc.z))) {
                        valid_indices.push_back(offsetPointIndex + i / xyzil_size);
                    }
                }
            }

            // 恢复gd/bg/ngd的索引信息
            const int gdLabel = static_cast<int>(RS_INDEX_TYPE::RS_INDEX_GD_INDEX);
            const int bgLabel = static_cast<int>(RS_INDEX_TYPE::RS_INDEX_BG_INDEX);
            const int ngdLabel = static_cast<int>(RS_INDEX_TYPE::RS_INDEX_NGD_INDEX);
            for (auto iterMap = labelMap.cbegin(); iterMap != labelMap.cend();
                 ++iterMap) {
                const auto &label = iterMap->first;
                const auto &indices = iterMap->second;

                if (label == gdLabel) {
                    recvMsg.msg->rs_lidar_result_ptr->ground_indices.insert(
                    recvMsg.msg->rs_lidar_result_ptr->ground_indices.end(),
                    indices.begin(), indices.end());
                } else if (label == bgLabel) {
                    recvMsg.msg->rs_lidar_result_ptr->background_indices.insert(
                    recvMsg.msg->rs_lidar_result_ptr->background_indices.end(),
                    indices.begin(), indices.end());
                } else if (label == ngdLabel) {
                    recvMsg.msg->rs_lidar_result_ptr->non_ground_indices.insert(
                    recvMsg.msg->rs_lidar_result_ptr->non_ground_indices.end(),
                    indices.begin(), indices.end());
                }
            }

            // 恢复Object中的点云索引信息, 注意未恢复tracker_id
            if (_isTranslateSupplement == true) {
                int objectCnt = recvMsg.msg->rs_lidar_result_ptr->objects.size();
                for (int j = 0; j < objectCnt; ++j) {
                    auto &objectPtr = recvMsg.msg->rs_lidar_result_ptr->objects[j];
                    const int label = objectPtr->core_infos_.tracker_id;

                    auto iterMap = labelMap.find(label);

                    if (iterMap == labelMap.end()) {
                        continue;
                    }
                    const auto &indices = iterMap->second;

                    objectPtr->supplement_infos_.cloud_indices.insert(
                    objectPtr->supplement_infos_.cloud_indices.end(), indices.begin(),
                    indices.end());
                }

                int attObjectCnt =
                recvMsg.msg->rs_lidar_result_ptr->attention_objects.size();
                for (int j = 0; j < attObjectCnt; ++j) {
                    auto &objectPtr =
                    recvMsg.msg->rs_lidar_result_ptr->attention_objects[j];
                    const int label = objectPtr->core_infos_.tracker_id;
                    auto iterMap = labelMap.find(label);

                    if (iterMap == labelMap.end()) {
                        continue;
                    }

                    const auto &indices = iterMap->second;
                    objectPtr->supplement_infos_.cloud_indices.insert(
                    objectPtr->supplement_infos_.cloud_indices.end(), indices.begin(),
                    indices.end());
                }
            }
        }
        return 0;
    }

private:
    bool _isTranslatePointCloud;
    bool _isTranslateSupplement;
    std::vector<int> _pointCloudLabels;
    const int _defaultTrackerId;
    int _currentUnTrackerId;
    RsCustomNativeBytesSDK3_0MsgParams _commConfig;
};

}  // namespace native_sdk_3_0


namespace native_sdk_3_1 {

class RSNativeByte3_1Serialize {
public:
    using Ptr = std::shared_ptr<RSNativeByte3_1Serialize>;
    using ConstPtr = std::shared_ptr<const RSNativeByte3_1Serialize>;

public:
    explicit RSNativeByte3_1Serialize(const RsCommonBytesCustomMsgParams &commConfig)
    : default_tracker_id_(static_cast<int>(RS_INDEX_TYPE::RS_INDEX_ALG_UNTRACKER)),
    current_untrack_id_(static_cast<int>(RS_INDEX_TYPE::RS_INDEX_ALG_UNTRACKER)),
    custom_params_(commConfig) {
        translate_point_cloud_ = commConfig.send_point_cloud;
        translate_supplement_ = commConfig.send_object_supplement;
    }

    /**
     * @brief serialize
     * @param msg: Robosense Perception Message Data
     * @param en_msg: Protobuf Encode Message Data
     * @return
     */
    int serialize(const RsPerceptionMsg::Ptr &msg, const ROBO_DATA_TYPE msgType,
                  RSBytes3_1SerializeBuffer &en_msg) {
        int offset = 0;
        if (!en_msg.offsets.empty()) {
            offset = *(en_msg.offsets.rbegin()) + *(en_msg.lengths.rbegin());
        }

        // 分配空间: 并恢复初始Label状态
        // 这个过程只会进行一次
        int totalPcCnt = static_cast<int>(msg->rs_lidar_result_ptr->scan_ptr->size());
        if (totalPcCnt > 0 && translate_point_cloud_ && static_cast<int>(point_cloud_label_.size()) != totalPcCnt) {
            point_cloud_label_.resize(totalPcCnt);
            for (int i = 0; i < totalPcCnt; ++i) {
                point_cloud_label_[i] = default_tracker_id_;
            }
        }
        if (msgType == ROBO_DATA_TYPE::ROBO_MSG_OBJECT) {
            const auto& objects = msg->rs_lidar_result_ptr->objects;
            size_t object_nums = objects.size();
            if (object_nums > 0) {
                // 如果又要发点云又要发supplement信息，那么需要给object的indices写标签。就牵涉到tracker_id为-1的物体要重新赋id。
                if (custom_params_.send_point_cloud && custom_params_.send_object_supplement) {
                    for (const auto& obj: objects) {
                        const auto& cloud_indices = obj->supplement_infos_.cloud_indices;
                        auto& tracker_id = obj->core_infos_.tracker_id;
                        if (tracker_id < 0) {
                            tracker_id = current_untrack_id_;
                            current_untrack_id_--;
                        }

                        for (int index: cloud_indices) {
                            point_cloud_label_[index] = tracker_id;
                        }
                    }
                }

                for (size_t i = 0; i < object_nums; i++) {
                    en_msg.offsets.push_back(offset);
                    int object_size = RSNativeSerializeUtil::serialize(objects[i],translate_supplement_,
                                                                       en_msg.buffers.data() + offset);
                    en_msg.lengths.push_back(object_size);
                    en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_OBJECT);
                    offset += object_size;
                }
            }
            else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_OBJECT_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap[msgType] = static_cast<int>(object_nums);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT) {
            const auto& att_objects = msg->rs_lidar_result_ptr->attention_objects;
            size_t att_objects_nums = att_objects.size();
            if (att_objects_nums > 0) {

                // 由于一度目标的定义关系，被标为attention object的物体可能也出现在了object中。
                // 因此，在开启了序列化supplement info的时候，可能会出现同样的cloud indices又被序列化了一次。这个太占地方了，没必要。
                // 所以要在这个地方判定一下每一个attention object是不是同时也是object。只为那些不是的attention object进行序列化cloud indices。
                if (custom_params_.send_point_cloud && custom_params_.send_object_supplement) {
                    for (const auto& att_obj: att_objects) {
                        bool isObject = false;
                        // 防止关了object只开attention object的骚操作带来的风险。
                        if (custom_params_.send_object) {
                            for (const auto& obj: msg->rs_lidar_result_ptr->objects) {
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
                            att_obj->core_infos_.tracker_id = current_untrack_id_;
                            current_untrack_id_--;
                        }
                        const auto& cloud_indices = att_obj->supplement_infos_.cloud_indices;
                        for (const auto& index: cloud_indices) {
                            point_cloud_label_[index] = att_obj->core_infos_.tracker_id;
                        }
                    }
                }

                for (size_t i = 0; i < att_objects_nums; i++) {
                    en_msg.offsets.push_back(offset);
                    int att_object_size = RSNativeSerializeUtil::serialize(att_objects[i], translate_supplement_,
                    en_msg.buffers.data() + offset);
                    en_msg.lengths.push_back(att_object_size);
                    en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT);
                    offset += att_object_size;
                }
            }
            else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap[msgType] = static_cast<int>(att_objects_nums);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_AXISSTATUS) {
            en_msg.offsets.push_back(offset);
            int status_size = RSNativeSerializeUtil::serialize(
            msg->rs_lidar_result_ptr->status, en_msg.buffers.data() + offset);
            en_msg.lengths.push_back(status_size);
            en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_AXISSTATUS);
            offset += status_size;

            en_msg.msgCntMap[msgType] = 1;
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_FREESPACE) {
            en_msg.offsets.push_back(offset);
            const auto& freespaces = msg->rs_lidar_result_ptr->freespace_ptr;
            size_t freespace_point_nums = freespaces->fs_pts.size();
            if (freespace_point_nums > 0) {
                int freespaces_size = RSNativeSerializeUtil::serialize(freespaces,en_msg.buffers.data() + offset);
                en_msg.lengths.push_back(freespaces_size);
                en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_FREESPACE);
                offset += freespaces_size;
            }
            else {
                en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_FREESPACE_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap[msgType] = static_cast<int>(freespace_point_nums);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_LANE) {
            en_msg.offsets.push_back(offset);
            const auto& lanes = msg->rs_lidar_result_ptr->lanes;
            size_t lane_point_nums = lanes.size();
            if (lane_point_nums > 0) {
                int lanes_size = RSNativeSerializeUtil::serialize(lanes, en_msg.buffers.data() + offset);
                en_msg.lengths.push_back(lanes_size);
                en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_LANE);
                offset += lanes_size;
            }
            else {
                en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_LANE_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap[msgType] = static_cast<int>(lane_point_nums);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_CURB) {
            en_msg.offsets.push_back(offset);
            const auto& roadedges = msg->rs_lidar_result_ptr->roadedges;
            size_t roadedge_point_nums = roadedges.size();
            if (roadedge_point_nums > 0) {
                int roadedges_size = RSNativeSerializeUtil::serialize(roadedges, en_msg.buffers.data() + offset);
                en_msg.lengths.push_back(roadedges_size);
                en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_CURB);
                offset += roadedges_size;
            }
            else {
                en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_CURB_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap[msgType] = static_cast<int>(roadedge_point_nums);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_AXISLIDAR_POSE) {
            en_msg.offsets.push_back(offset);

            int pose_size = RSNativeSerializeUtil::serialize(
            msg->rs_lidar_result_ptr->status_pose_map,en_msg.buffers.data() + offset);
            en_msg.lengths.push_back(pose_size);
            en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_AXISLIDAR_POSE);
            offset += pose_size;

            en_msg.msgCntMap[msgType] = 1;
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_GLOBALCAR_POSE) {
            en_msg.offsets.push_back(offset);
            AxisStatus status = AxisStatus::GLOBAL_AXIS;
            int pose_size = RSNativeSerializeUtil::serialize(
            status, msg->rs_lidar_result_ptr->global_pose_ptr,
            en_msg.buffers.data() + offset);
            en_msg.lengths.push_back(pose_size);
            en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_GLOBALCAR_POSE);
            offset += pose_size;

            en_msg.msgCntMap[msgType] = 1;
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_POINT) {  // isSupplement is the segment
            // include points count
            if (totalPcCnt > 0) {
                // single dgram frame include pointcloud
                int float_size = sizeof(float);
                int int_size = sizeof(int);
                int x_size = float_size;
                int xy_size = float_size * 2;
                int xyz_size = float_size * 3;
                int xyzi_size = float_size * 4;
                int xyzil_size = xyzi_size + int_size;

                // 计算最多传输点的个数
                int segmentCnt =
                static_cast<int>((custom_params_.max_msg_size - sizeof(st_RoboMsgHeader)) / xyzil_size);
                RSEndian<float> float32Endian;
                RSEndian<int> int32Endian;

                RS_DATA_ENDIAN_TYPE hostEndian = float32Endian.getHostEndian();

                const auto cloud_ptr = msg->rs_lidar_result_ptr->scan_ptr;
                if (hostEndian != RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
                    for (int i = 0; i < totalPcCnt; i += segmentCnt) {
                        int start = i;                                   // include
                        int end = std::min(static_cast<int>(i + segmentCnt), totalPcCnt);  // exclude
                        en_msg.offsets.push_back(offset);

                        int pc_size = 0;
                        char *data = en_msg.buffers.data();
                        for (int j = start; j < end; ++j) {
                            const auto &point = cloud_ptr->points[j];
                            int label = point_cloud_label_[j];
                            float32Endian.toTargetEndianArray(
                            point.x, data + offset, float_size,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            float32Endian.toTargetEndianArray(
                            point.y, data + offset + x_size, float_size,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            float32Endian.toTargetEndianArray(
                            point.z, data + offset + xy_size, float_size,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            float32Endian.toTargetEndianArray(
                            point.intensity, data + offset + xyz_size, float_size,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            int32Endian.toTargetEndianArray(
                            label, data + offset + xyzi_size, int_size,
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                            offset += xyzil_size;
                            pc_size += xyzil_size;
                        }
                        en_msg.lengths.push_back(pc_size);
                        en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_POINT);
                    }
                }
                else {
                    for (int i = 0; i < totalPcCnt; i += segmentCnt) {
                        en_msg.offsets.push_back(offset);                //
                        int start = i;                                   // include
                        int end = std::min(static_cast<int>(i + segmentCnt), totalPcCnt);  // exclude

                        int pc_size = 0;  // (end - start) * xyzi_size;
                        char *data = en_msg.buffers.data();
                        for (int j = start; j < end; ++j) {
                            const auto &point = cloud_ptr->points[j];
                            int label = point_cloud_label_[j];
                            memcpy(data + offset, &(point.x), xyz_size);
                            memcpy(data + offset + xyz_size, &(point.intensity), int_size);
                            memcpy(data + offset + xyzi_size, &label, int_size);

                            offset += xyzil_size;
                            pc_size += xyzil_size;
                        }
                        en_msg.lengths.push_back(pc_size);
                        en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_POINT);
                    }
                }
            }
            else {
                en_msg.offsets.push_back(offset);
                en_msg.types.push_back(ROBO_DATA_TYPE::ROBO_MSG_POINT_EMPTY);
                en_msg.lengths.push_back(0);
            }
            en_msg.msgCntMap[msgType] = totalPcCnt;
            // 恢复Un-Tracker Object Id
            current_untrack_id_ = static_cast<int>(RS_INDEX_TYPE::RS_INDEX_ALG_UNTRACKER);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_VALID_INDICES) {
            const auto& valid_indices = msg->rs_lidar_result_ptr->valid_indices;
            int maxMsgSize = static_cast<int>(custom_params_.max_msg_size - sizeof(st_RoboMsgHeader));
            RSNativeSerializeUtil::serialize(valid_indices, msgType, maxMsgSize, en_msg);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_BG_IDX) {
            const auto& bg_indices = msg->rs_lidar_result_ptr->background_indices;
            int maxMsgSize = static_cast<int>(custom_params_.max_msg_size - sizeof(st_RoboMsgHeader));
            RSNativeSerializeUtil::serialize(bg_indices, msgType, maxMsgSize, en_msg);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_GD_IDX) {
            const auto& gd_indices = msg->rs_lidar_result_ptr->ground_indices;
            int maxMsgSize = static_cast<int>(custom_params_.max_msg_size - sizeof(st_RoboMsgHeader));
            RSNativeSerializeUtil::serialize(gd_indices, msgType, maxMsgSize, en_msg);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_NON_GD_IDX) {
            const auto& non_gd_indices = msg->rs_lidar_result_ptr->non_ground_indices;
            int maxMsgSize = static_cast<int>(custom_params_.max_msg_size - sizeof(st_RoboMsgHeader));
            RSNativeSerializeUtil::serialize(non_gd_indices, msgType, maxMsgSize, en_msg);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_TIMESTAMP) {
            en_msg.offsets.push_back(offset);
            const auto& timestamp = msg->rs_lidar_result_ptr->timestamp;

            RSEndian<double> doubleEndian;
            int doubleSize = sizeof(double);
            bool isDstEndianTypeMatchHostEndian =
            (doubleEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            if (isDstEndianTypeMatchHostEndian) {
                memcpy(en_msg.buffers.data() + offset, &timestamp, doubleSize);
                offset += doubleSize;
            }
            else {
                doubleEndian.toTargetEndianArray(timestamp, en_msg.buffers.data() + offset,
                                                 doubleSize, RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += doubleSize;
            }
            en_msg.lengths.push_back(doubleSize);
            en_msg.types.push_back(msgType);
            en_msg.msgCntMap[msgType] = 1;
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_GPS_ORIGIN) {
            en_msg.offsets.push_back(offset);
            const auto& gps_origin = msg->rs_lidar_result_ptr->gps_origin;

            RSEndian<double> doubleEndian;
            int doubleSize = sizeof(double);
            int len = 0;
            bool isDstEndianTypeMatchHostEndian =
            (doubleEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            if (isDstEndianTypeMatchHostEndian) {
                memcpy(en_msg.buffers.data() + offset, &(gps_origin.x), doubleSize);
                len += doubleSize;
                offset += doubleSize;

                memcpy(en_msg.buffers.data() + offset, &(gps_origin.y), doubleSize);
                len += doubleSize;
                offset += doubleSize;

                memcpy(en_msg.buffers.data() + offset, &(gps_origin.z), doubleSize);
                len += doubleSize;
                offset += doubleSize;
            }
            else {
                doubleEndian.toTargetEndianArray(gps_origin.x, en_msg.buffers.data() + offset,
                                                 doubleSize,RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                len += doubleSize;
                offset += doubleSize;

                doubleEndian.toTargetEndianArray(gps_origin.y, en_msg.buffers.data() + offset,
                                                 doubleSize,RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                len += doubleSize;
                offset += doubleSize;

                doubleEndian.toTargetEndianArray(gps_origin.z, en_msg.buffers.data() + offset,
                                                 doubleSize,RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                len += doubleSize;
                offset += doubleSize;
            }
            en_msg.lengths.push_back(len);
            en_msg.types.push_back(msgType);
            en_msg.msgCntMap[msgType] = 1;
        }
        return 0;
    }

    /**
     * @brief deserialize
     * @param en_msg: Protobuf Encode Message Data
     * @param msg: Robosense Perception Message Data
     * @return
     */
    int deserialize(const char *data, const int length,
                    const ROBO_DATA_TYPE msgType, st_RoboRecvMessage &recvMsg) {
        // std::cout << __FUNCTION__ << "==> length = " << length << " ==> msgType =
        // " << static_cast<int>(msgType) << std::endl;
        if (msgType == ROBO_DATA_TYPE::ROBO_MSG_OBJECT) {
            Object::Ptr robo_object;
            RSNativeSerializeUtil::deserialize(data, length, robo_object);
            recvMsg.msg->rs_lidar_result_ptr->objects.push_back(robo_object);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT) {
            Object::Ptr robo_object;
            RSNativeSerializeUtil::deserialize(data, length, robo_object);
            recvMsg.msg->rs_lidar_result_ptr->attention_objects.push_back(
            robo_object);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_AXISSTATUS) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->status);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_FREESPACE) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->freespace_ptr);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_LANE) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->lanes);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_CURB) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->roadedges);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_AXISLIDAR_POSE) {
            RSNativeSerializeUtil::deserialize(
            data, length, recvMsg.msg->rs_lidar_result_ptr->status_pose_map);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_GLOBALCAR_POSE) {
            AxisStatus status;
            RSNativeSerializeUtil::deserialize(
            data, length, status,
            recvMsg.msg->rs_lidar_result_ptr->global_pose_ptr);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_POINT) {  // Only Support Binary Now
            RSEndian<float> float32Endian;
            RSEndian<int> int32Endian;
            RS_DATA_ENDIAN_TYPE hostEndian = float32Endian.getHostEndian();

            int float_size = sizeof(float);
            int int_size = sizeof(int);
            int x_size = float_size;
            int xy_size = float_size * 2;
            int xyz_size = float_size * 3;
            int xyzi_size = float_size * 4;
            int xyzil_size = xyzi_size + int_size;

            auto cloud_ptr = recvMsg.msg->rs_lidar_result_ptr->scan_ptr;
            const int offsetPointIndex = cloud_ptr->size();
            std::map<int, std::vector<int>> labelMap;

            if (hostEndian != RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
                for (int i = 0; i < length; i += xyzil_size) {
                    RSCommPoint pc;
                    float32Endian.toHostEndianValue(
                    pc.x, data + i, float_size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    float32Endian.toHostEndianValue(
                    pc.y, data + i + x_size, float_size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    float32Endian.toHostEndianValue(
                    pc.z, data + i + xy_size, float_size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    float32Endian.toHostEndianValue(
                    pc.intensity, data + i + xyz_size, float_size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    int32Endian.toHostEndianValue(
                    pc.label, data + i + xyzi_size, int_size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

                    if (labelMap.find(pc.label) == labelMap.end()) {
                        labelMap[pc.label] = std::vector<int>{offsetPointIndex + i / xyzil_size};
                    }
                    else {
                        labelMap[pc.label].push_back(offsetPointIndex + i / xyzil_size);
                    }

                    // std::cout << "Receive x, y, z, i , l = " << pc.x << ", " << pc.y <<
                    // ", " << pc.z << ", " << pc.intensity << ", " << pc.label <<
                    // std::endl;
                    RsPoint point;
                    point.x = pc.x;
                    point.y = pc.y;
                    point.z = pc.z;
                    point.intensity = static_cast<uint8_t>(pc.intensity);
                    cloud_ptr->points.emplace_back(point);
                }
            }
            else {
                for (int i = 0; i < length; i += xyzil_size) {
                    RSCommPoint pc;
                    pc.x = *reinterpret_cast<const float *>(data + i);
                    pc.y = *reinterpret_cast<const float *>(data + i + x_size);
                    pc.z = *reinterpret_cast<const float *>(data + i + xy_size);
                    pc.intensity = *reinterpret_cast<const float *>(data + i + xyz_size);
                    pc.label = *reinterpret_cast<const int *>(data + i + xyzi_size);

                    if (labelMap.find(pc.label) == labelMap.end()) {
                        labelMap[pc.label] =
                        std::vector<int>{offsetPointIndex + i / xyzil_size};
                    }
                    else {
                        labelMap[pc.label].push_back(offsetPointIndex + i / xyzil_size);
                    }

                    // std::cout << "Receive x, y, z, i , l = " << pc.x << ", " << pc.y <<
                    // ", " << pc.z << ", " << pc.intensity << ", " << pc.label <<
                    // std::endl;

                    RsPoint point;
                    point.x = pc.x;
                    point.y = pc.y;
                    point.z = pc.z;
                    point.intensity = static_cast<uint8_t>(pc.intensity);
                    cloud_ptr->points.emplace_back(point);
                }
            }

            // 恢复Object中的点云索引信息, 注意未恢复tracker_id
            if (translate_supplement_) {
                int objectCnt = recvMsg.msg->rs_lidar_result_ptr->objects.size();
                for (int j = 0; j < objectCnt; ++j) {
                    auto &objectPtr = recvMsg.msg->rs_lidar_result_ptr->objects[j];
                    const int label = objectPtr->core_infos_.tracker_id;

                    auto iterMap = labelMap.find(label);

                    if (iterMap == labelMap.end()) {
                        continue;
                    }
                    const auto &indices = iterMap->second;

                    objectPtr->supplement_infos_.cloud_indices.insert(
                    objectPtr->supplement_infos_.cloud_indices.end(), indices.begin(),
                    indices.end());
                }

                int attObjectCnt =
                recvMsg.msg->rs_lidar_result_ptr->attention_objects.size();
                for (int j = 0; j < attObjectCnt; ++j) {
                    auto &objectPtr =
                    recvMsg.msg->rs_lidar_result_ptr->attention_objects[j];
                    const int label = objectPtr->core_infos_.tracker_id;
                    auto iterMap = labelMap.find(label);

                    if (iterMap == labelMap.end()) {
                        continue;
                    }

                    const auto &indices = iterMap->second;
                    objectPtr->supplement_infos_.cloud_indices.insert(
                    objectPtr->supplement_infos_.cloud_indices.end(), indices.begin(),
                    indices.end());
                }
            }
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_VALID_INDICES) {
            RSNativeSerializeUtil::deserialize(data, length,recvMsg.msg->rs_lidar_result_ptr->valid_indices);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_BG_IDX) {
            RSNativeSerializeUtil::deserialize(data, length,recvMsg.msg->rs_lidar_result_ptr->background_indices);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_GD_IDX) {
            RSNativeSerializeUtil::deserialize(data, length,recvMsg.msg->rs_lidar_result_ptr->ground_indices);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_NON_GD_IDX) {
            RSNativeSerializeUtil::deserialize(data, length,recvMsg.msg->rs_lidar_result_ptr->non_ground_indices);
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_TIMESTAMP) {
            RSEndian<double> doubleEndian;
            bool isSrcEndianTypeMatchHostEndian =
            (doubleEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            int doubleSize = sizeof(double);

            int offset = 0;
            if (isSrcEndianTypeMatchHostEndian) {
                memcpy(&(recvMsg.msg->rs_lidar_result_ptr->timestamp), data + offset, doubleSize);
            }
            else {
                doubleEndian.toHostEndianValue(recvMsg.msg->rs_lidar_result_ptr->timestamp,
                                               data + offset, doubleSize,
                                               RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            }
        }
        else if (msgType == ROBO_DATA_TYPE::ROBO_MSG_GPS_ORIGIN) {
            RSEndian<double> doubleEndian;
            bool isSrcEndianTypeMatchHostEndian =
            (doubleEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            int doubleSize = sizeof(double);

            int offset = 0;
            if (isSrcEndianTypeMatchHostEndian) {
                memcpy(&(recvMsg.msg->rs_lidar_result_ptr->gps_origin.x), data + offset, doubleSize);
                offset += doubleSize;

                memcpy(&(recvMsg.msg->rs_lidar_result_ptr->gps_origin.y), data + offset, doubleSize);
                offset += doubleSize;

                memcpy(&(recvMsg.msg->rs_lidar_result_ptr->gps_origin.z), data + offset, doubleSize);
                offset += doubleSize;
            }
            else {
                doubleEndian.toHostEndianValue(recvMsg.msg->rs_lidar_result_ptr->gps_origin.x, data + offset,
                                               doubleSize, RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += doubleSize;

                doubleEndian.toHostEndianValue(recvMsg.msg->rs_lidar_result_ptr->gps_origin.y, data + offset,
                                               doubleSize, RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += doubleSize;

                doubleEndian.toHostEndianValue(recvMsg.msg->rs_lidar_result_ptr->gps_origin.z, data + offset,
                                               doubleSize, RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += doubleSize;
            }
        }
        return 0;
    }

private:
    bool translate_point_cloud_;
    bool translate_supplement_;
    std::vector<int> point_cloud_label_;
    const int default_tracker_id_;
    int current_untrack_id_;
    RsCommonBytesCustomMsgParams custom_params_;
};

}

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_CUSTOM_TRANSFORMATER_H_
