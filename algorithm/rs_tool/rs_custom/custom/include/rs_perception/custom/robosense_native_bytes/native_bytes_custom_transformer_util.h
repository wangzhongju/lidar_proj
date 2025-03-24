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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_CUSTOM_TRANSFORMATER_UTIL_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_CUSTOM_TRANSFORMATER_UTIL_H_
#include "rs_perception/communication/external/common/basic_type.h"
#include "rs_perception/custom/common/base_custom_params.h"

namespace robosense {
namespace perception {

class RsNativeCommon {
public:
    static void fromNativeByteToEigen(RsVector3f &eig, const char *data, int &len,
                                      const bool isSrcEndianTypeMatchHostEndian) {
        int floatSize = sizeof(float);
        len = 3 * floatSize;
        if (isSrcEndianTypeMatchHostEndian) {
            memcpy(&(eig.x), data, len);
        } else {
            RSEndian<float> floatEndian;
            floatEndian.toHostEndianValue(eig.x, data, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            floatEndian.toHostEndianValue(eig.y, data + floatSize, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            floatEndian.toHostEndianValue(eig.z, data + 2 * floatSize, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        }
    }

    static void fromEigenToNativeByte(const RsVector3f &eig, char *data, int &len,
                                      const bool isDstEndianTypeMatchHostEndian) {
        int floatSize = sizeof(float);
        len = floatSize * 3;
        if (isDstEndianTypeMatchHostEndian) {
            memcpy(data, &(eig.x), len);
        } else {
            RSEndian<float> floatEndian;
            floatEndian.toTargetEndianArray(
            eig.x, data, floatSize, RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            floatEndian.toTargetEndianArray(
            eig.y, data + floatSize, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            floatEndian.toTargetEndianArray(
            eig.z, data + 2 * floatSize, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        }
    }

    static void fromNativeByteToEigen(RsMatrix3f &eig, const char *data, int &len,
                                      const bool isSrcEndianTypeMatchHostEndian) {
        int floatSize = sizeof(float);
        len = floatSize * 9;
        if (isSrcEndianTypeMatchHostEndian) {
            memcpy(eig.val, data, len);
        } else {
            RSEndian<float> floatEndian;
            for (size_t i = 0; i < 3; ++i) {
                for (size_t j = 0; j < 3; ++j) {
                    floatEndian.toHostEndianValue(
                    eig.val[i][j], data + j * floatSize + i * 3 * floatSize,
                    floatSize, RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                }
            }
        }
    }

    static void fromEigenToNativeByte(const RsMatrix3f &eig, char *data, int &len,
                                      const bool isDstEndianTypeMatchHostEndian) {
        int floatSize = sizeof(float);
        len = floatSize * 9;
        if (isDstEndianTypeMatchHostEndian) {
            for (int i = 0; i < 9; ++i) {
                memcpy(data, eig.val, len);
            }
        } else {
            RSEndian<float> floatEndian;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    floatEndian.toTargetEndianArray(
                    eig.val[i][j], data + j * floatSize + i * 3 * floatSize,
                    floatSize, RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                }
            }
        }
    }

    template<typename T>
    static int fillTemplateElements(const T value, int count, char *data,
                                    const size_t maxdataLen,
                                    RS_DATA_ENDIAN_TYPE dstType) {
        size_t tSize = sizeof(T);
        size_t tCntSize = tSize * count;

        if (maxdataLen < tCntSize) {
            return -1;
        }

        RSEndian<T> tEndian;
        for (int i = 0; i < count; ++i) {
            tEndian.toTargetEndianArray(value, data + i * tSize, tSize, dstType);
        }

        return tCntSize;
    }
};

namespace native_sdk_2_x {

/**
 * @brief The st_RoboRecvMessage struct
 *
 * status: Index Message Receive Status
 * recorder: Recorder UDP Translate Log
 * device_id: device id
 * timestamp: message timestamp
 * msg: Receive Message
 *
 */
struct st_RoboRecvMessage {
    int device_id;
    double timestamp;
    std::map<ROBO_MSG_TYPE, st_RoboMsgRecorder> recorder;
    std::map<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS> status;
    RsPerceptionMsg::Ptr msg;

    st_RoboRecvMessage() {
        device_id = 0;
        timestamp = 0;
    }

    void updateStatusByConfig(
    const RsCustomNativeBytesSDK2_XMsgParams &translateConfig) {
        status.clear();

        // Message Receive Recorder
        st_RoboMsgRecorder msgRecorder;

        // Object Message Info
        if (translateConfig.send_objects == true) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_OBJECT, ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_OBJECT,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_OBJECT, msgRecorder));

        // Attention Object Message Info
        if (translateConfig.send_attention_objects == true) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT, msgRecorder));

        // Freespace Message Info
        if (translateConfig.send_freespace == true) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_FREESPACE,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_FREESPACE,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_FREESPACE, msgRecorder));

        // Lane Message Info
        if (translateConfig.send_lanes == true) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_LANE, ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_LANE,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_LANE, msgRecorder));

        // Curb Message Info
        if (translateConfig.send_curbs == true) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_CURB, ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_CURB,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_CURB, msgRecorder));

        // PointCloud Message Info
        if (translateConfig.send_pointcloud == true) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_POINT, ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_POINT,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_POINT, msgRecorder));

        // Non-Ground Indices Message Info
        if (translateConfig.send_non_ground_indices == true) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_NON_GD_IDX,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_NON_GD_IDX,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_NON_GD_IDX, msgRecorder));

        // Ground Indices Message Info
        if (translateConfig.send_ground_indices == true) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_GD_IDX, ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_GD_IDX,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_GD_IDX, msgRecorder));

        // Background Indices Message Info
        if (translateConfig.send_background_indices == true) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_BG_IDX, ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_BG_IDX,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_BG_IDX, msgRecorder));

        // Pose Message Info
        if (translateConfig.send_pose == true) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_POSE, ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_POSE,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_POSE, msgRecorder));
    }
};

class RSNativeSerializeUtil {
public:
    using Ptr = std::shared_ptr<RSNativeSerializeUtil>;
    using ConstPtr = std::shared_ptr<const RSNativeSerializeUtil>;

public:
    static int serialize(const Object::Ptr &object, bool isSupplement,
                         char *data) {
        const CoreInfos &robo_core = object->core_infos_;
        SupplementInfos &robo_supplement = object->supplement_infos_;

        RSEndian<float> floatEndian;
        RSEndian<int> int32Endian;
        RSEndian<unsigned int> uint32Endian;
        RSEndian<double> doubleEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int int32Size = sizeof(int);
        int uint32Size = sizeof(unsigned int);
        int floatSize = sizeof(float);
        int doubleSize = sizeof(double);

        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            // timestamp
            memcpy(data + offset, &(robo_core.timestamp), doubleSize);
            offset += doubleSize;

            // priority_id
            memcpy(data + offset, &(robo_core.priority_id), int32Size);
            offset += int32Size;

            // SDK 2.x
            // existence_confidence
            // memcpy(data + offset, &(robo_core.existence_confidence), floatSize);
            // offset += floatSize;
            // exist_confidence
            // SDK 3.x
            memcpy(data + offset, &(robo_core.exist_confidence), floatSize);
            offset += floatSize;
        } else {
            // timestamp
            doubleEndian.toTargetEndianArray(
            robo_core.timestamp, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            // priority_id
            int32Endian.toTargetEndianArray(
            robo_core.priority_id, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // SDK 2.x
            // existence_confidence
            //   floatEndian.toTargetEndianArray(
            //       robo_core.existence_confidence, data + offset, floatSize,
            //       RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            //   offset += floatSize;
            // SDK 3.x
            floatEndian.toTargetEndianArray(
            robo_core.exist_confidence, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }

        // center
        int len = 0;
        RsNativeCommon::fromEigenToNativeByte(robo_core.center, data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // SDK 2.x
        // center_cov
        // RsNativeCommon::fromEigenToNativeByte(robo_core.center_cov, data +
        // offset,
        //                                       len,
        //                                       isDstEndianTypeMatchHostEndian);
        // offset += len;

        // SDK 3.x
        RsNativeCommon::fromEigenToNativeByte(robo_core.center_cov, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // 填充: center_cov
        float fill_center_cov = 0;
        len = RsNativeCommon::fillTemplateElements<float>(
        fill_center_cov, 6, data + offset, 6 * floatSize,
        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += len;

        // size
        RsNativeCommon::fromEigenToNativeByte(robo_core.size, data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // SDK 2.x
        // // size_cov
        // RsNativeCommon::fromEigenToNativeByte(robo_core.size_cov, data + offset,
        //                                       len,
        //                                       isDstEndianTypeMatchHostEndian);
        // offset += len;
        // SDK 3.x
        // size_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.size_cov, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // 填充: center_cov
        float fill_size_cov = 0;
        len = RsNativeCommon::fillTemplateElements<float>(
        fill_size_cov, 6, data + offset, 6 * floatSize,
        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += len;

        // direction
        RsNativeCommon::fromEigenToNativeByte(robo_core.direction, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // SDK 3.x
        // direction_cov
        // RsNativeCommon::fromEigenToNativeByte(robo_core.direction_cov,
        //                                       data + offset, len,
        //                                       isDstEndianTypeMatchHostEndian);
        // offset += len;

        // SDK 2.x
        RsNativeCommon::fromEigenToNativeByte(robo_core.direction_cov,
                                              data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // 填充: direction_cov
        float fill_direction_cov = 0;
        len = RsNativeCommon::fillTemplateElements<float>(
        fill_direction_cov, 6, data + offset, 6 * floatSize,
        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += len;

        if (isDstEndianTypeMatchHostEndian) {
            // object type
            int objectType = static_cast<int>(robo_core.type);
            memcpy(data + offset, &objectType, int32Size);
            offset += int32Size;

            // object type confidence
            memcpy(data + offset, &(robo_core.type_confidence), floatSize);
            offset += floatSize;

            // attentions_type
            int attType = static_cast<int>(robo_core.attention_type);
            memcpy(data + offset, &attType, int32Size);
            offset += int32Size;

            // motion_state
            int motionState = static_cast<int>(robo_core.motion_state);
            memcpy(data + offset, &motionState, int32Size);
            offset += int32Size;

            // tracker_id
            memcpy(data + offset, &(robo_core.tracker_id), int32Size);
            offset += int32Size;

            // age
            memcpy(data + offset, &(robo_core.age), doubleSize);
            offset += doubleSize;
        } else {
            // object type
            int objectType = static_cast<int>(robo_core.type);
            int32Endian.toTargetEndianArray(
            objectType, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // object type confidence
            floatEndian.toTargetEndianArray(
            robo_core.type_confidence, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // attentions_type
            int attType = static_cast<int>(robo_core.attention_type);
            int32Endian.toTargetEndianArray(
            attType, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // motion_state
            int motionState = static_cast<int>(robo_core.motion_state);
            int32Endian.toTargetEndianArray(
            motionState, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // tracker_id
            int32Endian.toTargetEndianArray(
            robo_core.tracker_id, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // age
            doubleEndian.toTargetEndianArray(
            robo_core.age, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }

        // velocity
        RsNativeCommon::fromEigenToNativeByte(robo_core.velocity, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // SDK 2.x
        // velocity_cov
        // RsNativeCommon::fromEigenToNativeByte(robo_core.velocity_cov, data +
        // offset,
        //                                       len,
        //                                       isDstEndianTypeMatchHostEndian);
        // offset += len;

        // SDK 3.x
        RsNativeCommon::fromEigenToNativeByte(robo_core.velocity_cov, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // 填充: velocity_cov
        float fill_velocity_cov = 0;
        len = RsNativeCommon::fillTemplateElements<float>(
        fill_velocity_cov, 6, data + offset, 6 * floatSize,
        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += len;

        // acceleration
        RsNativeCommon::fromEigenToNativeByte(robo_core.acceleration, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // SDK 2.x
        // // acceleration_cov
        // RsNativeCommon::fromEigenToNativeByte(robo_core.acceleration_cov,
        //                                       data + offset, len,
        //                                       isDstEndianTypeMatchHostEndian);
        // offset += len;

        // SDK 3.x
        // acceleration_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.acceleration_cov,
                                              data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // 填充: acceleration_cov
        float fill_acceleration_cov = 0;
        len = RsNativeCommon::fillTemplateElements<float>(
        fill_acceleration_cov, 6, data + offset, 6 * floatSize,
        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += len;

        // SDK 2.x
        // angle_velocity
        // RsNativeCommon::fromEigenToNativeByte(robo_core.angle_velocity,
        //                                       data + offset, len,
        //                                       isDstEndianTypeMatchHostEndian);
        // offset += len;
        // SDK 3.x
        floatEndian.toTargetEndianArray(robo_core.angle_velocity, data + offset,
                                        floatSize,
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += floatSize;

        // 填充: 2 * float
        float fill_angle_velocity = 0;
        len = RsNativeCommon::fillTemplateElements<float>(
        fill_angle_velocity, 2, data + offset, 2 * floatSize,
        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += len;

        // anchor
        RsNativeCommon::fromEigenToNativeByte(robo_core.anchor, data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // nearest_point
        RsNativeCommon::fromEigenToNativeByte(robo_core.nearest_point,
                                              data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // SDK 2.x
        // // left_point
        // RsNativeCommon::fromEigenToNativeByte(robo_core.left_point, data +
        // offset,
        //                                       len,
        //                                       isDstEndianTypeMatchHostEndian);
        // offset += len;

        // // right_point
        // RsNativeCommon::fromEigenToNativeByte(robo_core.right_point, data +
        // offset,
        //                                       len,
        //                                       isDstEndianTypeMatchHostEndian);
        // offset += len;

        // SDK 3.x
        const RsVector3f left_point =
        robo_supplement.polygon[robo_supplement.left_point_index];
        const RsVector3f right_point =
        robo_supplement.polygon[robo_supplement.right_point_index];

        RsNativeCommon::fromEigenToNativeByte(left_point, data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        RsNativeCommon::fromEigenToNativeByte(right_point, data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        memcpy(data + offset, &isSupplement, sizeof(bool));
        offset += sizeof(bool);

        // Protobuf Serialize Supplement
        if (isSupplement == true) {
            // std::cout << "isSupplement = " << isSupplement << std::endl;

            // polygon
            int polygonSize = robo_supplement.polygon.size();
            int32Endian.toTargetEndianArray(
            polygonSize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            for (int i = 0; i < polygonSize; ++i) {
                RsNativeCommon::fromEigenToNativeByte(robo_supplement.polygon[i],
                                                      data + offset, len,
                                                      isDstEndianTypeMatchHostEndian);
                offset += len;
            }

            // SDK 2.x
            //   // reference
            //   int referenceSize = robo_supplement.reference.size();
            //   int32Endian.toTargetEndianArray(
            //       referenceSize, data + offset, int32Size,
            //       RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            //   offset += int32Size;

            //   for (int i = 0; i < referenceSize; ++i) {
            //     RsNativeCommon::fromEigenToNativeByte(robo_supplement.reference[i],
            //                                           data + offset, len,
            //                                           isDstEndianTypeMatchHostEndian);
            //     offset += len;
            //   }

            // SDK 3.x
            int referenceSize = 0;
            int32Endian.toTargetEndianArray(
            referenceSize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // latent_types
            int latentTypesSize = robo_supplement.latent_types.size();
            int32Endian.toTargetEndianArray(
            latentTypesSize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            for (int i = 0; i < latentTypesSize; ++i) {
                floatEndian.toTargetEndianArray(
                robo_supplement.latent_types[i], data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
            }

            // size_type
            int sizeType = static_cast<int>(robo_supplement.size_type);
            int32Endian.toTargetEndianArray(
            sizeType, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // in_roi
            memcpy(data + offset, &(robo_supplement.in_roi), sizeof(bool));
            offset += sizeof(bool);

            // SDK 2.x
            //   // angle_veloacity_cov
            //   RsNativeCommon::fromEigenToNativeByte(robo_supplement.angle_velocity_cov,
            //                                         data + offset, len,
            //                                         isDstEndianTypeMatchHostEndian);
            //   offset += len;

            //   // angle_acceleration
            //   RsNativeCommon::fromEigenToNativeByte(robo_supplement.angle_acceleration,
            //                                         data + offset, len,
            //                                         isDstEndianTypeMatchHostEndian);
            //   offset += len;

            //   // angle_acceleration
            //   RsNativeCommon::fromEigenToNativeByte(
            //       robo_supplement.angle_acceleration_cov, data + offset, len,
            //       isDstEndianTypeMatchHostEndian);
            //   offset += len;

            // SDK 3.x
            floatEndian.toTargetEndianArray(
            robo_core.angle_velocity_cov, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // 填充 angle_velocity_cov
            float fill_angle_velocity_cov = 0;
            len = RsNativeCommon::fillTemplateElements<float>(
            fill_angle_velocity_cov, 8, data + offset, 8 * floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += len;

            // angle_acceleration
            floatEndian.toTargetEndianArray(
            robo_core.angle_acceleration, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // 填充 angle_acceleration
            float fill_angle_acceleration = 0;
            len = RsNativeCommon::fillTemplateElements<float>(
            fill_angle_acceleration, 2, data + offset, 2 * floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += len;

            // angle_acceleration_cov
            floatEndian.toTargetEndianArray(
            robo_core.angle_acceleration_cov, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            float fill_angle_acceleration_cov = 0;
            len = RsNativeCommon::fillTemplateElements<float>(
            fill_angle_acceleration_cov, 8, data + offset, 8 * floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += len;

            // trajectory
            int trajectorySize = robo_supplement.trajectory.size();
            int32Endian.toTargetEndianArray(
            trajectorySize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            for (auto iter = robo_supplement.trajectory.begin();
                 iter != robo_supplement.trajectory.end(); ++iter) {
                RsNativeCommon::fromEigenToNativeByte(*iter, data + offset, len,
                                                      isDstEndianTypeMatchHostEndian);
                offset += len;
            }

            // history_velocity
            int historyVelocitySize = robo_supplement.trajectory.size();
            int32Endian.toTargetEndianArray(
            historyVelocitySize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            for (auto iter = robo_supplement.history_velocity.begin();
                 iter != robo_supplement.history_velocity.end(); ++iter) {
                RsNativeCommon::fromEigenToNativeByte(*iter, data + offset, len,
                                                      isDstEndianTypeMatchHostEndian);
                offset += len;
            }

            // history_type
            int historyTypeSize = robo_supplement.history_type.size();
            int32Endian.toTargetEndianArray(
            historyTypeSize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            for (auto iter = robo_supplement.history_type.begin();
                 iter != robo_supplement.history_type.end(); ++iter) {
                int objectType = static_cast<int>(*iter);
                int32Endian.toTargetEndianArray(
                objectType, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;
            }

            // gps_info
            doubleEndian.toTargetEndianArray(
            robo_supplement.gps_longtitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toTargetEndianArray(
            robo_supplement.gps_latitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toTargetEndianArray(
            robo_supplement.gps_altitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen,
                           Object::Ptr &object) {
        if (maxLen < 0) {
            return -1;
        }

        object.reset(new Object());

        // CoreInfos
        CoreInfos &robo_core = object->core_infos_;
        RSEndian<float> floatEndian;
        RSEndian<int> int32Endian;
        RSEndian<unsigned int> uint32Endian;
        RSEndian<double> doubleEndian;
        bool isSrcEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int int32Size = sizeof(int);
        int uint32Size = sizeof(unsigned int);
        int floatSize = sizeof(float);
        int doubleSize = sizeof(double);

        int offset = 0;
        if (isSrcEndianTypeMatchHostEndian) {
            // timestamp
            memcpy(&(robo_core.timestamp), data + offset, doubleSize);
            offset += doubleSize;

            // priority_id
            memcpy(&(robo_core.priority_id), data + offset, int32Size);
            offset += int32Size;

            // SDK 2.x
            //   // existence_confidence
            //   memcpy(&(robo_core.existence_confidence), data + offset, floatSize);
            //   offset += floatSize;

            // SDK 3.x
            memcpy(&(robo_core.exist_confidence), data + offset, floatSize);
            offset += floatSize;

        } else {
            // timestamp
            doubleEndian.toHostEndianValue(
            robo_core.timestamp, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            // priority_id
            int32Endian.toHostEndianValue(robo_core.priority_id, data + offset,
                                          int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // SDK 2.x
            //   // existence_confidence
            //   floatEndian.toHostEndianValue(robo_core.existence_confidence,
            //                                 data + offset, floatSize,
            //                                 RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            //   offset += floatSize;

            // SDK 3.x
            floatEndian.toHostEndianValue(robo_core.exist_confidence, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }
        // std::cout << "run here 1" << std::endl;

        // center
        int len = 0;
        RsNativeCommon::fromNativeByteToEigen(robo_core.center, data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;
        // std::cout << "center = " << robo_core.center << std::endl;

        // SDK 2.x
        // // center_cov
        // RsNativeCommon::fromNativeByteToEigen(robo_core.center_cov, data +
        // offset,
        //                                       len,
        //                                       isSrcEndianTypeMatchHostEndian);
        // offset += len;

        // SDK 3.x
        // center_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.center_cov, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        offset += 6 * floatSize;  // 填充: center_cov

        // size
        RsNativeCommon::fromNativeByteToEigen(robo_core.size, data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;
        // std::cout << "size = " << robo_core.size << std::endl;

        // SDK 2.x
        // // size_cov
        // RsNativeCommon::fromNativeByteToEigen(robo_core.size_cov, data + offset,
        //                                       len,
        //                                       isSrcEndianTypeMatchHostEndian);
        // offset += len;

        // SDK 3.x
        // size_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.size_cov, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        offset += 6 * floatSize;  // 填充: size_ cov

        // direction
        RsNativeCommon::fromNativeByteToEigen(robo_core.direction, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;
        // std::cout << "direction = " << robo_core.direction << std::endl;

        // SDK 2.x
        // // direction_cov
        // RsNativeCommon::fromNativeByteToEigen(robo_core.direction_cov,
        //                                       data + offset, len,
        //                                       isSrcEndianTypeMatchHostEndian);
        // offset += len;

        // SDK 3.x
        // direction_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.direction_cov,
                                              data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        offset += 6 * floatSize;  // 填充: direction_cov

        // std::cout << "run here 2" << std::endl;

        if (isSrcEndianTypeMatchHostEndian) {
            // object type
            int objectType;
            memcpy(&objectType, data + offset, int32Size);
            robo_core.type = static_cast<ObjectType>(objectType);
            offset += int32Size;

            // object type confidence
            memcpy(&(robo_core.type_confidence), data + offset, floatSize);
            offset += floatSize;

            // attentions_type
            int attType;
            memcpy(&attType, data + offset, int32Size);
            robo_core.attention_type = static_cast<AttentionType>(attType);
            offset += int32Size;

            // motion_state
            int motionState;
            memcpy(&motionState, data + offset, int32Size);
            robo_core.motion_state = static_cast<MotionType>(motionState);
            offset += int32Size;

            // tracker_id
            memcpy(&(robo_core.tracker_id), data + offset, int32Size);
            offset += int32Size;

            // age
            memcpy(&(robo_core.age), data + offset, doubleSize);
            offset += doubleSize;
        } else {
            // object type
            int objectType = static_cast<int>(robo_core.type);
            int32Endian.toHostEndianValue(objectType, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // object type confidence
            floatEndian.toHostEndianValue(robo_core.type_confidence, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // attentions_type
            int attType = static_cast<int>(robo_core.attention_type);
            int32Endian.toHostEndianValue(attType, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // motion_state
            int motionState = static_cast<int>(robo_core.motion_state);
            int32Endian.toHostEndianValue(motionState, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // tracker_id
            int32Endian.toHostEndianValue(robo_core.tracker_id, data + offset,
                                          int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // age
            doubleEndian.toHostEndianValue(
            robo_core.age, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }
        // std::cout << "run here 3" << std::endl;

        // velocity
        RsNativeCommon::fromNativeByteToEigen(robo_core.velocity, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // SDK 2.x
        // // velocity_cov
        // RsNativeCommon::fromNativeByteToEigen(robo_core.velocity_cov, data +
        // offset,
        //                                       len,
        //                                       isSrcEndianTypeMatchHostEndian);
        // offset += len;

        // SDK 3.x
        // velocity_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.velocity_cov, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        offset += 6 * floatSize;  // 填充: velocity_cov

        // acceleration
        RsNativeCommon::fromNativeByteToEigen(robo_core.acceleration, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // SDK 2.x
        // // acceleration_cov
        // RsNativeCommon::fromNativeByteToEigen(robo_core.acceleration_cov,
        //                                       data + offset, len,
        //                                       isSrcEndianTypeMatchHostEndian);
        // offset += len;

        // SDK 3.x
        // acceleration_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.acceleration_cov,
                                              data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        offset += 6 * floatSize;  // 填充: acceleration_cov

        // angle_velocity
        // SDK 2.x
        // RsNativeCommon::fromNativeByteToEigen(robo_core.angle_velocity,
        //                                       data + offset, len,
        //                                       isSrcEndianTypeMatchHostEndian);
        // offset += len;
        // SDK 3.x
        floatEndian.toHostEndianValue(robo_core.angle_velocity, data + offset,
                                      floatSize,
                                      RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += floatSize;

        // 填充: angle_velocity
        offset += floatSize * 2;

        // anchor
        RsNativeCommon::fromNativeByteToEigen(robo_core.anchor, data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        // nearest_point
        RsNativeCommon::fromNativeByteToEigen(robo_core.nearest_point,
                                              data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        // SDK 2.x
        // // left_point
        // RsNativeCommon::fromNativeByteToEigen(robo_core.left_point, data +
        // offset,
        //                                       len,
        //                                       isSrcEndianTypeMatchHostEndian);
        // offset += len;

        // // right_point
        // RsNativeCommon::fromNativeByteToEigen(robo_core.right_point, data +
        // offset,
        //                                       len,
        //                                       isSrcEndianTypeMatchHostEndian);
        // offset += len;

        // SDK 3.x
        offset += 3 * floatSize;  // left_point，未解析保存

        offset += 3 * floatSize;  // right_point，未解析保存

        // std::cout << "run here 4" << std::endl;

        bool isSupplement = false;
        memcpy(&isSupplement, data + offset, sizeof(bool));
        offset += sizeof(bool);

        // Protobuf Serialize Supplement
        if (isSupplement == true) {
            // std::cout << "run here 5" << std::endl;

            SupplementInfos &robo_supplement = object->supplement_infos_;
            // polygon
            int polygonSize = 0;
            int32Endian.toHostEndianValue(polygonSize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            if (polygonSize > 0) {
                robo_supplement.polygon.resize(polygonSize);
                for (int i = 0; i < polygonSize; ++i) {
                    RsNativeCommon::fromNativeByteToEigen(robo_supplement.polygon[i],
                                                          data + offset, len,
                                                          isSrcEndianTypeMatchHostEndian);
                    offset += len;
                }
            }

            // SDK 2.x
            //   // reference
            //   int referenceSize = 0;
            //   int32Endian.toHostEndianValue(referenceSize, data + offset,
            //   int32Size,
            //                                 RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            //   offset += int32Size;

            //   if (referenceSize > 0) {
            //     robo_supplement.reference.resize(referenceSize);
            //     for (int i = 0; i < referenceSize; ++i) {
            //       RsNativeCommon::fromNativeByteToEigen(robo_supplement.reference[i],
            //                                             data + offset, len,
            //                                             isSrcEndianTypeMatchHostEndian);
            //       offset += len;
            //     }
            //   }
            // SDK 3.x
            int referenceSize = 0;
            int32Endian.toHostEndianValue(referenceSize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            // 不包含任何reference点

            // latent_types
            int latentTypesSize = 0;
            int32Endian.toHostEndianValue(latentTypesSize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            if (latentTypesSize > 0) {
                robo_supplement.latent_types.resize(latentTypesSize);
                for (int i = 0; i < latentTypesSize; ++i) {
                    floatEndian.toHostEndianValue(
                    robo_supplement.latent_types[i], data + offset, floatSize,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    offset += floatSize;
                }
            }

            // size_type
            int sizeType;
            int32Endian.toHostEndianValue(sizeType, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_supplement.size_type = static_cast<SizeType>(sizeType);
            offset += int32Size;

            // in_roi
            memcpy(&(robo_supplement.in_roi), data + offset, sizeof(bool));
            offset += sizeof(bool);

            // SDK 2.x
            //   // angle_veloacity_cov
            //   RsNativeCommon::fromNativeByteToEigen(robo_supplement.angle_velocity_cov,
            //                                         data + offset, len,
            //                                         isSrcEndianTypeMatchHostEndian);
            //   offset += len;

            //   // angle_acceleration
            //   RsNativeCommon::fromNativeByteToEigen(robo_supplement.angle_acceleration,
            //                                         data + offset, len,
            //                                         isSrcEndianTypeMatchHostEndian);
            //   offset += len;

            //   // angle_acceleration_cov
            //   RsNativeCommon::fromNativeByteToEigen(
            //       robo_supplement.angle_acceleration_cov, data + offset, len,
            //       isSrcEndianTypeMatchHostEndian);
            //   offset += len;

            // SDK 3.x
            // angle_velocity_cov
            floatEndian.toHostEndianValue(robo_core.angle_velocity_cov, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            offset += 8 * floatSize;  // 填充: angle_velocity_cov

            // angle_acceleration
            floatEndian.toHostEndianValue(robo_core.angle_acceleration, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            offset += 2 * floatSize;  // 填充: angle_acceleration

            // angle_acceleration_cov
            floatEndian.toHostEndianValue(robo_core.angle_acceleration_cov,
                                          data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            offset += 8 * floatSize;  // 填充: angle_acceleration_cov

            // trajectory
            int trajectorySize = 0;
            int32Endian.toHostEndianValue(trajectorySize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            for (auto iter = 0; iter < trajectorySize; ++iter) {
                // Eigen::Vector3f trajectory;
                RsVector3f trajectory;
                RsNativeCommon::fromNativeByteToEigen(trajectory, data + offset, len,
                                                      isSrcEndianTypeMatchHostEndian);
                offset += len;
                robo_supplement.trajectory.push_back(trajectory);
            }

            // history_velocity
            int historyVelocitySize = robo_supplement.trajectory.size();
            int32Endian.toHostEndianValue(historyVelocitySize, data + offset,
                                          int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            if (historyVelocitySize > 0) {
                for (auto iter = 0; iter < historyVelocitySize; ++iter) {
                    //   Eigen::Vector3f velocity;
                    RsVector3f velocity;
                    RsNativeCommon::fromNativeByteToEigen(velocity, data + offset, len,
                                                          isSrcEndianTypeMatchHostEndian);
                    offset += len;
                    robo_supplement.history_velocity.push_back(velocity);
                }
            }

            // history_type
            int historyTypeSize = 0;
            int32Endian.toHostEndianValue(historyTypeSize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            if (historyTypeSize > 0) {
                for (auto iter = 0; iter < historyTypeSize; ++iter) {
                    int objectType;
                    int32Endian.toHostEndianValue(
                    objectType, data + offset, int32Size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    offset += int32Size;
                    robo_supplement.history_type.push_back(
                    static_cast<ObjectType>(objectType));
                }
            }

            // gps_info
            doubleEndian.toHostEndianValue(
            robo_supplement.gps_longtitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toHostEndianValue(
            robo_supplement.gps_latitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toHostEndianValue(
            robo_supplement.gps_altitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }

        // std::cout << "run here 6: offset = " << offset << "==> maxLen = " <<
        // maxLen
        // << std::endl;
        return 0;
    }

    //   static int serialize(const std::vector<FreeSpaceInfo::Ptr> &freespaces,
    //                        char *data) {
    static int serialize(const RsFreeSpace::Ptr &freespace, char *data) {
        int freespacesCnt = freespace->fs_pts.size();
        if (freespacesCnt == 0) {
            return 0;
        }

        // std::cout << "freespace cnt = " << freespacesCnt << std::endl;

        int floatSize = sizeof(float);

        RSEndian<float> floatEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int offset = 0;
        // free_spaces
        if (isDstEndianTypeMatchHostEndian) {
            for (int i = 0; i < freespacesCnt; ++i) {
                const RsVector3f &anchorPoint = freespace->fs_pts[i];
                const float &free_prob = freespace->fs_confidence[i];

                float distance =
                std::sqrt(std::pow(anchorPoint.x, 2) + std::pow(anchorPoint.y, 2));
                float yaw_angle = std::atan2(anchorPoint.x, anchorPoint.y);

                memcpy(data + offset, &distance, floatSize);
                offset += floatSize;
                memcpy(data + offset, &yaw_angle, floatSize);
                offset += floatSize;
                memcpy(data + offset, &free_prob, floatSize);
                offset += floatSize;
            }
        } else {
            for (int i = 0; i < freespacesCnt; ++i) {
                const RsVector3f &anchorPoint = freespace->fs_pts[i];
                const float &free_prob = freespace->fs_confidence[i];

                float distance =
                std::sqrt(std::pow(anchorPoint.x, 2) + std::pow(anchorPoint.y, 2));
                float yaw_angle = std::atan2(anchorPoint.x, anchorPoint.y);

                floatEndian.toTargetEndianArray(
                distance, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(
                yaw_angle, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(
                free_prob, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
            }
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen,
                           RsFreeSpace::Ptr &freespaces) {
        if (freespaces == nullptr) {
            freespaces.reset(new RsFreeSpace());
        }

        int floatSize = sizeof(float);
        int basicSize = floatSize * 3;

        if (maxLen % basicSize != 0 || maxLen < 0) {
            return -1;
        }

        int freespaceCnt = maxLen / basicSize;

        // std::cout << __FUNCTION__ << "=> A freespace cnt = " << freespaceCnt <<
        // std::endl;
        freespaces->fs_pts.clear();
        freespaces->fs_confidence.clear();

        // freespaces
        RSEndian<float> floatEndian;
        bool isSrcEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int offset = 0;
        if (isSrcEndianTypeMatchHostEndian) {
            for (int i = 0; i < freespaceCnt; ++i) {
                float distance = 0;
                float yaw_angle = 0;
                float free_prob = 0;

                memcpy(&distance, data + offset, floatSize);
                offset += floatSize;
                memcpy(&yaw_angle, data + offset, floatSize);
                offset += floatSize;
                memcpy(&free_prob, data + offset, floatSize);
                offset += floatSize;

                RsVector3f anchorPoint;
                anchorPoint.x = distance * std::cos(yaw_angle);
                anchorPoint.y = distance * std::sin(yaw_angle);
                anchorPoint.z = 0;  // 无法解析获得

                freespaces->fs_pts.push_back(anchorPoint);
                freespaces->fs_confidence.push_back(free_prob);
            }
        } else {
            for (int i = 0; i < freespaceCnt; ++i) {
                float distance = 0;
                float yaw_angle = 0;
                float free_prob = 0;

                floatEndian.toHostEndianValue(
                distance, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(
                yaw_angle, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(
                free_prob, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                RsVector3f anchorPoint;
                anchorPoint.x = distance * std::cos(yaw_angle);
                anchorPoint.y = distance * std::sin(yaw_angle);
                anchorPoint.z = 0;  // 无法解析获得

                freespaces->fs_pts.push_back(anchorPoint);
                freespaces->fs_confidence.push_back(free_prob);
            }
        }
        // std::cout << __FUNCTION__ << "=> B freespace cnt = " << freespaceCnt <<
        // std::endl;
        return 0;
    }

    //   static int serialize(const std::vector<Curb::Ptr> &curbs, char *data) {

    static int serialize(const std::vector<Roadedge::Ptr> &curbs, char *data) {
        int curbsCnt = curbs.size();
        if (curbsCnt == 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // curbs
        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            for (int i = 0; i < curbsCnt; ++i) {
                const Roadedge::Ptr &robo_curb_ptr = curbs[i];

                // pos_type
                int posType = static_cast<int>(robo_curb_ptr->roadedge_id);
                memcpy(data + offset, &posType, int32Size);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_curb_ptr->curve;
                memcpy(data + offset, &(robo_curve.x_start), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.x_end), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.a), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.b), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.c), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.d), floatSize);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_curb_ptr->end_point;

                memcpy(data + offset, &(robo_endpoints.start.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.start.y), floatSize);
                offset += floatSize;
                // memcpy(data + offset, &(robo_endpoints.start.z), floatSize);
                // offset += floatSize;
                memset(data + offset, 0, floatSize);
                offset += floatSize;

                memcpy(data + offset, &(robo_endpoints.end.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.end.y), floatSize);
                offset += floatSize;
                // memcpy(data + offset, &(robo_endpoints.end.z), floatSize);
                // offset += floatSize;
                memset(data + offset, 0, floatSize);
                offset += floatSize;

                // track_id
                // memcpy(data + offset, &(robo_curb_ptr->track_id), int32Size);
                // offset += int32Size;
                memset(data + offset, 0, int32Size);
                offset += int32Size;

                // confidence
                memcpy(data + offset, &(robo_curb_ptr->confidence), floatSize);
                offset += floatSize;
            }
        } else {
            for (int i = 0; i < curbsCnt; ++i) {
                const Roadedge::Ptr &robo_curb_ptr = curbs[i];

                // pos_type
                int posType = static_cast<int>(robo_curb_ptr->roadedge_id);
                int32Endian.toTargetEndianArray(
                posType, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_curb_ptr->curve;
                floatEndian.toTargetEndianArray(
                robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_curb_ptr->end_point;

                floatEndian.toTargetEndianArray(
                robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                // floatEndian.toTargetEndianArray(
                //     robo_endpoints.start.z, data + offset, floatSize,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += floatSize;
                memset(data + offset, 0, floatSize);  // 填充: start.z
                offset += floatSize;

                floatEndian.toTargetEndianArray(
                robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                // floatEndian.toTargetEndianArray(
                //     robo_endpoints.end.z, data + offset, floatSize,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += floatSize;
                memset(data + offset, 0, floatSize);  // 填充: end.z
                offset += floatSize;

                // track_id
                // int32Endian.toTargetEndianArray(
                //     robo_curb_ptr->track_id, data + offset, int32Size,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += int32Size;
                memset(data + offset, 0, int32Size);  // 填充: tracker_id
                offset += int32Size;

                // confidence
                floatEndian.toTargetEndianArray(
                robo_curb_ptr->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
            }
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen,
                           std::vector<Roadedge::Ptr> &curbs) {
        curbs.clear();
        if (maxLen < 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isScrEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // curbs
        int offset = 0;

        if (isScrEndianTypeMatchHostEndian) {
            while (offset < maxLen) {
                Roadedge::Ptr curb(new Roadedge());

                // pos_type
                int posType = 0;
                memcpy(&posType, data + offset, int32Size);
                curb->roadedge_id = static_cast<RoadedgePosition>(posType);
                offset += int32Size;

                // curve
                Curve &robo_curve = curb->curve;
                memcpy(&(robo_curve.x_start), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.x_end), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.a), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.b), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.c), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.d), data + offset, floatSize);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = curb->end_point;

                memcpy(&(robo_endpoints.start.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.start.y), data + offset, floatSize);
                offset += floatSize;
                // memcpy(&(robo_endpoints.start.z), data + offset, floatSize);
                // offset += floatSize;
                offset += floatSize;  // 不解析: start.z

                memcpy(&(robo_endpoints.end.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.end.y), data + offset, floatSize);
                offset += floatSize;
                // memcpy(&(robo_endpoints.end.z), data + offset, floatSize);
                // offset += floatSize;
                offset += floatSize;  // 不解析: end.z

                // track_id
                // memcpy(&(curb->track_id), data + offset, int32Size);
                // offset += int32Size;
                offset += int32Size;  // 不解析: track_id

                // confidence
                memcpy(&(curb->confidence), data + offset, floatSize);
                offset += floatSize;

                curbs.push_back(curb);
            }
        } else {
            while (offset < maxLen) {
                Roadedge::Ptr curb(new Roadedge());
                // pos_type
                int posType;
                int32Endian.toHostEndianValue(
                posType, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                curb->roadedge_id = static_cast<RoadedgePosition>(posType);
                offset += int32Size;

                // curve
                Curve &robo_curve = curb->curve;
                floatEndian.toHostEndianValue(
                robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = curb->end_point;

                floatEndian.toHostEndianValue(
                robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                // floatEndian.toHostEndianValue(
                //     robo_endpoints.start.z, data + offset, floatSize,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += floatSize;
                offset += floatSize;  // 不解析: start.z

                floatEndian.toHostEndianValue(
                robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                // floatEndian.toHostEndianValue(
                //     robo_endpoints.end.z, data + offset, floatSize,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += floatSize;
                offset += floatSize;  // 不解析: end.z

                // track_id
                // int32Endian.toHostEndianValue(
                //     curb->track_id, data + offset, int32Size,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += int32Size;
                offset += int32Size;  // 不解析: track_id

                // confidence
                floatEndian.toHostEndianValue(
                curb->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                curbs.push_back(curb);
            }
        }

        return 0;
    }

    static int serialize(const std::vector<Lane::Ptr> &lanes, char *data) {
        int lanesCnt = lanes.size();

        if (lanesCnt == 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // lanes
        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            for (int i = 0; i < lanesCnt; ++i) {
                const Lane::Ptr &robo_lane_ptr = lanes[i];

                // pos_type
                int posType = static_cast<int>(robo_lane_ptr->lane_id);
                memcpy(data + offset, &posType, int32Size);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_lane_ptr->curve;
                memcpy(data + offset, &(robo_curve.x_start), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.x_end), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.a), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.b), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.c), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.d), floatSize);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_lane_ptr->end_point;

                memcpy(data + offset, &(robo_endpoints.start.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.start.y), floatSize);
                offset += floatSize;
                // memcpy(data + offset, &(robo_endpoints.start.z), floatSize);
                // offset += floatSize;
                memset(data + offset, 0, floatSize);
                offset += floatSize;

                memcpy(data + offset, &(robo_endpoints.end.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.end.y), floatSize);
                offset += floatSize;
                // memcpy(data + offset, &(robo_endpoints.end.z), floatSize);
                // offset += floatSize;
                memset(data + offset, 0, floatSize);
                offset += floatSize;

                // track_id
                // memcpy(data + offset, &(robo_lane_ptr->track_id), int32Size);
                // offset += int32Size;
                memset(data + offset, 0, int32Size);
                offset += int32Size;

                // confidence
                memcpy(data + offset, &(robo_lane_ptr->confidence), floatSize);
                offset += floatSize;
            }
        } else {
            for (int i = 0; i < lanesCnt; ++i) {
                const Lane::Ptr &robo_lane_ptr = lanes[i];

                // pos_type
                int posType = static_cast<int>(robo_lane_ptr->lane_id);
                int32Endian.toTargetEndianArray(
                posType, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_lane_ptr->curve;
                floatEndian.toTargetEndianArray(
                robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_lane_ptr->end_point;

                floatEndian.toTargetEndianArray(
                robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                // floatEndian.toTargetEndianArray(
                //     robo_endpoints.start.z, data + offset, floatSize,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += floatSize;
                memset(data + offset, 0, floatSize);
                offset += floatSize;

                floatEndian.toTargetEndianArray(
                robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                // floatEndian.toTargetEndianArray(
                //     robo_endpoints.end.z, data + offset, floatSize,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += floatSize;
                memset(data + offset, 0, floatSize);
                offset += floatSize;

                // track_id
                // int32Endian.toTargetEndianArray(
                //     robo_lane_ptr->track_id, data + offset, int32Size,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += int32Size;
                memset(data + offset, 0, floatSize);
                offset += int32Size;

                // confidence
                floatEndian.toTargetEndianArray(
                robo_lane_ptr->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
            }
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen,
                           std::vector<Lane::Ptr> &lanes) {
        lanes.clear();
        if (maxLen < 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isScrEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // lanes
        int offset = 0;

        if (isScrEndianTypeMatchHostEndian) {
            while (offset < maxLen) {
                Lane::Ptr lane(new Lane());

                // pos_type
                int posType = 0;
                memcpy(&posType, data + offset, int32Size);
                lane->lane_id = static_cast<LanePosition>(posType);
                offset += int32Size;

                // curve
                Curve &robo_curve = lane->curve;
                memcpy(&(robo_curve.x_start), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.x_end), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.a), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.b), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.c), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.d), data + offset, floatSize);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = lane->end_point;

                memcpy(&(robo_endpoints.start.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.start.y), data + offset, floatSize);
                offset += floatSize;
                // memcpy(&(robo_endpoints.start.z), data + offset, floatSize);
                // offset += floatSize;
                offset += floatSize;  // 不解析: start.z

                memcpy(&(robo_endpoints.end.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.end.y), data + offset, floatSize);
                offset += floatSize;
                // memcpy(&(robo_endpoints.end.z), data + offset, floatSize);
                // offset += floatSize;
                offset += floatSize;  // 不解析: end.z

                // track_id
                // memcpy(&(lane->track_id), data + offset, int32Size);
                // offset += int32Size;
                offset += int32Size;  // 不解析: track_id

                // confidence
                memcpy(&(lane->confidence), data + offset, floatSize);
                offset += floatSize;

                lanes.push_back(lane);
            }
        } else {
            while (offset < maxLen) {
                Lane::Ptr lane(new Lane());
                // pos_type
                int posType;
                int32Endian.toHostEndianValue(
                posType, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                lane->lane_id = static_cast<LanePosition>(posType);
                offset += int32Size;

                // curve
                Curve &robo_curve = lane->curve;
                floatEndian.toHostEndianValue(
                robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = lane->end_point;

                floatEndian.toHostEndianValue(
                robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                // floatEndian.toHostEndianValue(
                //     robo_endpoints.start.z, data + offset, floatSize,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += floatSize;
                offset += floatSize;  // 不解析: start.z

                floatEndian.toHostEndianValue(
                robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                // floatEndian.toHostEndianValue(
                //     robo_endpoints.end.z, data + offset, floatSize,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += floatSize;
                offset += floatSize;  // 不解析: end.z

                // track_id
                // int32Endian.toHostEndianValue(
                //     lane->track_id, data + offset, int32Size,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += int32Size;
                offset += int32Size;  // 不解析: track_id

                // confidence
                floatEndian.toHostEndianValue(
                lane->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                lanes.push_back(lane);
            }
        }

        return 0;
    }

    //   static int serialize(const Pose::Ptr &pose, char *data) {
    static int serialize(const RsPose::Ptr &pose, char *data) {
        int floatSize = sizeof(float);

        RSEndian<float> floatEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            memcpy(data + offset, &(pose->x), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->y), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->z), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->roll), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->pitch), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->yaw), floatSize);
            offset += floatSize;
        } else {
            floatEndian.toTargetEndianArray(
            pose->x, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(
            pose->y, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(
            pose->z, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(
            pose->roll, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(
            pose->pitch, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(
            pose->yaw, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen,
                           RsPose::Ptr &pose) {
        if (maxLen < 24) {
            return 0;
        }

        pose.reset(new RsPose());

        int floatSize = sizeof(float);

        RSEndian<float> floatEndian;
        bool isScrEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int offset = 0;
        if (isScrEndianTypeMatchHostEndian) {
            memcpy(&(pose->x), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->y), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->z), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->roll), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->pitch), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->yaw), data + offset, floatSize);
            offset += floatSize;
        } else {
            floatEndian.toHostEndianValue(pose->x, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->y, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->z, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->roll, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->pitch, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->yaw, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }
        return 0;
    }
};

}  // namespace native_sdk_2_x

namespace native_sdk_3_0 {

/**
 * @brief The st_RoboRecvMessage struct
 *
 * status: Index Message Receive Status
 * recorder: Recorder UDP Translate Log
 * device_id: device id
 * timestamp: message timestamp
 * msg: Receive Message
 *
 */
struct st_RoboRecvMessage {
public:
    int device_id;
    double timestamp;
    std::map<ROBO_MSG_TYPE, st_RoboMsgRecorder> recorder;
    std::map<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS> status;
    RsPerceptionMsg::Ptr msg;

public:
    st_RoboRecvMessage() {
        device_id = 0;
        timestamp = 0;
    };

public:
    // #ifdef CUSTOM_GE_COMM_FOUND
    //     void updateStatusByConfig(const st_CustomMessageConfig
    //     &fordMessageConfig);
    // #endif // CUSTOM_GE_COMM_FOUND

    // #ifdef CUSTOM_CR_COMM_FOUND
    //     void updateStatusByConfig(const st_CustomMessageConfig
    //     &beiqiMessageConfig);
    // #endif // CUSTOM_CR_COMM_FOUND

    void updateStatusByConfig(const RsCustomNativeBytesSDK3_0MsgParams &translateConfig) {
        status.clear();
        recorder.clear();

        // Message Receive Recorder
        st_RoboMsgRecorder msgRecorder;

        // Axis Status Object Message Info
        if (translateConfig.send_axisstatus) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_AXISSTATUS,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_AXISSTATUS,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_AXISSTATUS, msgRecorder));

        // Object Message Info
        if (translateConfig.send_objects) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_OBJECT, ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_OBJECT,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_OBJECT, msgRecorder));

        // Attention Object Message Info
        if (translateConfig.send_attention_objects) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT, msgRecorder));

        // Freespace Message Info
        if (translateConfig.send_freespace) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_FREESPACE,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_FREESPACE,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_FREESPACE, msgRecorder));

        // Lane Message Info
        if (translateConfig.send_lanes) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_LANE, ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_LANE,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_LANE, msgRecorder));

        // Curb Message Info
        if (translateConfig.send_curbs) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_CURB, ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_CURB,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_CURB, msgRecorder));

        // Axis Lidar Pose Message Info
        if (translateConfig.send_axis_lidar_pose) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_AXISLIDAR_POSE,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_AXISLIDAR_POSE,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_AXISLIDAR_POSE, msgRecorder));

        // PointCloud Message Info
        if (translateConfig.send_pointcloud) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_POINT, ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_POINT,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_POINT, msgRecorder));

        // Global Case Pose Message Info
        if (translateConfig.send_global_car_pose) {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_GLOBALCAR_POSE,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_LOSS));
        } else {
            status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_RECV_STATUS>(
            ROBO_MSG_TYPE::ROBO_MSG_GLOBALCAR_POSE,
            ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_GLOBALCAR_POSE, msgRecorder));
    }
};

class RSNativeSerializeUtil {
public:
    using Ptr = std::shared_ptr<RSNativeSerializeUtil>;
    using ConstPtr = std::shared_ptr<const RSNativeSerializeUtil>;

public:
    static int serialize(const Object::Ptr &object, bool isSupplement,
                         char *data) {
        const CoreInfos &robo_core = object->core_infos_;

        RSEndian<float> floatEndian;
        RSEndian<int> int32Endian;
        RSEndian<unsigned int> uint32Endian;
        RSEndian<double> doubleEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int int32Size = sizeof(int);
        // int uint32Size = sizeof(unsigned int);
        int floatSize = sizeof(float);
        int doubleSize = sizeof(double);

        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            // timestamp
            memcpy(data + offset, &(robo_core.timestamp), doubleSize);
            offset += doubleSize;

            // priority_id
            memcpy(data + offset, &(robo_core.priority_id), int32Size);
            offset += int32Size;

            // exist_confidence
            memcpy(data + offset, &(robo_core.exist_confidence), floatSize);
            offset += floatSize;
        } else {
            // timestamp
            doubleEndian.toTargetEndianArray(
            robo_core.timestamp, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            // priority_id
            int32Endian.toTargetEndianArray(
            robo_core.priority_id, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // exist_confidence
            floatEndian.toTargetEndianArray(
            robo_core.exist_confidence, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }

        // center
        int len = 0;
        RsNativeCommon::fromEigenToNativeByte(robo_core.center, data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // center_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.center_cov, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // size
        RsNativeCommon::fromEigenToNativeByte(robo_core.size, data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // size_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.size_cov, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // direction
        RsNativeCommon::fromEigenToNativeByte(robo_core.direction, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // direction_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.direction_cov,
                                              data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        if (isDstEndianTypeMatchHostEndian) {
            // object type
            int objectType = static_cast<int>(robo_core.type);
            memcpy(data + offset, &objectType, int32Size);
            offset += int32Size;

            // object type confidence
            memcpy(data + offset, &(robo_core.type_confidence), floatSize);
            offset += floatSize;

            // attentions_type
            int attType = static_cast<int>(robo_core.attention_type);
            memcpy(data + offset, &attType, int32Size);
            offset += int32Size;

            // motion_state
            int motionState = static_cast<int>(robo_core.motion_state);
            memcpy(data + offset, &motionState, int32Size);
            offset += int32Size;

            // lane_pos
            int lane_pos = static_cast<int>(robo_core.lane_pos);
            memcpy(data + offset, &lane_pos, int32Size);
            offset += int32Size;

            // tracker_id
            memcpy(data + offset, &(robo_core.tracker_id), int32Size);
            offset += int32Size;

            // age
            memcpy(data + offset, &(robo_core.age), doubleSize);
            offset += doubleSize;
        } else {
            // object type
            int objectType = static_cast<int>(robo_core.type);
            int32Endian.toTargetEndianArray(
            objectType, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // object type confidence
            floatEndian.toTargetEndianArray(
            robo_core.type_confidence, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // attentions_type
            int attType = static_cast<int>(robo_core.attention_type);
            int32Endian.toTargetEndianArray(
            attType, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // motion_state
            int motionState = static_cast<int>(robo_core.motion_state);
            int32Endian.toTargetEndianArray(
            motionState, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // lane_pos
            int lane_pos = static_cast<int>(robo_core.lane_pos);
            int32Endian.toTargetEndianArray(
            lane_pos, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // tracker_id
            int32Endian.toTargetEndianArray(
            robo_core.tracker_id, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // age
            doubleEndian.toTargetEndianArray(
            robo_core.age, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }

        // velocity
        RsNativeCommon::fromEigenToNativeByte(robo_core.velocity, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // velocity_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.velocity_cov, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // acceleration
        RsNativeCommon::fromEigenToNativeByte(robo_core.acceleration, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // acceleration_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.acceleration_cov,
                                              data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        if (isDstEndianTypeMatchHostEndian) {
            // angle_velocity
            memcpy(data + offset, &(robo_core.angle_velocity), floatSize);
            offset += floatSize;

            // angle_velocity_cov
            memcpy(data + offset, &(robo_core.angle_velocity_cov), floatSize);
            offset += floatSize;

            // angle_acceleration
            memcpy(data + offset, &(robo_core.angle_acceleration), floatSize);
            offset += floatSize;

            // angle_acceleration_cov
            memcpy(data + offset, &(robo_core.angle_acceleration_cov), floatSize);
            offset += floatSize;
        } else {
            // angle_velocity
            floatEndian.toTargetEndianArray(
            robo_core.angle_velocity, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_velocity_cov
            floatEndian.toTargetEndianArray(
            robo_core.angle_velocity_cov, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_acceleration
            floatEndian.toTargetEndianArray(
            robo_core.angle_acceleration, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_acceleration_cov
            floatEndian.toTargetEndianArray(
            robo_core.angle_acceleration_cov, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }

        // anchor
        RsNativeCommon::fromEigenToNativeByte(robo_core.anchor, data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // nearest_point
        RsNativeCommon::fromEigenToNativeByte(robo_core.nearest_point,
                                              data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        memcpy(data + offset, &isSupplement, sizeof(bool));
        offset += sizeof(bool);

        // Protobuf Serialize Supplement
        if (isSupplement == true) {
            // std::cout << "isSupplement = " << isSupplement << std::endl;
            const SupplementInfos &robo_supplement = object->supplement_infos_;

            if (isDstEndianTypeMatchHostEndian) {
                // unique_id
                unsigned int unique_id = robo_supplement.unique_id;
                memcpy(data + offset, &unique_id, int32Size);
                offset += int32Size;
            } else {
                // unique_id
                uint32Endian.toTargetEndianArray(
                robo_supplement.unique_id, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;
            }

            //   // status
            //   int status = static_cast<int>(robo_supplement.status);
            //   int32Endian.toTargetEndianArray(
            //       status, data + offset, int32Size,
            //       RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            //   offset += int32Size;

            memset(data + offset, 0, int32Size);  // 填充: status
            offset += int32Size;

            // polygon
            int polygonSize = robo_supplement.polygon.size();
            int32Endian.toTargetEndianArray(
            polygonSize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            for (int i = 0; i < polygonSize; ++i) {
                RsNativeCommon::fromEigenToNativeByte(robo_supplement.polygon[i],
                                                      data + offset, len,
                                                      isDstEndianTypeMatchHostEndian);
                offset += len;
            }

            if (isDstEndianTypeMatchHostEndian) {
                // left_point_index
                int left_point_index = robo_supplement.left_point_index;
                memcpy(data + offset, &left_point_index, int32Size);
                offset += int32Size;

                // right_point_index
                int right_point_index = robo_supplement.right_point_index;
                memcpy(data + offset, &right_point_index, int32Size);
                offset += int32Size;
            } else {
                // left_point_index
                uint32Endian.toTargetEndianArray(
                robo_supplement.left_point_index, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // right_point_index
                uint32Endian.toTargetEndianArray(
                robo_supplement.right_point_index, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;
            }

            // latent_types
            int latentTypesSize = robo_supplement.latent_types.size();
            int32Endian.toTargetEndianArray(
            latentTypesSize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            // RINFO << "latentTypesSize = " << latentTypesSize;
            for (int i = 0; i < latentTypesSize; ++i) {
                floatEndian.toTargetEndianArray(
                robo_supplement.latent_types[i], data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
            }

            // size_type
            int sizeType = static_cast<int>(robo_supplement.size_type);
            int32Endian.toTargetEndianArray(
            sizeType, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // mode
            int mode = static_cast<int>(robo_supplement.mode);
            int32Endian.toTargetEndianArray(
            mode, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // in_roi
            memcpy(data + offset, &(robo_supplement.in_roi), sizeof(bool));
            offset += sizeof(bool);

            // stability_type
            int stability_type = static_cast<int>(robo_supplement.tracking_state);
            int32Endian.toTargetEndianArray(
            stability_type, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // geo_center
            RsNativeCommon::fromEigenToNativeByte(robo_supplement.geo_center,
                                                  data + offset, len,
                                                  isDstEndianTypeMatchHostEndian);
            offset += len;

            // geo_size
            RsNativeCommon::fromEigenToNativeByte(robo_supplement.geo_size,
                                                  data + offset, len,
                                                  isDstEndianTypeMatchHostEndian);
            offset += len;

            // trajectory
            int trajectorySize = robo_supplement.trajectory.size();
            int32Endian.toTargetEndianArray(
            trajectorySize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            for (int i = 0; i < trajectorySize; ++i) {
                RsNativeCommon::fromEigenToNativeByte(robo_supplement.trajectory[i],
                                                      data + offset, len,
                                                      isDstEndianTypeMatchHostEndian);
                offset += len;
            }

            // history_velocity
            int historyVelocitySize = robo_supplement.history_velocity.size();
            int32Endian.toTargetEndianArray(
            historyVelocitySize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            for (int i = 0; i < historyVelocitySize; ++i) {
                RsNativeCommon::fromEigenToNativeByte(
                robo_supplement.history_velocity[i], data + offset, len,
                isDstEndianTypeMatchHostEndian);
                offset += len;
            }

            // history_type
            int historyTypeSize = robo_supplement.history_type.size();
            int32Endian.toTargetEndianArray(
            historyTypeSize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            for (int i = 0; i < historyTypeSize; ++i) {
                int objectType = static_cast<int>(robo_supplement.history_type[i]);
                int32Endian.toTargetEndianArray(
                objectType, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;
            }

            // gps_info
            int gps_mode = static_cast<int>(robo_supplement.gps_mode);
            int32Endian.toTargetEndianArray(
            gps_mode, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            doubleEndian.toTargetEndianArray(
            robo_supplement.gps_longtitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toTargetEndianArray(
            robo_supplement.gps_latitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toTargetEndianArray(
            robo_supplement.gps_altitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen,
                           Object::Ptr &object) {
        if (maxLen < 0) {
            return -1;
        }

        object.reset(new Object());

        // CoreInfos
        CoreInfos &robo_core = object->core_infos_;
        RSEndian<float> floatEndian;
        RSEndian<int> int32Endian;
        RSEndian<unsigned int> uint32Endian;
        RSEndian<double> doubleEndian;
        bool isSrcEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int int32Size = sizeof(int);
        // int uint32Size = sizeof(unsigned int);
        int floatSize = sizeof(float);
        int doubleSize = sizeof(double);

        int offset = 0;
        if (isSrcEndianTypeMatchHostEndian) {
            // timestamp
            memcpy(&(robo_core.timestamp), data + offset, doubleSize);
            offset += doubleSize;

            // priority_id
            memcpy(&(robo_core.priority_id), data + offset, int32Size);
            offset += int32Size;

            // exist_confidence
            memcpy(&(robo_core.exist_confidence), data + offset, floatSize);
            offset += floatSize;

        } else {
            // timestamp

            doubleEndian.toHostEndianValue(
            robo_core.timestamp, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            // priority_id

            int32Endian.toHostEndianValue(robo_core.priority_id, data + offset,
                                          int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // exist_confidence
            floatEndian.toHostEndianValue(robo_core.exist_confidence, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }
        // std::cout << "run here 1" << std::endl;

        // center
        int len = 0;
        RsNativeCommon::fromNativeByteToEigen(robo_core.center, data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;
        // std::cout << "center = " << robo_core.center << std::endl;

        // center_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.center_cov, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // size
        RsNativeCommon::fromNativeByteToEigen(robo_core.size, data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;
        // std::cout << "size = " << robo_core.size << std::endl;

        // size_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.size_cov, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // direction
        RsNativeCommon::fromNativeByteToEigen(robo_core.direction, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;
        // std::cout << "direction = " << robo_core.direction << std::endl;

        // direction_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.direction_cov,
                                              data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        // std::cout << "run here 2" << std::endl;

        if (isSrcEndianTypeMatchHostEndian) {
            // object type
            int objectType;
            memcpy(&objectType, data + offset, int32Size);
            robo_core.type = static_cast<ObjectType>(objectType);
            offset += int32Size;

            // object type confidence
            memcpy(&(robo_core.type_confidence), data + offset, floatSize);
            offset += floatSize;

            // attentions_type
            int attType;
            memcpy(&attType, data + offset, int32Size);
            robo_core.attention_type = static_cast<AttentionType>(attType);
            offset += int32Size;

            // motion_state
            int motionState;
            memcpy(&motionState, data + offset, int32Size);
            robo_core.motion_state = static_cast<MotionType>(motionState);
            offset += int32Size;

            // lane_poe
            int lane_pos;
            memcpy(&lane_pos, data + offset, int32Size);
            robo_core.lane_pos = static_cast<LanePosition>(lane_pos);
            offset += int32Size;

            // tracker_id
            memcpy(&(robo_core.tracker_id), data + offset, int32Size);
            offset += int32Size;

            // age
            memcpy(&(robo_core.age), data + offset, doubleSize);
            offset += doubleSize;
        } else {
            // object type
            int objectType = static_cast<int>(robo_core.type);
            int32Endian.toHostEndianValue(objectType, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // object type confidence
            floatEndian.toHostEndianValue(robo_core.type_confidence, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // attentions_type
            int attType = static_cast<int>(robo_core.attention_type);
            int32Endian.toHostEndianValue(attType, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // motion_state
            int motionState = static_cast<int>(robo_core.motion_state);
            int32Endian.toHostEndianValue(motionState, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // lane_pos
            int lane_pos;
            int32Endian.toHostEndianValue(lane_pos, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // tracker_id
            int32Endian.toHostEndianValue(robo_core.tracker_id, data + offset,
                                          int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // age
            doubleEndian.toHostEndianValue(
            robo_core.age, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }
        // std::cout << "run here 3" << std::endl;

        // velocity
        RsNativeCommon::fromNativeByteToEigen(robo_core.velocity, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // velocity_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.velocity_cov, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // acceleration
        RsNativeCommon::fromNativeByteToEigen(robo_core.acceleration, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // acceleration_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.acceleration_cov,
                                              data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        if (isSrcEndianTypeMatchHostEndian) {
            // angle_velocity
            memcpy(&(robo_core.angle_velocity), data + offset, floatSize);
            offset += floatSize;

            // angle_velocity_cov
            memcpy(&(robo_core.angle_velocity_cov), data + offset, floatSize);
            offset += floatSize;

            // angle_acceleration
            memcpy(&(robo_core.angle_acceleration), data + offset, floatSize);
            offset += floatSize;

            // angle_acceleration_cov
            memcpy(&(robo_core.angle_acceleration_cov), data + offset, floatSize);
            offset += floatSize;
        } else {
            // angle_velocity
            floatEndian.toHostEndianValue(robo_core.angle_velocity, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_velocity_cov
            floatEndian.toHostEndianValue(robo_core.angle_velocity_cov, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_acceleration
            floatEndian.toHostEndianValue(robo_core.angle_acceleration, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_acceleration_cov
            floatEndian.toHostEndianValue(robo_core.angle_acceleration_cov,
                                          data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }

        // anchor
        RsNativeCommon::fromNativeByteToEigen(robo_core.anchor, data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        // nearest_point
        RsNativeCommon::fromNativeByteToEigen(robo_core.nearest_point,
                                              data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        // std::cout << "run here 4" << std::endl;

        bool isSupplement = false;
        memcpy(&isSupplement, data + offset, sizeof(bool));
        offset += sizeof(bool);

        // Protobuf Serialize Supplement
        if (isSupplement == true) {
            // std::cout << "run here 5" << std::endl;

            SupplementInfos &robo_supplement = object->supplement_infos_;

            // unique_id
            uint32Endian.toHostEndianValue(
            robo_supplement.unique_id, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            //   // status
            //   int status;
            //   int32Endian.toHostEndianValue(status, data + offset, int32Size,
            //                                 RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            //   robo_supplement.status = static_cast<AxisStatus>(status);
            //   offset += int32Size;

            offset += int32Size;  // 填充: status

            // polygon
            int polygonSize = 0;
            int32Endian.toHostEndianValue(polygonSize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            if (polygonSize > 0) {
                robo_supplement.polygon.resize(polygonSize);
                for (int i = 0; i < polygonSize; ++i) {
                    RsNativeCommon::fromNativeByteToEigen(robo_supplement.polygon[i],
                                                          data + offset, len,
                                                          isSrcEndianTypeMatchHostEndian);
                    offset += len;
                }
            }

            // left_point_index
            int32Endian.toHostEndianValue(robo_supplement.left_point_index,
                                          data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // right_point_index
            int32Endian.toHostEndianValue(robo_supplement.right_point_index,
                                          data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // latent_types
            int latentTypesSize = 0;
            int32Endian.toHostEndianValue(latentTypesSize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            // RERROR << "latentTypesSize = " << latentTypesSize;
            if (latentTypesSize > 0) {
                robo_supplement.latent_types.resize(latentTypesSize);
                for (int i = 0; i < latentTypesSize; ++i) {
                    floatEndian.toHostEndianValue(
                    robo_supplement.latent_types[i], data + offset, floatSize,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    offset += floatSize;
                }
            } else {
                robo_supplement.latent_types.clear();
            }

            // size_type
            int sizeType;
            int32Endian.toHostEndianValue(sizeType, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_supplement.size_type = static_cast<SizeType>(sizeType);
            offset += int32Size;

            // mode
            int mode;
            int32Endian.toHostEndianValue(mode, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_supplement.mode = static_cast<ModeType>(mode);
            offset += int32Size;

            // in_roi
            memcpy(&(robo_supplement.in_roi), data + offset, sizeof(bool));
            offset += sizeof(bool);

            // stability_type
            int tracking_state;
            int32Endian.toHostEndianValue(tracking_state, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_supplement.tracking_state =
            static_cast<TrackingState>(tracking_state);
            offset += int32Size;

            // geo_center
            RsNativeCommon::fromNativeByteToEigen(robo_supplement.geo_center,
                                                  data + offset, len,
                                                  isSrcEndianTypeMatchHostEndian);
            offset += len;

            // geo_size
            RsNativeCommon::fromNativeByteToEigen(robo_supplement.geo_size,
                                                  data + offset, len,
                                                  isSrcEndianTypeMatchHostEndian);
            offset += len;

            // trajectory
            int trajectorySize = 0;
            int32Endian.toHostEndianValue(trajectorySize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            if (trajectorySize > 0) {
                for (auto iter = 0; iter < trajectorySize; ++iter) {
                    // Eigen::Vector3f trajectory;
                    RsVector3f trajectory;
                    RsNativeCommon::fromNativeByteToEigen(trajectory, data + offset, len,
                                                          isSrcEndianTypeMatchHostEndian);
                    offset += len;
                    robo_supplement.trajectory.push_back(trajectory);
                }
            }

            // history_velocity
            int historyVelocitySize = 0;
            int32Endian.toHostEndianValue(historyVelocitySize, data + offset,
                                          int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            if (historyVelocitySize > 0) {
                for (auto iter = 0; iter < historyVelocitySize; ++iter) {
                    // Eigen::Vector3f velocity;
                    RsVector3f velocity;
                    RsNativeCommon::fromNativeByteToEigen(velocity, data + offset, len,
                                                          isSrcEndianTypeMatchHostEndian);
                    offset += len;
                    robo_supplement.history_velocity.push_back(velocity);
                }
            }

            // history_type
            int historyTypeSize = 0;
            int32Endian.toHostEndianValue(historyTypeSize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            if (historyTypeSize > 0) {
                for (auto iter = 0; iter < historyTypeSize; ++iter) {
                    int objectType;
                    int32Endian.toHostEndianValue(
                    objectType, data + offset, int32Size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    offset += int32Size;
                    robo_supplement.history_type.push_back(
                    static_cast<ObjectType>(objectType));
                }
            }

            // gps_info
            int gps_mode;
            int32Endian.toHostEndianValue(gps_mode, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_supplement.gps_mode = static_cast<GpsType>(gps_mode);
            offset += int32Size;

            doubleEndian.toHostEndianValue(
            robo_supplement.gps_longtitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toHostEndianValue(
            robo_supplement.gps_latitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toHostEndianValue(
            robo_supplement.gps_altitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }

        // std::cout << "run here 6: offset = " << offset << "==> maxLen = " <<
        // maxLen << std::endl;
        return 0;
    }

    static int serialize(const RsFreeSpace::Ptr &robo_freespace_ptr, char *data) {
        int freespacesCnt = robo_freespace_ptr->fs_pts.size();
        if (freespacesCnt == 0) {
            return 0;
        }

        // std::cout << "freespace cnt = " << freespacesCnt << std::endl;

        int floatSize = sizeof(float);
        int intSize = sizeof(int);

        RSEndian<float> floatEndian;
        RSEndian<int> intEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int offset = 0;
        // free_spaces
        if (isDstEndianTypeMatchHostEndian) {
            for (int i = 0; i < freespacesCnt; ++i) {
                const RsVector3f &robo_freespace = robo_freespace_ptr->fs_pts[i];

                memcpy(data + offset, &(robo_freespace.x), floatSize);
                offset += floatSize;

                memcpy(data + offset, &(robo_freespace.y), floatSize);
                offset += floatSize;

                memcpy(data + offset, &(robo_freespace.z), floatSize);
                offset += floatSize;
            }
        } else {
            for (int i = 0; i < freespacesCnt; ++i) {
                int len = 0;
                RsNativeCommon::fromNativeByteToEigen(robo_freespace_ptr->fs_pts[i],
                                                      data + offset, len,
                                                      isDstEndianTypeMatchHostEndian);
                offset += len;
            }
        }

        // int status = static_cast<int>(robo_freespace_ptr->status);
        // intEndian.toTargetEndianArray(status, data + offset, intSize,
        //                               RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        // offset += intSize;

        memset(data + offset, 0, intSize);
        offset += intSize;

        return offset;
    }

    static int deserialize(const char *data, const int maxLen,
                           RsFreeSpace::Ptr &robo_freespace_ptr) {
        if (robo_freespace_ptr == nullptr) {
            robo_freespace_ptr.reset(new RsFreeSpace());
        }
        robo_freespace_ptr->fs_pts.clear();

        int floatSize = sizeof(float);
        int intSize = sizeof(int);
        int basicSize = floatSize * 3;

        if ((maxLen - intSize) % basicSize != 0 || maxLen < 0) {
            return -1;
        }

        int freespaceCnt = maxLen / basicSize;

        // std::cout << __FUNCTION__ << "=> A freespace cnt = " << freespaceCnt <<
        // std::endl;

        // freespaces
        RSEndian<float> floatEndian;
        RSEndian<int> intEndian;
        bool isSrcEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int offset = 0;
        RsVector3f freespaceInfo;
        if (isSrcEndianTypeMatchHostEndian) {
            for (int i = 0; i < freespaceCnt; ++i) {
                memcpy(&(freespaceInfo.x), data + offset, floatSize);
                offset += floatSize;

                memcpy(&(freespaceInfo.y), data + offset, floatSize);
                offset += floatSize;

                memcpy(&(freespaceInfo.z), data + offset, floatSize);
                offset += floatSize;

                robo_freespace_ptr->fs_pts.push_back(freespaceInfo);
            }
        } else {
            for (int i = 0; i < freespaceCnt; ++i) {
                int len = 0;
                RsNativeCommon::fromNativeByteToEigen(freespaceInfo, data + offset, len,
                                                      isSrcEndianTypeMatchHostEndian);
                offset += len;

                robo_freespace_ptr->fs_pts.push_back(freespaceInfo);
            }
        }

        // int status;
        // memcpy(&status, data + offset, intSize);
        // offset += intSize;
        // robo_freespace_ptr->status = static_cast<AxisStatus>(status);

        offset += intSize;  // 填充: status

        // std::cout << __FUNCTION__ << "=> B freespace cnt = " << freespaceCnt <<
        // std::endl;
        return 0;
    }

    static int serialize(const std::vector<Roadedge::Ptr> &curbs, char *data) {
        int curbsCnt = curbs.size();
        if (curbsCnt == 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // curbs
        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            for (int i = 0; i < curbsCnt; ++i) {
                const Roadedge::Ptr &robo_curb_ptr = curbs[i];

                // // status
                // int status = static_cast<int>(robo_curb_ptr->status);
                // memcpy(data + offset, &status, int32Size);
                // offset += int32Size;

                memset(data + offset, 0, int32Size);  // 填充 status
                offset += int32Size;

                // curb_id
                int curb_id = static_cast<int>(robo_curb_ptr->roadedge_id);
                memcpy(data + offset, &curb_id, int32Size);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_curb_ptr->curve;
                memcpy(data + offset, &(robo_curve.x_start), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.x_end), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.a), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.b), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.c), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.d), floatSize);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_curb_ptr->end_point;

                memcpy(data + offset, &(robo_endpoints.start.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.start.y), floatSize);
                offset += floatSize;

                memcpy(data + offset, &(robo_endpoints.end.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.end.y), floatSize);
                offset += floatSize;

                // measure_status
                int measure_status = static_cast<int>(robo_curb_ptr->measure_status);
                memcpy(data + offset, &measure_status, int32Size);
                offset += int32Size;

                // confidence
                memcpy(data + offset, &(robo_curb_ptr->confidence), floatSize);
                offset += floatSize;
            }
        } else {
            for (int i = 0; i < curbsCnt; ++i) {
                const Roadedge::Ptr &robo_curb_ptr = curbs[i];

                // // status
                // int status = static_cast<int>(robo_curb_ptr->status);
                // int32Endian.toTargetEndianArray(
                //     status, data + offset, int32Size,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += int32Size;

                memset(data + offset, 0, int32Size);  // 填充: status
                offset += int32Size;

                // curb_id
                int curb_id = static_cast<int>(robo_curb_ptr->roadedge_id);
                int32Endian.toTargetEndianArray(
                curb_id, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_curb_ptr->curve;
                floatEndian.toTargetEndianArray(
                robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_curb_ptr->end_point;

                floatEndian.toTargetEndianArray(
                robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(
                robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // measure_status
                int measure_status = static_cast<int>(robo_curb_ptr->measure_status);
                int32Endian.toTargetEndianArray(
                measure_status, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // confidence
                floatEndian.toTargetEndianArray(
                robo_curb_ptr->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
            }
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen,
                           std::vector<Roadedge::Ptr> &curbs) {
        curbs.clear();
        if (maxLen < 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isScrEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // curbs
        int offset = 0;

        if (isScrEndianTypeMatchHostEndian) {
            while (offset < maxLen) {
                Roadedge::Ptr curb(new Roadedge());

                // // status
                // int status = 0;
                // memcpy(&status, data + offset, int32Size);
                // curb->status = static_cast<AxisStatus>(status);
                // offset += int32Size;

                offset += int32Size;  // 填充: status

                // track_id
                int roadedge_id = 0;
                memcpy(&roadedge_id, data + offset, int32Size);
                curb->roadedge_id = static_cast<RoadedgePosition>(roadedge_id);
                offset += int32Size;

                // curve
                Curve &robo_curve = curb->curve;
                memcpy(&(robo_curve.x_start), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.x_end), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.a), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.b), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.c), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.d), data + offset, floatSize);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = curb->end_point;

                memcpy(&(robo_endpoints.start.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.start.y), data + offset, floatSize);
                offset += floatSize;

                memcpy(&(robo_endpoints.end.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.end.y), data + offset, floatSize);
                offset += floatSize;

                // measure_status
                int measure_status;
                memcpy(&measure_status, data + offset, int32Size);
                curb->measure_status = static_cast<MeasureStatus>(measure_status);
                offset += int32Size;

                // confidence
                memcpy(&(curb->confidence), data + offset, floatSize);
                offset += floatSize;

                curbs.push_back(curb);
            }
        } else {
            while (offset < maxLen) {
                Roadedge::Ptr curb(new Roadedge());
                // // status
                // int status;
                // int32Endian.toHostEndianValue(
                //     status, data + offset, int32Size,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // curb->status = static_cast<AxisStatus>(status);
                // offset += int32Size;

                offset += int32Size;  // 填充: status

                // roadedge_id
                int roadedge_id;
                int32Endian.toHostEndianValue(
                roadedge_id, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                curb->roadedge_id = static_cast<RoadedgePosition>(roadedge_id);
                offset += int32Size;

                // curve
                Curve &robo_curve = curb->curve;
                floatEndian.toHostEndianValue(
                robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = curb->end_point;

                floatEndian.toHostEndianValue(
                robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(
                robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // measure_status
                int measure_status;
                int32Endian.toHostEndianValue(
                measure_status, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                curb->measure_status = static_cast<MeasureStatus>(measure_status);
                offset += int32Size;

                // confidence
                floatEndian.toHostEndianValue(
                curb->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                curbs.push_back(curb);
            }
        }

        return 0;
    }

    static int serialize(const std::vector<Lane::Ptr> &lanes, char *data) {
        int lanesCnt = lanes.size();

        if (lanesCnt == 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // lanes
        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            for (int i = 0; i < lanesCnt; ++i) {
                const Lane::Ptr &robo_lane_ptr = lanes[i];

                // // status
                // int status = static_cast<int>(robo_lane_ptr->status);
                // memcpy(data + offset, &status, int32Size);
                // offset += int32Size;

                memset(data + offset, 0, int32Size);  // 填充: status
                offset += int32Size;

                // lane_id
                int lane_id = static_cast<int>(robo_lane_ptr->lane_id);
                memcpy(data + offset, &lane_id, int32Size);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_lane_ptr->curve;
                memcpy(data + offset, &(robo_curve.x_start), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.x_end), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.a), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.b), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.c), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.d), floatSize);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_lane_ptr->end_point;

                memcpy(data + offset, &(robo_endpoints.start.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.start.y), floatSize);
                offset += floatSize;

                memcpy(data + offset, &(robo_endpoints.end.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.end.y), floatSize);
                offset += floatSize;

                // measure_status
                int measure_status = static_cast<int>(robo_lane_ptr->measure_status);
                memcpy(data + offset, &measure_status, int32Size);
                offset += int32Size;

                // confidence
                memcpy(data + offset, &(robo_lane_ptr->confidence), floatSize);
                offset += floatSize;
            }
        } else {
            for (int i = 0; i < lanesCnt; ++i) {
                const Lane::Ptr &robo_lane_ptr = lanes[i];

                // // status
                // int status = static_cast<int>(robo_lane_ptr->status);
                // int32Endian.toTargetEndianArray(
                //     status, data + offset, int32Size,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // offset += int32Size;

                memset(data + offset, 0, int32Size);  // 填充: status
                offset += int32Size;

                // lane_id
                int lane_id = static_cast<int>(robo_lane_ptr->lane_id);
                int32Endian.toTargetEndianArray(
                lane_id, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_lane_ptr->curve;
                floatEndian.toTargetEndianArray(
                robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_lane_ptr->end_point;

                floatEndian.toTargetEndianArray(
                robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(
                robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // measure_status
                int measure_status = static_cast<int>(robo_lane_ptr->measure_status);
                int32Endian.toTargetEndianArray(
                measure_status, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // confidence
                floatEndian.toTargetEndianArray(
                robo_lane_ptr->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
            }
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen,
                           std::vector<Lane::Ptr> &lanes) {
        lanes.clear();
        if (maxLen < 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isScrEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // lanes
        int offset = 0;

        if (isScrEndianTypeMatchHostEndian) {
            while (offset < maxLen) {
                Lane::Ptr lane(new Lane());

                // // status
                // int status = 0;
                // memcpy(&status, data + offset, int32Size);
                // lane->status = static_cast<AxisStatus>(status);
                // offset += int32Size;

                offset += int32Size;  // 填充: status

                // lane_id
                int lane_id = 0;
                memcpy(&lane_id, data + offset, int32Size);
                lane->lane_id = static_cast<LanePosition>(lane_id);
                offset += int32Size;

                // curve
                Curve &robo_curve = lane->curve;
                memcpy(&(robo_curve.x_start), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.x_end), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.a), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.b), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.c), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.d), data + offset, floatSize);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = lane->end_point;

                memcpy(&(robo_endpoints.start.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.start.y), data + offset, floatSize);
                offset += floatSize;

                memcpy(&(robo_endpoints.end.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.end.y), data + offset, floatSize);
                offset += floatSize;

                // measure_status
                int measure_status = 0;
                memcpy(&measure_status, data + offset, int32Size);
                lane->measure_status = static_cast<MeasureStatus>(measure_status);
                offset += int32Size;

                // confidence
                memcpy(&(lane->confidence), data + offset, floatSize);
                offset += floatSize;

                lanes.push_back(lane);
            }
        } else {
            while (offset < maxLen) {
                Lane::Ptr lane(new Lane());

                // // status
                // int status;
                // int32Endian.toHostEndianValue(
                //     status, data + offset, int32Size,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // lane->status = static_cast<AxisStatus>(status);
                // offset += int32Size;

                offset += int32Size;  // 填充: status

                // lane_id
                int lane_id;
                int32Endian.toHostEndianValue(
                lane_id, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                lane->lane_id = static_cast<LanePosition>(lane_id);
                offset += int32Size;

                // curve
                Curve &robo_curve = lane->curve;
                floatEndian.toHostEndianValue(
                robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = lane->end_point;

                floatEndian.toHostEndianValue(
                robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(
                robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // measure_status
                int measure_status;
                int32Endian.toHostEndianValue(
                measure_status, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                lane->measure_status = static_cast<MeasureStatus>(measure_status);
                offset += int32Size;

                // confidence
                floatEndian.toHostEndianValue(
                lane->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                lanes.push_back(lane);
            }
        }

        return 0;
    }

    static int serialize(const std::map<AxisStatus, RsPose::Ptr> &pose_map,
                         char *data) {
        int offset = 0;
        for (auto iterMap = pose_map.begin(); iterMap != pose_map.end();
             ++iterMap) {
            int pose_size = serialize(iterMap->first, iterMap->second, data + offset);
            offset += pose_size;
        }

        return offset;
    }

    static int serialize(const AxisStatus poseStatus, const RsPose::Ptr &pose,
                         char *data) {
        int floatSize = sizeof(float);
        int intSize = sizeof(int);

        RSEndian<float> floatEndian;
        RSEndian<int> intEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            memcpy(data + offset, &(pose->x), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->y), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->z), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->roll), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->pitch), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->yaw), floatSize);
            offset += floatSize;

            int iStatus = static_cast<int>(poseStatus);
            memcpy(data + offset, &iStatus, intSize);
            offset += intSize;
        } else {
            floatEndian.toTargetEndianArray(
            pose->x, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(
            pose->y, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(
            pose->z, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(
            pose->roll, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(
            pose->pitch, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(
            pose->yaw, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            int iStatus = static_cast<int>(poseStatus);
            intEndian.toTargetEndianArray(iStatus, data + offset, intSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += intSize;
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen, std::map<AxisStatus, RsPose::Ptr> &pose_map) {
        if (!pose_map.empty()) {
            pose_map.clear();
        }
        for (int offset = 0; offset < maxLen && maxLen - offset >= 28;
             offset += 28) {
            AxisStatus status;
            RsPose::Ptr pose;
            deserialize(data + offset, maxLen - offset, status, pose);

            pose_map.insert(std::pair<AxisStatus, RsPose::Ptr>(status, pose));
        }

        return 0;
    }

    static int deserialize(const char *data, const int maxLen, AxisStatus &status,
                           RsPose::Ptr &pose) {
        if (maxLen < 24) {
            return 0;
        }

        pose.reset(new RsPose());

        const int floatSize = sizeof(float);
        const int intSize = sizeof(int);

        RSEndian<float> floatEndian;
        bool isScrEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int offset = 0;
        if (isScrEndianTypeMatchHostEndian) {
            memcpy(&(pose->x), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->y), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->z), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->roll), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->pitch), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->yaw), data + offset, floatSize);
            offset += floatSize;

            int iStatus;
            memcpy(&iStatus, data + offset, floatSize);
            status = static_cast<AxisStatus>(iStatus);
        } else {
            floatEndian.toHostEndianValue(pose->x, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->y, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->z, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->roll, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->pitch, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->yaw, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            int iStatus;
            RSEndian<int> int32Endian;
            int32Endian.toHostEndianValue(iStatus, data + offset, intSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += intSize;
            status = static_cast<AxisStatus>(iStatus);
        }
        return 0;
    }

    static int serialize(const AxisStatus status, char *data) {
        int intStatus = static_cast<int>(status);

        RSEndian<int> int32Endian;

        int offset = 0;
        int32Endian.toTargetEndianArray(intStatus, data + offset, sizeof(int),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(int);

        return offset;
    }

    static int deserialize(const char *data, const int maxLen,
                           AxisStatus &status) {
        int intStatus = 0;

        RSEndian<int> int32Endian;

        int32Endian.toHostEndianValue(intStatus, data, maxLen,
                                      RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        status = static_cast<AxisStatus>(intStatus);

        return 0;
    }
};

class RSNativeDeserializeUtil {
public:
    using Ptr = std::shared_ptr<RSNativeDeserializeUtil>;
    using ConstPtr = std::shared_ptr<const RSNativeDeserializeUtil>;

public:
    bool isEmptyMsgType(const ROBO_MSG_TYPE msg_type) {
        if (msg_type == ROBO_MSG_TYPE::ROBO_MSG_OBJECT_EMPTY
        || msg_type == ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT_EMPTY
        || msg_type == ROBO_MSG_TYPE::ROBO_MSG_AXISSTATUS_EMPTY
        || msg_type == ROBO_MSG_TYPE::ROBO_MSG_FREESPACE_EMPTY
        || msg_type == ROBO_MSG_TYPE::ROBO_MSG_LANE_EMPTY
        || msg_type == ROBO_MSG_TYPE::ROBO_MSG_CURB_EMPTY
        || msg_type == ROBO_MSG_TYPE::ROBO_MSG_AXISLIDAR_POSE_EMPTY
        || msg_type == ROBO_MSG_TYPE::ROBO_MSG_GLOBALCAR_POSE_EMPTY
        || msg_type == ROBO_MSG_TYPE::ROBO_MSG_POINT_EMPTY) {
            return true;
        }

        return false;
    }

    bool checkDeserializeStatus(const st_RoboRecvMessage &recv_msg, uint16_t msg_type_) {
        // 发的不是空消息，但是不解析，返回true
        // 发的不是空消息，且要解析，返回false
        native_sdk_3_0::ROBO_MSG_TYPE msg_type = static_cast<native_sdk_3_0::ROBO_MSG_TYPE>(msg_type_);
        const auto& status_map = recv_msg.status;
        if (status_map.at(msg_type) == native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_NO_TRANSLATE) {
            return true;
        }
        else {
            return false;
        }
    }

    bool checkMessageComplete(const RsCustomNativeBytesSDK3_0MsgParams::Ptr& custom_params_,
                              st_RoboRecvMessage & recv_msg) {
        // Object Incomplete
        if (custom_params_->send_objects &&
        recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_OBJECT] != native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_EMPTY
        && recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_OBJECT] != ROBO_MSG_RECV_STATUS::ROBO_MSG_SUCCESS) {
//            std::cout << "1 Incomplete" << std::endl;
            return false;
        }

        // Attention Object Incomplete
        if (custom_params_->send_attention_objects &&
        recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT] != native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_EMPTY
        && recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT] != ROBO_MSG_RECV_STATUS::ROBO_MSG_SUCCESS) {
//            std::cout << "2 Incomplete" << std::endl;
            return false;
        }

        // Axis status Incomplete
        if (custom_params_->send_axisstatus &&
        recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_AXISSTATUS] != native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_EMPTY
        && recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_AXISSTATUS] != ROBO_MSG_RECV_STATUS::ROBO_MSG_SUCCESS) {
//            std::cout << "3 Incomplete" << std::endl;
            return false;
        }

        // Freespace Incomplete
        if (custom_params_->send_freespace &&
        recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_FREESPACE] != native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_EMPTY
        && recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_FREESPACE] != ROBO_MSG_RECV_STATUS::ROBO_MSG_SUCCESS) {
//            std::cout << "4 Incomplete" << std::endl;
            return false;
        }

        // Lanes Incomplete
        if (custom_params_->send_lanes &&
        recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_LANE] != native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_EMPTY
        && recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_LANE] != ROBO_MSG_RECV_STATUS::ROBO_MSG_SUCCESS) {
//            std::cout << "5 Incomplete" << std::endl;
            return false;
        }

        // Curbs Incomplete
        if (custom_params_->send_curbs &&
        recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_CURB] != native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_EMPTY
        && recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_CURB] != ROBO_MSG_RECV_STATUS::ROBO_MSG_SUCCESS) {
//            std::cout << "6 Incomplete" << std::endl;
            return false;
        }

        // Axis Lidar Pose Incomplete
        if (custom_params_->send_axis_lidar_pose &&
        recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_AXISLIDAR_POSE] != native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_EMPTY
        && recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_AXISLIDAR_POSE] != ROBO_MSG_RECV_STATUS::ROBO_MSG_SUCCESS) {
//            std::cout << "7 Incomplete" << std::endl;
            return false;
        }

        // Global Car Pose Incomplete
        if (custom_params_->send_global_car_pose &&
        recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_GLOBALCAR_POSE] != native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_EMPTY
        && recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_GLOBALCAR_POSE] != ROBO_MSG_RECV_STATUS::ROBO_MSG_SUCCESS) {
//            std::cout << "8 Incomplete" << std::endl;
            return false;
        }

        // PointCloud Incomplete
        if (custom_params_->send_pointcloud &&
        recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_POINT] != native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_EMPTY
        && recv_msg.status[ROBO_MSG_TYPE::ROBO_MSG_POINT] != ROBO_MSG_RECV_STATUS::ROBO_MSG_SUCCESS) {
//            std::cout << "9 Incomplete" << std::endl;
            return false;
        }

        return true;
    }
};

}  // namespace native_sdk_3_0

namespace native_sdk_3_1 {
struct st_RoboRecvMessage {
public:
    int device_id;
    double timestamp;
    std::map<ROBO_DATA_TYPE, st_RoboMsgRecorder> recorder;
    std::map<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS> status;
    RsPerceptionMsg::Ptr msg;

public:
    st_RoboRecvMessage() {
        device_id = 0;
        timestamp = 0;
    };

public:
    // #ifdef CUSTOM_GE_COMM_FOUND
    //     void updateStatusByConfig(const st_CustomMessageConfig
    //     &fordMessageConfig);
    // #endif // CUSTOM_GE_COMM_FOUND

    // #ifdef CUSTOM_CR_COMM_FOUND
    //     void updateStatusByConfig(const st_CustomMessageConfig
    //     &beiqiMessageConfig);
    // #endif // CUSTOM_CR_COMM_FOUND

    void updateStatusByConfig(const RsCommonBytesCustomMsgParams &translateConfig) {
        status.clear();
        recorder.clear();

        // Message Receive Recorder
        st_RoboMsgRecorder msgRecorder;

        // timestamp Message Info
        if (translateConfig.send_timestamp) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_TIMESTAMP,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_TIMESTAMP,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_TIMESTAMP, msgRecorder));

        // gps_origin Message Info
        if (translateConfig.send_gps_origin) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_GPS_ORIGIN,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_GPS_ORIGIN,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_GPS_ORIGIN, msgRecorder));

        // valid_indices Message Info
        if (translateConfig.send_valid_indices) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_VALID_INDICES,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_VALID_INDICES,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_VALID_INDICES, msgRecorder));

        // Axis Status Object Message Info
        if (translateConfig.send_status) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_AXISSTATUS,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_AXISSTATUS,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_AXISSTATUS, msgRecorder));

        // Object Message Info
        if (translateConfig.send_object) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_OBJECT, ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_OBJECT,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_OBJECT, msgRecorder));

        // Attention Object Message Info
        if (translateConfig.send_attention_objects) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT, msgRecorder));

        // Freespace Message Info
        if (translateConfig.send_freespace) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_FREESPACE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_FREESPACE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_FREESPACE, msgRecorder));

        // Lane Message Info
        if (translateConfig.send_lane) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_LANE, ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_LANE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_LANE, msgRecorder));

        // roadedge Message Info
        if (translateConfig.send_roadedge) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_CURB, ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_CURB,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_CURB, msgRecorder));

        // Axis Lidar Pose Message Info
        if (translateConfig.send_status_pose_map) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_AXISLIDAR_POSE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_AXISLIDAR_POSE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_AXISLIDAR_POSE, msgRecorder));

        // non_ground_indices Message Info
        if (translateConfig.send_non_ground_indices) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_NON_GD_IDX, ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_NON_GD_IDX,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_NON_GD_IDX, msgRecorder));

        // ground_indices Message Info
        if (translateConfig.send_ground_indices) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_GD_IDX, ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_GD_IDX,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_GD_IDX, msgRecorder));

        // background_indices Message Info
        if (translateConfig.send_background_indices) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_BG_IDX, ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_BG_IDX,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_BG_IDX, msgRecorder));

        // PointCloud Message Info
        if (translateConfig.send_point_cloud) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_POINT, ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_POINT,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_POINT, msgRecorder));

        // Global Case Pose Message Info
        if (translateConfig.send_global_pose) {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_GLOBALCAR_POSE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_DATA_TYPE::ROBO_MSG_GLOBALCAR_POSE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_DATA_TYPE::ROBO_MSG_GLOBALCAR_POSE, msgRecorder));
    }
};

class RSNativeSerializeUtil {
public:
    using Ptr = std::shared_ptr<RSNativeSerializeUtil>;
    using ConstPtr = std::shared_ptr<const RSNativeSerializeUtil>;

public:
    static int serialize(const Object::Ptr &object, bool isSupplement, char *data) {
        const CoreInfos &robo_core = object->core_infos_;

        RSEndian<float> floatEndian;
        RSEndian<int> int32Endian;
        RSEndian<unsigned int> uint32Endian;
        RSEndian<double> doubleEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int int32Size = sizeof(int);
        // int uint32Size = sizeof(unsigned int);
        int floatSize = sizeof(float);
        int doubleSize = sizeof(double);

        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            // timestamp
            memcpy(data + offset, &(robo_core.timestamp), doubleSize);
            offset += doubleSize;

            // priority_id
            memcpy(data + offset, &(robo_core.priority_id), int32Size);
            offset += int32Size;

            // exist_confidence
            memcpy(data + offset, &(robo_core.exist_confidence), floatSize);
            offset += floatSize;
        } else {
            // timestamp
            doubleEndian.toTargetEndianArray(
            robo_core.timestamp, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            // priority_id
            int32Endian.toTargetEndianArray(
            robo_core.priority_id, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // exist_confidence
            floatEndian.toTargetEndianArray(
            robo_core.exist_confidence, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }

        // center
        int len = 0;
        RsNativeCommon::fromEigenToNativeByte(robo_core.center, data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // center_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.center_cov, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // size
        RsNativeCommon::fromEigenToNativeByte(robo_core.size, data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // size_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.size_cov, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // direction
        RsNativeCommon::fromEigenToNativeByte(robo_core.direction, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // direction_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.direction_cov,
                                              data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        if (isDstEndianTypeMatchHostEndian) {
            // object type
            int objectType = static_cast<int>(robo_core.type);
            memcpy(data + offset, &objectType, int32Size);
            offset += int32Size;

            // object type confidence
            memcpy(data + offset, &(robo_core.type_confidence), floatSize);
            offset += floatSize;

            // attentions_type
            int attType = static_cast<int>(robo_core.attention_type);
            memcpy(data + offset, &attType, int32Size);
            offset += int32Size;

            // motion_state
            int motionState = static_cast<int>(robo_core.motion_state);
            memcpy(data + offset, &motionState, int32Size);
            offset += int32Size;

            // lane_pos
            int lane_pos = static_cast<int>(robo_core.lane_pos);
            memcpy(data + offset, &lane_pos, int32Size);
            offset += int32Size;

            // tracker_id
            memcpy(data + offset, &(robo_core.tracker_id), int32Size);
            offset += int32Size;

            // age
            memcpy(data + offset, &(robo_core.age), doubleSize);
            offset += doubleSize;
        } else {
            // object type
            int objectType = static_cast<int>(robo_core.type);
            int32Endian.toTargetEndianArray(
            objectType, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // object type confidence
            floatEndian.toTargetEndianArray(
            robo_core.type_confidence, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // attentions_type
            int attType = static_cast<int>(robo_core.attention_type);
            int32Endian.toTargetEndianArray(
            attType, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // motion_state
            int motionState = static_cast<int>(robo_core.motion_state);
            int32Endian.toTargetEndianArray(
            motionState, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // lane_pos
            int lane_pos = static_cast<int>(robo_core.lane_pos);
            int32Endian.toTargetEndianArray(
            lane_pos, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // tracker_id
            int32Endian.toTargetEndianArray(
            robo_core.tracker_id, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // age
            doubleEndian.toTargetEndianArray(
            robo_core.age, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }

        // velocity
        RsNativeCommon::fromEigenToNativeByte(robo_core.velocity, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // relative_velocity
        RsNativeCommon::fromEigenToNativeByte(robo_core.relative_velocity, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // velocity_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.velocity_cov, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // relative_velocity_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.relative_velocity_cov, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // acceleration
        RsNativeCommon::fromEigenToNativeByte(robo_core.acceleration, data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        // acceleration_cov
        RsNativeCommon::fromEigenToNativeByte(robo_core.acceleration_cov,data + offset,
                                              len, isDstEndianTypeMatchHostEndian);
        offset += len;

        if (isDstEndianTypeMatchHostEndian) {
            // angle_velocity
            memcpy(data + offset, &(robo_core.angle_velocity), floatSize);
            offset += floatSize;

            // angle_velocity_cov
            memcpy(data + offset, &(robo_core.angle_velocity_cov), floatSize);
            offset += floatSize;

            // angle_acceleration
            memcpy(data + offset, &(robo_core.angle_acceleration), floatSize);
            offset += floatSize;

            // angle_acceleration_cov
            memcpy(data + offset, &(robo_core.angle_acceleration_cov), floatSize);
            offset += floatSize;
        } else {
            // angle_velocity
            floatEndian.toTargetEndianArray(
            robo_core.angle_velocity, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_velocity_cov
            floatEndian.toTargetEndianArray(
            robo_core.angle_velocity_cov, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_acceleration
            floatEndian.toTargetEndianArray(
            robo_core.angle_acceleration, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_acceleration_cov
            floatEndian.toTargetEndianArray(
            robo_core.angle_acceleration_cov, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }

        // anchor
        RsNativeCommon::fromEigenToNativeByte(robo_core.anchor, data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        // nearest_point
        RsNativeCommon::fromEigenToNativeByte(robo_core.nearest_point,data + offset, len,
                                              isDstEndianTypeMatchHostEndian);
        offset += len;

        memcpy(data + offset, &isSupplement, sizeof(bool));
        offset += sizeof(bool);

        // Protobuf Serialize Supplement
        if (isSupplement) {
            // std::cout << "isSupplement = " << isSupplement << std::endl;
            const SupplementInfos &robo_supplement = object->supplement_infos_;

            if (isDstEndianTypeMatchHostEndian) {
                // unique_id
                unsigned int unique_id = robo_supplement.unique_id;
                memcpy(data + offset, &unique_id, int32Size);
                offset += int32Size;
            } else {
                // unique_id
                uint32Endian.toTargetEndianArray(
                robo_supplement.unique_id, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;
            }

            // polygon
            int polygonSize = static_cast<int>(robo_supplement.polygon.size());
            int32Endian.toTargetEndianArray(
            polygonSize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            for (int i = 0; i < polygonSize; ++i) {
                RsNativeCommon::fromEigenToNativeByte(robo_supplement.polygon[i],
                                                      data + offset, len,
                                                      isDstEndianTypeMatchHostEndian);
                offset += len;
            }

            if (isDstEndianTypeMatchHostEndian) {
                // left_point_index
                int left_point_index = robo_supplement.left_point_index;
                memcpy(data + offset, &left_point_index, int32Size);
                offset += int32Size;

                // right_point_index
                int right_point_index = robo_supplement.right_point_index;
                memcpy(data + offset, &right_point_index, int32Size);
                offset += int32Size;
            } else {
                // left_point_index
                uint32Endian.toTargetEndianArray(
                robo_supplement.left_point_index, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // right_point_index
                uint32Endian.toTargetEndianArray(
                robo_supplement.right_point_index, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;
            }

            // cloud_indices太大了，随点云一起发送

            // latent_types
            int latentTypesSize = static_cast<int>(robo_supplement.latent_types.size());
            int32Endian.toTargetEndianArray(
            latentTypesSize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            // RINFO << "latentTypesSize = " << latentTypesSize;
            for (int i = 0; i < latentTypesSize; ++i) {
                floatEndian.toTargetEndianArray(
                robo_supplement.latent_types[i], data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
            }

            // size_type
            int sizeType = static_cast<int>(robo_supplement.size_type);
            int32Endian.toTargetEndianArray(
            sizeType, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // mode
            int mode = static_cast<int>(robo_supplement.mode);
            int32Endian.toTargetEndianArray(
            mode, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // in_roi
            memcpy(data + offset, &(robo_supplement.in_roi), sizeof(bool));
            offset += sizeof(bool);

            // tarcking state
            int stability_type = static_cast<int>(robo_supplement.tracking_state);
            int32Endian.toTargetEndianArray(
            stability_type, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // geo_center
            RsNativeCommon::fromEigenToNativeByte(robo_supplement.geo_center,
                                                  data + offset, len,
                                                  isDstEndianTypeMatchHostEndian);
            offset += len;

            // geo_size
            RsNativeCommon::fromEigenToNativeByte(robo_supplement.geo_size,
                                                  data + offset, len,
                                                  isDstEndianTypeMatchHostEndian);
            offset += len;

            // trajectory
            int trajectorySize = static_cast<int>(robo_supplement.trajectory.size());
            int32Endian.toTargetEndianArray(
            trajectorySize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            for (int i = 0; i < trajectorySize; ++i) {
                RsNativeCommon::fromEigenToNativeByte(robo_supplement.trajectory[i],
                                                      data + offset, len,
                                                      isDstEndianTypeMatchHostEndian);
                offset += len;
            }

            // history_velocity
            int historyVelocitySize = static_cast<int>(robo_supplement.history_velocity.size());
            int32Endian.toTargetEndianArray(
            historyVelocitySize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            for (int i = 0; i < historyVelocitySize; ++i) {
                RsNativeCommon::fromEigenToNativeByte(
                robo_supplement.history_velocity[i], data + offset, len,
                isDstEndianTypeMatchHostEndian);
                offset += len;
            }

            // history_type
            int historyTypeSize = static_cast<int>(robo_supplement.history_type.size());
            int32Endian.toTargetEndianArray(
            historyTypeSize, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            for (int i = 0; i < historyTypeSize; ++i) {
                int objectType = static_cast<int>(robo_supplement.history_type[i]);
                int32Endian.toTargetEndianArray(
                objectType, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;
            }

            // gps_info
            int gps_mode = static_cast<int>(robo_supplement.gps_mode);
            int32Endian.toTargetEndianArray(
            gps_mode, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            doubleEndian.toTargetEndianArray(
            robo_supplement.gps_longtitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toTargetEndianArray(
            robo_supplement.gps_latitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toTargetEndianArray(
            robo_supplement.gps_altitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen, Object::Ptr &object) {
        if (maxLen < 0) {
            return -1;
        }

        object.reset(new Object());

        // CoreInfos
        CoreInfos &robo_core = object->core_infos_;
        RSEndian<float> floatEndian;
        RSEndian<int> int32Endian;
        RSEndian<unsigned int> uint32Endian;
        RSEndian<double> doubleEndian;
        bool isSrcEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int int32Size = sizeof(int);
        // int uint32Size = sizeof(unsigned int);
        int floatSize = sizeof(float);
        int doubleSize = sizeof(double);

        int offset = 0;
        if (isSrcEndianTypeMatchHostEndian) {
            // timestamp
            memcpy(&(robo_core.timestamp), data + offset, doubleSize);
            offset += doubleSize;

            // priority_id
            memcpy(&(robo_core.priority_id), data + offset, int32Size);
            offset += int32Size;

            // exist_confidence
            memcpy(&(robo_core.exist_confidence), data + offset, floatSize);
            offset += floatSize;

        } else {
            // timestamp

            doubleEndian.toHostEndianValue(
            robo_core.timestamp, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            // priority_id

            int32Endian.toHostEndianValue(robo_core.priority_id, data + offset,
                                          int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // exist_confidence
            floatEndian.toHostEndianValue(robo_core.exist_confidence, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }
        // std::cout << "run here 1" << std::endl;

        // center
        int len = 0;
        RsNativeCommon::fromNativeByteToEigen(robo_core.center, data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;
        // std::cout << "center = " << robo_core.center << std::endl;

        // center_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.center_cov, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // size
        RsNativeCommon::fromNativeByteToEigen(robo_core.size, data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;
        // std::cout << "size = " << robo_core.size << std::endl;

        // size_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.size_cov, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // direction
        RsNativeCommon::fromNativeByteToEigen(robo_core.direction, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;
        // std::cout << "direction = " << robo_core.direction << std::endl;

        // direction_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.direction_cov,
                                              data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        // std::cout << "run here 2" << std::endl;

        if (isSrcEndianTypeMatchHostEndian) {
            // object type
            int objectType = 0;
            memcpy(&objectType, data + offset, int32Size);
            robo_core.type = static_cast<ObjectType>(objectType);
            offset += int32Size;

            // object type confidence
            memcpy(&(robo_core.type_confidence), data + offset, floatSize);
            offset += floatSize;

            // attentions_type
            int attType = 0;
            memcpy(&attType, data + offset, int32Size);
            robo_core.attention_type = static_cast<AttentionType>(attType);
            offset += int32Size;

            // motion_state
            int motionState = 0;
            memcpy(&motionState, data + offset, int32Size);
            robo_core.motion_state = static_cast<MotionType>(motionState);
            offset += int32Size;

            // lane_poe
            int lane_pos = 0;
            memcpy(&lane_pos, data + offset, int32Size);
            robo_core.lane_pos = static_cast<LanePosition>(lane_pos);
            offset += int32Size;

            // tracker_id
            memcpy(&(robo_core.tracker_id), data + offset, int32Size);
            offset += int32Size;

            // age
            memcpy(&(robo_core.age), data + offset, doubleSize);
            offset += doubleSize;
        } else {
            // object type
            int objectType = 0;
            int32Endian.toHostEndianValue(objectType, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_core.type = static_cast<ObjectType>(objectType);
            offset += int32Size;

            // object type confidence
            floatEndian.toHostEndianValue(robo_core.type_confidence, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // attentions_type
            int attType = 0;
            int32Endian.toHostEndianValue(attType, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_core.attention_type = static_cast<AttentionType>(attType);
            offset += int32Size;

            // motion_state
            int motionState = 0;
            int32Endian.toHostEndianValue(motionState, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_core.motion_state = static_cast<MotionType>(motionState);
            offset += int32Size;

            // lane_pos
            int lane_pos = 0;
            int32Endian.toHostEndianValue(lane_pos, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            robo_core.lane_pos = static_cast<LanePosition>(lane_pos);
            offset += int32Size;

            // tracker_id
            int32Endian.toHostEndianValue(robo_core.tracker_id, data + offset,
                                          int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // age
            doubleEndian.toHostEndianValue(robo_core.age, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }
        // std::cout << "run here 3" << std::endl;

        // velocity
        RsNativeCommon::fromNativeByteToEigen(robo_core.velocity, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // relative_velocity
        RsNativeCommon::fromNativeByteToEigen(robo_core.relative_velocity, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // velocity_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.velocity_cov, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // relative_velocity_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.relative_velocity_cov, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // acceleration
        RsNativeCommon::fromNativeByteToEigen(robo_core.acceleration, data + offset,
                                              len, isSrcEndianTypeMatchHostEndian);
        offset += len;

        // acceleration_cov
        RsNativeCommon::fromNativeByteToEigen(robo_core.acceleration_cov,
                                              data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        if (isSrcEndianTypeMatchHostEndian) {
            // angle_velocity
            memcpy(&(robo_core.angle_velocity), data + offset, floatSize);
            offset += floatSize;

            // angle_velocity_cov
            memcpy(&(robo_core.angle_velocity_cov), data + offset, floatSize);
            offset += floatSize;

            // angle_acceleration
            memcpy(&(robo_core.angle_acceleration), data + offset, floatSize);
            offset += floatSize;

            // angle_acceleration_cov
            memcpy(&(robo_core.angle_acceleration_cov), data + offset, floatSize);
            offset += floatSize;
        } else {
            // angle_velocity
            floatEndian.toHostEndianValue(robo_core.angle_velocity, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_velocity_cov
            floatEndian.toHostEndianValue(robo_core.angle_velocity_cov, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_acceleration
            floatEndian.toHostEndianValue(robo_core.angle_acceleration, data + offset,
                                          floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            // angle_acceleration_cov
            floatEndian.toHostEndianValue(robo_core.angle_acceleration_cov,
                                          data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;
        }

        // anchor
        RsNativeCommon::fromNativeByteToEigen(robo_core.anchor, data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        // nearest_point
        RsNativeCommon::fromNativeByteToEigen(robo_core.nearest_point,
                                              data + offset, len,
                                              isSrcEndianTypeMatchHostEndian);
        offset += len;

        // std::cout << "run here 4" << std::endl;

        bool isSupplement = false;
        memcpy(&isSupplement, data + offset, sizeof(bool));
        offset += sizeof(bool);

        // Protobuf Serialize Supplement
        if (isSupplement) {
            // std::cout << "run here 5" << std::endl;

            SupplementInfos &robo_supplement = object->supplement_infos_;

            // unique_id
            uint32Endian.toHostEndianValue(
            robo_supplement.unique_id, data + offset, int32Size,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // polygon
            int polygonSize = 0;
            int32Endian.toHostEndianValue(polygonSize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            if (polygonSize > 0) {
                robo_supplement.polygon.resize(polygonSize);
                for (int i = 0; i < polygonSize; ++i) {
                    RsNativeCommon::fromNativeByteToEigen(robo_supplement.polygon[i],
                                                          data + offset, len,
                                                          isSrcEndianTypeMatchHostEndian);
                    offset += len;
                }
            }

            // left_point_index
            int32Endian.toHostEndianValue(robo_supplement.left_point_index,
                                          data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // right_point_index
            int32Endian.toHostEndianValue(robo_supplement.right_point_index,
                                          data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            // latent_types
            int latentTypesSize = 0;
            int32Endian.toHostEndianValue(latentTypesSize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            // RERROR << "latentTypesSize = " << latentTypesSize;
            if (latentTypesSize > 0) {
                robo_supplement.latent_types.resize(latentTypesSize);
                for (int i = 0; i < latentTypesSize; ++i) {
                    floatEndian.toHostEndianValue(
                    robo_supplement.latent_types[i], data + offset, floatSize,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    offset += floatSize;
                }
            } else {
                robo_supplement.latent_types.clear();
            }

            // size_type
            int sizeType = 0;
            int32Endian.toHostEndianValue(sizeType, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_supplement.size_type = static_cast<SizeType>(sizeType);
            offset += int32Size;

            // mode
            int mode = 0;
            int32Endian.toHostEndianValue(mode, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_supplement.mode = static_cast<ModeType>(mode);
            offset += int32Size;

            // in_roi
            memcpy(&(robo_supplement.in_roi), data + offset, sizeof(bool));
            offset += sizeof(bool);

            // tracking state
            int tracking_state = 0;
            int32Endian.toHostEndianValue(tracking_state, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_supplement.tracking_state =
            static_cast<TrackingState>(tracking_state);
            offset += int32Size;

            // geo_center
            RsNativeCommon::fromNativeByteToEigen(robo_supplement.geo_center,data + offset, len,
                                                  isSrcEndianTypeMatchHostEndian);
            offset += len;

            // geo_size
            RsNativeCommon::fromNativeByteToEigen(robo_supplement.geo_size,data + offset, len,
                                                  isSrcEndianTypeMatchHostEndian);
            offset += len;

            // trajectory
            int trajectorySize = 0;
            int32Endian.toHostEndianValue(trajectorySize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            if (trajectorySize > 0) {
                for (auto iter = 0; iter < trajectorySize; ++iter) {
                    // Eigen::Vector3f trajectory;
                    RsVector3f trajectory;
                    RsNativeCommon::fromNativeByteToEigen(trajectory, data + offset, len,
                                                          isSrcEndianTypeMatchHostEndian);
                    offset += len;
                    robo_supplement.trajectory.push_back(trajectory);
                }
            }

            // history_velocity
            int historyVelocitySize = 0;
            int32Endian.toHostEndianValue(historyVelocitySize, data + offset,
                                          int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;
            if (historyVelocitySize > 0) {
                for (auto iter = 0; iter < historyVelocitySize; ++iter) {
                    // Eigen::Vector3f velocity;
                    RsVector3f velocity;
                    RsNativeCommon::fromNativeByteToEigen(velocity, data + offset, len,
                                                          isSrcEndianTypeMatchHostEndian);
                    offset += len;
                    robo_supplement.history_velocity.push_back(velocity);
                }
            }

            // history_type
            int historyTypeSize = 0;
            int32Endian.toHostEndianValue(historyTypeSize, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += int32Size;

            if (historyTypeSize > 0) {
                for (auto iter = 0; iter < historyTypeSize; ++iter) {
                    int objectType = 0;
                    int32Endian.toHostEndianValue(objectType, data + offset, int32Size,
                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                    offset += int32Size;
                    robo_supplement.history_type.push_back(static_cast<ObjectType>(objectType));
                }
            }

            // gps_info
            int gps_mode = 0;
            int32Endian.toHostEndianValue(gps_mode, data + offset, int32Size,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            robo_supplement.gps_mode = static_cast<GpsType>(gps_mode);
            offset += int32Size;

            doubleEndian.toHostEndianValue(robo_supplement.gps_longtitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toHostEndianValue(robo_supplement.gps_latitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;

            doubleEndian.toHostEndianValue(robo_supplement.gps_altitude, data + offset, doubleSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += doubleSize;
        }

        // std::cout << "run here 6: offset = " << offset << "==> maxLen = " <<
        // maxLen << std::endl;
        return 0;
    }

    static int serialize(const RsFreeSpace::Ptr &robo_freespace_ptr, char *data) {
        int freespacesCnt = static_cast<int>(robo_freespace_ptr->fs_pts.size());
        int fs_confidence_size = static_cast<int>(robo_freespace_ptr->fs_confidence.size());
        if (freespacesCnt == 0 || freespacesCnt != fs_confidence_size) {
            return -1;
        }

        // std::cout << "freespace cnt = " << freespacesCnt << std::endl;

        int floatSize = sizeof(float);

        RSEndian<float> floatEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int offset = 0;
        // free_spaces and fs_confidence
        if (isDstEndianTypeMatchHostEndian) {
            for (int i = 0; i < freespacesCnt; ++i) {
                const RsVector3f &robo_freespace = robo_freespace_ptr->fs_pts[i];

                memcpy(data + offset, &(robo_freespace.x), floatSize);
                offset += floatSize;

                memcpy(data + offset, &(robo_freespace.y), floatSize);
                offset += floatSize;

                memcpy(data + offset, &(robo_freespace.z), floatSize);
                offset += floatSize;

                memcpy(data + offset, &(robo_freespace_ptr->fs_confidence[i]), floatSize);
                offset += floatSize;
            }
        } else {
            for (int i = 0; i < freespacesCnt; ++i) {
                int len = 0;
                RsNativeCommon::fromNativeByteToEigen(robo_freespace_ptr->fs_pts[i],
                                                      data + offset, len,
                                                      isDstEndianTypeMatchHostEndian);
                offset += len;

                floatEndian.toTargetEndianArray(robo_freespace_ptr->fs_confidence[i],
                                                data + offset, floatSize,
                                                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
            }
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen, RsFreeSpace::Ptr &robo_freespace_ptr) {
        if (robo_freespace_ptr == nullptr) {
            robo_freespace_ptr.reset(new RsFreeSpace());
        }
        robo_freespace_ptr->fs_pts.clear();
        robo_freespace_ptr->fs_confidence.clear();
        robo_freespace_ptr->fs_types.clear();

        int floatSize = sizeof(float);
        int basicSize = floatSize * 4;

        if (maxLen % basicSize != 0 || maxLen < 0) {
            return -1;
        }

        int freespaceCnt = maxLen / basicSize;

        // std::cout << __FUNCTION__ << "=> A freespace cnt = " << freespaceCnt <<
        // std::endl;

        // freespaces
        RSEndian<float> floatEndian;
        RSEndian<int> intEndian;
        bool isSrcEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // fs_pts and fs_confidence
        int offset = 0;
        RsVector3f freespaceInfo;
        float confidence = 0.;
        if (isSrcEndianTypeMatchHostEndian) {
            for (int i = 0; i < freespaceCnt; ++i) {
                memcpy(&(freespaceInfo.x), data + offset, floatSize);
                offset += floatSize;

                memcpy(&(freespaceInfo.y), data + offset, floatSize);
                offset += floatSize;

                memcpy(&(freespaceInfo.z), data + offset, floatSize);
                offset += floatSize;

                robo_freespace_ptr->fs_pts.push_back(freespaceInfo);

                memcpy(&confidence, data + offset, floatSize);
                offset += floatSize;

                robo_freespace_ptr->fs_confidence.push_back(confidence);
            }
        } else {
            for (int i = 0; i < freespaceCnt; ++i) {
                int len = 0;
                RsNativeCommon::fromNativeByteToEigen(freespaceInfo, data + offset, len,
                                                      isSrcEndianTypeMatchHostEndian);
                offset += len;

                robo_freespace_ptr->fs_pts.push_back(freespaceInfo);

                floatEndian.toHostEndianValue(confidence, data + offset, floatSize,
                                              RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                robo_freespace_ptr->fs_confidence.push_back(confidence);
            }
        }
        return 0;
    }

    static int serialize(const std::vector<Roadedge::Ptr> &curbs, char *data) {
        int curbsCnt = static_cast<int>(curbs.size());
        if (curbsCnt == 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // curbs
        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            for (int i = 0; i < curbsCnt; ++i) {
                const Roadedge::Ptr &robo_curb_ptr = curbs[i];

                // curb_id
                int curb_id = static_cast<int>(robo_curb_ptr->roadedge_id);
                memcpy(data + offset, &curb_id, int32Size);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_curb_ptr->curve;
                memcpy(data + offset, &(robo_curve.x_start), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.x_end), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.a), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.b), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.c), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.d), floatSize);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_curb_ptr->end_point;

                memcpy(data + offset, &(robo_endpoints.start.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.start.y), floatSize);
                offset += floatSize;

                memcpy(data + offset, &(robo_endpoints.end.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.end.y), floatSize);
                offset += floatSize;

                // measure_status
                int measure_status = static_cast<int>(robo_curb_ptr->measure_status);
                memcpy(data + offset, &measure_status, int32Size);
                offset += int32Size;

                // confidence
                memcpy(data + offset, &(robo_curb_ptr->confidence), floatSize);
                offset += floatSize;
            }
        } else {
            for (int i = 0; i < curbsCnt; ++i) {
                const Roadedge::Ptr &robo_curb_ptr = curbs[i];

                // curb_id
                int curb_id = static_cast<int>(robo_curb_ptr->roadedge_id);
                int32Endian.toTargetEndianArray(
                curb_id, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_curb_ptr->curve;
                floatEndian.toTargetEndianArray(
                robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_curb_ptr->end_point;

                floatEndian.toTargetEndianArray(
                robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(
                robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toTargetEndianArray(
                robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // measure_status
                int measure_status = static_cast<int>(robo_curb_ptr->measure_status);
                int32Endian.toTargetEndianArray(
                measure_status, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // confidence
                floatEndian.toTargetEndianArray(
                robo_curb_ptr->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
            }
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen, std::vector<Roadedge::Ptr> &curbs) {
        curbs.clear();
        if (maxLen < 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isScrEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // curbs
        int offset = 0;

        if (isScrEndianTypeMatchHostEndian) {
            while (offset < maxLen) {
                Roadedge::Ptr curb(new Roadedge());

                // roadedge_id
                int roadedge_id = 0;
                memcpy(&roadedge_id, data + offset, int32Size);
                curb->roadedge_id = static_cast<RoadedgePosition>(roadedge_id);
                offset += int32Size;

                // curve
                Curve &robo_curve = curb->curve;
                memcpy(&(robo_curve.x_start), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.x_end), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.a), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.b), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.c), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.d), data + offset, floatSize);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = curb->end_point;

                memcpy(&(robo_endpoints.start.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.start.y), data + offset, floatSize);
                offset += floatSize;

                memcpy(&(robo_endpoints.end.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.end.y), data + offset, floatSize);
                offset += floatSize;

                // measure_status
                int measure_status = 0;
                memcpy(&measure_status, data + offset, int32Size);
                curb->measure_status = static_cast<MeasureStatus>(measure_status);
                offset += int32Size;

                // confidence
                memcpy(&(curb->confidence), data + offset, floatSize);
                offset += floatSize;

                curbs.push_back(curb);
            }
        } else {
            while (offset < maxLen) {
                Roadedge::Ptr curb(new Roadedge());

                // roadedge_id
                int roadedge_id = 0;
                int32Endian.toHostEndianValue(
                roadedge_id, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                curb->roadedge_id = static_cast<RoadedgePosition>(roadedge_id);
                offset += int32Size;

                // curve
                Curve &robo_curve = curb->curve;
                floatEndian.toHostEndianValue(robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = curb->end_point;

                floatEndian.toHostEndianValue(robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // measure_status
                int measure_status = 0;
                int32Endian.toHostEndianValue(measure_status, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                curb->measure_status = static_cast<MeasureStatus>(measure_status);
                offset += int32Size;

                // confidence
                floatEndian.toHostEndianValue(curb->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                curbs.push_back(curb);
            }
        }

        return 0;
    }

    static int serialize(const std::vector<Lane::Ptr> &lanes, char *data) {
        int lanesCnt = static_cast<int>(lanes.size());

        if (lanesCnt == 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // lanes
        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            for (int i = 0; i < lanesCnt; ++i) {
                const Lane::Ptr &robo_lane_ptr = lanes[i];

                // lane_id
                int lane_id = static_cast<int>(robo_lane_ptr->lane_id);
                memcpy(data + offset, &lane_id, int32Size);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_lane_ptr->curve;
                memcpy(data + offset, &(robo_curve.x_start), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.x_end), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.a), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.b), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.c), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_curve.d), floatSize);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_lane_ptr->end_point;

                memcpy(data + offset, &(robo_endpoints.start.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.start.y), floatSize);
                offset += floatSize;

                memcpy(data + offset, &(robo_endpoints.end.x), floatSize);
                offset += floatSize;
                memcpy(data + offset, &(robo_endpoints.end.y), floatSize);
                offset += floatSize;

                // measure_status
                int measure_status = static_cast<int>(robo_lane_ptr->measure_status);
                memcpy(data + offset, &measure_status, int32Size);
                offset += int32Size;

                // confidence
                memcpy(data + offset, &(robo_lane_ptr->confidence), floatSize);
                offset += floatSize;
            }
        } else {
            for (int i = 0; i < lanesCnt; ++i) {
                const Lane::Ptr &robo_lane_ptr = lanes[i];

                // lane_id
                int lane_id = static_cast<int>(robo_lane_ptr->lane_id);
                int32Endian.toTargetEndianArray(lane_id, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // curve
                const Curve &robo_curve = robo_lane_ptr->curve;
                floatEndian.toTargetEndianArray(robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                const EndPoints &robo_endpoints = robo_lane_ptr->end_point;

                floatEndian.toTargetEndianArray(robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toTargetEndianArray(robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // measure_status
                int measure_status = static_cast<int>(robo_lane_ptr->measure_status);
                int32Endian.toTargetEndianArray(measure_status, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += int32Size;

                // confidence
                floatEndian.toTargetEndianArray(robo_lane_ptr->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
            }
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen, std::vector<Lane::Ptr> &lanes) {
        lanes.clear();
        if (maxLen < 0) {
            return 0;
        }

        int int32Size = sizeof(int);
        int floatSize = sizeof(float);

        RSEndian<int> int32Endian;
        RSEndian<float> floatEndian;
        bool isScrEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() ==
        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        // lanes
        int offset = 0;

        if (isScrEndianTypeMatchHostEndian) {
            while (offset < maxLen) {
                Lane::Ptr lane(new Lane());

                // // status
                // int status = 0;
                // memcpy(&status, data + offset, int32Size);
                // lane->status = static_cast<AxisStatus>(status);
                // offset += int32Size;

//                offset += int32Size;  // 填充: status

                // lane_id
                int lane_id = 0;
                memcpy(&lane_id, data + offset, int32Size);
                lane->lane_id = static_cast<LanePosition>(lane_id);
                offset += int32Size;

                // curve
                Curve &robo_curve = lane->curve;
                memcpy(&(robo_curve.x_start), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.x_end), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.a), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.b), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.c), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_curve.d), data + offset, floatSize);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = lane->end_point;

                memcpy(&(robo_endpoints.start.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.start.y), data + offset, floatSize);
                offset += floatSize;

                memcpy(&(robo_endpoints.end.x), data + offset, floatSize);
                offset += floatSize;
                memcpy(&(robo_endpoints.end.y), data + offset, floatSize);
                offset += floatSize;

                // measure_status
                int measure_status = 0;
                memcpy(&measure_status, data + offset, int32Size);
                lane->measure_status = static_cast<MeasureStatus>(measure_status);
                offset += int32Size;

                // confidence
                memcpy(&(lane->confidence), data + offset, floatSize);
                offset += floatSize;

                lanes.push_back(lane);
            }
        } else {
            while (offset < maxLen) {
                Lane::Ptr lane(new Lane());

                // // status
                // int status;
                // int32Endian.toHostEndianValue(
                //     status, data + offset, int32Size,
                //     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                // lane->status = static_cast<AxisStatus>(status);
                // offset += int32Size;

//                offset += int32Size;  // 填充: status

                // lane_id
                int lane_id;
                int32Endian.toHostEndianValue(
                lane_id, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                lane->lane_id = static_cast<LanePosition>(lane_id);
                offset += int32Size;

                // curve
                Curve &robo_curve = lane->curve;
                floatEndian.toHostEndianValue(
                robo_curve.x_start, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.x_end, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.a, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.b, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.c, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_curve.d, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // end_point
                EndPoints &robo_endpoints = lane->end_point;

                floatEndian.toHostEndianValue(
                robo_endpoints.start.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_endpoints.start.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                floatEndian.toHostEndianValue(
                robo_endpoints.end.x, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;
                floatEndian.toHostEndianValue(
                robo_endpoints.end.y, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                // measure_status
                int measure_status;
                int32Endian.toHostEndianValue(
                measure_status, data + offset, int32Size,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                lane->measure_status = static_cast<MeasureStatus>(measure_status);
                offset += int32Size;

                // confidence
                floatEndian.toHostEndianValue(
                lane->confidence, data + offset, floatSize,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                offset += floatSize;

                lanes.push_back(lane);
            }
        }

        return 0;
    }

    static int serialize(const std::map<AxisStatus, RsPose::Ptr> &pose_map, char *data) {
        int offset = 0;
        for (auto iterMap = pose_map.begin(); iterMap != pose_map.end(); ++iterMap) {
            int pose_size = serialize(iterMap->first, iterMap->second, data + offset);
            offset += pose_size;
        }

        return offset;
    }

    static int serialize(const AxisStatus poseStatus, const RsPose::Ptr &pose, char *data) {
        int floatSize = sizeof(float);
        int intSize = sizeof(int);

        RSEndian<float> floatEndian;
        RSEndian<int> intEndian;
        bool isDstEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int offset = 0;
        if (isDstEndianTypeMatchHostEndian) {
            memcpy(data + offset, &(pose->x), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->y), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->z), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->roll), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->pitch), floatSize);
            offset += floatSize;

            memcpy(data + offset, &(pose->yaw), floatSize);
            offset += floatSize;

            int iStatus = static_cast<int>(poseStatus);
            memcpy(data + offset, &iStatus, intSize);
            offset += intSize;
        } else {
            floatEndian.toTargetEndianArray(pose->x, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(pose->y, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(pose->z, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(pose->roll, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(pose->pitch, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toTargetEndianArray(pose->yaw, data + offset, floatSize,
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            int iStatus = static_cast<int>(poseStatus);
            intEndian.toTargetEndianArray(iStatus, data + offset, intSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += intSize;
        }

        return offset;
    }

    static int deserialize(const char *data, const int maxLen, std::map<AxisStatus, RsPose::Ptr> &pose_map) {
        if (!pose_map.empty()) {
            pose_map.clear();
        }
        for (int offset = 0; offset < maxLen && maxLen - offset >= 28; offset += 28) {
            AxisStatus status;
            RsPose::Ptr pose;
            deserialize(data + offset, maxLen - offset, status, pose);

            pose_map.insert(std::pair<AxisStatus, RsPose::Ptr>(status, pose));
        }

        return 0;
    }

    static int deserialize(const char *data, const int maxLen, AxisStatus &status, RsPose::Ptr &pose) {
        if (maxLen < 24) {
            return 0;
        }

        pose.reset(new RsPose());

        const int floatSize = sizeof(float);
        const int intSize = sizeof(int);

        RSEndian<float> floatEndian;
        bool isScrEndianTypeMatchHostEndian =
        (floatEndian.getHostEndian() == RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        int offset = 0;
        if (isScrEndianTypeMatchHostEndian) {
            memcpy(&(pose->x), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->y), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->z), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->roll), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->pitch), data + offset, floatSize);
            offset += floatSize;

            memcpy(&(pose->yaw), data + offset, floatSize);
            offset += floatSize;

            int iStatus = 0;
            memcpy(&iStatus, data + offset, floatSize);
            status = static_cast<AxisStatus>(iStatus);
        } else {
            floatEndian.toHostEndianValue(pose->x, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->y, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->z, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->roll, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->pitch, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            floatEndian.toHostEndianValue(pose->yaw, data + offset, floatSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += floatSize;

            int iStatus = 0;
            RSEndian<int> int32Endian;
            int32Endian.toHostEndianValue(iStatus, data + offset, intSize,
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += intSize;
            status = static_cast<AxisStatus>(iStatus);
        }
        return 0;
    }

    static int serialize(const AxisStatus status, char *data) {
        int intStatus = static_cast<int>(status);

        RSEndian<int> int32Endian;

        int offset = 0;
        int32Endian.toTargetEndianArray(intStatus, data + offset, sizeof(int),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(int);

        return offset;
    }

    static int deserialize(const char *data, const int maxLen, AxisStatus &status) {
        int intStatus = 0;

        RSEndian<int> int32Endian;

        int32Endian.toHostEndianValue(intStatus, data, maxLen,
                                      RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        status = static_cast<AxisStatus>(intStatus);

        return 0;
    }

    static int serialize(const VecInt& indices, const ROBO_DATA_TYPE msg_type, const int& maxMsgSize,
                         RSBytes3_1SerializeBuffer &en_msg) {
        int offset = 0;
        if (!en_msg.offsets.empty()) {
            offset = *(en_msg.offsets.rbegin()) + *(en_msg.lengths.rbegin());
        }

        int total_indices_nums = static_cast<int>(indices.size());

        // 一个数据包所包含的索引数量
        int segment_indices_nums = static_cast<int>(maxMsgSize / sizeof(int));

        if (total_indices_nums > 0) {
            RSEndian<int> int32Endian;
            RS_DATA_ENDIAN_TYPE hostEndian = int32Endian.getHostEndian();

            if (hostEndian != RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
                for (int i = 0; i < total_indices_nums; i += segment_indices_nums) {
                    int start = i;
                    int end = std::min(i+segment_indices_nums, total_indices_nums);

                    en_msg.offsets.push_back(offset);
                    int indices_size = 0;
                    for (int j = start; j < end; j++) {
                        int32Endian.toTargetEndianArray(indices[j], en_msg.buffers.data() + offset,
                                                        4, RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                        offset += 4;
                        indices_size += 4;
                    }
                    en_msg.lengths.push_back(indices_size);
                    en_msg.types.push_back(msg_type);
                }
            }
            else {
                for (int i = 0; i < total_indices_nums; i += segment_indices_nums) {
                    en_msg.offsets.push_back(offset);
                    int start = i;
                    int end = std::min(i + segment_indices_nums, total_indices_nums);

                    // direclty copy memory
                    int indices_size = (end - start) * sizeof(int);
                    memcpy(en_msg.buffers.data() + offset, indices.data() + start,
                           indices_size);
                    offset += indices_size;

                    en_msg.lengths.push_back(indices_size);
                    en_msg.types.push_back(msg_type);
                }
            }
        }
        else {
            en_msg.offsets.push_back(offset);
            en_msg.types.push_back(static_cast<ROBO_DATA_TYPE>(static_cast<int>(msg_type) + 1));
            en_msg.lengths.push_back(0);
        }
        en_msg.msgCntMap[msg_type] = total_indices_nums;
        return 0;
    }

    static int deserialize(const char *data, const int length, VecInt &indices) {
        int oldSize = static_cast<int>(indices.size());
        int newSize = static_cast<int>(indices.size()) + length / 4;
        indices.resize(newSize);

        RSEndian<int> int32Endian;
        RS_DATA_ENDIAN_TYPE hostEndian = int32Endian.getHostEndian();
        if (hostEndian != RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
            for (int i = 0; i < length; i += 4) {
                int j = oldSize + i / 4;
                int32Endian.toHostEndianValue(indices[j], data + i, 4,
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            }
        } else {  // directly copy data
            memcpy(indices.data() + oldSize, data, length);
        }

        return 0;
    }
};

class RSNativeDeserializeUtil {
public:
    using Ptr = std::shared_ptr<RSNativeDeserializeUtil>;
    using ConstPtr = std::shared_ptr<const RSNativeDeserializeUtil>;

public:
    bool isEmptyMsgType(const ROBO_DATA_TYPE msgType) {
        if (msgType == ROBO_DATA_TYPE::ROBO_MSG_GLOBALCAR_POSE_EMPTY
        || msgType == ROBO_DATA_TYPE::ROBO_MSG_AXISLIDAR_POSE_EMPTY
        || msgType == ROBO_DATA_TYPE::ROBO_MSG_VALID_INDICES_EMPTY
        || msgType == ROBO_DATA_TYPE::ROBO_MSG_OBJECT_EMPTY
        || msgType == ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT_EMPTY
        || msgType == ROBO_DATA_TYPE::ROBO_MSG_FREESPACE_EMPTY
        || msgType == ROBO_DATA_TYPE::ROBO_MSG_LANE_EMPTY
        || msgType == ROBO_DATA_TYPE::ROBO_MSG_CURB_EMPTY
        || msgType == ROBO_DATA_TYPE::ROBO_MSG_NON_GD_IDX_EMPTY
        || msgType == ROBO_DATA_TYPE::ROBO_MSG_GD_IDX_EMPTY
        || msgType == ROBO_DATA_TYPE::ROBO_MSG_BG_IDX_EMPTY
        || msgType == ROBO_DATA_TYPE::ROBO_MSG_POINT_EMPTY) {
            return true;
        }

        return false;
    }

    bool checkDeserializeStatus(const st_RoboRecvMessage &recvMsg, uint16_t msgType) {
        // 发的不是空消息，但是不解析，返回true
        // 发的不是空消息，且要解析，返回false
        native_sdk_3_1::ROBO_DATA_TYPE msg_type = static_cast<native_sdk_3_1::ROBO_DATA_TYPE>(msgType);
        const auto& status_map = recvMsg.status;
        if (status_map.at(msg_type) == native_sdk_3_1::ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE) {
            return true;
        }
        else {
            return false;
        }
    }

    bool checkMessageComplete(const RsCommonBytesCustomMsgParams::Ptr& custom_params_, st_RoboRecvMessage &recvMsg)
    {
        // PointCloud Incomplete
        if (custom_params_->send_point_cloud &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_POINT] != native_sdk_3_1::ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_POINT] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "1 Incomplete" << std::endl;
            return false;
        }

        // Non-Ground Indices Incomplete
        if (custom_params_->send_non_ground_indices &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_NON_GD_IDX] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_NON_GD_IDX] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "2 Incomplete" << std::endl;
            return false;
        }

        // Ground Indices Incomplete
        if (custom_params_->send_ground_indices &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_GD_IDX] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_GD_IDX] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "3 Incomplete" << std::endl;
            return false;
        }

        // Background Indices Incomplete
        if (custom_params_->send_background_indices &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_BG_IDX] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_BG_IDX] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "4 Incomplete" << std::endl;
            return false;
        }

        // Object Incomplete
        if (custom_params_->send_object &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_OBJECT] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_OBJECT] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "5 Incomplete" << std::endl;
            return false;
        }

        // Attention Object Incomplete
        if (custom_params_->send_attention_objects &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "6 Incomplete" << std::endl;
            return false;
        }

        // Freespace Incomplete
        if (custom_params_->send_freespace &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_FREESPACE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_FREESPACE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "7 Incomplete" << std::endl;
            return false;
        }

        // Lane Incomplete
        if (custom_params_->send_lane &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_LANE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_LANE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "8 Incomplete" << std::endl;
            return false;
        }

        // Curb Incomplete
        if (custom_params_->send_roadedge &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_CURB] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_CURB] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "9 Incomplete" << std::endl;
            return false;
        }

        // Pose Incomplete
        if (custom_params_->send_global_pose &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_GLOBALCAR_POSE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_GLOBALCAR_POSE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "10 Incomplete" << std::endl;
            return false;
        }

        // Axis Pose Map Incomplete
        if (custom_params_->send_status_pose_map &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_AXISLIDAR_POSE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_AXISLIDAR_POSE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "11 Incomplete" << std::endl;
            return false;
        }

        // Valid indices Incomplete
        if (custom_params_->send_valid_indices &&
        recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_VALID_INDICES] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_DATA_TYPE::ROBO_MSG_VALID_INDICES] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "12 Incomplete" << std::endl;
            return false;
        }

        return true;
    }
};
}  // namespace native_sdk_3_1

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_CUSTOM_TRANSFORMATER_UTIL_H_
