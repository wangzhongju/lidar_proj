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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_SDK_2_X_CUSTOM_MSG_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_SDK_2_X_CUSTOM_MSG_H_
#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_perception/custom/robosense_native_bytes/native_bytes_custom_transformer.h"

namespace robosense {
namespace perception {

class RsNativeBytesSDK2_XCustomMsg : public RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsNativeBytesSDK2_XCustomMsg>;
    using ConstPtr = std::shared_ptr<const RsNativeBytesSDK2_XCustomMsg>;

public:

    // load configures from yaml and init native bytes 2.x perception message serialize function.
    // input: yaml node
    void init(const RsYamlNode &config_node_) override {
        frameId_ = 0;
        RsYamlNode general_node, control_node, custom_node, version_2_x_node;
        rsYamlSubNode(config_node_, "general", general_node);
        rsYamlSubNode(config_node_, "socket", control_node);
        rsYamlSubNode(config_node_, "native", custom_node);
//        rsYamlSubNode(custom_node, "version_2_x", version_2_x_node);

        customParams_.reset(new RsCustomNativeBytesSDK2_XMsgParams);
        // general
        rsYamlRead(general_node, "device_id", customParams_->device_id);

        // control
        rsYamlRead(control_node, "max_msg_size", customParams_->max_msg_size);

        // native
//        rsYamlRead(custom_node, "enable", customParams_->enable);
        rsYamlRead(custom_node, "objects",customParams_->send_objects);
        rsYamlRead(custom_node, "attention_objects",customParams_->send_attention_objects);
        rsYamlRead(custom_node, "object_supplement",customParams_->send_object_supplements);
        rsYamlRead(custom_node, "freespace",customParams_->send_freespace);
        rsYamlRead(custom_node, "lanes",customParams_->send_lanes);
        rsYamlRead(custom_node, "curbs",customParams_->send_curbs);
        rsYamlRead(custom_node, "non_ground_indices",customParams_->send_non_ground_indices);
        rsYamlRead(custom_node, "gound_indices",customParams_->send_ground_indices);
        rsYamlRead(custom_node, "background_indices",customParams_->send_background_indices);
        rsYamlRead(custom_node, "point_cloud",customParams_->send_pointcloud);
        rsYamlRead(custom_node, "pose", customParams_->send_pose);

        deviceId_ = customParams_->device_id;
        maxMsgSize_ = customParams_->max_msg_size;
    }

    // entrance of perception message serialize.
    // input: robosense perception message.
    // output: void. the result of serialization will be recorded in an internal variable.
    void serialization(const RsPerceptionMsg::Ptr &msg) override {
        msg_ = msg;
        serialize();
    }

    // entrance of perception message deserialize.
    // input: robosense perception message.
    // output: void. the result of deserialization will be recorded in an internal variable.
    void deSerialization(const RsPerceptionMsg::Ptr &msg) override {
        auto& res_ptr = msg->rs_lidar_result_ptr;
        res_ptr = msg_->rs_lidar_result_ptr;
    }

    // entrance of perception message deserialize.
    // input: robosense perception message and message buffer.
    // output: void.
    void deSerialization(const std::string& msg, std::vector<RsPerceptionMsg::Ptr>& res) override{
        (void)(res);
    }

    // implementation of serialization.
    int serialize() {
        serializeBuffer_.clearInfo();
        timestamp_ = msg_->rs_lidar_result_ptr->timestamp;
        if (customParams_->send_objects) {
            native_sdk_2_x::RSNativeSerialize::serialize(
                    msg_, native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_OBJECT, deviceId_,
                    customParams_->send_object_supplements, serializeBuffer_);
        }

        if (customParams_->send_attention_objects) {
            native_sdk_2_x::RSNativeSerialize::serialize(
                    msg_, native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT, deviceId_,
                    customParams_->send_object_supplements, serializeBuffer_);
        }

        if (customParams_->send_freespace) {
            native_sdk_2_x::RSNativeSerialize::serialize(
                    msg_, native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_FREESPACE, deviceId_,
                    0, serializeBuffer_);
        }

        if (customParams_->send_lanes) {
            native_sdk_2_x::RSNativeSerialize::serialize(
                    msg_, native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_LANE, deviceId_, 0,
                    serializeBuffer_);
        }

        if (customParams_->send_curbs) {
            native_sdk_2_x::RSNativeSerialize::serialize(
                    msg_, native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_CURB, deviceId_, 0,
                    serializeBuffer_);
        }

        if (customParams_->send_non_ground_indices) {
            native_sdk_2_x::RSNativeSerialize::serialize(
                    msg_, native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_NON_GD_IDX, deviceId_,
                    maxMsgSize_ - sizeof(native_sdk_2_x::st_RoboMsgHeader),
                    serializeBuffer_);
        }

        if (customParams_->send_ground_indices) {
            native_sdk_2_x::RSNativeSerialize::serialize(
                    msg_, native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_GD_IDX, deviceId_,
                    maxMsgSize_ - sizeof(native_sdk_2_x::st_RoboMsgHeader),
                    serializeBuffer_);
        }

        if (customParams_->send_background_indices) {
            native_sdk_2_x::RSNativeSerialize::serialize(
                    msg_, native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_BG_IDX, deviceId_,
                    maxMsgSize_ - sizeof(native_sdk_2_x::st_RoboMsgHeader),
                    serializeBuffer_);
        }

        if (customParams_->send_pointcloud) {
            native_sdk_2_x::RSNativeSerialize::serialize(
                    msg_, native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_POINT, deviceId_,
                    maxMsgSize_ - sizeof(native_sdk_2_x::st_RoboMsgHeader),
                    serializeBuffer_);
        }

        if (customParams_->send_pose) {
            native_sdk_2_x::RSNativeSerialize::serialize(
                    msg_, native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_POSE, deviceId_,
                    maxMsgSize_ - sizeof(native_sdk_2_x::st_RoboMsgHeader),
                    serializeBuffer_);
        }
        ++frameId_;
        return 0;
    }

    // entrance of getting the result of serialization.
    // input: a vector that used to save the buffers.
    // output: void. the result of serialization is added into the vector with a message header.
    int getSerializeMessage(std::vector<RsCharBufferPtr> &msgs) {
        msgs.clear();
        size_t typesCnt = serializeBuffer_.types.size();
        native_sdk_2_x::st_RoboMsgHeader msgHeader;
        for (size_t i = 0; i < typesCnt; ++i) {
            const native_sdk_2_x::ROBO_MSG_TYPE msgType = serializeBuffer_.types[i];
            const int msgOffset = serializeBuffer_.offsets[i];
            const int msgLength = serializeBuffer_.lengths[i];

            // Make Message Header
            msgHeader.msgTimestampMs = timestamp_;
            msgHeader.deviceId = static_cast<uint16_t>(deviceId_);
            msgHeader.msgType = static_cast<int>(msgType);
            msgHeader.msgLocalLen = msgLength;
            msgHeader.msgTotalCnt = serializeBuffer_.msgCntMap[msgType];
            msgHeader.msgFrameId = frameId_;

            int totalMsgSize = sizeof(native_sdk_2_x::st_RoboMsgHeader) + msgLength;
            RsCharBufferPtr charBufferPtr(new RsCharBuffer());
            charBufferPtr->resize(totalMsgSize, '\0');
            if (msgLength == 0) {
                msgHeader.toTargetEndianArray(
                (unsigned char *) (charBufferPtr->data()),
                sizeof(native_sdk_2_x::st_RoboMsgHeader),
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            } else {
                const char *msgData =
                (const char *) (serializeBuffer_.buffers.data() + msgOffset);

                if (msgType == native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_OBJECT ||
                    msgType == native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT) {
                    // Single Frame Just One Object
                    msgHeader.msgLocalCnt = 1;
                } else if (msgType ==
                           native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_FREESPACE) {
                    // All in One Frame
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                } else if (msgType == native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_LANE) {
                    // All in one Frame
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                } else if (msgType == native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_CURB) {
                    // All in one Frame
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                } else if (msgType == native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_POSE) {
                    // All in one Frame
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                } else if (msgType == native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_POINT) {
                    // Single Frame Point Count
                    msgHeader.msgLocalCnt = msgHeader.msgLocalLen / 16;
                } else if (msgType ==
                           native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_NON_GD_IDX ||
                           msgType == native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_GD_IDX ||
                           msgType == native_sdk_2_x::ROBO_MSG_TYPE::ROBO_MSG_BG_IDX) {
                    // Single Frame Indices Count
                    msgHeader.msgLocalCnt = msgHeader.msgLocalLen / 4;
                }

                msgHeader.toTargetEndianArray(
                (unsigned char *) (charBufferPtr->data()),
                sizeof(native_sdk_2_x::st_RoboMsgHeader),
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

                memcpy(charBufferPtr->data() + sizeof(native_sdk_2_x::st_RoboMsgHeader),
                       msgData, msgLength);
            }
            msgs.push_back(charBufferPtr);
        }
        return 0;
    }

public:
    int deviceId_;
    int maxMsgSize_;
    double timestamp_;
    unsigned int frameId_;
    native_sdk_2_x::RSSerializeBuffer serializeBuffer_;
    RsCustomNativeBytesSDK2_XMsgParams::Ptr customParams_;
};

}  // namespace perception
}  // namespace robosense

CEREAL_REGISTER_TYPE(robosense::perception::RsNativeBytesSDK2_XCustomMsg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(
    robosense::perception::RsBaseCustomMsg,
    robosense::perception::RsNativeBytesSDK2_XCustomMsg)
#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_SDK_2_X_CUSTOM_MSG_H_
