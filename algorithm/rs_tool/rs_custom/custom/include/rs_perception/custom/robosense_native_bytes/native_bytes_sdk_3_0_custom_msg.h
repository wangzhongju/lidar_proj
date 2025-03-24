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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_SDK_3_0_CUSTOM_MSG_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_SDK_3_0_CUSTOM_MSG_H_
#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_perception/custom/robosense_native_bytes/native_bytes_custom_transformer.h"

namespace robosense {
namespace perception {

class RsNativeBytesSDK3_0CustomMsg : public RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsNativeBytesSDK3_0CustomMsg>;
    using ConstPtr = std::shared_ptr<const RsNativeBytesSDK3_0CustomMsg>;

public:

    // load configures from yaml and init native bytes 3.0 perception message serialize function.
    // input: yaml node
    void init(const RsYamlNode &config_node_) override {
        frameId_ = 0;
        RsYamlNode general_node, control_node, custom_node, version_3_0_node;
        rsYamlSubNode(config_node_, "general", general_node);
        rsYamlSubNode(config_node_, "socket", control_node);
        rsYamlSubNode(config_node_, "native", custom_node);
//        rsYamlSubNode(custom_node, "version_2_x", version_3_0_node);

        customParams_.reset(new RsCustomNativeBytesSDK3_0MsgParams);
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
        rsYamlRead(custom_node, "curbs",customParams_->send_curbs);
        rsYamlRead(custom_node, "lanes",customParams_->send_lanes);
        rsYamlRead(custom_node, "axis_lidar_pose",customParams_->send_axis_lidar_pose);
        rsYamlRead(custom_node, "point_cloud",customParams_->send_pointcloud);
        rsYamlRead(custom_node, "global_car_pose",customParams_->send_global_car_pose);

        customParams_->send_axisstatus =(customParams_->send_objects || customParams_->send_attention_objects);

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
        native_sdk_3_0::st_RoboMsgHeader msg_header;
        const int msg_header_size = sizeof(native_sdk_3_0::st_RoboMsgHeader);
        msg_header.toHostEndianValue(msg.c_str(), msg_header_size,
                                     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        std::map<unsigned int, native_sdk_3_0::st_RoboRecvMessage>::iterator iter_map = receive_msg_.end();
        native_sdk_3_0::RSNativeDeserializeUtil deserialize_utils;
        {
            std::lock_guard<std::mutex> lg(buffer_mtx_);
            if (msg_header.msgFrameId >= frameId_) {
                iter_map = receive_msg_.find(msg_header.msgFrameId);
                if (iter_map == receive_msg_.end()) {
                    native_sdk_3_0::st_RoboRecvMessage msg_receive_status;
                    msg_receive_status.updateStatusByConfig(*customParams_);
                    msg_receive_status.msg.reset(new RsPerceptionMsg());
                    msg_receive_status.device_id = static_cast<int>(msg_header.deviceId);
                    msg_receive_status.timestamp = msg_header.msgTimestampMs;
                    msg_receive_status.msg->rs_lidar_result_ptr->timestamp = msg_header.msgTimestampMs;
                    receive_msg_.insert(std::pair<int, native_sdk_3_0::st_RoboRecvMessage>(
                    msg_header.msgFrameId, msg_receive_status));

                    iter_map = receive_msg_.find(msg_header.msgFrameId);

                }

                // deserialization
                if (iter_map != receive_msg_.end()) {
                    if (deserialize_utils.isEmptyMsgType(
                    static_cast<native_sdk_3_0::ROBO_MSG_TYPE>(msg_header.msgType))) {
                        native_sdk_3_0::ROBO_MSG_TYPE non_empty_msg_type = static_cast<native_sdk_3_0::ROBO_MSG_TYPE>(
                        static_cast<int>(msg_header.msgType) - 1);
                        native_sdk_3_0::st_RoboRecvMessage &msgs = iter_map->second;
                        msgs.status[non_empty_msg_type] = native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_EMPTY;
                    }
                    else if (deserialize_utils.checkDeserializeStatus(iter_map->second, msg_header.msgType)) {
                        // if true, do nothing
                        //                        RINFO << "131 =====> msg_type = " << std::to_string(msg_header.msgType);
                    }
                    else {
                        native_sdk_3_0::RSNativeSerialize::Ptr native_serialize_ptr(
                        new native_sdk_3_0::RSNativeSerialize(*customParams_));

                        if (static_cast<native_sdk_3_0::ROBO_MSG_TYPE>(msg_header.msgType)
                        == native_sdk_3_0:: ROBO_MSG_TYPE::ROBO_MSG_POINT) {
                            native_sdk_3_0::st_RoboRecvMessage &msgs = iter_map->second;
                            if (msgs.msg->rs_lidar_result_ptr->scan_ptr->width == 0) {
                                msgs.msg->rs_lidar_result_ptr->scan_ptr->points.reserve(msg_header.msgTotalCnt);
                                msgs.msg->rs_lidar_result_ptr->scan_ptr->width = msg_header.msgTotalCnt;
                                msgs.msg->rs_lidar_result_ptr->scan_ptr->height = 1;
                            }
                        }

                        auto msgType = static_cast<native_sdk_3_0::ROBO_MSG_TYPE>(msg_header.msgType);
                        native_sdk_3_0::st_RoboRecvMessage &msgs = iter_map->second;

                        // Binary Byte Deserialize
                        native_serialize_ptr->deserialize(msg.c_str() + msg_header_size, msg_header.msgLocalLen,
                                                          static_cast<native_sdk_3_0::ROBO_MSG_TYPE>(msg_header.msgType),
                                                          msgs);

                        native_sdk_3_0::st_RoboMsgRecorder &msg_recorder = msgs.recorder[msgType];
                        msg_recorder.totalCnt = static_cast<int>(msg_header.msgTotalCnt);
                        msg_recorder.receivedCnt += static_cast<int>(msg_header.msgLocalCnt);

                        if (msg_recorder.receivedCnt > 0 && msg_recorder.receivedCnt != msg_recorder.totalCnt) {
                            //                            RINFO << "// Incomplete";
                            msgs.status[msgType] = native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_INCOMMPLETE;
                        }
                        else if (msg_recorder.receivedCnt == msg_recorder.totalCnt) {
                            //                            RINFO << "// success";
                            msgs.status[msgType] = native_sdk_3_0::ROBO_MSG_RECV_STATUS::ROBO_MSG_SUCCESS;
                        }
                    }
                }
//                 int i = 0;
                // Check Receive Success
                for (auto iterMap = receive_msg_.begin(); iterMap != receive_msg_.end();) {
                    unsigned int frameId = iterMap->first;
                    auto& msg_receive_status = iterMap->second;

                    if (deserialize_utils.checkMessageComplete(customParams_, iterMap->second)) {
                        RTRACE << name() << ": udp native receive complete ! ";
                        // 接收完整，把处理好的RsPerceptionMsg::Ptr传递给回调函数
                        res.push_back(msg_receive_status.msg);

                        frameId_ = std::max(frameId_, iterMap->first);
                        iterMap = receive_msg_.erase(iterMap);
                        RINFO << name() << ": udp native receive successed: frameId = " << frameId;
                    }
                    else if (frameId < frameId_) {
                        iterMap = receive_msg_.erase(iterMap);
                        RWARNING << name()
                        << ": udp native receive out of order: frame_id = " << frameId
                        << ", force output(loss data) !";
                    }
                    else {
                        ++iterMap;
//                        RINFO << "191";
//                        ++i;
                    }
                }
//                RINFO << "i = " << i;
            }
            else {
                RWARNING << name() << ": udp native receive out of order: frameid = "
                << msg_header.msgFrameId << ", not process !";
            }
        }
    }

    // implementation of serialization.
    int serialize() {
        timestamp_ = msg_->rs_lidar_result_ptr->timestamp;
        serializeBuffer_.clearInfo();
        native_sdk_3_0::RSNativeSerialize::Ptr nativeSerializePtr(
                new native_sdk_3_0::RSNativeSerialize(*customParams_));
        if (customParams_->send_objects) {
            nativeSerializePtr->serialize(
                    msg_, native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_OBJECT,
                    serializeBuffer_);
        }

        if (customParams_->send_attention_objects) {
            nativeSerializePtr->serialize(
                    msg_, native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT,
                    serializeBuffer_);
        }

        if (customParams_->send_axisstatus) {
            nativeSerializePtr->serialize(
                    msg_, native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_AXISSTATUS,
                    serializeBuffer_);
        }

        if (customParams_->send_freespace) {
            nativeSerializePtr->serialize(
                    msg_, native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_FREESPACE,
                    serializeBuffer_);
        }

        if (customParams_->send_lanes) {
            nativeSerializePtr->serialize(
                    msg_, native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_LANE,
                    serializeBuffer_);
        }

        if (customParams_->send_curbs) {
            nativeSerializePtr->serialize(
                    msg_, native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_CURB,
                    serializeBuffer_);
        }

        if (customParams_->send_axis_lidar_pose) {
            nativeSerializePtr->serialize(
                    msg_, native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_AXISLIDAR_POSE,
                    serializeBuffer_);
        }

        if (customParams_->send_pointcloud) {
            nativeSerializePtr->serialize(
                    msg_, native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_POINT,
                    serializeBuffer_);
        }

        if (customParams_->send_global_car_pose) {
            nativeSerializePtr->serialize(
                    msg_, native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_GLOBALCAR_POSE,
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
        native_sdk_3_0::st_RoboMsgHeader msgHeader;
        for (size_t i = 0; i < typesCnt; ++i) {
            const native_sdk_3_0::ROBO_MSG_TYPE msgType = serializeBuffer_.types[i];
            const int msgOffset = serializeBuffer_.offsets[i];
            const int msgLength = serializeBuffer_.lengths[i];

            // Make Message Header
            msgHeader.msgTimestampMs = timestamp_;
            msgHeader.deviceId = static_cast<uint16_t>(deviceId_);
            msgHeader.msgType = static_cast<int>(msgType);
            msgHeader.msgLocalLen = msgLength;
            msgHeader.msgTotalCnt = serializeBuffer_.msgCntMap[msgType];
            msgHeader.msgFrameId = frameId_;

            int totalMsgSize = sizeof(native_sdk_3_0::st_RoboMsgHeader) + msgLength;
            RsCharBufferPtr charBufferPtr(new RsCharBuffer());
            charBufferPtr->resize(totalMsgSize, '\0');
            if (msgLength == 0) {
                msgHeader.toTargetEndianArray(
                (unsigned char *) (charBufferPtr->data()),
                sizeof(native_sdk_3_0::st_RoboMsgHeader),
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            } else {
                const char *msgData =
                (const char *) (serializeBuffer_.buffers.data() + msgOffset);

                if (msgType == native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_OBJECT ||
                    msgType == native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_ATT_OBJECT) {
                    // Single Frame Just One Object
                    msgHeader.msgLocalCnt = 1;
                } else if (msgType ==
                           native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_FREESPACE) {
                    // All in One Frame
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                } else if (msgType == native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_LANE) {
                    // All in one Frame
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                } else if (msgType == native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_CURB) {
                    // All in one Frame
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                } else if (msgType ==
                           native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_AXISLIDAR_POSE) {
                    // All in one Frame
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                } else if (msgType == native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_POINT) {
                    // Single Frame Point Count: xyzil
                    msgHeader.msgLocalCnt = msgHeader.msgLocalLen / 20;
                } else if (msgType ==
                           native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_GLOBALCAR_POSE) {
                    msgHeader.msgLocalCnt = 1;
                } else if (msgType ==
                           native_sdk_3_0::ROBO_MSG_TYPE::ROBO_MSG_AXISSTATUS) {
                    msgHeader.msgLocalCnt = 1;
                }

                msgHeader.toTargetEndianArray(
                (unsigned char *) (charBufferPtr->data()),
                sizeof(native_sdk_3_0::st_RoboMsgHeader),
                RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

                memcpy(charBufferPtr->data() + sizeof(native_sdk_3_0::st_RoboMsgHeader),
                       msgData, msgLength);
            }
            msgs.push_back(charBufferPtr);
        }
        return 0;
    }

public:
    std::string name() { return "Native_Bytes_3_0_msg"; }
    int deviceId_;
    int maxMsgSize_;
    double timestamp_;
    unsigned int frameId_;
    RsCustomNativeBytesSDK3_0MsgParams::Ptr customParams_;
    native_sdk_3_0::RSSerializeBuffer serializeBuffer_;

    std::map<unsigned int, native_sdk_3_0::st_RoboRecvMessage> receive_msg_;
    std::mutex buffer_mtx_;
};

}  // namespace perception
}  // namespace robosense

CEREAL_REGISTER_TYPE(robosense::perception::RsNativeBytesSDK3_0CustomMsg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(
    robosense::perception::RsBaseCustomMsg,
    robosense::perception::RsNativeBytesSDK3_0CustomMsg)
#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NATIVE_BYTES_SDK_3_0_CUSTOM_MSG_H_
