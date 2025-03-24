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
#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NAIVE_BYTES_3_1_CUSTOM_MSG_H
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NAIVE_BYTES_3_1_CUSTOM_MSG_H

#include "rs_perception/custom/robosense_native_bytes/native_bytes_custom_transformer.h"
#include "rs_perception/custom/common/base_custom_msg.h"

namespace robosense {
namespace perception {

class RsNativeBytesSDK3_1CustomMsg : public RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsNativeBytesSDK3_1CustomMsg>;
    using ConstPtr = std::shared_ptr<const RsNativeBytesSDK3_1CustomMsg>;

    RsNativeBytesSDK3_1CustomMsg() {
        msg_.reset(new RsPerceptionMsg);
        custom_params_.reset(new RsCommonBytesCustomMsgParams);
        frame_id_ = 0;
    }

    // load configures from yaml and init native bytes 3.1 perception message serialize function.
    // input: yaml node
    void init(const RsYamlNode& config_node_) override {
        RsYamlNode general_node, custom_node, control_node, send_control_node;
        rsYamlSubNode(config_node_, "general", general_node);
        rsYamlSubNode(config_node_, "native", custom_node);
        rsYamlSubNode(config_node_, "socket", control_node);
        // general
        rsYamlRead(general_node, "device_id", custom_params_->device_id);

        // control
        rsYamlRead(control_node, "max_msg_size", custom_params_->max_msg_size);
        rsYamlSubNode(control_node, "send_control", send_control_node);
        rsYamlRead(send_control_node, "send_control_compress_enable", custom_params_->compress_enable);

        // native
        rsYamlRead(custom_node, "send_object_supplement", custom_params_->send_object_supplement);
        rsYamlRead(custom_node, "send_point_cloud", custom_params_->send_point_cloud);
        rsYamlRead(custom_node, "send_attention_objects", custom_params_->send_attention_objects);
        rsYamlRead(custom_node, "send_freespace", custom_params_->send_freespace);
        rsYamlRead(custom_node, "send_lane", custom_params_->send_lane);
        rsYamlRead(custom_node, "send_roadedge", custom_params_->send_roadedge);
        rsYamlRead(custom_node, "send_sematic_indices", custom_params_->send_sematic);

        if (custom_params_->send_sematic) {
            custom_params_->send_background_indices = true;
            custom_params_->send_non_ground_indices = true;
            custom_params_->send_ground_indices = true;
        }
    }

    // entrance of perception message serialize.
    // input: robosense perception message.
    // output: void. the result of serialization will be recorded in an internal variable.
    void serialization(const RsPerceptionMsg::Ptr& msg) override {
        msg_ = msg;
        serialize();
    }

    // entrance of perception message deserialize.
    // input: robosense perception message.
    // output: void. the result of deserialization will be recorded in an internal variable.
    void deSerialization(const RsPerceptionMsg::Ptr& msg) override {
        auto& res_ptr = msg->rs_lidar_result_ptr;
        res_ptr = msg_->rs_lidar_result_ptr;
    }

    // entrance of perception message deserialize.
    // input: robosense perception message and message buffer.
    // output: void.
    void deSerialization(const std::string& msg, std::vector<RsPerceptionMsg::Ptr>& res) override {
        native_sdk_3_1::st_RoboMsgHeader msg_header;
        const int msgHeaderSize = sizeof(native_sdk_3_1::st_RoboMsgHeader);
        msg_header.toHostEndianValue(msg.c_str(), msgHeaderSize,
                                     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        std::map<unsigned int, native_sdk_3_1::st_RoboRecvMessage>::iterator iterMap = receive_msg_.end();
        native_sdk_3_1::RSNativeDeserializeUtil deserialize_utils;

        {
            std::lock_guard<std::mutex> lg(buffer_mtx_);
            if (msg_header.msgFrameId >= frame_id_) {
                iterMap = receive_msg_.find(msg_header.msgFrameId);
                if (iterMap == receive_msg_.end()) {
                    native_sdk_3_1::st_RoboRecvMessage msg_receive_status;
                    msg_receive_status.updateStatusByConfig(*custom_params_);
                    msg_receive_status.msg.reset(new RsPerceptionMsg());
                    msg_receive_status.device_id = msg_header.deviceId;
                    msg_receive_status.timestamp = msg_header.msgTimestampS;
                    msg_receive_status.msg->rs_lidar_result_ptr->timestamp = msg_header.msgTimestampS;
                    receive_msg_.insert(std::pair<int, native_sdk_3_1::st_RoboRecvMessage>(
                    msg_header.msgFrameId, msg_receive_status));

                    iterMap = receive_msg_.find(msg_header.msgFrameId);

                }

                // deserialization
                if (iterMap != receive_msg_.end()) {
                    if (deserialize_utils.isEmptyMsgType(
                    static_cast<native_sdk_3_1::ROBO_DATA_TYPE>(msg_header.msgType))) {
                        native_sdk_3_1::ROBO_DATA_TYPE non_empty_msg_type = static_cast<native_sdk_3_1::ROBO_DATA_TYPE>(
                        static_cast<int>(msg_header.msgType) - 1);
                        native_sdk_3_1::st_RoboRecvMessage &msgs = iterMap->second;
                        msgs.status[non_empty_msg_type] = native_sdk_3_1::ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY;
                    }
                    else if (deserialize_utils.checkDeserializeStatus(iterMap->second, msg_header.msgType)) {
                        // if true, do nothing
                        // RINFO << "131 =====> msg_type = " << std::to_string(msg_header.msgType);
                    }
                    else {
                        native_sdk_3_1::RSNativeByte3_1Serialize::Ptr native_serialize_ptr(
                        new native_sdk_3_1::RSNativeByte3_1Serialize(*custom_params_));

                        if (static_cast<native_sdk_3_1::ROBO_DATA_TYPE>(msg_header.msgType)
                        == native_sdk_3_1:: ROBO_DATA_TYPE::ROBO_MSG_POINT) {
                            native_sdk_3_1::st_RoboRecvMessage &msgs = iterMap->second;
                            if (msgs.msg->rs_lidar_result_ptr->scan_ptr->width == 0) {
                                msgs.msg->rs_lidar_result_ptr->scan_ptr->points.reserve(msg_header.msgTotalCnt);
                                msgs.msg->rs_lidar_result_ptr->scan_ptr->width = msg_header.msgTotalCnt;
                                msgs.msg->rs_lidar_result_ptr->scan_ptr->height = 1;
                            }
                        }

                        native_sdk_3_1::ROBO_DATA_TYPE msgType
                        = static_cast<native_sdk_3_1::ROBO_DATA_TYPE>(msg_header.msgType);
                        native_sdk_3_1::st_RoboRecvMessage &msgs = iterMap->second;

                        // Binary Byte Deserialize
                        native_serialize_ptr->deserialize(msg.c_str() + msgHeaderSize, msg_header.msgLocalLen,
                                                          static_cast<native_sdk_3_1::ROBO_DATA_TYPE>(msg_header.msgType),
                                                          msgs);

                        native_sdk_3_1::st_RoboMsgRecorder &msg_recorder = msgs.recorder[msgType];
                        msg_recorder.totalCnt = msg_header.msgTotalCnt;
                        msg_recorder.receivedCnt += msg_header.msgLocalCnt;

                        if (msg_recorder.receivedCnt > 0 && msg_recorder.receivedCnt != msg_recorder.totalCnt) {
                            // RINFO << "// Incomplete";
                            msgs.status[msgType] = native_sdk_3_1::ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_INCOMMPLETE;
                        }
                        else if (msg_recorder.receivedCnt == msg_recorder.totalCnt) {
                            // RINFO << "// success";
                            msgs.status[msgType] = native_sdk_3_1::ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS;
                        }
                    }
                }
                // int i = 0;
                // Check Receive Success
                for (auto iterMap = receive_msg_.begin(); iterMap != receive_msg_.end();) {
                    unsigned int frameId = iterMap->first;
                    auto& msg_receive_status = iterMap->second;

                    if (deserialize_utils.checkMessageComplete(custom_params_, iterMap->second)) {
                        RTRACE << name() << ": udp native receive complete ! ";
                        // 接收完整，把处理好的RsPerceptionMsg::Ptr传递给回调函数
                        res.push_back(msg_receive_status.msg);

                        frame_id_ = std::max(frame_id_, iterMap->first);
                        iterMap = receive_msg_.erase(iterMap);
                        RINFO << name() << ": udp native receive successed: frameId = " << frameId;
                    }
                    else if (frameId < frame_id_) {
                        iterMap = receive_msg_.erase(iterMap);
                        RWARNING << name()
                        << ": udp native receive out of order: frame_id = " << frameId
                        << ", force output(loss data) !";
                    }
                    else {
                        ++iterMap;
                        // RINFO << "191";
                        //  ++i;
                    }
                }
                // RINFO << "i = " << i;
            }
            else {
                RWARNING << name() << ": udp native receive out of order: frameid = "
                << msg_header.msgFrameId << ", not process !";
            }
        }
    }

    // implementation of serialization.
    int serialize() {
        serializeBuffer_.clearInfo();
        native_sdk_3_1::RSNativeByte3_1Serialize::Ptr native_serialize_ptr(
        new native_sdk_3_1::RSNativeByte3_1Serialize(*custom_params_));

        // base
        {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_GLOBALCAR_POSE, serializeBuffer_);
        }

        {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_GPS_ORIGIN, serializeBuffer_);
        }

        {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_AXISLIDAR_POSE, serializeBuffer_);
        }

        {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_VALID_INDICES, serializeBuffer_);
        }

        {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_OBJECT, serializeBuffer_);
        }

        // optional
        if (custom_params_->send_attention_objects) {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT, serializeBuffer_);
        }

        if (custom_params_->send_freespace) {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_FREESPACE, serializeBuffer_);
        }

        if (custom_params_->send_lane) {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_LANE, serializeBuffer_);
        }

        if (custom_params_->send_roadedge) {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_CURB, serializeBuffer_);
        }

        if (custom_params_->send_non_ground_indices) {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_NON_GD_IDX, serializeBuffer_);
        }

        if (custom_params_->send_ground_indices) {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_GD_IDX, serializeBuffer_);
        }

        if (custom_params_->send_background_indices) {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_BG_IDX, serializeBuffer_);
        }

        if (custom_params_->send_point_cloud) {
            native_serialize_ptr->serialize(
            msg_, native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_POINT, serializeBuffer_);
        }
        frame_id_++;
        return 0;
    }

    // entrance of getting the result of serialization.
    // input: a vector that used to save the buffers.
    // output: void. the result of serialization is added into the vector with a message header.
    int getSerializeMessage(std::vector<RsCharBufferPtr>& msgs) {
        msgs.clear();
        size_t types_nums = serializeBuffer_.types.size();
        native_sdk_3_1::st_RoboMsgHeader msgHeader;
        for (size_t i = 0; i < types_nums; i++) {
            const auto& msg_type = serializeBuffer_.types[i];
            const int msg_offset = serializeBuffer_.offsets[i];
            const int msg_length = serializeBuffer_.lengths[i];

            // 制作消息头
            msgHeader.msgTimestampS = msg_->rs_lidar_result_ptr->timestamp;
            msgHeader.deviceId = static_cast<int>(custom_params_->device_id);
            msgHeader.msgType = static_cast<uint16_t>(msg_type);
            msgHeader.msgLocalLen = msg_length;
            msgHeader.msgTotalCnt = serializeBuffer_.msgCntMap[msg_type];
            msgHeader.msgFrameId = frame_id_;

            int total_msg_size = sizeof(native_sdk_3_1::st_RoboMsgHeader) + msg_length;
            RsCharBufferPtr charBufferPtr(new RsCharBuffer());
            charBufferPtr->resize(total_msg_size, '\0');

            if (msg_length > 0) {
                const char *msg_data = (const char *) (serializeBuffer_.buffers.data() + msg_offset);

                if (msg_type == native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_OBJECT ||
                    msg_type == native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_ATT_OBJECT) {
                    // 一帧只有1个物体
                    msgHeader.msgLocalCnt = 1;
                }
                else if (msg_type == native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_POINT) {
                    // 一个点共有x,y,z,i,l 5个数据。每个4字节。存储的时候长度就是4*5=20。因此这里要除以20。
                    msgHeader.msgLocalCnt = msgHeader.msgLocalLen / 20;
                }
                else if (msg_type == native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_NON_GD_IDX ||
                         msg_type == native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_GD_IDX ||
                         msg_type == native_sdk_3_1::ROBO_DATA_TYPE::ROBO_MSG_BG_IDX) {
                    // 一个索引占了4个字节。存储的时候长度就是1*4=4。因此这里要除以4。
                    msgHeader.msgLocalCnt = msgHeader.msgLocalLen / 4;
                }
                else {
                    // 剩余的消息，都是一帧就全部发完了。
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                }

                memcpy(charBufferPtr->data() + sizeof(native_sdk_3_1::st_RoboMsgHeader),
                       msg_data, msg_length);
            }

            msgHeader.toTargetEndianArray((unsigned char *) (charBufferPtr->data()),
                                          sizeof(native_sdk_3_1::st_RoboMsgHeader),
                                          RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            msgs.push_back(charBufferPtr);
        }
        return 0;
    }

private:
    std::string name() { return "Native_Bytes_3_1_msg"; }
    unsigned int frame_id_;
    RsCommonBytesCustomMsgParams::Ptr custom_params_;
    native_sdk_3_1::RSBytes3_1SerializeBuffer serializeBuffer_;

    std::map<unsigned int, native_sdk_3_1::st_RoboRecvMessage> receive_msg_;
    std::mutex buffer_mtx_;
};

}  // namespace perception
}  // namespace robosense

CEREAL_REGISTER_TYPE(robosense::perception::RsNativeBytesSDK3_1CustomMsg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(
robosense::perception::RsBaseCustomMsg,
robosense::perception::RsNativeBytesSDK3_1CustomMsg)

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_BYTES_NAIVE_BYTES_3_1_CUSTOM_MSG_H
