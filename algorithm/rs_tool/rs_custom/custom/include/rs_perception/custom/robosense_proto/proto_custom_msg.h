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
#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_PROTO_CUSTOM_MSG_H
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_PROTO_CUSTOM_MSG_H

#include "rs_perception/custom/robosense_proto/proto_custom_transformer.h"

#ifdef RS_PROTO_FOUND

namespace robosense {
namespace perception {

class RsProtoCustomMsg : public RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsProtoCustomMsg>;
    using ConstPtr = std::shared_ptr<const RsProtoCustomMsg>;

    // load configures from yaml and init protobuf perception message serialize function.
    // input: yaml node
    void init(const RsYamlNode &config_node_) override {
        frame_id = 0;
        RsYamlNode general_node, control_node, custom_node, version_3_0_node;
        rsYamlSubNode(config_node_, "general", general_node);
        rsYamlSubNode(config_node_, "socket", control_node);
        rsYamlSubNode(config_node_, "native", custom_node);

        customParams_.reset(new RsProtoCustomMsgParams);

        // general
        rsYamlRead(general_node, "device_id", customParams_->device_id);

        // control
        rsYamlRead(control_node, "max_msg_size", customParams_->max_msg_size);

        // native
        rsYamlRead(custom_node, "send_objects", customParams_->send_objects);
        rsYamlRead(custom_node, "send_attention_objects",  customParams_->send_attention_objects);
        rsYamlRead(custom_node, "send_object_supplement", customParams_->send_object_supplement);
        rsYamlRead(custom_node, "send_freespace", customParams_->send_freespace);
        rsYamlRead(custom_node, "send_lane", customParams_->send_lane);
        rsYamlRead(custom_node, "send_roadedge", customParams_->send_roadedge);
        rsYamlRead(custom_node, "send_pose", customParams_->send_pose);
        rsYamlRead(custom_node, "send_point_cloud", customParams_->send_point_cloud);
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
    // output: void.
    void deSerialization(const RsPerceptionMsg::Ptr& msg) override {
        (void)(msg);
    }

    // entrance of perception message deserialize.
    // input: robosense perception message and message buffer.
    // output: void.
    void deSerialization(const std::string& msg, std::vector<RsPerceptionMsg::Ptr>& res) override {
        native_sdk_3_1::st_RoboMsgHeader msg_header;
        const int msgHeaderSize = sizeof(native_sdk_3_1::st_RoboMsgHeader);
        msg_header.toHostEndianValue(msg.c_str(), msgHeaderSize,
                                     RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        std::map<unsigned int, native_sdk_3_1::st_RoboProtoRecvMessage>::iterator iterMap = receive_msg_.end();
        native_sdk_3_1::RSProtoDeserializeUtil deserialize_utils;

        {
            std::lock_guard<std::mutex> lg(buffer_mtx_);
            if (msg_header.msgFrameId >= frame_id) 
            {
                iterMap = receive_msg_.find(msg_header.msgFrameId);
                if (iterMap == receive_msg_.end()) {
                    native_sdk_3_1::st_RoboProtoRecvMessage msg_receive_status;
                    msg_receive_status.updateStatusByConfig(*customParams_);
                    msg_receive_status.msg.reset(new RsPerceptionMsg());
                    msg_receive_status.device_id = msg_header.deviceId;
                    msg_receive_status.timestamp = msg_header.msgTimestampS;
                    msg_receive_status.msg->rs_lidar_result_ptr->timestamp = msg_header.msgTimestampS;
                    receive_msg_.insert(std::pair<int, native_sdk_3_1::st_RoboProtoRecvMessage>(
                    msg_header.msgFrameId, msg_receive_status));

                    iterMap = receive_msg_.find(msg_header.msgFrameId);

                }

                // deserialization
                if (iterMap != receive_msg_.end()) {
                    if (deserialize_utils.isEmptyMsgType(
                    static_cast<native_sdk_3_1::ROBO_PROTO_DATA_TYPE>(msg_header.msgType))) {
                        native_sdk_3_1::ROBO_PROTO_DATA_TYPE non_empty_msg_type = static_cast<native_sdk_3_1::ROBO_PROTO_DATA_TYPE>(
                        static_cast<int>(msg_header.msgType) - 1);
                        native_sdk_3_1::st_RoboProtoRecvMessage &msgs = iterMap->second;
                        msgs.status[non_empty_msg_type] = native_sdk_3_1::ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY;
                    }
                    else if (deserialize_utils.checkDeserializeStatus(iterMap->second, msg_header.msgType)) {
                        // if true, do nothing
                        // RINFO << "131 =====> msg_type = " << std::to_string(msg_header.msgType);
                    }
                    else {
                        //Proto::RsProtoSerialize::Ptr native_serialize_ptr(new Proto::RsProtoSerialize(*customParams_));
                        Proto::RsProtoSerialize::Ptr native_serialize_ptr;
                        native_serialize_ptr.reset(new Proto::RsProtoSerialize(customParams_));

                        if (static_cast<native_sdk_3_1::ROBO_PROTO_DATA_TYPE>(msg_header.msgType)
                        == native_sdk_3_1:: ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT) {
                            native_sdk_3_1::st_RoboProtoRecvMessage &msgs = iterMap->second;
                            if (msgs.msg->rs_lidar_result_ptr->scan_ptr->width == 0) {
                                msgs.msg->rs_lidar_result_ptr->scan_ptr->points.reserve(msg_header.msgTotalCnt);
                                msgs.msg->rs_lidar_result_ptr->scan_ptr->width = msg_header.msgTotalCnt;
                                msgs.msg->rs_lidar_result_ptr->scan_ptr->height = 1;
                            }
                        }

                        native_sdk_3_1::ROBO_PROTO_DATA_TYPE msgType
                        = static_cast<native_sdk_3_1::ROBO_PROTO_DATA_TYPE>(msg_header.msgType);
                        native_sdk_3_1::st_RoboProtoRecvMessage &msgs = iterMap->second;

                        // Binary Byte Deserialize
                        native_serialize_ptr->deserialize(msg.c_str() + msgHeaderSize, msg_header.msgLocalLen,
                                                          static_cast<native_sdk_3_1::ROBO_PROTO_DATA_TYPE>(msg_header.msgType),
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

                    if (deserialize_utils.checkMessageComplete(customParams_, iterMap->second)) {
                        RTRACE << name() << ": udp native receive complete ! ";
                        // 接收完整，把处理好的RsPerceptionMsg::Ptr传递给回调函数
                        res.push_back(msg_receive_status.msg);
                        //for(int i=0;i<res[0]->rs_lidar_result_ptr->objects.size();i++)
                        //    {
                        //       std::cout<<res[0]->rs_lidar_result_ptr->objects[i]->core_infos_.center.x<<std::endl;
                        //    }
                        frame_id = std::max(frame_id, int(iterMap->first));
                        iterMap = receive_msg_.erase(iterMap);
                        RINFO << name() << ": udp native receive successed: frameId = " << frameId;
                    }
                    else if (frameId < frame_id) {
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
            }
            else {
                RWARNING << name() << ": udp native receive out of order: frameid = "
                << msg_header.msgFrameId << ", not process !";
            }
        }       
    }

    // implementation of serialization.
    int serialize() {
        timestamps_ = msg_->rs_lidar_result_ptr->timestamp;
        serializeBuffer_.clearInfo();
        Proto::RsProtoSerialize::Ptr protoSerializePtr;
        protoSerializePtr.reset(new Proto::RsProtoSerialize(customParams_));

        if (customParams_->send_objects) {
            protoSerializePtr->serialize(msg_, native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT,
                                         serializeBuffer_);
        }

        if (customParams_->send_attention_objects) {
            protoSerializePtr->serialize(msg_, native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT,
                                         serializeBuffer_);
        }

        if (customParams_->send_freespace) {
            protoSerializePtr->serialize(msg_, native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE,
                                         serializeBuffer_);
        }

        if (customParams_->send_lane) {
            protoSerializePtr->serialize(msg_, native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE,
                                         serializeBuffer_);
        }

        if (customParams_->send_roadedge) {
            protoSerializePtr->serialize(msg_, native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB,
                                         serializeBuffer_);
        }

        if (customParams_->send_pose) {
            protoSerializePtr->serialize(msg_, native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_GLOBALCAR_POSE,
                                         serializeBuffer_);
        }

        if (customParams_->send_point_cloud) {
            protoSerializePtr->serialize(msg_, native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT,
                                         serializeBuffer_);
        }
        frame_id++;
        return 0;
    }

    // entrance of getting the result of serialization.
    // input: a vector that used to save the buffers.
    // output: void. the result of serialization is added into the vector with a message header.
    int getSerializeMessage(std::vector<RsCharBufferPtr>& msgs) override {
        msgs.clear();
        size_t typesCnt = serializeBuffer_.types.size();
        native_sdk_3_1::st_RoboMsgHeader msgHeader;
        for (size_t i = 0; i < typesCnt; i++) {
            const native_sdk_3_1::ROBO_PROTO_DATA_TYPE msg_type = serializeBuffer_.types[i];
            const int msg_offset = serializeBuffer_.offsets[i];
            const int msg_length = serializeBuffer_.lengths[i];

            // 生成消息头
            msgHeader.msgTimestampS = timestamps_;
            msgHeader.deviceId = static_cast<uint16_t>(customParams_->device_id);
            msgHeader.msgType = static_cast<int>(msg_type);
            msgHeader.msgLocalLen = msg_length;
            msgHeader.msgTotalCnt = serializeBuffer_.msgCntMap[msg_type];
            msgHeader.msgFrameId = frame_id;

            size_t total_msg_size = sizeof(native_sdk_3_1::st_RoboMsgHeader) + msg_length;
            RsCharBufferPtr charBufferPtr(new RsCharBuffer());
            charBufferPtr->resize(total_msg_size, '\0');
            if (msg_length == 0) {
                msgHeader.toTargetEndianArray((unsigned char*)(charBufferPtr->data()),
                                              sizeof(native_sdk_3_1::st_RoboMsgHeader),
                                              RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            }
            else {
                const char *msg_data = (const char*)(serializeBuffer_.buffers.data() + msg_offset);

                if (msg_type == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT ||
                    msg_type == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT) {
                    // 一个物体一个坑
                    msgHeader.msgLocalCnt = 1;
                }
                else if (msg_type == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE) {
                    // 所有freespace共用1个坑
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                }
                else if (msg_type == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE) {
                    // 所有lane共用1个坑
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                }
                else if (msg_type == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB) {
                    // 所有roadedge共用1个坑
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                }
                else if (msg_type == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_GLOBALCAR_POSE) {
                    // 所有pose（其实就1个pose）共用1个坑
                    msgHeader.msgLocalCnt = msgHeader.msgTotalCnt;
                }
                else if (msg_type == native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT) {
                    // 1个坑含有的点数
                    msgHeader.msgLocalCnt = serializeBuffer_.msgPointCntMap[static_cast<int>(i)];
                }

                msgHeader.toTargetEndianArray((unsigned char*)(charBufferPtr->data()),
                                              sizeof(native_sdk_3_1::st_RoboMsgHeader),
                                              RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

                memcpy(charBufferPtr->data() + sizeof(native_sdk_3_1::st_RoboMsgHeader),
                       msg_data, msg_length);
            }
            msgs.push_back(charBufferPtr);
        }
        return 0;
    }
private:
    std::string name() { return "Proto_msg"; }
    std::vector<RsCharBufferPtr> charBuffers_;
    int frame_id = 0;
    double timestamps_ = 0.;
    RsProtoCustomMsgParams::Ptr customParams_;
    native_sdk_3_1::RSProtoSerializeBuffer serializeBuffer_;

    std::map<unsigned int, native_sdk_3_1::st_RoboProtoRecvMessage> receive_msg_;
    std::mutex buffer_mtx_;

};

}  // namespace perception
}  // namespace robosense

CEREAL_REGISTER_TYPE(robosense::perception::RsProtoCustomMsg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(
robosense::perception::RsBaseCustomMsg,
robosense::perception::RsProtoCustomMsg)

#endif  // RS_PROTO_FOUND
#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_PROTO_CUSTOM_MSG_H
