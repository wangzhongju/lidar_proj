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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_NATIVE_CUSTOM_MSG_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_NATIVE_CUSTOM_MSG_H_

#include "rs_perception/custom/robosense_native/native_custom_transformer.h"

namespace robosense {
namespace perception {

class RsNativeCustomMsg : public RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsNativeCustomMsg>;

    RsNativeCustomMsg() {
        msg_.reset(new RsPerceptionMsg);
    }

    // load configures from yaml and init native perception message serialize function.
    // input: yaml node
    void init(const RsYamlNode &config_node_) override {
        frameId_ = 0;
        RsYamlNode general_node, custom_node, control_node, send_control_node;
        rsYamlSubNode(config_node_, "general", general_node);
        rsYamlSubNode(config_node_, "native", custom_node);
        rsYamlSubNode(config_node_, "socket", control_node);
        // general
        rsYamlRead(general_node, "device_id", custom_params.device_id);

        // control
        rsYamlRead(control_node, "max_msg_size", custom_params.max_msg_size);
        rsYamlSubNode(control_node, "send_control", send_control_node);
        rsYamlRead(send_control_node, "send_control_compress_enable", custom_params.compress_enable);

        // native
        rsYamlRead(custom_node, "send_point_cloud", custom_params.send_point_cloud);
        rsYamlRead(custom_node, "send_attention_objects",
                   custom_params.send_attention_objects);
        rsYamlRead(custom_node, "send_freespace", custom_params.send_freespace);
        rsYamlRead(custom_node, "send_lane", custom_params.send_lane);
        rsYamlRead(custom_node, "send_roadedge", custom_params.send_roadedge);
        rsYamlRead(custom_node, "send_sematic_indices", custom_params.send_sematic);

        native_msg.reset(new Native::NativeConvert);
        native_msg->init(custom_params);
    }

    // entrance of perception message serialize.
    // input: robosense perception message.
    // output: void. the result of serialization will be recorded in an internal variable.
    void serialization(const RsPerceptionMsg::Ptr &msg) override {
        content.clear();
        msg_ = msg;
        native_msg->serialization(msg_);

        //serialize
        std::stringstream ss;
        cereal::PortableBinaryOutputArchive oarchive(ss);
        oarchive(native_msg);
        content = ss.str();
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
    // output: void. the result of deserialization will be recorded in an internal variable.
    void deSerialization(const std::string& msg, std::vector<RsPerceptionMsg::Ptr>& res) override{
        native_sdk_3_1::st_RoboMsgHeader msgHeader;
        const int msgHeaderSize = sizeof(native_sdk_3_1::st_RoboMsgHeader);
        msgHeader.toHostEndianValue(msg.c_str(), msgHeaderSize,
                                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        std::map<unsigned int, st_MsgReceiveStat>::iterator iterMap = buffers_.end();
        {
            std::lock_guard<std::mutex> lg(buffer_mtx_);
            if (msgHeader.msgFrameId >= frameId_) {
                iterMap = buffers_.find(msgHeader.msgFrameId);
                if (iterMap == buffers_.end()) {
                    st_MsgReceiveStat msgReceiveStat;
                    msgReceiveStat.buffer.resize(msgHeader.msgTotalLen, '\0');
                    msgReceiveStat.totalMsgLen = msgHeader.msgTotalLen;
                    msgReceiveStat.totalMsgUncompressLen = msgHeader.msgTotalUncompressLen;
                    buffers_.insert(std::pair<unsigned int, st_MsgReceiveStat>(
                            msgHeader.msgFrameId, msgReceiveStat));
                    iterMap = buffers_.find(msgHeader.msgFrameId);
                }

                if (iterMap != buffers_.end()) {
                    st_MsgReceiveStat &msgReceiveStat = iterMap->second;

                    if (msgHeader.msgFragmentIndex != msgHeader.msgTotalFragment - 1) {
                        memcpy(msgReceiveStat.buffer.data() +
                        msgHeader.msgLocalLen * msgHeader.msgFragmentIndex,
                        msg.c_str() + msgHeaderSize, msgHeader.msgLocalLen);
                    }
                    else {
                        memcpy(msgReceiveStat.buffer.data() + msgHeader.msgTotalLen -
                        msgHeader.msgLocalLen,
                        msg.c_str() + msgHeaderSize, msgHeader.msgLocalLen);
                    }
                    msgReceiveStat.receiveMsgLen += msgHeader.msgLocalLen;
                }

                for (auto iterMap = buffers_.begin(); iterMap != buffers_.end();) {
                    unsigned int frameId = iterMap->first;
                    st_MsgReceiveStat &msgReceiveStat = iterMap->second;
                    //                RINFO << "frameId = " << frameId;
                    //                RINFO << "TOTAL LENGTH = " << msgReceiveStat.totalMsgLen;
                    //                RINFO << "RECEIVE LENGTH = " << msgReceiveStat.receiveMsgLen;
                    if (msgReceiveStat.checkIsComplete()) { // 检查完整性
                        RTRACE << name() << ": udp native receive complete : "
                        << msgReceiveStat.totalMsgLen;

                        std::string msg;

                        if (msgReceiveStat.totalMsgUncompressLen != 0) {
                            auto starttime = getTime();
                            std::vector<char> uncompressBuffer(
                                    msgReceiveStat.totalMsgUncompressLen, '\0');
                            const int decompressed_size = LZ4_decompress_safe(
                                    static_cast<const char *>(msgReceiveStat.buffer.data()),
                                    uncompressBuffer.data(), msgReceiveStat.totalMsgLen,
                                    msgReceiveStat.totalMsgUncompressLen);
                            if (decompressed_size < 0) {
                                RWARNING << name()
                                << ": udp native receive uncompress failed: frameId = "
                                << frameId;
                                iterMap = buffers_.erase(iterMap);
                                continue;
                            } else {
                                msg.assign(static_cast<const char *>(uncompressBuffer.data()),
                                           uncompressBuffer.size());
                                auto endtime = getTime();
                                RINFO << name()
                                << ": udp native receive uncompress successed: frameId = "
                                << frameId
                                << ", timecost = " << (endtime - starttime) * 1000
                                << " (ms)";
                            }
                        } else {
                            msg.assign((const char *) (msgReceiveStat.buffer.data()),
                                       msgReceiveStat.totalMsgLen);
                        }

                        RsPerceptionMsg::Ptr msg_ptr(new RsPerceptionMsg);
                        native_msg.reset(new Native::NativeConvert);
                        native_msg->init(custom_params);
                        std::stringstream ss;
                        ss << msg;
                        cereal::PortableBinaryInputArchive iarchive(ss);
                        iarchive(native_msg);
                        native_msg->deserialization(msg_ptr);

                        res.push_back(msg_ptr);

                        frameId_ = std::max(frameId_, iterMap->first);
                        iterMap = buffers_.erase(iterMap);
                        RINFO << name()
                        << ": udp native receive successed: frameId = " << frameId
                        << ", totalMsgLen = " << msgReceiveStat.totalMsgLen;
                    }
                    else if (frameId < frameId_) {
                        iterMap = buffers_.erase(iterMap);
                        RWARNING << name()
                        << ": udp native receive out of order: frameid = " << frameId
                        << ", totalMsgLen = " << msgReceiveStat.totalMsgLen
                        << ", force output(loss data) !";
                    }
                    else {
                        ++iterMap;
                        //                    RINFO << "HELLO";
                    }
                }
            }
            else {
                RWARNING << name() << ": udp native receive out of order: frameid = "
                << msgHeader.msgFrameId << ", not process !";
            }
        }
    }

    // entrance of getting the result of serialization.
    // input: a vector that used to save the buffers.
    // output: void. the result of serialization is added into the vector with a message header.
    int getSerializeMessage(std::vector<RsCharBufferPtr>& msgs) override {
        msgs.clear();
        native_sdk_3_1::st_RoboMsgHeader header;

        if (custom_params.compress_enable) {
            auto starttime = getTime();
            const int content_size = content.size();
            const int max_dst_size = LZ4_compressBound(content_size);
            std::vector<char> compress(max_dst_size, '\0');
            const int compressed_data_size = LZ4_compress_default(
                    content.c_str(), static_cast<char *>(compress.data()), content_size,
                    max_dst_size);
            if (compressed_data_size > 0) {
                content.assign(compress.data(), compressed_data_size);
                header.msgTotalUncompressLen = content_size;
                auto endtime = getTime();
                RINFO << "compress successed: ratio = "
                << (content_size * 1.0) / compressed_data_size
                << ", timecost = " << (endtime - starttime) * 1000 << " (ms)";
            }
            else {
                RWARNING << "compress failed !";
                header.msgTotalUncompressLen = 0;
            }
        }

        const int totalContentLen = content.size();
        const int msgHeaderSize = sizeof(native_sdk_3_1::st_RoboMsgHeader);
        const int usefulMsgLen = custom_params.max_msg_size - msgHeaderSize;

        header.msgType = static_cast<uint16_t>(
                native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_CRREAL_NATIVE);
        header.deviceId = custom_params.device_id;
        header.msgTimestampS = msg_->rs_lidar_result_ptr->timestamp;
        header.msgFrameId = frameId_;
        header.msgTotalCnt = 1;
        header.msgTotalLen = totalContentLen;

        int msgIndexFragmentCnt = totalContentLen / usefulMsgLen;
        if (totalContentLen % usefulMsgLen != 0) {
            msgIndexFragmentCnt += 1;
        }

        int contentOffset = 0;
        int contentFragmentLen = 0;
        int lastFragmentIndex = msgIndexFragmentCnt - 1;

        for (int fragmentIndex = 0; fragmentIndex < msgIndexFragmentCnt; ++fragmentIndex) {
            contentOffset = usefulMsgLen * fragmentIndex;

            RsCharBufferPtr charBufferPtr(new RsCharBuffer());
            charBufferPtr->resize(usefulMsgLen, '\0');

            if (fragmentIndex != lastFragmentIndex) {
                contentFragmentLen = usefulMsgLen;
            }
            else {
                contentFragmentLen = totalContentLen - contentOffset;
            }

            charBufferPtr->resize(contentFragmentLen + msgHeaderSize);

            header.msgLocalCnt = 1;
            header.msgLocalLen = contentFragmentLen;
            //            RINFO << "header.msgLocalLen = " << header.msgLocalLen;
            header.msgIndex = 0;
            header.msgTotalFragment = msgIndexFragmentCnt;
            header.msgFragmentIndex = fragmentIndex;

            header.toTargetEndianArray(reinterpret_cast<unsigned char *>(charBufferPtr->data()),
                                       msgHeaderSize,
                                       RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            memcpy(charBufferPtr->data() + msgHeaderSize, content.c_str() + contentOffset,
                   contentFragmentLen);

            msgs.push_back(charBufferPtr);

        }
        frameId_++;
        return 0;
    }

    // implementation of serialization.
    template<class Archive>
    void serialize(Archive &archive) {
        archive(msg_->rs_lidar_result_ptr->frame_id);
        archive(msg_->rs_lidar_result_ptr->timestamp);
        archive(msg_->rs_lidar_result_ptr->global_pose_ptr);
        archive(msg_->rs_lidar_result_ptr->gps_origin);
        archive(msg_->rs_lidar_result_ptr->status_pose_map);
        archive(msg_->rs_lidar_result_ptr->status);
        archive(msg_->rs_lidar_result_ptr->valid_indices);
        archive(msg_->rs_lidar_result_ptr->objects);

        archive(custom_params.send_point_cloud);
        archive(custom_params.send_attention_objects);
        archive(custom_params.send_freespace);
        archive(custom_params.send_lane);
        archive(custom_params.send_roadedge);
        archive(custom_params.send_sematic);

        if (custom_params.send_point_cloud) {
            archive(msg_->rs_lidar_result_ptr->scan_ptr);
        }
        if (custom_params.send_attention_objects) {
            archive(msg_->rs_lidar_result_ptr->attention_objects);
        }
        if (custom_params.send_freespace) {
            archive(msg_->rs_lidar_result_ptr->freespace_ptr);
        }
        if (custom_params.send_lane) {
            archive(msg_->rs_lidar_result_ptr->lanes);
        }
        if (custom_params.send_roadedge) {
            archive(msg_->rs_lidar_result_ptr->roadedges);
        }
        if (custom_params.send_sematic) {
            archive(msg_->rs_lidar_result_ptr->non_ground_indices);
            archive(msg_->rs_lidar_result_ptr->ground_indices);
            archive(msg_->rs_lidar_result_ptr->background_indices);
        }

        // extra_infos
        archive(custom_params.device_id);
    }

    std::string content;
    unsigned int frameId_;
    RsCommonCustomMsgParams custom_params;
    Native::NativeConvert::Ptr native_msg;

private:
    std::string name() { return "Native_Custom_msg"; }
    struct st_MsgReceiveStat {
        unsigned int totalMsgLen = 0;
        unsigned int totalMsgUncompressLen = 0;
        unsigned int receiveMsgLen = 0;
        std::vector<char> buffer;

        bool checkIsComplete() { return totalMsgLen == receiveMsgLen; }
    };

    std::mutex buffer_mtx_;
    std::map<unsigned int, st_MsgReceiveStat> buffers_;

};

}  // namespace perception
}  // namespace robosense

CEREAL_REGISTER_TYPE(robosense::perception::RsNativeCustomMsg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(robosense::perception::RsBaseCustomMsg,
                                     robosense::perception::RsNativeCustomMsg)
#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_NATIVE_CUSTOM_MSG_H_
