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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_V2R_ROBOSENSE_V2R_CUSTOM_MSG_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_V2R_ROBOSENSE_V2R_CUSTOM_MSG_H_
#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_perception/custom/robosense_v2r/robosense_v2r_custom_transformer.h"

namespace robosense {
namespace perception {

class RsRobosenseV2RCustomMsg : public RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsRobosenseV2RCustomMsg>;
    using ConstPtr = std::shared_ptr<const RsRobosenseV2RCustomMsg>;

public:

    // load configures from yaml and init ros perception message serialize function.
    // input: yaml node
    void init(const RsYamlNode &config_node_) override {
        frame_id = 0;
        RsYamlNode v2r_heading_node, custom_node, general_node;
        rsYamlSubNode(config_node_, "v2r", custom_node);
        rsYamlSubNode(config_node_, "general", general_node);
        customParams_.reset(new RsCustomRobosenseV2RMsgParams);

        // general
        rsYamlRead(general_node, "device_id", customParams_->device_id);

        // v2r
        rsYamlRead(custom_node, "method", customParams_->sMethod);
        rsYamlRead(custom_node, "version", customParams_->sVersion);
        rsYamlRead(custom_node, "center", customParams_->sCenter);

        rsYamlSubNode(custom_node, "heading", v2r_heading_node);
        rsYamlRead(v2r_heading_node, "velocity_heading_th",
                   customParams_->headingConfig.velocityHeadingTh);
        rsYamlRead(v2r_heading_node, "acceleration_heading_th",
                   customParams_->headingConfig.accelerationHeadingTh);
        rsYamlRead(v2r_heading_node, "acceleration_valid",
                   customParams_->headingConfig.accelerationValid);
        customParams_->updateFromString();

        serializePtr_ = RsRobosenseV2RSerializeFactory::getInstance(customParams_->version);

        if (serializePtr_ == nullptr) {
            RERROR << "robosense v2r serialize create failed: version = " << customParams_->sVersion;
            return;
        }

        int ret = serializePtr_->init(*customParams_);

        if (ret != 0) {
            RERROR << "robosense v2r serialize initial failed: version = " << customParams_->sVersion;
            return;
        }
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
    // output: void.
    void deSerialization(const RsPerceptionMsg::Ptr &msg) override {
        (void)(msg);
    }

    // entrance of perception message deserialize.
    // input: robosense perception message and message buffer.
    // output: void.
    void deSerialization(const std::string& msg, std::vector<RsPerceptionMsg::Ptr>& res) override {
        native_sdk_3_1::st_RoboMsgHeader msg_header;
        method = customParams_->sMethod;
        msg_header.msgLocalLen=1;
        msg_header.msgLocalCnt=1;
        msg_header.msgTotalCnt=1;
        // 数据帧类型
        char data_type; 
        memcpy(&data_type, msg.c_str() + 3, 1);
        msg_header.msgType = (static_cast<int>(data_type)) ? 4095 : 4094;

        std::map<unsigned int, native_sdk_3_1::st_Robov2rRecvMessage>::iterator iterMap = receive_msg_.end();
        native_sdk_3_1::RSv2rDeserializeUtil deserialize_utils;
        {
            std::lock_guard<std::mutex> lg(buffer_mtx_);
            
            if (recieve_id >= frame_id) {
                iterMap = receive_msg_.find(recieve_id);
                if (iterMap == receive_msg_.end()) {
                    native_sdk_3_1::st_Robov2rRecvMessage msg_receive_status;
                    msg_receive_status.updateStatusByConfig();
                    msg_receive_status.msg.reset(new RsPerceptionMsg());
                    msg_receive_status.device_id = 0;
                    
                    receive_msg_.insert(std::pair<int,native_sdk_3_1::st_Robov2rRecvMessage>(
                    recieve_id, msg_receive_status));
                    iterMap = receive_msg_.find(recieve_id);

                }

                // deserialization
                if (iterMap != receive_msg_.end())  {
                    if (deserialize_utils.checkDeserializeStatus(iterMap->second, msg_header.msgType)) {
                        // if true, do nothing
                        // RINFO << "131 =====> msg_type = " << std::to_string(msg_header.msgType);
                    }
                    else {
                        native_sdk_3_1::ROBO_MSG_TYPE msgType
                        = static_cast<native_sdk_3_1::ROBO_MSG_TYPE>(msg_header.msgType);
                        native_sdk_3_1::st_Robov2rRecvMessage &msgs = iterMap->second;

                        // Binary Byte Deserialize
                        serializePtr_->deserialize(msg.c_str(),msg_header.msgLocalLen,
                                                            static_cast<native_sdk_3_1::ROBO_MSG_TYPE>(msg_header.msgType),
                                                            msgs); 

                        //std::cout<<msg_header.msgType<<std::endl;
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

                for (auto iterMap = receive_msg_.begin(); iterMap != receive_msg_.end();) {
                    unsigned int frameId = iterMap->first;
                    auto& msg_receive_status = iterMap->second;

                    if (deserialize_utils.checkMessageComplete(iterMap->second)) {
                        RTRACE << name() << ":" << method << " receive complete ! ";
                        // 接收完整，把处理好的RsPerceptionMsg::Ptr传递给回调函数
                        res.push_back(msg_receive_status.msg);
                        recieve_id++;

                        
                        for(int i=0;i<res[0]->rs_lidar_result_ptr->objects.size();i++)
                           {
                              Object::Ptr object_test = res[0]->rs_lidar_result_ptr->objects[i];
                              RDEBUG << "--------------物体: "<< i << "--------------";
                              RDEBUG << "--------------Time: "
                                    << std::to_string(res[0]->rs_lidar_result_ptr->timestamp)
                                    << "--------------";
                              RDEBUG << "区域: "<< object_test->supplement_infos_.roi_id<< ";"
                                    << "类型: "<< static_cast<int>(object_test->core_infos_.type)<< ";"
                                    << "跟踪ID: "<< object_test->core_infos_.tracker_id;
 



                              RDEBUG << "长度: " << object_test->core_infos_.size.x << ";"
                                    << "宽度: " << object_test->core_infos_.size.y << ";"
                                    << "高度: " << object_test->core_infos_.size.z;

                              RDEBUG << "朝向: " << object_test->core_infos_.direction.y << ";"
                                    << "速度: " << object_test->core_infos_.velocity.x << ";"
                                    << "航向: " << object_test->core_infos_.velocity.y;

                              RDEBUG << "加速度: " << object_test->core_infos_.acceleration.x << ";"
                                    << "加速度方向: " << object_test->core_infos_.acceleration.y ;
                     
                              RDEBUG << "经度: " << object_test->supplement_infos_.gps_longtitude << ";"
                                    << "纬度: " << object_test->supplement_infos_.gps_latitude ;
                                    
                              if(customParams_->sVersion == "ROBOSENSE_V2R_V1.5")
                              {
                                RDEBUG << "相对距离: " << object_test->core_infos_.center.y; 
                              }
                              if(customParams_->sVersion == "ROBOSENSE_V2R_V1.6")
                              {
                                RDEBUG << "中心点: " << "X: " << object_test->core_infos_.center.x << ";"
                                                    << "Y: " << object_test->core_infos_.center.y << ";"
                                                    << "Z: " << object_test->core_infos_.center.z;
                                RDEBUG << "海拔: " << object_test->supplement_infos_.gps_altitude;                             
                              }




                           }
                        frame_id = std::max(frame_id, int(iterMap->first));
                        iterMap = receive_msg_.erase(iterMap);
                        RINFO << name() << ":" << method << " receive successed: frameId = " << frameId;
                    }
                    else if (frameId < frame_id) {
                        iterMap = receive_msg_.erase(iterMap);
                        RWARNING << name()
                        << ":" << method << " receive out of order: frame_id = " << frameId
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
                RWARNING << name() << ":" << method << " receive out of order: frameid = "
                << recieve_id << ", not process !";
            }
        }
    }

    // implementation of serialization
    int serialize() {
        if (serializePtr_ == nullptr) {
            RERROR << "robosense v2r serialize is nullptr";
            return -1;
        }

        int ret = serializePtr_->serialize(msg_, serializeBuffer_);
        frame_id++;
        if (ret != 0) {
            RERROR << "robosense v2r serialize failed !";
            return -2;
        }
        return 0;
    }

    // entrance of getting the result of serialization.
    // input: a vector that used to save the buffers.
    // output: void. the result of serialization is added into the vector with a message header.
    int getSerializeMessage(std::vector<RsCharBufferPtr> &msgs) override {
        msgs.clear();
        size_t typesCnt = serializeBuffer_.types.size();
        native_sdk_3_1::st_RoboMsgHeader msgHeader;
        for (size_t i = 0; i < typesCnt; ++i) {
            const native_sdk_3_1::ROBO_MSG_TYPE msgType = serializeBuffer_.types[i];
            const int msgOffset = serializeBuffer_.offsets[i];
            const int msgLength = serializeBuffer_.lengths[i];

            // 生成消息头
            RsCharBufferPtr charBufferPtr(new RsCharBuffer());
            charBufferPtr->resize(msgLength, '\0');
            memcpy(charBufferPtr->data(), serializeBuffer_.buffers.data() + msgOffset, msgLength);

            msgs.push_back(charBufferPtr);
        }
        return 0;
    }

public:
    std::string method;
    std::string name() { return "v2r_msg"; }
    RsBaseRobosenseV2RSerialize::Ptr serializePtr_;
    native_sdk_3_1::RSSerializeBuffer serializeBuffer_;
    RsCustomRobosenseV2RMsgParams::Ptr customParams_;
    int frame_id = 0;
    std::mutex buffer_mtx_;
    int recieve_id = 1;
    std::map<unsigned int, native_sdk_3_1::st_Robov2rRecvMessage> receive_msg_;
};

}  // namespace perception
}  // namespace robosense

CEREAL_REGISTER_TYPE(robosense::perception::RsRobosenseV2RCustomMsg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(
    robosense::perception::RsBaseCustomMsg,
    robosense::perception::RsRobosenseV2RCustomMsg)
#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_V2R_ROBOSENSE_V2R_CUSTOM_MSG_H_
