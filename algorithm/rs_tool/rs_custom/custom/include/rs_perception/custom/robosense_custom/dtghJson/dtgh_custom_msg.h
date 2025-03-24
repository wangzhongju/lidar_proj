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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_DTGH_CUSTOM_MSG_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_DTGH_CUSTOM_MSG_H_

#include "rs_perception/custom/robosense_custom/dtghJson/dtgh_custom_transformer.h"

namespace robosense {
namespace perception {

class DtghMsg {
public:
    using Ptr = std::shared_ptr<DtghMsg>;

    DtghMsg(){
        dtgh_res_ptr.reset(new Dtgh::EventMsg);
    }
    Dtgh::EventMsg::Ptr dtgh_res_ptr;
};

class RsDtghCustomMsg : public RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsDtghCustomMsg>;

    RsDtghCustomMsg() {
        msg_.reset(new RsPerceptionMsg);
        dtgh_msg_ptr.reset(new DtghMsg);
    }

    // load configures from yaml and init custom message serialize function.
    // input: yaml node
    void init(const RsYamlNode &config_node_) override {
        
        RsYamlNode general_node, custom_node, control_node, send_control_node;
        rsYamlSubNode(config_node_, "general", general_node);
        rsYamlSubNode(config_node_, "native", custom_node);
        rsYamlSubNode(config_node_, "socket", control_node);

        // general
        rsYamlRead(general_node, "device_id", custom_params_.device_id);

        // custom
        rsYamlRead(custom_node, "enable_upload", custom_params_.enable_upload);
        rsYamlRead(custom_node, "enable_lidar_object", custom_params_.enable_lidar_object);
        rsYamlRead(custom_node, "enable_wgs84_object", custom_params_.enable_wgs84_object);
        rsYamlRead(custom_node, "upload_time_ms", custom_params_.upload_time_ms);
        rsYamlRead(custom_node, "lidar_work_time_ms", custom_params_.lidar_work_time_ms);
        rsYamlRead(custom_node, "keep_alive_time_ms", custom_params_.keep_alive_time_ms);
        rsYamlRead(custom_node, "device_time_status_ms", custom_params_.device_time_status_ms);
        rsYamlRead(custom_node, "device_no", custom_params_.device_no);
        rsYamlRead(custom_node, "mec_no", custom_params_.mec_no);

        send_frequent = custom_params_.upload_time_ms / custom_params_.lidar_work_time_ms;

        event_convert_ptr.reset(new Dtgh::EventConvert);
        event_convert_ptr->init();

        keep_alive_.init(custom_params_);
        device_status_.init(custom_params_);

    }

    // entrance of perception message serialize.
    // input: robosense perception message.
    // output: void. the result of serialization will be recorded in an internal variable.
    void serialization(const RsPerceptionMsg::Ptr &msg) override {
        msg_ = msg;
        auto& any_map = msg_->rs_lidar_result_ptr->any_map;
        if (any_map.find("hd_map_disable") != any_map.end()) {
            Any::Ptr any_ptr_hd_map_disable = any_map["hd_map_disable"];
            auto hd_map_disable = any_ptr_hd_map_disable->AnyCast<bool>();
            hd_map_disable_ = *hd_map_disable;
        }
        else {
            hd_map_disable_ = true;
        }

        event_convert_ptr->convert(msg_, dtgh_msg_ptr->dtgh_res_ptr, hd_map_disable_);
        serializeToJson();
    }

    void deSerialization(const RsPerceptionMsg::Ptr &msg) override {
        // 自行定义转换函数
        (void)(msg);
    }

    void deSerialization(const std::string& msg, std::vector<RsPerceptionMsg::Ptr>& res) override {
        (void)(msg);
        (void)(res);
    }

    void serializeToJson() {
        charBuffers_.clear();
        if (!custom_params_.enable_upload) {
            RINFO << "upload disable! No perception result upload!";
            return;
        }

        std::string deviceNo = custom_params_.device_no;
        const int FixDeviceNoLen = 20;
        while (deviceNo.size() < FixDeviceNoLen) {
            deviceNo = std::string("0") + deviceNo;
        }
        deviceNo.resize(FixDeviceNoLen);

        if (frame_id % send_frequent == 0) {
            std::string sTimestamp = Dtgh::RSDtghCommUtil::Timestamp2time1(msg_->rs_lidar_result_ptr->timestamp);
            RsCharBufferPtr charBufferPtr(new RsCharBuffer());
            // Object 信息
            if (custom_params_.enable_lidar_object && dtgh_msg_ptr->dtgh_res_ptr->obj_send_ptr) {
                Dtgh::ObjectSend info = *(dtgh_msg_ptr->dtgh_res_ptr->obj_send_ptr);
                // 更新消息头
                info.MsgType = static_cast<int>(Dtgh::RS_DTGH_MSGTYPE::RS_DTGH_LIDAROBJECT);
                info.DevNo = custom_params_.device_no;
                info.MecNo = custom_params_.mec_no;
                info.Timestamp = sTimestamp;

                // for test
//                RINFO << "Object size: " << info.Obj_List.size() << " timestamp: " << info.Timestamp;

                // 进行序列化
                std::string encodeData;
                Dtgh::RSCustomSerialize::serialize(info, deviceNo, encodeData);
                // 保存序列化信息
                charBufferPtr->resize(encodeData.size(), '\0');
                memcpy(charBufferPtr->data(), encodeData.data(), encodeData.size());
                charBuffers_.push_back(charBufferPtr);
//                RINFO << "object message add success!";
            }
            charBufferPtr.reset(new RsCharBuffer());
            if (custom_params_.enable_wgs84_object && !hd_map_disable_ && dtgh_msg_ptr->dtgh_res_ptr->event_detect) {
                Dtgh::EventSend info = *(dtgh_msg_ptr->dtgh_res_ptr->event_send_ptr);
                // 更新消息头
                info.MsgType = static_cast<int>(Dtgh::RS_DTGH_MSGTYPE::RS_DTGH_WGS84OBJECT);
                info.DevNo = custom_params_.device_no;
                info.MecNo = custom_params_.mec_no;
                info.Timestamp = sTimestamp;
//                RINFO << 139;
                // for test
//                RINFO << "Event size: " << info.Evt_List.size() << " timestamp: " << info.Timestamp;

                // 进行序列化
                if (info.Evt_List.size() > 0) {
                    std::string encodeData;
                    Dtgh::RSCustomSerialize::serialize(info, deviceNo, encodeData);
                    // 保存序列化消息
                    charBufferPtr->resize(encodeData.size(), '\0');
                    memcpy(charBufferPtr->data(), encodeData.data(), encodeData.size());
                    charBuffers_.push_back(charBufferPtr);
//                    RINFO << "event message add success!";
                }
            }
        }

        frame_id++;
    }

    // entrance of getting the result of serialization.
    // input: a vector that used to save the buffers.
    // output: void. the result of serialization is added into the vector with a message header.
    int getSerializeMessage(std::vector<RsCharBufferPtr>& msgs) override {
        msgs.clear();
        msgs = charBuffers_;

        if (hd_map_disable_) {
            RDEBUG << "Event detect is turned off, only send perception message.";
            return -1;
        }

        Any::Ptr any_ptr_event_detect = msg_->rs_lidar_result_ptr->any_map.at("event_detect");
        auto event_detect = any_ptr_event_detect->AnyCast<bool>();
        if (!(*event_detect)) {
            return -1;
        }

        std::string data;
        RsCharBufferPtr charBufferPtr(new RsCharBuffer());

        // 插入心跳信息
        std::string new_keep_alive_timestamp;
        std::vector<std::string> keep_alive_tmp = keep_alive_.getTimestampAndRes();
        new_keep_alive_timestamp = keep_alive_tmp[0];
        if (keep_alive_timestamp != new_keep_alive_timestamp) {
            keep_alive_timestamp = new_keep_alive_timestamp;
            data = keep_alive_tmp[1];
            charBufferPtr->resize(data.size(), '\0');
            memcpy(charBufferPtr->data(), data.data(), data.size());
            msgs.push_back(charBufferPtr);
//            RINFO << "keep alive message add success!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
        }

        // 插入设备信息
        std::string new_device_status_timestamp;
        std::vector<std::string> device_status_tmp = device_status_.getTimestampAndRes();
        new_device_status_timestamp = device_status_tmp[0];
        if (device_status_timestamp != new_device_status_timestamp) {
            device_status_timestamp = new_device_status_timestamp;
            data.clear();
//            RINFO << "DEVICE DATA SIZE: " << device_status_.encodeData_.size();
            data = device_status_tmp[1];
            charBufferPtr.reset(new RsCharBuffer);
            charBufferPtr->resize(data.size(), '\0');
            memcpy(charBufferPtr->data(), data.data(), data.size());
            msgs.push_back(charBufferPtr);
//            RINFO << "device status message add success!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
        }
        return 0;
    }

private:
    RsDtghCustomMsgParams custom_params_;
    Dtgh::EventConvert::Ptr event_convert_ptr;
    DtghMsg::Ptr dtgh_msg_ptr;
    bool hd_map_disable_ = false;
    std::vector<RsCharBufferPtr> charBuffers_;
    Dtgh::RSCustomKeepAlive keep_alive_;
    Dtgh::RSCustomDeviceStatus device_status_;
    std::string keep_alive_timestamp, device_status_timestamp;
    int send_frequent = 0;
    int frame_id = 0;
};

}
}
CEREAL_REGISTER_TYPE(robosense::perception::RsDtghCustomMsg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(robosense::perception::RsBaseCustomMsg,
                                     robosense::perception::RsDtghCustomMsg)

#endif //RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_DTGH_CUSTOM_MSG_H_
