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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_DTGH_CUSTOM_MSG_1_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_DTGH_CUSTOM_MSG_1_H_

#include "rs_perception/custom/robosense_custom/dtghCustom/dtgh_custom_transformer_1.h"

namespace robosense {
namespace perception {

class DtghMsg_1 {
public:
    using Ptr = std::shared_ptr<DtghMsg_1>;
    DtghMsg_1() {
        dtgh_res_ptr.reset(new Dtgh1::EventMsg);
    }
    Dtgh1::EventMsg::Ptr dtgh_res_ptr;
};

class RsDtgh_1CustomMsg : public RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsDtgh_1CustomMsg>;
    RsDtgh_1CustomMsg() {
        msg_.reset(new RsPerceptionMsg);
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
        rsYamlRead(custom_node, "send_object", custom_params_.send_object);
        rsYamlRead(custom_node, "send_event", custom_params_.send_event);
        rsYamlRead(custom_node, "send_traffic", custom_params_.send_traffic);

        serialize_ptr.reset(new Dtgh1::RsDtghSerialize);
        serialize_ptr->init();

    }

    // entrance of perception message serialize.
    // input: robosense perception message.
    // output: void. the result of serialization will be recorded in an internal variable.
    void serialization(const RsPerceptionMsg::Ptr& msg) override {
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

        serialize_ptr->serialize(msg_, serializeBuffer_, custom_params_, hd_map_disable_);
//        auto& any_map = msg_->rs_lidar_result_ptr->any_map;
//        if (any_map.find("hd_map_disable") != any_map.end()) {
//            Any::Ptr any_ptr_hd_map_disable = any_map["hd_map_disable"];
//            auto hd_map_disable = any_ptr_hd_map_disable->AnyCast<bool>();
//            if (!(*hd_map_disable)) {
//                serialize_ptr->serialize(msg_, serializeBuffer_, custom_params_);
//                return;
//            }
//        }
    }

    void deSerialization(const RsPerceptionMsg::Ptr& msg) override {
        (void)(msg);
    }

    void deSerialization(const std::string &msg, std::vector<RsPerceptionMsg::Ptr> & res) override {
        (void)(msg);
        (void)(res);
    }

    // entrance of getting the result of serialization.
    // input: a vector that used to save the buffers.
    // output: void. the result of serialization is added into the vector with a message header.
    int getSerializeMessage(std::vector<RsCharBufferPtr>& msgs) override {
        msgs.clear();

        if (hd_map_disable_) {
            RDEBUG << "hd_map is turned off, only send perception message!";
        }

        size_t typesCnt = serializeBuffer_.types.size();
        for (size_t i = 0; i < typesCnt; ++i) {
            const int msgOffset = serializeBuffer_.offsets[i];
            const int msgLength = serializeBuffer_.lengths[i];
            RsCharBufferPtr charBufferPtr(new RsCharBuffer);
            charBufferPtr->resize(msgLength, '\0');
            memcpy(charBufferPtr->data(), serializeBuffer_.buffers.data() + msgOffset, msgLength);
            msgs.push_back(charBufferPtr);
        }
        return 0;
    }

private:
    bool hd_map_disable_ = false;
    native_sdk_3_1::RSSerializeBuffer serializeBuffer_;

    DtghMsg_1::Ptr dtgh_msg_ptr;
    RsDtgh_1CustomMsgParams custom_params_;
    Dtgh1::RsDtghSerialize::Ptr serialize_ptr;
};

} // namespace perception
} // namespace robosense

CEREAL_REGISTER_TYPE(robosense::perception::RsDtgh_1CustomMsg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(robosense::perception::RsBaseCustomMsg,
                                     robosense::perception::RsDtgh_1CustomMsg)

#endif //RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_DTGH_CUSTOM_MSG_1_H_
