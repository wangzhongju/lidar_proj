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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_WEBSOCKET_WEBSOCKET_CUSTOM_MSG_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_WEBSOCKET_WEBSOCKET_CUSTOM_MSG_H_
#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_perception/custom/robosense_websocket/websocket_custom_transformer.h"

#ifdef ROBOSENSE_WEBSOCKET_FOUND

namespace robosense {
namespace perception {

class RsWebsocketCustomMsg : public RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsWebsocketCustomMsg>;
    using ConstPtr = std::shared_ptr<const RsWebsocketCustomMsg>;

public:

    // load configures from yaml and init websocket perception message serialize function.
    // input: yaml node
    void init(const RsYamlNode &config_node_) override {
        customParams_.reset(new RsCustomWebsocketMsgParams);
        int ret = 0;
        ret = customParams_->parserWebsocketConfig(config_node_);
        if (ret != 0) {
            RERROR  << name() << ": parse websocket params failed!";
        }

        serializePtr_.reset(new RsWebsocketSerialize(*customParams_));

        ret = serializePtr_->init(-1);

        if (ret != 0) {
            RERROR << "robosense websocket serialize initial failed";
            return;
        }

        ret = serializePtr_->serialize(
        nullptr, native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_WEBSOCEKT_METADATA,
        metaSerializeBuffer_);

        if (ret != 0) {
            RERROR << "robosense websocket serialize metadata failed";
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
        (void)(msg);
        (void)(res);
    }

    // entrance of getting the result of serialization.
    // input: a vector that used to save the buffers.
    // output: void.
    int getSerializeMessage(std::vector<RsCharBufferPtr>& msgs) override {
        (void)(msgs);
    }
    // implementation of serialization
    int serialize() {
        if (serializePtr_ == nullptr) {
            RERROR << "robosense v2r serialize is nullptr";
            return -1;
        }

        int ret = serializePtr_->serialize(
        msg_, native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_WEBSOCKET_MESSAGE,
        msgSerializeBuffer_);

        if (ret != 0) {
            RERROR << "robosense v2r serialize failed !";
            return -2;
        }

        return 0;
    }

    int getSerializeMeta(native_sdk_3_1::RSSerializeBuffer &buffer) {
        buffer = metaSerializeBuffer_;
        return 0;
    }

    int getSerializeMessage(native_sdk_3_1::RSSerializeBuffer &buffer) {
        buffer = msgSerializeBuffer_;
        return 0;
    }

    std::string name() {
        return "WebSocket";
    }

public:
    RsWebsocketSerialize::Ptr serializePtr_;
    native_sdk_3_1::RSSerializeBuffer msgSerializeBuffer_;
    native_sdk_3_1::RSSerializeBuffer metaSerializeBuffer_;
    RsCustomWebsocketMsgParams::Ptr customParams_;
};

}  // namespace perception
}  // namespace robosense

CEREAL_REGISTER_TYPE(robosense::perception::RsWebsocketCustomMsg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(
    robosense::perception::RsBaseCustomMsg,
    robosense::perception::RsWebsocketCustomMsg)

#endif  // ROBOSENSE_WEBSOCKET_FOUND

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_WEBSOCKET_WEBSOCKET_CUSTOM_MSG_H_
