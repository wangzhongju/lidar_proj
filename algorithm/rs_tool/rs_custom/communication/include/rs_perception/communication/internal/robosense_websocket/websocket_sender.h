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

#ifndef RS_PERCEPTION_COMMUNICATION_INTERNAL_ROBOSENSE_WEBSOCKET_WEBSOCKET_SENDER_H_
#define RS_PERCEPTION_COMMUNICATION_INTERNAL_ROBOSENSE_WEBSOCKET_WEBSOCKET_SENDER_H_

#include "rs_perception/communication/external/base_sender.h"
#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_perception/custom/common/base_custom_params.h"
#include "rs_dependence/rs_dependence_manager.h"

#ifdef ROBOSENSE_WEBSOCKET_FOUND

#include <boost/circular_buffer.hpp>

#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"

namespace robosense {
namespace perception {

class WebsocketSender : public BaseSender {
public:
    using Ptr = std::shared_ptr<WebsocketSender>;
    using server = websocketpp::server<websocketpp::config::asio>;

    virtual ~WebsocketSender() { stopServer(); }

    enum class RS_CONNECT_STATUE : int {
        RS_CONNECT_CLOSE = 0,
        RS_CONNECT_OPEN,
    };

    struct st_xvizConnectHdl {
        unsigned long long int hdlId;
        websocketpp::connection_hdl hdl;

        bool operator<(const st_xvizConnectHdl& val) const {
            return (hdlId < val.hdlId);
        }

        bool operator==(const st_xvizConnectHdl& val) const {
            return (hdlId == val.hdlId);
        }
    };

    // load configures from yaml and init websocket send function.
    // input: yaml node
    int init(const RsYamlNode& config_node) override;

    // entrance of sending perception result by socket
    // input: robosense perception message struct
    // output: a robosense communication error code.
    //         this code can indicate whether the transmission was successful or not.
    COMMUNICATION_ERROR_CODE send(const RsPerceptionMsg::Ptr& msg_ptr) override;

    // entrance of registering error code callback function
    // input: a callback function
    // output: void.
    void registerErrorComm(const CommunicaterErrorCallback& cb) override;

private:
    int initServer(const RsYamlNode &config_node_);

    void stopServer();

    void openHandle(websocketpp::connection_hdl hdl);

    void failHandle(websocketpp::connection_hdl hld);

    void closeHandle(websocketpp::connection_hdl hld);

    void receiveHandle(websocketpp::connection_hdl hdl, server::message_ptr msg);

    void xvizLiveThread();

    void xvizListenThread();

private:
    std::string name() { return "WebsocketSender"; }

    RsCustomWebsocketMsgParams::Ptr custom_websocket_params_;
    RsCommunicationParams communication_params;
    SocketParams params_;

    server _server;
    unsigned short _serverBindPort;
    std::atomic_bool _connectStop;
    std::thread _webSocketThread;
    std::thread _listenThread;
    websocketpp::connection_hdl _connectHdl;
    boost::circular_buffer<RsPerceptionMsg::Ptr> _messageBuffers;
    std::mutex _msgMutex;
    std::condition_variable _msgCond;

    native_sdk_3_1::RSSerializeBuffer _sendMessageBuffer;
    native_sdk_3_1::RSSerializeBuffer _sendMetadataBuffer;

    RsBaseCustomMsg::Ptr custom_msg;
    // Mutli-Connects Information
    std::map<st_xvizConnectHdl, RS_CONNECT_STATUE> _connectHdls;
    unsigned long long int _maxConnectHdlId;
    unsigned int _totalFrameCnt;
    int _web_update_frame_gap;
    int _web_buffer_size;
};

}  // namespace perception
}  // namespace robosense
#endif  // ROBOSENSE_WEBSOCKET_FOUND

#endif  // RS_PERCEPTION_COMMUNICATION_INTERNAL_ROBOSENSE_WEBSOCKET_WEBSOCKET_SENDER_H_
