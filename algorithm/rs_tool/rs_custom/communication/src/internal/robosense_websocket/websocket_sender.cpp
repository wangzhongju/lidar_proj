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

#include "rs_perception/communication/internal/robosense_websocket/websocket_sender.h"


#ifdef ROBOSENSE_WEBSOCKET_FOUND

#include "rs_perception/custom/robosense_websocket/websocket_custom_msg.h"

namespace robosense {
namespace perception {

int WebsocketSender::init(const RsYamlNode &config_node_) {
    //  RDEBUG << name() << "=> config_node: " << config_node;
    RsYamlNode config_node;
    rsYamlSubNode(config_node_, "config", config_node);

    RsYamlNode general_node;
    rsYamlSubNode(config_node, "general", general_node);

    // general
    int device_id = 0;
    rsYamlRead(general_node, "device_id", device_id);

    // common parameters
    RsYamlNode common_node;
    rsYamlSubNode(config_node, "common", common_node);
    _web_buffer_size = common_node["socket_parallel_bufer_size"].as<int>();
    _serverBindPort = common_node["socket_listen_port"].as<unsigned int>();
    _web_update_frame_gap = common_node["web_update_frame_gap"].as<unsigned int>();

    // init sender server
    int ret = initServer(config_node);
    if (ret != 0) {
        RERROR << name() << ": initial websocket server failed !";
        return -2;
    }
    RINFO << "robosense websocket init successed!";
    return 0;
}

COMMUNICATION_ERROR_CODE WebsocketSender::send(const RsPerceptionMsg::Ptr &msg_ptr) {
    std::lock_guard<std::mutex> lg(_msgMutex);
    _totalFrameCnt++;
    if (_totalFrameCnt % _web_update_frame_gap == 0) {
        _messageBuffers.push_back(msg_ptr);
        _msgCond.notify_one();
    }
    RINFO << "robosense websocket send successed!";
    return COMMUNICATION_ERROR_CODE::Success;
}

void WebsocketSender::registerErrorComm(const CommunicaterErrorCallback &cb) {
    if (cb != nullptr) {
        socket_sender_ptr_->registerErrorComm(cb);
    }
}

int WebsocketSender::initServer(const RsYamlNode &config_node_) {
    _connectStop = true;

    _messageBuffers = boost::circular_buffer<RsPerceptionMsg::Ptr>(
    _web_buffer_size < 2 ? 2 : _web_buffer_size);

    _totalFrameCnt = 0;

    _maxConnectHdlId = 0;

    // Initial Server
    _server.set_error_channels(websocketpp::log::elevel::all);

    _server.set_access_channels(websocketpp::log::alevel::none);

    _server.init_asio();

    _server.set_reuse_addr(true);

    _server.set_open_handler(std::bind(&WebsocketSender::openHandle, this, std::placeholders::_1));

    _server.set_fail_handler(std::bind(&WebsocketSender::failHandle, this, std::placeholders::_1));

    _server.set_close_handler(std::bind(&WebsocketSender::closeHandle, this, std::placeholders::_1));

    _server.set_message_handler(std::bind(&WebsocketSender::receiveHandle, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));

    custom_msg.reset(RsBaseCustomMsgRegisterer::getInstanceByName("RsWebsocketCustomMsg"));
    custom_msg->init(config_node_);

    RsWebsocketCustomMsg::Ptr websocketCustomMsgPtr = std::static_pointer_cast<RsWebsocketCustomMsg>(custom_msg);
    if (websocketCustomMsgPtr == nullptr) {
        return -1;
    }

    _sendMetadataBuffer.clearInfo();
    websocketCustomMsgPtr->getSerializeMeta(_sendMetadataBuffer);

    _connectStop = false;
    _webSocketThread = std::move(std::thread(&WebsocketSender::xvizLiveThread, this));
    _listenThread = std::move(std::thread(&WebsocketSender::xvizListenThread, this));

    return 0;
}

void WebsocketSender::stopServer() {
    if (_connectStop == false) {
        _connectStop = true;
        _msgCond.notify_all();

        if (_webSocketThread.joinable()) {
            _webSocketThread.join();
        }
    }

    _server.stop();
}

void WebsocketSender::openHandle(websocketpp::connection_hdl hdl) {
    if (_sendMetadataBuffer.lengths[0] > 0) {
        _server.send(hdl, _sendMetadataBuffer.buffers.data(),
                     _sendMetadataBuffer.lengths[0],
                     websocketpp::frame::opcode::binary);
    }

    st_xvizConnectHdl connectHdl;
    connectHdl.hdlId = _maxConnectHdlId;
    connectHdl.hdl = hdl;

    ++_maxConnectHdlId;

    _connectHdls.insert(std::pair<st_xvizConnectHdl, RS_CONNECT_STATUE>(
    connectHdl, RS_CONNECT_STATUE::RS_CONNECT_OPEN));

    RINFO << ": New Connect: hdlId = " << connectHdl.hdlId << "==> Current Connect Count = " << _connectHdls.size();
}

void WebsocketSender::failHandle(websocketpp::connection_hdl hld) {
    (void)hld;
    RINFO << ": websocket connect fail ";
}

void WebsocketSender::closeHandle(websocketpp::connection_hdl hld) {
    for (auto iterMap = _connectHdls.begin(); iterMap != _connectHdls.end(); ++iterMap) {
        if (iterMap->first.hdl.lock() == hld.lock()) {
            iterMap->second = RS_CONNECT_STATUE::RS_CONNECT_CLOSE;
            RINFO << ": websocket connect close: " << iterMap->first.hdlId;
        }
    }
}

void WebsocketSender::receiveHandle(websocketpp::connection_hdl hdl, server::message_ptr msg) {
    (void)hdl;
    RINFO << "receiveHandle: " << std::string(msg->get_payload());
}

void WebsocketSender::xvizLiveThread() {
    RsWebsocketCustomMsg::Ptr websocketCustomMsgPtr = std::static_pointer_cast<RsWebsocketCustomMsg>(custom_msg);
    if (websocketCustomMsgPtr == nullptr) {
        return;
    }

    while (!_connectStop) {
    // Step1: If No Any Connect Hdl
        if (_connectHdls.size() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::lock_guard<std::mutex> lg(_msgMutex);
            _messageBuffers.clear();
            RINFO << "No Any Connect..." << std::endl;
            continue;
        }

        {
            // Step1: Safely Get Message
            RsPerceptionMsg::Ptr msg = nullptr;
            {
                std::unique_lock<std::mutex> lg(_msgMutex);
                _msgCond.wait(lg, [&] {return (_messageBuffers.size() > 0 || _connectStop == true);});
                if (_connectStop == true) {
                    break;
                }

                // Message
                msg = _messageBuffers.front();
                _messageBuffers.pop_front();
            }

            if (msg != nullptr) {
                // std::cout << __FUNCTION__ << " ==> " << __LINE__ << std::endl;
                _sendMessageBuffer.clearInfo();
                websocketCustomMsgPtr->serialization(msg);
                int ret = websocketCustomMsgPtr->getSerializeMessage(_sendMessageBuffer);
                if (ret != 0) {
                    RERROR << "get serialize websocket message data failed";
                    continue;
                }
                // std::cout << __FUNCTION__ << " ==> " << __LINE__ << std::endl;

                // Step3: Delete Closed Connect Hdl
                for (auto iterMap = _connectHdls.begin(); iterMap != _connectHdls.end();) {
                    RS_CONNECT_STATUE statue = iterMap->second;
                    if (statue == RS_CONNECT_STATUE::RS_CONNECT_CLOSE) {
                        iterMap = _connectHdls.erase(iterMap);
                    }
                    else {
                        ++iterMap;
                    }
                }

                for (auto iterMap = _connectHdls.begin(); iterMap != _connectHdls.end(); ++iterMap) {
                    // std::cout << __FUNCTION__ << " ==> " << __LINE__ << std::endl;
                    _connectHdl = iterMap->first.hdl;

                    if (iterMap->second == RS_CONNECT_STATUE::RS_CONNECT_OPEN) {
                        try {
                            _server.send(_connectHdl, _sendMessageBuffer.buffers.data(),
                                         _sendMessageBuffer.lengths[0], websocketpp::frame::opcode::binary);
                        }
                        catch (const std::exception &e) {
                            RERROR << "WebSocket Error: " << e.what() << std::endl;
                            iterMap->second = RS_CONNECT_STATUE::RS_CONNECT_CLOSE;
                        }
                    }
                }
            }
        }
    }
}

void WebsocketSender::xvizListenThread() {
    _server.listen(_serverBindPort);

    _server.start_accept();

    _server.run();
}

RS_REGISTER_SENDER(WebsocketSender);


}  // namespace perception
}  // namespace robosense

#endif  // ROBOSENSE_WEBSOCKET_FOUND