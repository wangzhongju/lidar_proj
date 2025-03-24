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

#include "rs_perception/communication/internal/robosense_native/native_sender.h"


namespace robosense {
namespace perception {

int NativeSender::init(const RsYamlNode &config_node_) {
    RsYamlNode config_node, general_node, socket_node, communicate_node, send_control_node;
    rsYamlSubNode(config_node_, "config", config_node);
    rsYamlSubNode(config_node, "general", general_node);
    rsYamlSubNode(config_node, "socket", socket_node);

    // general
    rsYamlRead(general_node, "device_id", deviceId_);

    // socket
    rsYamlRead(socket_node, "socket_address", params_.remote_ip);
    rsYamlRead(socket_node, "socket_port", params_.remote_port);
    rsYamlRead(socket_node, "socket_buffer_size", params_.socket_buffer_size);
    rsYamlRead(socket_node, "max_msg_size", params_.max_msg_size);
    rsYamlRead(socket_node, "timeout_ms", params_.timeout_ms);

    rsYamlSubNode(socket_node, "send_control", send_control_node);
    rsYamlRead(send_control_node, "send_control_enable",
               params_.send_control.send_control_enable);
    rsYamlRead(send_control_node, "send_control_thres",
               params_.send_control.send_control_thres);
    rsYamlRead(send_control_node, "send_control_ms",
               params_.send_control.send_control_ms);
    rsYamlRead(send_control_node, "send_control_compress_enable",
               params_.send_control.send_control_compress_enable);

    // Customized parameters
    std::string msg_name, custom_method;
    RsYamlNode native_node;
    rsYamlSubNode(config_node, "native", native_node);
    bool yesss = rsYamlRead(native_node, "custom_method", custom_method);
    msg_name = "Rs" + custom_method + "CustomMsg";
    custom_msg.reset(RsBaseCustomMsgRegisterer::getInstanceByName(msg_name));
    custom_msg->init(config_node);
    params_.log(name());

    // init send method
    send_method_ = ROBOSENSE_SEND_METHOD::UDP_SENDER;
    if (custom_method == "RobosenseV2R") {
        RsYamlNode v2r_node;
        std::string method, version;
        rsYamlSubNode(config_node, "v2r", v2r_node);
        rsYamlRead(v2r_node, "method", method);
        rsYamlRead(v2r_node, "version", version);
        custom_method = version;
        if (method == "TCP_CLIENT") {
            send_method_ = ROBOSENSE_SEND_METHOD::TCP_CLIENT_SENDER;
        }
        else if (method == "TCP_SERVER") {
            send_method_ = ROBOSENSE_SEND_METHOD::TCP_SERVER_SENDER;
        }
    }

    if (send_method_ == ROBOSENSE_SEND_METHOD::UDP_SENDER){
        try {
            socket_sender_ptr_.reset(new RsSocketUdpSender(params_));
        }
        catch (const std::exception &e) {
            return -1;
        }

        COMMUNICATION_ERROR_CODE errCode =
                socket_sender_ptr_->initSocket(std::vector<CommunicaterErrorCallback>());

        if (errCode != COMMUNICATION_ERROR_CODE::Success) {
            return -2;
        }
    }
    else if (send_method_ == ROBOSENSE_SEND_METHOD::TCP_SERVER_SENDER) {
        try {
            socket_tcp_server_sender_ptr_.reset(new RsSocketTcpSenderServer(params_));
        }
        catch (const std::exception &e) {
            return -3;
        }

        COMMUNICATION_ERROR_CODE errCode =
                socket_tcp_server_sender_ptr_->initSocket(std::vector<CommunicaterErrorCallback>());

        if (errCode != COMMUNICATION_ERROR_CODE::Success) {
            return -4;
        }
    }
    else if (send_method_ == ROBOSENSE_SEND_METHOD::TCP_CLIENT_SENDER) {
        try {
            socket_tcp_client_sender_ptr_.reset(new RsSocketTcpSenderClient(params_));
        }
        catch (const std::exception &e) {
            return -5;
        }

        COMMUNICATION_ERROR_CODE errCode =
                socket_tcp_client_sender_ptr_->initSocket(std::vector<CommunicaterErrorCallback>());

        if (errCode != COMMUNICATION_ERROR_CODE::Success) {
            return -6;
        }
    }
    frameId_ = 0;
    RINFO << "RsCommunication: " << custom_method << " method init success!";

    return 0;
}

COMMUNICATION_ERROR_CODE NativeSender::send(const RsPerceptionMsg::Ptr &msg_ptr) {
    std::vector<RsCharBufferPtr> charBuffers;
    charBuffers.clear();

    custom_msg->serialization(msg_ptr);
    custom_msg->getSerializeMessage(charBuffers);

    COMMUNICATION_ERROR_CODE errCode = COMMUNICATION_ERROR_CODE::Success;
    frameId_++;

    if (charBuffers.empty()) {
        RINFO << "robosense native send frame_id = " << frameId_ << " has no message !";
        return errCode;
    }

    for (size_t i=0; i < charBuffers.size(); i++) {
        if (socket_sender_ptr_ != nullptr) {
            errCode = socket_sender_ptr_->send(*(charBuffers.at(i)));
        }
        else if (socket_tcp_server_sender_ptr_ != nullptr) {
            errCode = socket_tcp_server_sender_ptr_->send(*(charBuffers.at(i)));
        }
        else if (socket_tcp_client_sender_ptr_ != nullptr) {
            errCode = socket_tcp_client_sender_ptr_->send(*(charBuffers.at(i)));
        }
        else {
            errCode = COMMUNICATION_ERROR_CODE::SocketSndError;
        }
    }


    if (errCode == COMMUNICATION_ERROR_CODE::Success) {
        RINFO << "robosense native send frame_id = " << frameId_ << " successed !";
    }

    return errCode;
}

void NativeSender::registerErrorComm(const CommunicaterErrorCallback &cb) {
    if (cb != nullptr) {
        socket_sender_ptr_->registerErrorComm(cb);
    }
}

RS_REGISTER_SENDER(NativeSender);

}  // namespace perception
}  // namespace robosense
