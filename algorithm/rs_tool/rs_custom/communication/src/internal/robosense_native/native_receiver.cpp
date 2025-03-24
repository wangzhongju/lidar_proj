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

#include "rs_perception/communication/internal/robosense_native/native_receiver.h"

namespace robosense {
namespace perception {

int NativeReceiver::init(const RsYamlNode &config_node_) {
    RsYamlNode config_node, general_node, socket_node, communicate_node, receive_control_node;
    rsYamlRead(config_node_, "method", communicate_method_);
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

    rsYamlSubNode(socket_node, "receive_control", receive_control_node);
    rsYamlRead(receive_control_node, "receive_io_parallel_enable",
               params_.receive_control.receive_io_parallel_enable);
    rsYamlRead(receive_control_node, "receive_io_parallel_degree",
               params_.receive_control.receive_io_parallel_degree);
    rsYamlRead(receive_control_node, "receive_process_parallel_enable",
               params_.receive_control.receive_process_parallel_enable);

    // Customized parameters
    std::string msg_name;
    if (communicate_method_ == "Native") {
        msg_name = "Rs" + communicate_method_ + "CustomMsg";
    }
    else if (communicate_method_ == "NativeBytes"
            || communicate_method_ == "Proto"
            || communicate_method_ == "RobosenseV2R") {
        std::string custom_method;
        RsYamlNode custom_node;
        rsYamlSubNode(config_node, "native", custom_node);
        rsYamlRead(custom_node, "custom_method", custom_method);
        msg_name = "Rs" + custom_method + "CustomMsg";
    }
    else if (communicate_method_ == "Custom") {
        // std::string custom_method;
        // RsYamlNode custom_node;
        // rsYamlSubNode(config_node, "custom", custom_node);
        // rsYamlRead(custom_node, "custom_method", custom_method);
        // msg_name = custom_method + "CustomMsg";
        // todo
    }

    custom_msg.reset(RsBaseCustomMsgRegisterer::getInstanceByName(msg_name));
    custom_msg->init(config_node);
    params_.log(name());

    if (communicate_method_ == "Native" || communicate_method_ == "Proto" || communicate_method_ == "NativeBytes") {
        try {
            socket_receiver_ptr_.reset(new RsSocketUdpReceiver(params_));
        }
        catch (const std::exception &e) {
            return -1;
        }
    } else if (communicate_method_ == "RobosenseV2R") {
        
        RsYamlNode v2r_node;
        std::string method;
        rsYamlSubNode(config_node, "v2r", v2r_node);
        rsYamlRead(v2r_node, "method", method);
    
        if (method == "TCP_CLIENT") {
            try {
                socket_tcp_server_receiver_ptr_.reset(new RsSocketTcpReceiverServer(params_));
            }
            catch (const std::exception &e) {
                return -2;
            }
        }
        else if (method == "TCP_SERVER") {
            try {
                socket_tcp_client_receiver_ptr_.reset(new RsSocketTcpReceiverClient(params_));
            }
            catch (const std::exception &e) {
                return -3;
            }
        } else {
            try {
                socket_receiver_ptr_.reset(new RsSocketUdpReceiver(params_));
            } 
            catch (const std::exception &e) {
                return -4;
            }
        }

    } else if (communicate_method_ == "Custom") {
        // todo
    }

    if (socket_receiver_ptr_ != nullptr) {
        socket_receiver_ptr_->registerRcvComm(std::bind(&NativeReceiver::localRecvCallback, this, std::placeholders::_1));

        COMMUNICATION_ERROR_CODE errCode = socket_receiver_ptr_->initSocket(std::vector<SocketReceiveCallback>(),std::vector<CommunicaterErrorCallback>());
        if (errCode != COMMUNICATION_ERROR_CODE::Success) {
            return -5;
        }
    } else if (socket_tcp_server_receiver_ptr_ != nullptr) {
        socket_tcp_server_receiver_ptr_->registerRcvComm(std::bind(&NativeReceiver::localRecvCallback, this, std::placeholders::_1));
        COMMUNICATION_ERROR_CODE errCode = socket_tcp_server_receiver_ptr_->initSocket(std::vector<SocketReceiveCallback>(),std::vector<CommunicaterErrorCallback>());
        if (errCode != COMMUNICATION_ERROR_CODE::Success) {
            return -6;
        }
    } else if (socket_tcp_client_receiver_ptr_ != nullptr) {
        socket_tcp_client_receiver_ptr_->registerRcvComm(std::bind(&NativeReceiver::localRecvCallback, this, std::placeholders::_1));
        COMMUNICATION_ERROR_CODE errCode = socket_tcp_client_receiver_ptr_->initSocket(std::vector<SocketReceiveCallback>(),std::vector<CommunicaterErrorCallback>());
        if (errCode != COMMUNICATION_ERROR_CODE::Success) {
            return -7;
        }
    }
    
    frameId_ = 0;
    RINFO << "RsCommunication: " << msg_name << " receive method init success!";
    return 0;
}

void NativeReceiver::registerRcvComm(const PerceptReceiveCallback &cb) {
    if (cb != nullptr) {
        std::lock_guard<std::mutex> lg(precept_recv_mtx_);
        percept_recv_cbs_.push_back(cb);
    }
}

void NativeReceiver::registerErrorComm(const CommunicaterErrorCallback &cb) {
    if (cb != nullptr) {
        socket_receiver_ptr_->registerErrorComm(cb);
    }
}

void NativeReceiver::localRecvCallback(const std::string &msg) {
//    if (communicate_method_ != "Native") {
//        RERROR << "SDK 3.1 Not Support " << communicate_method_ << " Data Receiver !";
//        return;
//    }

    std::vector<RsPerceptionMsg::Ptr> result;
    result.clear();
    custom_msg->deSerialization(msg, result);

    if (!result.empty()) {
        for (auto msg: result) {
            for(auto cb: percept_recv_cbs_) {
                if (cb != nullptr) {
                    cb(msg);
                }
            }
        }
    }

}

RS_REGISTER_RECEIVER(NativeReceiver);

}  // namespace perception
}  // namespace robosense
