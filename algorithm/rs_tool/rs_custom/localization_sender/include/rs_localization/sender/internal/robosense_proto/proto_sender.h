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
#ifndef RS_LOCALIZATION_INTERNAL_ROBOSENSE_PROTO_PROTO_SENDER_H
#define RS_LOCALIZATION_INTERNAL_ROBOSENSE_PROTO_PROTO_SENDER_H

#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_PROTO_FOUND

#include "rs_localization/sender/external/base_sender.h"
#include "rs_localization/sender/internal/robosense_proto/proto_msg_translator.h"
#include "rs_perception/communication/external/common/basic_type.h"

namespace robosense {
namespace localization {

class ProtoLocalizationSender : public BaseLocalizationSender {
public:
    using Ptr = std::shared_ptr<ProtoLocalizationSender>;

    // load configures from yaml and init the localization info protobuf sender function.
    // input: yaml node
    // output: void.
    int init(const RsYamlNode& config_node, const std::shared_ptr<LocalizationInterface>& localization_ptr) override;

    // entrance of sending localization info message
    // input: void.
    // output: void. the localization info will be serialized by protobuf and sent by socket udp.
    void send() override;

    // start the thread and wait for data.
    void start() override;

    // stop the thread.
    void stop() override;

private:
    std::string name() {
        return "ProtoLocalizationSender";
    }
    std::thread localization_sender_thread_;
    std::thread t_;
    int status_id = 0, get_status_id = 0;
    std::vector<char> status_cache, cur_state_cache;
    int status_length_cache;
    std::vector<robosense::perception::RsCharBufferPtr> charBuffers_;
    int frameId_;

    struct Params {
        int localization_freq_ = 30;
        bool start_flag_ = false;
        bool send_pos_and_path_ = false;

        void load(const RsYamlNode& config_node) {
            rsYamlRead(config_node, "localization_freq_", localization_freq_);
            rsYamlRead(config_node, "send_pos_and_path_ros", send_pos_and_path_);
        }

        void log(const std::string& name) {
            RINFO << "localization_freq_ " << localization_freq_;
            RINFO << "send_pos_and_path_ros_ " << send_pos_and_path_;
        }
    }params;

    void getSerializeMessage(const int& msg_length, const double& timestamp);

    int getLocalizationProto(VehicleStateMsg& cur_state);

    void sendLocalizationProto();

    int initSocket(const RsYamlNode& config_node) {
        RsYamlNode socket_node, send_control_node;
        rsYamlSubNode(config_node, "socket", socket_node);
        rsYamlRead(socket_node, "socket_address", socket_params_.remote_ip);
        rsYamlRead(socket_node, "socket_port", socket_params_.remote_port);
        rsYamlRead(socket_node, "socket_buffer_size", socket_params_.socket_buffer_size);
        rsYamlRead(socket_node, "max_msg_size", socket_params_.max_msg_size);
        rsYamlRead(socket_node, "timeout_ms", socket_params_.timeout_ms);

        rsYamlSubNode(socket_node, "send_control", send_control_node);
        rsYamlRead(send_control_node, "send_control_enable",
                   socket_params_.send_control.send_control_enable);
        rsYamlRead(send_control_node, "send_control_thres",
                   socket_params_.send_control.send_control_thres);
        rsYamlRead(send_control_node, "send_control_ms",
                   socket_params_.send_control.send_control_ms);
        rsYamlRead(send_control_node, "send_control_compress_enable",
                   socket_params_.send_control.send_control_compress_enable);

        socket_params_.log(name());

        try {
            socket_sender_ptr_.reset(new perception::RsSocketUdpSender(socket_params_));
        }
        catch (const std::exception &e) {
            return -1;
        }

        perception::COMMUNICATION_ERROR_CODE errCode =
        socket_sender_ptr_->initSocket(std::vector<perception::CommunicaterErrorCallback>());

        if (errCode != perception::COMMUNICATION_ERROR_CODE::Success) {
            return -2;
        }
        return 0;
    }
};

}
}

#endif  // RS_PROTO_FOUND
#endif  // RS_LOCALIZATION_INTERNAL_ROBOSENSE_PROTO_PROTO_SENDER_H
