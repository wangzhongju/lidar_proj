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
#include "rs_localization/sender/internal/robosense_proto/proto_sender.h"

#ifdef RS_PROTO_FOUND
namespace robosense {
namespace localization {

int ProtoLocalizationSender::init(const RsYamlNode &config_node,
                                  const std::shared_ptr<LocalizationInterface> &localization_ptr) {
    localization_ptr_ = localization_ptr;
    params.load(config_node);
    params.log(name());
    status_cache.resize(8*1024*1024); // 8M
    cur_state_cache.resize(8*8*1024); //8M
    frameId_ = 0;
    int ret = initSocket(config_node);
    return ret;
}

void ProtoLocalizationSender::send() {
    while (params.start_flag_) {
        const double total_duration = 1.0 / params.localization_freq_ * 1000000.0;
        auto status = localization_ptr_->getModuleStatus().localization_status;
        auto start_time = getTime();
        static uint32_t prev_seq = 0;
        if (status != loc_status::LOC_LOST) {
            VehicleStateMsg cur_state;
            if (localization_ptr_->getVehicleState(cur_state) == common::ErrCode_Success) {
                if (cur_state.seq == prev_seq) {
                    continue;
                }

                int msg_length = getLocalizationProto(cur_state);
                getSerializeMessage(msg_length, cur_state.timestamp);
                sendLocalizationProto();

                prev_seq = cur_state.seq;
            }
        }
        auto end_time = getTime();
        auto exec_duration = (end_time - start_time) * 1000000;
        if (total_duration > exec_duration) {
            usleep(total_duration - exec_duration);
        }
    }
}

void ProtoLocalizationSender::start() {
    params.start_flag_ = true;
    auto func = [this]() {
        while (params.start_flag_) {
            localization::LocalizationInterface::ModuleStatus status = localization_ptr_->getModuleStatus();
            Proto_msg::Status proto_status;
            proto_status.set_status_int(status.status_int);

            status_cache.clear();
            status_cache.resize(status_length_cache);
            status_length_cache = proto_status.ByteSize();
            proto_status.SerializeToArray(status_cache.data(), status_length_cache);

            status_id++;
            sleep(1);
        }
    };
    std::thread t(func);
    t.detach();
    localization_sender_thread_ = std::thread(std::bind(&ProtoLocalizationSender::send, this));
}

void ProtoLocalizationSender::stop() {
    params.start_flag_ = false;
    if (localization_sender_thread_.joinable()) {
        localization_sender_thread_.join();
    }
}

int ProtoLocalizationSender::getLocalizationProto(VehicleStateMsg &cur_state) {
    Proto_msg::VehicleStateMsg proto_cur_state;
    toProtoMsg(cur_state, proto_cur_state);
    int cur_state_size = proto_cur_state.ByteSize();
    cur_state_cache.clear();
    cur_state_cache.resize(cur_state_size);
    proto_cur_state.SerializeToArray(cur_state_cache.data(), cur_state_size);
    return cur_state_size;
}

void ProtoLocalizationSender::getSerializeMessage(const int &msg_length, const double &timestamp) {
    charBuffers_.clear();
    robosense::perception::native_sdk_3_1::st_RoboMsgHeader msgHeader;

    // 制作共用消息头
    msgHeader.msgTimestampS = timestamp;
    msgHeader.deviceId = 0;
    msgHeader.msgTotalCnt = 1;
    msgHeader.msgFrameId = frameId_;

    // cur_state
    perception::native_sdk_3_1::ROBO_PROTO_DATA_TYPE msg_type
        = perception::native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LOC_STATUS;
    msgHeader.msgType = static_cast<int>(msg_type);
    msgHeader.msgLocalLen = msg_length;
    msgHeader.msgLocalCnt = 1;

    size_t total_msg_size = sizeof(perception::native_sdk_3_1::st_RoboMsgHeader) + msg_length;
    perception::RsCharBufferPtr charBufferPtr(new perception::RsCharBuffer);
    charBufferPtr->resize(total_msg_size, '\0');

    const char *msg_data = (const char*)(cur_state_cache.data()); // 起点

    msgHeader.toTargetEndianArray((unsigned char*)(charBufferPtr->data()),
                                  sizeof(perception::native_sdk_3_1::st_RoboMsgHeader),
                                  perception::RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

    memcpy(charBufferPtr->data() + sizeof(perception::native_sdk_3_1::st_RoboMsgHeader),
           msg_data, msg_length);

    charBuffers_.push_back(charBufferPtr);

    // state
    if (status_id != get_status_id) {
        get_status_id = status_id;
        std::vector<char> status_data = status_cache;

        msgHeader.msgLocalLen = status_length_cache;
        msg_type = perception::native_sdk_3_1::ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LOC_CUR_STATE;
        msgHeader.msgType = static_cast<int>(msg_type);
        msgHeader.msgLocalCnt = 1;

        total_msg_size = sizeof(perception::native_sdk_3_1::st_RoboMsgHeader) + msgHeader.msgLocalLen;
        charBufferPtr.reset(new perception::RsCharBuffer);
        charBufferPtr->resize(total_msg_size, '\0');

        const char *status_msg_data = (const char*)(status_data.data());

        msgHeader.toTargetEndianArray((unsigned char*)(charBufferPtr->data()),
                                      sizeof(perception::native_sdk_3_1::st_RoboMsgHeader),
                                      perception::RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        memcpy(charBufferPtr->data() + sizeof(perception::native_sdk_3_1::st_RoboMsgHeader),
               status_msg_data, msgHeader.msgLocalLen);

        charBuffers_.push_back(charBufferPtr);
    }
}

void ProtoLocalizationSender::sendLocalizationProto() {
    perception::COMMUNICATION_ERROR_CODE errCode = perception::COMMUNICATION_ERROR_CODE::Success;
    for (size_t i=0; i < charBuffers_.size(); i++) {
        if (socket_sender_ptr_ != nullptr) {
            errCode = socket_sender_ptr_->send(*(charBuffers_.at(i)));
        }
        else {
            errCode = perception::COMMUNICATION_ERROR_CODE::SocketSndError;
        }
    }

    frameId_++;
    if (errCode == perception::COMMUNICATION_ERROR_CODE::Success) {
        RINFO << "robosense native send localization msg frame_id = " << frameId_ << " successed !";
    }
}

RS_REGISTER_LOCALIZATION_SENDER(ProtoLocalizationSender)

}
}

#endif  // RS_PROTO_FOUND