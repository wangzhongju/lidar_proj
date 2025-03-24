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
#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_V2R_ROBOSENSE_V2R_CUSTOM_TRANSFORMER_UTILS_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_V2R_ROBOSENSE_V2R_CUSTOM_TRANSFORMER_UTILS_H_

#include "rs_dependence/rs_dependence_manager.h"


#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_perception/communication/external/common/basic_type.h"

namespace robosense {
namespace perception {
namespace native_sdk_3_1 {

struct st_Robov2rRecvMessage {
public:
    int device_id;
    double timestamp;
    std::map<ROBO_MSG_TYPE, st_RoboMsgRecorder> recorder;
    std::map<ROBO_MSG_TYPE, ROBO_MSG_DATA_RECV_STATUS> status;
    RsPerceptionMsg::Ptr msg;

public:
    st_Robov2rRecvMessage() {
        device_id = 0;
        timestamp = 0;
    };
public:
    // #ifdef CUSTOM_GE_COMM_FOUND
    //     void updateStatusByConfig(const st_CustomMessageConfig
    //     &fordMessageConfig);
    // #endif // CUSTOM_GE_COMM_FOUND

    // #ifdef CUSTOM_CR_COMM_FOUND
    //     void updateStatusByConfig(const st_CustomMessageConfig
    //     &beiqiMessageConfig);
    // #endif // CUSTOM_CR_COMM_FOUND

    void updateStatusByConfig() {
        status.clear();
        recorder.clear();

        // Message Receive Recorder
        st_RoboMsgRecorder msgRecorder;

        status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
        ROBO_MSG_TYPE::ROBO_MSG_V2R_DATA_MESSAGE,
        ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));

        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_V2R_DATA_MESSAGE, msgRecorder));


        status.insert(std::pair<ROBO_MSG_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
        ROBO_MSG_TYPE::ROBO_MSG_V2R_STATUS_MESSAGE,
        ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));

        recorder.insert(std::pair<ROBO_MSG_TYPE, st_RoboMsgRecorder>(
        ROBO_MSG_TYPE::ROBO_MSG_V2R_STATUS_MESSAGE, msgRecorder));
    }
};

class RSv2rDeserializeUtil {
public:
    using Ptr = std::shared_ptr<RSv2rDeserializeUtil>;
    using ConstPtr = std::shared_ptr<const RSv2rDeserializeUtil>;

public:
    bool checkDeserializeStatus(const st_Robov2rRecvMessage &recvMsg, uint16_t msgType) {
        // 发的不是空消息，但是不解析，返回true
        // 发的不是空消息，且要解析，返回false
        native_sdk_3_1::ROBO_MSG_TYPE msg_type = static_cast<native_sdk_3_1::ROBO_MSG_TYPE>(msgType);
        const auto& status_map = recvMsg.status;
        if (status_map.at(msg_type) == native_sdk_3_1::ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE) {
            return true;
        }
        else {
            return false;
        }
    }

    bool checkMessageComplete(st_Robov2rRecvMessage &recvMsg) {
        if (recvMsg.status[ROBO_MSG_TYPE::ROBO_MSG_V2R_DATA_MESSAGE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "1 Incomplete" << std::endl;
            return false;
        }

        // if (recvMsg.status[ROBO_MSG_TYPE::ROBO_MSG_V2R_STATUS_MESSAGE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
        //     //            std::cout << "2 Incomplete" << std::endl;
        //     return false;
        // }
        return true;
    }
};

}  // namespace native_sdk_3_1
}  // namespace perception
}  // namespace robosense


#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_V2R_ROBOSENSE_V2R_CUSTOM_TRANSFORMER_UTILS_H_
