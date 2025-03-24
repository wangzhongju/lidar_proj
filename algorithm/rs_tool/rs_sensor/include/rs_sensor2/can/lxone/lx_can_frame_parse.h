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
#ifndef RS_SENSOR_CAN_LXONE_ONLINE_LX_CAN_FRAME_PARSE_H
#define RS_SENSOR_CAN_LXONE_ONLINE_LX_CAN_FRAME_PARSE_H

#include "rs_sensor2/can/lxone/lx_can_msg.h"
#include "rs_sensor2/can/common/can_frame.h"

namespace robosense {
namespace sensor {
namespace LX_CAN {

inline Basic_Msg::Ptr can_lx_frame_parse(const unsigned int Msg_Id) {
    Basic_Msg::Ptr msgPtr = nullptr;

    if (Msg_Id == LX_CAN::ESP_0x295().msgId()) {
        msgPtr.reset(new LX_CAN::ESP_0x295());
    } else if (Msg_Id == LX_CAN::ESP_0x225().msgId()) {
        msgPtr.reset(new LX_CAN::ESP_0x225);
    } else if (Msg_Id == LX_CAN::ESP_0x155().msgId()) {
        msgPtr.reset(new LX_CAN::ESP_0x155);
    } else if (Msg_Id == LX_CAN::ESP_0x46F().msgId()) {
        msgPtr.reset(new LX_CAN::ESP_0x46F);
    } else if (Msg_Id == LX_CAN::ESP_0x655().msgId()) {
        msgPtr.reset(new LX_CAN::ESP_0x655);
    } else if (Msg_Id == LX_CAN::ESP_0x675().msgId()) {
        msgPtr.reset(new LX_CAN::ESP_0x675);
    } else if (Msg_Id == LX_CAN::ESP_0x7CF().msgId()) {
        msgPtr.reset(new LX_CAN::ESP_0x7CF);
    } else if (Msg_Id == LX_CAN::ESP_0x460().msgId()) {
        msgPtr.reset(new LX_CAN::ESP_0x460);
    }

    return msgPtr;
}

inline Basic_Msg::Ptr can_lx_frame_process(const struct RSCanFrame &frame) {
    Basic_Msg::Ptr msgPtr = nullptr;

    if (frame.can_frame_ptr != nullptr) {

        msgPtr = can_lx_frame_parse(frame.can_frame_ptr->can_id);

        if (msgPtr != nullptr) {
            msgPtr->parseSFF(frame.can_frame_ptr->data);
        }
    } else if (frame.canfd_frame_ptr != nullptr) {

        msgPtr = can_lx_frame_parse(frame.canfd_frame_ptr->can_id);

        if (msgPtr != nullptr) {
            msgPtr->parseSFF(frame.canfd_frame_ptr->data);
        }
    }

    return msgPtr;
}

}  // namespace LX_CAN
}  // namespace sensor
}  // namespace robosense

#endif  // RS_SENSOR_CAN_LXONE_ONLINE_LX_CAN_FRAME_PARSE_H
