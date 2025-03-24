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
#ifndef RS_SENSOR_CAN_COMMON_CAN_PARSER_H
#define RS_SENSOR_CAN_COMMON_CAN_PARSER_H

#include "rs_sensor2/can/common/can_frame.h"

namespace robosense {
namespace sensor {

inline RSCanFrame can_frame_parse(unsigned int canId, unsigned int canLen,
                                  const unsigned char *canBuffer) {
    RSCanFrame rs_frame;
        if (canLen <= 8) {
        can_frame frame;
        frame.can_id = canId;
        frame.can_dlc = canLen;
        memcpy(frame.data, canBuffer, canLen);

        rs_frame = RSCanFrame(CAN_PROTOCOL_TYPE::PROTOCOL_TYPE_CAN);
        rs_frame.update_can_frame(frame);
    } else {
        canfd_frame frame;
        frame.can_id = canId;
        frame.len = canLen;
        memcpy(frame.data, canBuffer, canLen);

        rs_frame = RSCanFrame(CAN_PROTOCOL_TYPE::PROTOCOL_TYPE_CANFD);
        rs_frame.update_canfd_frame(frame);
    }
    return rs_frame;
}

}  // namespace sensor
}  // namespace robosense

#endif  // RS_SENSOR_CAN_COMMON_CAN_PARSER_H
