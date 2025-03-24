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
#ifndef RS_SENSOR_CAN_COMMON_CAN_FRAME_H
#define RS_SENSOR_CAN_COMMON_CAN_FRAME_H

#include <cstring>
#include <memory>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace robosense {
namespace sensor {

enum class CAN_PROTOCOL_TYPE : int {
    PROTOCOL_TYPE_CAN = 0,
    PROTOCOL_TYPE_CANFD,
};

class RSCanFrame {
public:
    std::shared_ptr<can_frame> can_frame_ptr;
    std::shared_ptr<canfd_frame> canfd_frame_ptr;

public:
    RSCanFrame() {
        // TODO...
    }

    RSCanFrame(const CAN_PROTOCOL_TYPE type) {
        switch (type) {
            case CAN_PROTOCOL_TYPE::PROTOCOL_TYPE_CAN: {
                can_frame_ptr.reset(new can_frame);
                break;
            }
            case CAN_PROTOCOL_TYPE::PROTOCOL_TYPE_CANFD: {
                canfd_frame_ptr.reset(new canfd_frame);
                break;
            }
        }
    }

public:
    void update_can_frame(const can_frame &frame) {
        if (can_frame_ptr == nullptr) {
            can_frame_ptr.reset(new can_frame);
        }

        can_frame_ptr->can_id = frame.can_id;
        can_frame_ptr->can_dlc = frame.can_dlc;
        can_frame_ptr->__pad = frame.__pad;
        can_frame_ptr->__res0 = frame.__res0;
        memcpy(can_frame_ptr->data, frame.data, CAN_MAX_DLEN);
    }

    void update_canfd_frame(const canfd_frame &frame) {
        if (canfd_frame_ptr == nullptr) {
            canfd_frame_ptr.reset(new canfd_frame);
        }

        canfd_frame_ptr->can_id = frame.can_id;
        canfd_frame_ptr->len = frame.len;
        canfd_frame_ptr->__res0 = frame.__res0;
        canfd_frame_ptr->__res1 = frame.__res1;
        memcpy(canfd_frame_ptr->data, frame.data, CANFD_MAX_DLEN);
    }
};

}  // namespace sensor
}  // namespace robosense

#endif  // RS_SENSOR_CAN_COMMON_CAN_FRAME_H
