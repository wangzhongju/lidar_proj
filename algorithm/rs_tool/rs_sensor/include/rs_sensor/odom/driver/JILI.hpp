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
#pragma once

#include "rs_common/external/common.h"
#include "rs_sensor/odom/adapter/driver_adapter.hpp"

namespace robosense {
namespace odom {
class OdomJILI : virtual public DriverAdapter {
public:
    OdomJILI() = default;

    ~OdomJILI() = default;

private:
    void init(const RsYamlNode &config) {
        direction_ = 0;
        gear_current_mode_ = 0;
        this->DriverAdapter::init(config);
    }

    void prepareMsg(vector <canbusData> buf);

    int direction_;
    int gear_current_mode_;
};

inline void OdomJILI::prepareMsg(vector <canbusData> buf) {
    using namespace robosense::common;

    OdomMsg odom_msg;
    for (auto it : buf) {
        // DEBUG<<"id"<<std::hex<<it.id<<REND;
        switch (it.id) {
            case (0x191): {
                switch (gear_current_mode_) {
                    case (0x45):
                        direction_ = -1;
                        break;
                    case (0xFF):
                        direction_ = 0;
                        break;
                    default:
                        direction_ = 1;
                        break;
                }
                break;
            }
            case (0x125): {
                odom_seq_++;
                int low = (it.data[2] & 0xf8) >> 3;
                int high = it.data[1];
                odom_msg.linear_vel[0] = ((high << 5 | low) * 0.05625) / 3.6;
                odom_msg.seq = odom_seq_;
                odom_msg.frame_id = odom_parameter_.frame_id;
                odom_msg.parent_frame_id = odom_msg.frame_id;
                odom_msg.timestamp = getTime();
                runCallBack(odom_msg);
            }
            default:
                break;
        }
    }
}
}  // namespace sensor
}  // namespace robosense
