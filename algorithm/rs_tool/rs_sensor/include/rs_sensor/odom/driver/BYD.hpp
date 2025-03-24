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
class OdomBYD : virtual public DriverAdapter {
public:
    OdomBYD() = default;

    ~OdomBYD() = default;

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

inline void OdomBYD::prepareMsg(vector <canbusData> buf) {
    using namespace robosense::common;
    OdomMsg odom_msg;
    for (auto it : buf) {
        switch (it.id) {
            case (0x35C): {
                gear_current_mode_ = it.data[1] & 0x7;

                switch (gear_current_mode_) {
                    case (0x2):
                        direction_ = -1;
                        break;
                    case (0x4):
                        direction_ = 1;
                        break;
                    default:
                        direction_ = 0;
                        break;
                }
                break;
            }
            case (0x121): {
                odom_seq_++;
                int low = it.data[0];
                int high = it.data[1] & 0xf;
                odom_msg.linear_vel[0] = ((high << 8 | low) * 0.06875 * direction_) / 3.6;
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
}  // namespace odom
}  // namespace robosense
