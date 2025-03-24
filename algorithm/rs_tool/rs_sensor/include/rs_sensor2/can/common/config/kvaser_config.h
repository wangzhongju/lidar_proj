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
#ifndef RS_SENSOR_CAN_COMMON_CONFIG_KVASER_CONFIG_H
#define RS_SENSOR_CAN_COMMON_CONFIG_KVASER_CONFIG_H

#include <vector>
#include <linux/can.h>
#include "rs_sensor2/can/common/base/base_can_config.h"

namespace robosense {
namespace sensor {

class RSSingleKvaserCanConfig {
public:
    std::string vehicleType; 
    int kvaserChannle;
    int kvaserOpenMode;
    int kvaserFreq;
    int kvaserControlMode;
    int affinity_id;
    std::vector<struct can_filter> filters;
public: 
    RSSingleKvaserCanConfig() : vehicleType("lx_one_black")
    {
        
    }
};

class RSKvaserCanConfig : public RSCanConfigBasic {
public:
    using Ptr = std::shared_ptr<RSKvaserCanConfig>;
    using ConstPtr = std::shared_ptr<const RSKvaserCanConfig>;

public:
    RSKvaserCanConfig() {
        configName = "KVASER_CAN_CONFIG";
    }

    virtual ~RSKvaserCanConfig() {
        //TODO...
    }

public:
    std::vector<RSSingleKvaserCanConfig> kvaserCanConfigs;
};

}  // namespace sensor
}  // namespace robosense

#endif  // RS_SENSOR_CAN_COMMON_CONFIG_KVASER_CONFIG_H
