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
#ifndef RS_SENSOR_CAN_COMMON_BASE_BASE_CAN_IO_H
#define RS_SENSOR_CAN_COMMON_BASE_BASE_CAN_IO_H

#include <vector>
#include <mutex>
#include <functional>
#include "rs_sensor2/can/common/base/base_can_msg.h"
#include "rs_sensor2/can/common/base/base_can_config.h"
#include "rs_sensor2/can/common/can_frame.h"

namespace robosense {
namespace sensor {

using CanExceptCallback = std::function<void(const int errNo, const std::string &)>;
// ChannelCanReceiveCallback's Parameters: canId, canData, canFlag, canStamp
using ChannelCanReceiveCallback = std::function<void(const std::string&, long,
                                                     const std::vector<unsigned char> &,
                                                     unsigned int, unsigned long)>;
using CanRecieveNativeCallback = ChannelCanReceiveCallback; 

class RSCanIOBasic {
public:
    using Ptr = std::shared_ptr<RSCanIOBasic>;
    using ConstPtr = std::shared_ptr<const RSCanIOBasic>;

public:
    RSCanIOBasic() {
        _canIoType = "CAN_IO_BASIC";
    }

    virtual ~RSCanIOBasic() {
        // TODO...
    }

public:
    void can_register_exception(const CanExceptCallback &callback) {
        if (callback == nullptr) {
            return;
        }
        std::lock_guard<std::mutex> lg(_excMtx);
        _excCallbacks.push_back(callback);
    }

    void can_register_callback(const CanRecieveNativeCallback& callback){
        if(callback == nullptr){
            return ; 
        }
        std::lock_guard<std::mutex> lg(_rcvNativeMtx); 
        _rcvNativeCallbacks.push_back(callback); 
    }

public:
    virtual int can_init(const RSCanConfigBasic::Ptr &basicCanConfigPtr) = 0;

    virtual bool can_is_open() = 0;

    virtual int can_start() = 0;

    virtual void can_stop() = 0;

protected:
    std::string _canIoType;
    std::mutex _excMtx;
    std::vector<CanExceptCallback> _excCallbacks;
    std::mutex _rcvNativeMtx; 
    std::vector<CanRecieveNativeCallback> _rcvNativeCallbacks; 
};

}  // namespace sensor
}  // namespace robosense

#endif  // RS_SENSOR_CAN_COMMON_BASE_BASE_CAN_IO_H
