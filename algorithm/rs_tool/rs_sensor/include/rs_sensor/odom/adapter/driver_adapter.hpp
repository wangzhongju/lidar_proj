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
#include "rs_sensor/adapter_base/adapter_base.h"
#include "rs_sensor/odom/can/can_bridge.hpp"

namespace robosense {
namespace odom {
class DriverAdapter : virtual public sensor::AdapterBase {
public:
    DriverAdapter() = default;

    ~DriverAdapter() { stop(); }

    virtual void init(const RsYamlNode &config);

    void start();

    void stop();

    inline void regRecvCallback(const std::function<void(const OdomMsg &)> callBack) {
        odomcb_.emplace_back(callBack);
    }

    inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> excallBack) {
        excb_ = excallBack;
    }

protected:
    std::string name() {
        return "DriverAdapter";
    }
    virtual void prepareMsg(vector <canbusData> buf) = 0;

protected:
    enum state {
        IDLE = 0,
        READ,
        READ_DATA,
    };

    void stateMachine();

protected:
    struct Odom_Parameter {
        std::string device_type = "";
        std::string frame_id = "/odom";
    };

protected:
    inline void reportError(const common::ErrCode &error) {
        if (excb_ != NULL) {
            excb_(error);
        }
    }

    inline void runCallBack(const OdomMsg &msg) {
        for (auto &it : odomcb_) {
            it(msg);
        }
    }

protected:
    std::function<void(const common::ErrCode &)> excb_;
    std::vector <std::function<void(const OdomMsg &)>> odomcb_;
    std::shared_ptr <CanBridge> ptrCanbridge_ptr_;
    std::shared_ptr <std::thread> odom_thread_ptr_;
    bool thread_flag_;
    state self_state_;
    uint32_t odom_seq_;
    Odom_Parameter odom_parameter_;

private:
    static const uint16_t supported_api_ = 0x0001;
};

using namespace robosense::common;

inline void DriverAdapter::init(const RsYamlNode &config) {
    int timeing0;
    int timeing1;
    unsigned int can_index = 0;
    RsYamlNode driver_config;
    rsYamlSubNode(config, "driver", driver_config);
    rsYamlRead(driver_config, "device_type", odom_parameter_.device_type);
    rsYamlRead(driver_config, "frame_id", odom_parameter_.frame_id);
    if (!rsYamlRead(driver_config, "timeing0", timeing0)) {
        RERROR << name() << ": failed to load timeing0 node!";
        RS_THROW("failed to load yaml node!");
    }
    if (!rsYamlRead(driver_config, "timeing1", timeing1)) {
        RERROR << name() << ": failed to load timeing1 node!";
        RS_THROW("failed to load yaml node!");
    }
    rsYamlRead(driver_config, "can_index", can_index);
    odom_thread_ptr_ = std::make_shared<std::thread>();
    ptrCanbridge_ptr_ = std::make_shared<CanBridge>(can_index);
    ptrCanbridge_ptr_->CanOpen(timeing0, timeing1);
    odom_seq_ = 0;
    odomcb_.reserve(10);
    excb_ = NULL;
    thread_flag_ = false;
    self_state_ = IDLE;
}

inline void DriverAdapter::start() {
    if (odomcb_.empty()) {
        RERROR << "DriverAdapter: Please register at least one callback function first!";
        exit(-1);
    }
    if (thread_flag_ == false) {
        odom_seq_ = 0;
        thread_flag_ = true;
        self_state_ = IDLE;
        const auto &func1 = [this] { stateMachine(); };
        odom_thread_ptr_ = std::make_shared<std::thread>(func1);
        RTRACE << name() << ": DriverAdapter Start!";
    }
}

inline void DriverAdapter::stop() {
    if (thread_flag_ == true) {
        thread_flag_ = false;
        odom_thread_ptr_->join();
    }
    RTRACE << "DriverAdapter Stop";
}

inline void DriverAdapter::stateMachine() {
    vector <canbusData> buf;
    while (thread_flag_) {
        switch (self_state_) {

            case IDLE: {
                self_state_ = READ;
                break;
            }
            case READ: {
                usleep(1000);
                ptrCanbridge_ptr_->Read(buf);
                if (buf.empty())
                    break;
                self_state_ = READ_DATA;
                break;
            }
            case READ_DATA: {
                prepareMsg(buf);
                self_state_ = READ;
                break;
            }
        }
    }
}
} // namespace sensor
} // namespace robosense
