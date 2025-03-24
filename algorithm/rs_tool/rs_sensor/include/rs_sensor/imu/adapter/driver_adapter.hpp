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
#include "rs_sensor/serial_port/serial_port.hpp"

namespace robosense {
namespace imu {

class DriverAdapter : virtual public sensor::AdapterBase {
public:
    DriverAdapter() = default;

    ~DriverAdapter() {
        stop();
    }

    virtual void init(const RsYamlNode &config);

    void start();

    void stop();

    void regRecvCallback(const std::function<void(const ImuMsg &)> callBack) {
        imucb_.emplace_back(callBack);
    }

    void regExceptionCallback(const std::function<void(const common::ErrCode &)> excallBack) {
        excb_ = excallBack;
    }

protected:
    std::string name() {
        return "DriverAdapter";
    }
    virtual void prepareMsg() = 0;

    virtual bool autoConnect() = 0;

    virtual int checkMarkBit() = 0;

protected:
    void reportError(const common::ErrCode &error) {
        if (excb_ != NULL) {
            excb_(error);
        }
    }

    void runCallBack(const ImuMsg &imu_msg) {
        for (auto &it : imucb_) {
            it(imu_msg);
        }
    }

protected:
    enum state {
        IDLE = 0,
        CONNECT,
        CHECK_MARK_BIT,
        READ_DATA,
        CHECK_CONNECTION,
    };

    void stateMachine();

protected:
    struct Imu_Parameter {
        std::string device_type = "";
        bool auto_scan_port = true;
        std::string port_name = "/dev/ttyUSB0";
        int baudrate = 115200;
        std::string frame_id = "/imu";
        bool do_reconnect = false;
        int reconnect_interval = 3;
        int reconnect_attemps = 3;
        double warning_gyro_z = 3.14;
        double timeout = 0.1;
        int imu_correction = 0;
    };

protected:
    std::vector<std::function<void(const ImuMsg &)>> imucb_;
    std::function<void(const common::ErrCode &)> excb_;
    std::shared_ptr<sensor::Serial> imu_ser_;
    std::shared_ptr<std::thread> imu_thread_;
    bool thread_flag_;
    state self_state_;
    uint32_t imu_seq_;
    Imu_Parameter imu_parameter_;
};

inline void DriverAdapter::init(const RsYamlNode &config) {
    using namespace robosense::common;
    RsYamlNode driver_config;
    if (!rsYamlSubNode(config, "driver", driver_config)) {
        RERROR << name() << ": load driver_config node failed!";
        RS_THROW("load yaml node failed!");
    }

    rsYamlRead(driver_config, "device_type", imu_parameter_.device_type);
    rsYamlRead(driver_config, "auto_scan_port", imu_parameter_.auto_scan_port);
    rsYamlRead(driver_config, "port_name", imu_parameter_.port_name);
    rsYamlRead(driver_config, "baudrate", imu_parameter_.baudrate);
    rsYamlRead(driver_config, "frame_id", imu_parameter_.frame_id);
    rsYamlRead(driver_config, "do_reconnect", imu_parameter_.do_reconnect);
    rsYamlRead(driver_config, "reconnect_interval", imu_parameter_.reconnect_interval);
    rsYamlRead(driver_config, "reconnect_attemps", imu_parameter_.reconnect_attemps);
    rsYamlRead(driver_config, "warning_gyro_z", imu_parameter_.warning_gyro_z);
    rsYamlRead(driver_config, "timeout", imu_parameter_.timeout);

    imu_thread_ = std::make_shared<std::thread>();
    imu_ser_ = std::make_shared<sensor::Serial>();
    imu_seq_ = 0;
    imucb_.reserve(10);
    excb_ = NULL;
    thread_flag_ = false;
    self_state_ = IDLE;
}

inline void DriverAdapter::start() {
    if (imucb_.empty()) {
        RERROR << "DriverAdapter: Please register at least one callback function first!";
        exit(-1);
    }
    if (thread_flag_ == false) {
        imu_seq_ = 0;
        thread_flag_ = true;
        self_state_ = IDLE;
        const auto &func1 = [this] { stateMachine(); };
        imu_thread_ = std::make_shared<std::thread>(func1);
    }
}

inline void DriverAdapter::stop() {
    if (thread_flag_ == true) {
        thread_flag_ = false;
        imu_thread_->join();
    }
}

inline void DriverAdapter::stateMachine() {
    int count = 0;
    while (thread_flag_) {
        switch (self_state_) {
            case IDLE: {
                self_state_ = CONNECT;
                break;
            }
            case CONNECT: {
                if (imu_parameter_.auto_scan_port) {
                    if (!autoConnect()) {
                        RWARNING << name() << ": IMU AUTO_CONNECT failed";
                        reportError(common::ErrCode_ImuDriverConnectfail);
                        self_state_ = CHECK_CONNECTION;
                    } else {
                        RTRACE << name() << ": IMU Auto connect successfully!";
                        count = 0;
                        self_state_ = CHECK_MARK_BIT;
                    }
                    break;
                } else {
                    if (!imu_ser_->connect(imu_parameter_.port_name, imu_parameter_.baudrate, 8, 'N', 1)) {
                        RWARNING << name() << ": IMU MANUAL_CONNECT failed";
                        reportError(common::ErrCode_ImuDriverConnectfail);
                        self_state_ = CHECK_CONNECTION;
                    } else {
                        RTRACE << name() << ": IMU Manually connect successfully!";
                        count = 0;
                        self_state_ = CHECK_MARK_BIT;
                    }
                    break;
                }
            }
            case CHECK_MARK_BIT: {
                switch (checkMarkBit()) {
                    case (0):
                        self_state_ = CHECK_CONNECTION;
                        break;
                    case (1):
                        break;
                    case (2):
                        self_state_ = READ_DATA;
                        break;
                }
                break;
            }
            case READ_DATA: {
                prepareMsg();
                self_state_ = CHECK_MARK_BIT;
                break;
            }
            case CHECK_CONNECTION: {
                if (imu_parameter_.do_reconnect) {
                    count++;
                    if (count > imu_parameter_.reconnect_attemps) {
                        reportError(common::ErrCode_ImuDriverDisconnect);
                        sleep(1);
                        break;
                    }
                    RWARNING << name() << ": IMU driver will start reconnect in " <<
                             imu_parameter_.reconnect_interval << " seconds";
                    sleep(imu_parameter_.reconnect_interval);
                    self_state_ = CONNECT;
                } else {
                    reportError(common::ErrCode_ImuDriverDisconnect);
                    sleep(1);
                    break;
                }
            }
        }
    }
}
}  // namespace sensor
}  // namespace robosense
