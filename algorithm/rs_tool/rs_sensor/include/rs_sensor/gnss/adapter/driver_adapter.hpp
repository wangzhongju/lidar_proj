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

#include "rs_sensor/adapter_base/adapter_base.h"
#include "rs_sensor/serial_port/serial_port.hpp"

namespace robosense {
namespace gnss {
class DriverAdapter : virtual public sensor::AdapterBase {
public:
    DriverAdapter() {
        leaps_ = {46828800, 78364801, 109900802, 173059203, 252028804, 315187205, 346723206, 393984007, 425520008,
                  457056009, 504489610, 551750411, 599184012, 820108813, 914803214, 1025136015, 1119744016, 1167264017};
    }

    ~DriverAdapter() {
        stop();
    }

    std::string name() {
        return "DriverAdapter";
    }

    virtual void init(const RsYamlNode &config) {
        imu_seq_ = 0;
        RsYamlNode driver_config;
        if (! rsYamlSubNode(config, "driver", driver_config, name(), false)) {
            RERROR << name() << ": failed load driver node!";
            RS_THROW("load yaml node failed!");
        }
        rsYamlRead(driver_config, "device_type", parameter_.device_type);
        rsYamlRead(driver_config, "auto_scan_port", parameter_.auto_scan_port);
        rsYamlRead(driver_config, "port_name", parameter_.port_name);
        rsYamlRead(driver_config, "baudrate", parameter_.baudrate);
        rsYamlRead(driver_config, "frame_id", parameter_.frame_id);

        rsYamlRead(driver_config, "do_reconnect", parameter_.do_reconnect);
        rsYamlRead(driver_config, "reconnect_interval", parameter_.reconnect_interval);
        rsYamlRead(driver_config, "reconnect_attempts", parameter_.reconnect_attempts);

        rsYamlRead(driver_config, "min_satellite", parameter_.min_satellite);
        rsYamlRead(driver_config, "warning_gyro_z", parameter_.warning_gyro_z);
        rsYamlRead(driver_config, "timeout", parameter_.timeout);
        rsYamlRead(driver_config, "use_gnss_clock", parameter_.use_gnss_clock);

        thread_flag_ = false;
        gnss_thread_ = std::make_shared<std::thread>();
        gnss_ser_ = std::make_shared<sensor::Serial>();
        gnss_seq_ = 0;
        odom_seq_ = 0;
        excb_ = NULL;
        gnsscb_.reserve(10);
        odomcb_.reserve(10);
        self_state_ = IDLE;
    }

    inline void start() {
        if (thread_flag_ == false) {
            thread_flag_ = true;
            self_state_ = IDLE;
            const auto &func1 = [this] { stateMachine(); };
            gnss_thread_ = std::make_shared<std::thread>(func1);
        }
    }

    inline void stop() {
        if (thread_flag_ == true) {
            thread_flag_ = false;
            gnss_thread_->join();
        }
    }

    inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> excallBack) {
        excb_ = excallBack;
    }

    inline void regRecvCallback(const std::function<void(const GnssMsg &)> callBack) {
        gnsscb_.emplace_back(callBack);
    }

    inline void regRecvCallback(const std::function<void(const odom::OdomMsg &)> callBack) {
        odomcb_.emplace_back(callBack);
    }

    inline void regRecvCallback(const std::function<void(const imu::ImuMsg &)> callBack) {
        imucb_.emplace_back(callBack);
    }

protected:
    inline void runCallBack(const imu::ImuMsg &imu_msg) {
        for (auto it : imucb_) {
            it(imu_msg);
        }
    }

    inline bool isleap(const double &gpstime) {
        bool isleap = false;
        for (auto iter : leaps_) {
            if (gpstime == iter)
                isleap = true;
        }
        return isleap;
    }

    inline int countleaps(const double &gpstime) {
        int nleaps = 0;  // number of leap seconds prior to gpsTime
        for (auto iter : leaps_) {
            if (gpstime >= iter) {
                nleaps++;
            }
        }
        return nleaps;
    }

    inline double gps2unix(const double &gpstime) {
        double unixtime = gpstime + 315964800;
        int nleaps = countleaps(gpstime);
        unixtime = unixtime - nleaps;
        if (isleap(gpstime)) {
            unixtime = unixtime + 0.5;
        }
        return unixtime;
    }

    std::vector<std::function<void(const imu::ImuMsg &)>> imucb_;
    uint32_t imu_seq_;
    std::array<double, 18> leaps_;

protected:
    virtual void prepareMsg(const std::vector<std::string> &msg, bool use_gnss_clock) = 0;

    virtual bool autoConnect() = 0;

    inline void runCallBack(const GnssMsg &gnss_msg) {
        for (auto &it : gnsscb_) {
            it(gnss_msg);
        }
    }

    inline void runCallBack(const odom::OdomMsg &odom_msg) {
        for (auto &it : odomcb_) {
            it(odom_msg);
        }
    }

    inline void reportError(const common::ErrCode &error) {
        if (excb_ != NULL) {
            excb_(error);
        }
    }

    bool processData(std::vector<uint8_t> &data_buf, std::vector<std::string> &data) {
        uint32_t i = 0;
        for (; i < data_buf.size(); i++) {
            if (data_buf[i] == '$') {
                break;
            }
        }
        uint32_t j = i;
        for (; j < data_buf.size(); j++) {
            if (data_buf[j] == '\n') {
                break;
            }
        }
        if (j == data_buf.size())
            return false;
        data_buf[j] = ',';
        for (uint32_t k = i; k <= j; ++k) {
            std::string tmp_str;
            while (data_buf[k] != ',') {
                tmp_str.push_back(data_buf[k]);
                ++k;
            }
            data.emplace_back(std::move(tmp_str));
        }
        if (j + 1 < data_buf.size()) {
            data_buf.erase(data_buf.begin(), data_buf.begin() + j + 1);
        } else {
            data_buf.clear();
        }
        return true;
    }

    enum state {
        IDLE = 0,
        CONNECT,
        PREPARE_DATA,
        READ_DATA,
        CHECK_CONNECTION,
    };

    void stateMachine() {
        {
            std::vector<uint8_t> data_buf;
            std::vector<std::string> data;
            int count = 0;
            while (thread_flag_) {
                switch (self_state_) {
                    case IDLE: {
                        self_state_ = CONNECT;
                        break;
                    }
                    case CONNECT: {
                        if (parameter_.auto_scan_port) {
                            if (!autoConnect()) {
                                RWARNING << name() << ": GNSS AUTO_CONNECT failed";
                                reportError(common::ErrCode_GnssDriverConnectfail);
                                self_state_ = CHECK_CONNECTION;
                            } else {
                                RTRACE << name() << ": GNSS Auto connect successfully!";
                                count = 0;
                                self_state_ = PREPARE_DATA;
                            }
                            break;
                        } else {
                            if (!gnss_ser_->connect(parameter_.port_name, parameter_.baudrate, 8, 'N', 1)) {
                                RWARNING << name() << ": GNSS MANUAL_CONNECT failed";
                                reportError(common::ErrCode_GnssDriverConnectfail);
                                self_state_ = CHECK_CONNECTION;
                            } else {
                                RTRACE << name() << ": GNSS Manually connect successfully!";
                                count = 0;
                                self_state_ = PREPARE_DATA;
                            }
                            break;
                        }
                    }
                    case PREPARE_DATA: {
                        std::vector<uint8_t> tmp_buf;

                        if ((int) gnss_ser_->serialRead(tmp_buf, 50, parameter_.timeout) < 0) {
                            reportError(common::ErrCode_GnssDriverInterrupt);
                            self_state_ = CHECK_CONNECTION;
                            break;
                        }
                        data_buf.insert(data_buf.begin() + data_buf.size(), tmp_buf.begin(),
                                        tmp_buf.begin() + tmp_buf.size());
                        if (processData(data_buf, data)) {
                            self_state_ = READ_DATA;
                        } else {
                            self_state_ = PREPARE_DATA;
                        }
                        break;
                    }
                    case READ_DATA: {
                        prepareMsg(data, parameter_.use_gnss_clock);
                        data.clear();
                        self_state_ = PREPARE_DATA;
                        break;
                    }
                    case CHECK_CONNECTION: {
                        if (parameter_.do_reconnect) {
                            count++;
                            if (count > parameter_.reconnect_attempts) {
                                reportError(common::ErrCode_GnssDriverDisconnect);
                                sleep(1);
                                break;
                            }
                            RWARNING << name() << ": GNSS driver will start reconnect in " <<
                                     parameter_.reconnect_interval << " seconds";
                            sleep(parameter_.reconnect_interval);
                            self_state_ = IDLE;
                        } else {
                            reportError(common::ErrCode_GnssDriverDisconnect);
                            sleep(1);
                            break;
                        }
                    }
                }
            }
        }
    }

    struct Parameter {
        std::string device_type = "";
        bool auto_scan_port = true;
        std::string port_name = "/dev/tyUSB0";
        int baudrate = 115200;
        std::string frame_id = "/gnss";
        bool do_reconnect = false;
        int reconnect_interval = 3;
        int reconnect_attempts = 3;
        int min_satellite = 3;
        double warning_gyro_z = 3.14;
        double timeout = 0.1;
        bool use_gnss_clock = false;
    };

    std::vector<std::function<void(const GnssMsg &)>> gnsscb_;
    std::vector<std::function<void(const odom::OdomMsg &)>> odomcb_;
    std::function<void(const common::ErrCode &)> excb_;
    std::shared_ptr<std::thread> gnss_thread_;
    std::shared_ptr<sensor::Serial> gnss_ser_;
    bool thread_flag_;
    state self_state_;
    uint32_t gnss_seq_;
    uint32_t odom_seq_;
    Parameter parameter_;
};
}  // namespace gnss
}  // namespace robosense
