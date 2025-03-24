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

#include "rs_sensor/lidar/lidar.h"
#include "gnss/gnss.h"
#include "imu/imu.h"
#include "odom/odom.h"
#include "rs_common/external/error_code.h"
#include "rs_common/external/common.h"

namespace robosense {
namespace sensor {
class SensorManager {
public:
    SensorManager();

    ~SensorManager();

    void init(const RsYamlNode &sensor_config);

    void start();

    void stop();

    inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> &callBack) {
        excbs_.emplace_back(callBack);
    }

    inline void regRecvCallback(const std::function<void(const imu::ImuMsg &)> &callBack) {
        imucbs_.emplace_back(callBack);
    }

    inline void regRecvCallback(const std::function<void(const gnss::GnssMsg &)> &callBack) {
        gnsscbs_.emplace_back(callBack);
    }

    inline void regRecvCallback(const std::function<void(const odom::OdomMsg &)> &callBack) {
        odomcbs_.emplace_back(callBack);
    }

    inline void regRecvCallback(const std::function<void(const lidar::LidarPointCloudMsg &)> &callBack) {
        point_cloud_cbs_.emplace_back(callBack);
    }

    inline bool getUseImu() {
        return use_imu_;
    }

    inline bool getUseGnss() {
        return use_gnss_;
    }

    inline bool getUseOdom() {
        return use_odom_;
    }

    inline bool getUseLidar() {
        return use_lidar_;
    }

    /* IMU */
private:
    void updateConfig();
    std::string name() {
        return "SensorManager";
    }
    void initImu(const RsYamlNode &config);

    std::shared_ptr<AdapterBase> createImuReceiver(const RsYamlNode &config, const ImuAdapterType &adapter_type);

    std::shared_ptr<AdapterBase> createImuTransmitter(const RsYamlNode &config, const ImuAdapterType &adapter_type);

    inline void localImuCallback(const imu::ImuMsg &msg);

    RsYamlNode config_node_;
    AdapterBase::Ptr imu_recv_ptr_;
    std::vector<AdapterBase::Ptr> imu_transmit_adapter_vec_;
    std::vector<std::function<void(const imu::ImuMsg &)>> imucbs_;
    bool use_imu_;

    /* GNSS */
private:
    void initGnss(const RsYamlNode &config);

    std::shared_ptr<AdapterBase> createGnssReceiver(const RsYamlNode &config, const GnssAdapterType &adapter_type);

    std::shared_ptr<AdapterBase> createGnssTransmitter(const RsYamlNode &config, const GnssAdapterType &adapter_type);

    inline void localGnssCallback(const gnss::GnssMsg &msg);

    AdapterBase::Ptr gnss_recv_ptr_;
    std::vector<AdapterBase::Ptr> gnss_transmit_adapter_vec_;
    std::vector<std::function<void(const gnss::GnssMsg &)>> gnsscbs_;
    bool use_gnss_;

    /* ODOM */
private:
    void initOdom(const RsYamlNode &config);

    std::shared_ptr<AdapterBase> createOdomReceiver(const RsYamlNode &config, const OdomAdapterType &adapter_type);

    std::shared_ptr<AdapterBase> createOdomTransmitter(const RsYamlNode &config, const OdomAdapterType &adapter_type);

    inline void localOdomCallback(const odom::OdomMsg &msg);

    AdapterBase::Ptr odom_recv_ptr_;
    std::vector<AdapterBase::Ptr> odom_transmit_adapter_vec_;
    std::vector<std::function<void(const odom::OdomMsg &)>> odomcbs_;
    bool use_odom_;

    /* LiDAR */
private:
    void initLidar(const RsYamlNode &config);

    std::shared_ptr<AdapterBase> createLidarReceiver(const RsYamlNode &config, const LidarAdapterType &adapter_type);

    std::shared_ptr<AdapterBase> createLidarTransmitter(const RsYamlNode &config, const LidarAdapterType &adapter_type);

    inline void localLidarPointsCallback(const lidar::LidarPointCloudMsg &msg);

    std::vector<AdapterBase::Ptr> packet_receive_adapter_vec_;
    std::vector<AdapterBase::Ptr> point_cloud_receive_adapter_vec_;
    std::vector<AdapterBase::Ptr> packet_transmit_adapter_vec_;
    std::vector<AdapterBase::Ptr> point_cloud_transmit_adapter_vec_;
    std::vector<std::function<void(const lidar::LidarPointCloudMsg &)>> point_cloud_cbs_;
    bool use_lidar_;

private:
    inline void localExceptionCallback(const common::ErrCode &code);

    std::vector<std::function<void(const common::ErrCode &)>> excbs_;
};
};  // namespace sensor
}  // namespace robosense
