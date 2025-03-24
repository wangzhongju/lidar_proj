/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
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
#include "rs_common/external/msg/lidar_point_cloud_msg.h"
#include "rs_driver/msg/scan_msg.h"
#include "rs_driver/msg/packet_msg.h"
#include "rs_common/external/msg/imu_msg.h"
#include "rs_common/external/msg/gnss_msg.h"
#include "rs_common/external/msg/odom_msg.h"

namespace robosense {
namespace sensor {
enum class LidarMsgSource {
    MSG_FROM_DRIVER = 1,
    PACKET_FROM_ROS,
    POINT_CLOUD_FROM_ROS,
};

enum class LidarAdapterType {
    DriverAdapter = 1,
    PointCloudRosAdapter,
    PacketRosAdapter,
};

enum class ImuMsgSource {
    MSG_FROM_DRIVER = 1,
    MSG_FROM_ROS,
    MSG_FROM_GNSS
};

enum class ImuAdapterType {
    DriverAdapter = 1,
    RosAdapter
};

enum class GnssMsgSource {
    MSG_FROM_DRIVER = 1,
    MSG_FROM_ROS
};

enum class GnssAdapterType {
    DriverAdapter = 1,
    RosAdapter
};

enum class OdomMsgSource {
    MSG_FROM_DRIVER = 1,
    MSG_FROM_ROS,
    MSG_FROM_GNSS
};

enum class OdomAdapterType {
    DriverAdapter = 1,
    RosAdapter
};

class AdapterBase {
public:
    typedef std::shared_ptr<AdapterBase> Ptr;

    AdapterBase() = default;

    virtual ~AdapterBase() = default;

    virtual void init(const RsYamlNode &config) {
        return;
    }

    virtual void start() {
        return;
    }

    virtual void stop() {
        return;
    }

    virtual void send(const imu::ImuMsg &msg) {
        return;
    }

    virtual void send(const gnss::GnssMsg &msg) {
        return;
    }

    virtual void send(const odom::OdomMsg &msg) {
        return;
    }

    virtual void send(const lidar::ScanMsg &msg) {
        return;
    }

    virtual void send(const lidar::PacketMsg &msg) {
        return;
    }

    virtual void send(const lidar::LidarPointCloudMsg &msg) {
        return;
    }

    virtual void regRecvCallback(const std::function<void(const lidar::ScanMsg &)> callback) {
        return;
    }

    virtual void regRecvCallback(const std::function<void(const lidar::PacketMsg &)> callback) {
        return;
    }

    virtual void regRecvCallback(const std::function<void(const lidar::LidarPointCloudMsg &)> callBack) {
    }

    virtual void regRecvCallback(const std::function<void(const imu::ImuMsg &)> callback) {
        return;
    }

    virtual void regRecvCallback(const std::function<void(const gnss::GnssMsg &)> callback) {
        return;
    }

    virtual void regRecvCallback(const std::function<void(const odom::OdomMsg &)> callback) {
        return;
    }

    virtual void regExceptionCallback(const std::function<void(const common::ErrCode &)> callback) {
        return;
    }

    virtual void decodeScan(const lidar::ScanMsg &msg) {
        return;
    }

    virtual void decodePacket(const lidar::PacketMsg &msg) {
        return;
    }
};
}  // namespace sensor
}  // namespace robosense
