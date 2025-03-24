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
#include "rs_driver/api/lidar_driver.h"

namespace robosense {
namespace lidar {
class DriverAdapter : virtual public sensor::AdapterBase {
public:
    DriverAdapter();

    ~DriverAdapter();

    void init(const RsYamlNode &config);

    void start();

    void stop();

    inline void regRecvCallback(const std::function<void(const LidarPointCloudMsg &)> callBack) {
        pointscb_.emplace_back(callBack);
    }

    inline void regRecvCallback(const std::function<void(const ScanMsg &)> callBack) {
        pkts_msop_cb_.emplace_back(callBack);
    }

    inline void regRecvCallback(const std::function<void(const PacketMsg &)> callBack) {
        pkts_difop_cb_.emplace_back(callBack);
    }

    inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> excallBack) {
        excb_ = excallBack;
    }

    void decodeScan(const ScanMsg &pkt_msg);

    void decodePacket(const PacketMsg &pkt_msg);

private:
    std::string name() {
        return "DriverAdapter";
    }

    void localScanCallback(const lidar::ScanMsg &msg);

    void localPacketCallback(const lidar::PacketMsg &msg);

    void localPointsCallback(const lidar::PointCloudMsg<RsPoint> &msg);

    void localExceptionCallback(const lidar::Error &msg);

    LidarPointCloudMsg lPoints2CPoints(const lidar::PointCloudMsg<RsPoint> &msg);

private:
    std::shared_ptr<lidar::LidarDriver<RsPoint>> driver_ptr_;
    std::vector<std::function<void(const ScanMsg &)>> pkts_msop_cb_;
    std::vector<std::function<void(const PacketMsg &)>> pkts_difop_cb_;
    std::vector<std::function<void(const LidarPointCloudMsg &)>> pointscb_;
    std::function<void(const common::ErrCode &)> excb_;
};

using namespace robosense::common;
using namespace robosense::lidar;

inline DriverAdapter::DriverAdapter() {
    driver_ptr_.reset(new lidar::LidarDriver<RsPoint>());
    driver_ptr_->regExceptionCallback(std::bind(&DriverAdapter::localExceptionCallback, this, std::placeholders::_1));
}

inline DriverAdapter::~DriverAdapter() {
    driver_ptr_->stop();
}

inline void DriverAdapter::init(const RsYamlNode &config) {
    lidar::RSDriverParam driver_param;
    int msg_source = 0;
    std::string device_type = "RS16";
    unsigned int split_frame_mode = 1;
    RsYamlNode driver_config;
    if (!rsYamlSubNode(config, "driver", driver_config)) {
        RERROR << name() << ": failed to load driver_config node!";
        RS_THROW("failed to load yaml node!");
    }
    rsYamlRead(config, "msg_source", msg_source);
    rsYamlRead(driver_config, "frame_id", driver_param.frame_id);
    rsYamlRead(driver_config, "angle_path", driver_param.angle_path);
    rsYamlRead(driver_config, "device_type", device_type);
    rsYamlRead(driver_config, "wait_for_difop", driver_param.wait_for_difop);
    rsYamlRead(driver_config, "use_lidar_clock", driver_param.decoder_param.use_lidar_clock);
    rsYamlRead(driver_config, "min_distance", driver_param.decoder_param.min_distance);
    rsYamlRead(driver_config, "max_distance", driver_param.decoder_param.max_distance);
    rsYamlRead(driver_config, "start_angle", driver_param.decoder_param.start_angle);
    rsYamlRead(driver_config, "end_angle", driver_param.decoder_param.end_angle);
    rsYamlRead(driver_config, "split_frame_mode", split_frame_mode);
    rsYamlRead(driver_config, "num_pkts_split", driver_param.decoder_param.num_pkts_split);
    rsYamlRead(driver_config, "cut_angle", driver_param.decoder_param.cut_angle);
    rsYamlRead(driver_config, "device_ip", driver_param.input_param.device_ip);
    rsYamlRead(driver_config, "msop_port", driver_param.input_param.msop_port);
    rsYamlRead(driver_config, "difop_port", driver_param.input_param.difop_port);
    rsYamlRead(driver_config, "read_pcap", driver_param.input_param.read_pcap);
    rsYamlRead(driver_config, "pcap_rate", driver_param.input_param.pcap_rate);
    rsYamlRead(driver_config, "pcap_repeat", driver_param.input_param.pcap_repeat);
    rsYamlRead(driver_config, "pcap_path", driver_param.input_param.pcap_path);

    driver_param.lidar_type = driver_param.strToLidarType(device_type);
    driver_param.decoder_param.split_frame_mode = SplitFrameMode(split_frame_mode);

    if (msg_source == 1) {
        if (!driver_ptr_->init(driver_param)) {
            RERROR << "Driver Initialize Error....";
            exit(-1);
        }
    } else {
        driver_ptr_->initDecoderOnly(driver_param);
    }
    driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localPointsCallback, this, std::placeholders::_1));
    driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localScanCallback, this, std::placeholders::_1));
    driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localPacketCallback, this, std::placeholders::_1));
}

inline void DriverAdapter::start() {
    driver_ptr_->start();
}

inline void DriverAdapter::stop() {
    driver_ptr_->stop();
}

inline void DriverAdapter::decodeScan(const ScanMsg &msg) {
    lidar::PointCloudMsg<RsPoint> point_cloud_msg;
    if (driver_ptr_->decodeMsopScan(msg, point_cloud_msg)) {
        localPointsCallback(point_cloud_msg);
    }
}

inline void DriverAdapter::decodePacket(const PacketMsg &msg) {
    driver_ptr_->decodeDifopPkt(msg);
}

inline void DriverAdapter::localScanCallback(const lidar::ScanMsg &msg) {
    for (auto &it : pkts_msop_cb_) {
        it(msg);
    }
}

inline void DriverAdapter::localPacketCallback(const lidar::PacketMsg &msg) {
    for (auto &it : pkts_difop_cb_) {
        it(msg);
    }
}

inline void DriverAdapter::localPointsCallback(const lidar::PointCloudMsg<RsPoint> &msg) {
    for (auto &iter : pointscb_) {
        iter(lPoints2CPoints(msg));
    }
}

inline void DriverAdapter::localExceptionCallback(const lidar::Error &msg) {
    switch (msg.error_code_type) {
        case lidar::ErrCodeType::INFO_CODE:
            RS_INFO << msg.toString() << RS_REND;
            break;
        case lidar::ErrCodeType::WARNING_CODE:
            RS_WARNING << msg.toString() << RS_REND;
            break;
        case lidar::ErrCodeType::ERROR_CODE:
            RS_ERROR << msg.toString() << RS_REND;
            break;
    }
}

inline LidarPointCloudMsg DriverAdapter::lPoints2CPoints(const lidar::PointCloudMsg<RsPoint> &msg) {
    LidarPointCloudMsg::PointCloudPtr point_cloud(new RsPointCloudGPT);
    point_cloud->points.assign(msg.point_cloud_ptr->begin(), msg.point_cloud_ptr->end());
    point_cloud->height = msg.height;
    point_cloud->width = msg.width;
    LidarPointCloudMsg point_cloud_msg(point_cloud);
    point_cloud_msg.frame_id = msg.frame_id;
    point_cloud_msg.parent_frame_id = msg.frame_id;
    point_cloud_msg.timestamp = msg.timestamp;
    point_cloud_msg.seq = msg.seq;
    point_cloud_msg.height = msg.height;
    point_cloud_msg.width = msg.width;
    point_cloud_msg.is_dense = msg.is_dense;
    return std::move(point_cloud_msg);
}

}  // namespace lidar
}  // namespace robosense
