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

#include "rs_dependence/rs_dependence_manager.h"
#include "rs_sensor/manager.h"

namespace robosense {
namespace sensor {

SensorManager::SensorManager() : use_imu_(false), use_gnss_(false), use_odom_(false), use_lidar_(false) {
}

SensorManager::~SensorManager() {
    stop();
}

void SensorManager::init(const RsYamlNode &config) {
    config_node_ = config;
    updateConfig();
    initLidar(config_node_);
    initGnss(config_node_);
    initImu(config_node_);
    initOdom(config_node_);
    {
        std::stringstream ss;
        ss << name() << ": update sensor config " << std::endl;
        ss << config_node_ << std::endl;
        RsConfigureManager().append(ss.str());
    }
}

void SensorManager::updateConfig() {
    RsYamlNode calib_node, calib_lidars_node, driver_node, driver_lidar_node;
    rsYamlSubNode(config_node_, "calibration", calib_node);
    rsYamlSubNode(calib_node, "lidar", calib_lidars_node);
    rsYamlSubNode(config_node_, "lidar", driver_node);
    rsYamlSubNode(driver_node, "lidar", driver_lidar_node);

    if (static_cast<int>(calib_lidars_node.size()) > 0 &&
        static_cast<int>(driver_lidar_node.size()) < static_cast<int>(calib_lidars_node.size())) {
        RERROR << name() << ": driver lidar config size is " << driver_lidar_node.size() <<
               ", but calibration lidar size is " << calib_lidars_node.size() << "!";
        RERROR << name() << ": please check the config!";
        RS_THROW("config error!");
    }

    for (size_t i = 0; i < calib_lidars_node.size(); ++i) {
        const auto& iter = calib_lidars_node[i];
        std::string calib_frame_id;
        rsYamlRead(iter, "frame_id", calib_frame_id);
        std::string calib_device_type;
        rsYamlRead(iter, "device_type", calib_device_type);

        driver_lidar_node[i]["driver"]["device_type"] = calib_device_type;
        driver_lidar_node[i]["driver"]["frame_id"] = calib_frame_id;
    }
}

void SensorManager::start() {
    if (imu_recv_ptr_ != nullptr) {
        imu_recv_ptr_->start();
    }
    if (gnss_recv_ptr_ != nullptr) {
        gnss_recv_ptr_->start();
    }
    if (odom_recv_ptr_ != nullptr) {
        odom_recv_ptr_->start();
    }

    for (auto &iter : point_cloud_receive_adapter_vec_) {
        if (iter != nullptr)
            iter->start();
    }

    for (auto &iter : packet_receive_adapter_vec_) {
        if (iter != nullptr) {
            iter->start();
        }
    }
}

void SensorManager::stop() {
    if (imu_recv_ptr_ != nullptr) {
        imu_recv_ptr_->stop();
    }
    if (gnss_recv_ptr_ != nullptr) {
        gnss_recv_ptr_->stop();
    }
    if (odom_recv_ptr_ != nullptr) {
        odom_recv_ptr_->stop();
    }
    for (auto &iter : point_cloud_receive_adapter_vec_) {
        if (iter != nullptr)
            iter->stop();
    }
    for (auto &iter : packet_receive_adapter_vec_) {
        if (iter != nullptr) {
            iter->stop();
        }
    }
}

/*************************************************/
/****************      IMU     *******************/
/*************************************************/
void SensorManager::initImu(const RsYamlNode &config) {
    RsYamlNode imu_config;
    if (!rsYamlSubNode(config, "imu", imu_config)) {
        return;
    }
    RsYamlNode imu_common_config;
    if (!rsYamlSubNode(imu_config, "common", imu_common_config)) {
        RERROR << name() << ": load common node failed!";
        RS_THROW("load yaml node failed!");
    }
    bool send_msg_ros = false;
    int msg_source = 0;
    rsYamlRead(imu_common_config, "msg_source", msg_source);
    rsYamlRead(imu_common_config, "send_msg_ros", send_msg_ros);
    switch (ImuMsgSource(msg_source)) {
        case ImuMsgSource::MSG_FROM_DRIVER:
            imu_recv_ptr_ = createImuReceiver(imu_config, ImuAdapterType::DriverAdapter);
            break;
        case ImuMsgSource::MSG_FROM_ROS:
            send_msg_ros = false;
            imu_recv_ptr_ = createImuReceiver(imu_config, ImuAdapterType::RosAdapter);
            break;
        case ImuMsgSource::MSG_FROM_GNSS:
            break;
        default:
            return;
    }
    use_imu_ = true;
    if (ImuMsgSource(msg_source) == ImuMsgSource::MSG_FROM_GNSS) {
        gnss_recv_ptr_->regRecvCallback(std::bind(&SensorManager::localImuCallback, this, std::placeholders::_1));
    } else {
        imu_recv_ptr_->regRecvCallback(std::bind(&SensorManager::localImuCallback, this, std::placeholders::_1));
    }
    if (send_msg_ros) {
        AdapterBase::Ptr transmitter_ptr = createImuTransmitter(imu_config, ImuAdapterType::RosAdapter);
        imu_transmit_adapter_vec_.emplace_back(transmitter_ptr);
        if (ImuMsgSource(msg_source) == ImuMsgSource::MSG_FROM_GNSS) {
            gnss_recv_ptr_->regRecvCallback([transmitter_ptr](const imu::ImuMsg &msg) { transmitter_ptr->send(msg); });
        } else {
            imu_recv_ptr_->regRecvCallback([transmitter_ptr](const imu::ImuMsg &msg) { transmitter_ptr->send(msg); });
        }
    }
}

std::shared_ptr<AdapterBase> SensorManager::createImuReceiver(const RsYamlNode &config,
                                                              const ImuAdapterType &adapter_type) {
    std::shared_ptr<AdapterBase> receiver;
    std::string device_type;
    switch (adapter_type) {
        case ImuAdapterType::DriverAdapter:
            if (!rsYamlRead(config["driver"], "device_type", device_type)) {
                RERROR << name() << ": load device_type node failed!";
                RS_THROW("load yaml node failed!");
            }
            if (device_type == "TL740D") {
                receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<imu::ImuTL740D>());
            } else if (device_type == "HWT605") {
                receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<imu::ImuHWT605>());
            } else if (device_type == "XWG13668") {
                receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<gnss::InsXWG13668>());
            }
            receiver->init(config);
            break;

        case ImuAdapterType::RosAdapter:
#ifdef RS_ROS_FOUND
            receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<imu::RosAdapter>());
            receiver->init(config);
            break;
#else  // RS_ROS_FOUND
        RS_ERROR << "ROS not found! Could not use IMU ROS functions!" << RS_REND;
        exit(-1);
#endif  // RS_ROS_FOUND
        default:
            RS_ERROR << "Create IMU receiver failed. Abort!" << RS_REND;
            exit(-1);
    }
    return receiver;
}

std::shared_ptr<AdapterBase> SensorManager::createImuTransmitter(const RsYamlNode &config,
                                                                 const ImuAdapterType &adapter_type) {
    std::shared_ptr<AdapterBase> transmitter;
    switch (adapter_type) {
        case ImuAdapterType::RosAdapter:
#ifdef RS_ROS_FOUND
            transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<imu::RosAdapter>());
            transmitter->init(config);
            break;
#else  // RS_ROS_FOUND
        RS_ERROR << "ROS not found! Could not use IMU ROS functions!" << RS_REND;
        exit(-1);
#endif  // RS_ROS_FOUND
        default:
            RS_ERROR << "Create IMU transmitter failed. Abort!" << RS_REND;
            exit(-1);
    }

    return transmitter;
}

void SensorManager::localImuCallback(const imu::ImuMsg &msg) {
    for (auto &cb : imucbs_)
        cb(msg);
}

/*************************************************/
/****************      GNSS     ******************/
/*************************************************/
void SensorManager::initGnss(const RsYamlNode &config) {
    RsYamlNode gnss_config;
    if (!rsYamlSubNode(config, "gnss", gnss_config)) {
        return;
    }
    RsYamlNode gnss_common_config;
    if (!rsYamlSubNode(gnss_config, "common", gnss_common_config)) {
        RERROR << name() << ": load common node failed!";
        RS_THROW("load yaml node failed!");
    }
    bool send_msg_ros = false;
    int msg_source = 0;
    rsYamlRead(gnss_common_config, "msg_source", msg_source);
    rsYamlRead(gnss_common_config, "send_msg_ros", send_msg_ros);

    switch (GnssMsgSource(msg_source)) {
        case GnssMsgSource::MSG_FROM_DRIVER:
            gnss_recv_ptr_ = createGnssReceiver(gnss_config, GnssAdapterType::DriverAdapter);
            break;
        case GnssMsgSource::MSG_FROM_ROS:
            send_msg_ros = false;
            gnss_recv_ptr_ = createGnssReceiver(gnss_config, GnssAdapterType::RosAdapter);
            break;
        default:
            return;
    }
    use_gnss_ = true;
    gnss_recv_ptr_->regRecvCallback(std::bind(&SensorManager::localGnssCallback, this, std::placeholders::_1));
    if (send_msg_ros) {
        AdapterBase::Ptr transmitter_ptr = createGnssTransmitter(gnss_config, GnssAdapterType::RosAdapter);
        gnss_transmit_adapter_vec_.emplace_back(transmitter_ptr);
        gnss_recv_ptr_->regRecvCallback([transmitter_ptr](const gnss::GnssMsg &msg) { transmitter_ptr->send(msg); });
    }
}

std::shared_ptr<AdapterBase> SensorManager::createGnssReceiver(const RsYamlNode &config,
                                                               const GnssAdapterType &adapter_type) {
    std::shared_ptr<AdapterBase> receiver;
    std::string device_type;
    switch (adapter_type) {
        case GnssAdapterType::DriverAdapter:
            if (!rsYamlRead(config["driver"], "device_type", device_type)) {
                RERROR << name() << ": load device_type node failed!";
                RS_THROW("load yaml node failed!");
            }
            if (device_type == "XWG13668") {
                receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<gnss::InsXWG13668>());
            }
            receiver->init(config);
            break;

        case GnssAdapterType::RosAdapter:
#ifdef RS_ROS_FOUND
            receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<gnss::RosAdapter>());
            receiver->init(config);
            break;
#else  // RS_ROS_FOUND
        RS_ERROR << "ROS not found! Could not use GNSS ROS functions!" << RS_REND;
        exit(-1);
#endif  // RS_ROS_FOUND

        default:
            RS_ERROR << "Create GNSS receiver failed. Abort!" << RS_REND;
            exit(-1);
    }
    return receiver;
}

std::shared_ptr<AdapterBase> SensorManager::createGnssTransmitter(const RsYamlNode &config,
                                                                  const GnssAdapterType &adapter_type) {
    std::shared_ptr<AdapterBase> transmitter;
    switch (adapter_type) {
        case GnssAdapterType::RosAdapter:
#ifdef RS_ROS_FOUND
            transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<gnss::RosAdapter>());
            transmitter->init(config);
            break;
#else  // RS_ROS_FOUND
        RS_ERROR << "ROS not found! Could not use GNSS ROS functions!" << RS_REND;
        exit(-1);
#endif  // RS_ROS_FOUND

        default:
            RS_ERROR << "Create GNSS transmitter failed. Abort!" << RS_REND;
            exit(-1);
    }
    return transmitter;
}

void SensorManager::localGnssCallback(const gnss::GnssMsg &msg) {
    for (auto &cb : gnsscbs_)
        cb(msg);
}

/*************************************************/
/****************      ODOM     ******************/
/*************************************************/
void SensorManager::initOdom(const RsYamlNode &config) {
    RsYamlNode odom_config;
    if (!rsYamlSubNode(config, "odom", odom_config)) {
        return;
    }
    RsYamlNode odom_common_config;
    if (!rsYamlSubNode(odom_config, "common", odom_common_config)) {
        RERROR << name() << ": load common node failed!";
        RS_THROW("load yaml node failed!");
    }
    bool send_msg_ros = false;
    int msg_source = 0;
    rsYamlRead(odom_common_config, "msg_source", msg_source);
    rsYamlRead(odom_common_config, "send_msg_ros", send_msg_ros);

    switch (OdomMsgSource(msg_source)) {
        case OdomMsgSource::MSG_FROM_DRIVER:
            odom_recv_ptr_ = createOdomReceiver(odom_config, OdomAdapterType::DriverAdapter);
            break;
        case OdomMsgSource::MSG_FROM_ROS:
            send_msg_ros = false;
            odom_recv_ptr_ = createOdomReceiver(odom_config, OdomAdapterType::RosAdapter);
            break;
        case OdomMsgSource::MSG_FROM_GNSS:
            break;
        default:
            return;
    }
    use_odom_ = true;
    if (OdomMsgSource(msg_source) == OdomMsgSource::MSG_FROM_GNSS) {
        gnss_recv_ptr_->regRecvCallback(std::bind(&SensorManager::localOdomCallback, this, std::placeholders::_1));
    } else {
        odom_recv_ptr_->regRecvCallback(std::bind(&SensorManager::localOdomCallback, this, std::placeholders::_1));
    }

    if (send_msg_ros) {
        AdapterBase::Ptr transmitter_ptr = createOdomTransmitter(odom_config, OdomAdapterType::RosAdapter);
        odom_transmit_adapter_vec_.emplace_back(transmitter_ptr);
        if (OdomMsgSource(msg_source) == OdomMsgSource::MSG_FROM_GNSS) {
            gnss_recv_ptr_->regRecvCallback([transmitter_ptr](const odom::OdomMsg &msg) { transmitter_ptr->send(msg); });
        } else {
            odom_recv_ptr_->regRecvCallback([transmitter_ptr](const odom::OdomMsg &msg) { transmitter_ptr->send(msg); });
        }
    }
}

std::shared_ptr<AdapterBase> SensorManager::createOdomReceiver(const RsYamlNode &config,
                                                               const OdomAdapterType &adapter_type) {
    std::shared_ptr<AdapterBase> receiver;
    std::string device_type;
    switch (adapter_type) {
        case OdomAdapterType::DriverAdapter:
#ifdef __x86_64__
            if (!rsYamlRead(config["driver"], "device_type", device_type)) {
                RERROR << name() << ": load device_type node failed!";
                RS_THROW("load yaml node failed!");
            }
            if (device_type == "BYD") {
              receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<odom::OdomBYD>());
            } else if (device_type == "JILI") {
              receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<odom::OdomJILI>());
            } else if (device_type == "BIEKE") {
              receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<odom::OdomBIEKE>());
            } else if (device_type == "XWG13668") {
              receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<gnss::InsXWG13668>());
            }
            receiver->init(config);
#else
            RERROR << name() << ": Only support odom driver on X86_64 platform!";
            exit(-1);
#endif

            break;

        case OdomAdapterType::RosAdapter:
#ifdef RS_ROS_FOUND
            receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<odom::RosAdapter>());
            receiver->init(config);
            break;
#else  // RS_ROS_FOUND
        RS_ERROR << "ROS not found! Could not use ODOM ROS functions!" << RS_REND;
        exit(-1);
#endif  // RS_ROS_FOUND

        default:
            RS_ERROR << "Create ODOM receiver failed. Abort!" << RS_REND;
            exit(-1);
    }
    return receiver;
}

std::shared_ptr<AdapterBase> SensorManager::createOdomTransmitter(const RsYamlNode &config,
                                                                  const OdomAdapterType &adapter_type) {
    std::shared_ptr<AdapterBase> transmitter;
    switch (adapter_type) {
        case OdomAdapterType::RosAdapter:
#ifdef RS_ROS_FOUND
            transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<odom::RosAdapter>());
            transmitter->init(config);
            break;
#else  // RS_ROS_FOUND
        RS_ERROR << "ROS not found! Could not use ODOM ROS functions!" << RS_REND;
        exit(-1);
#endif  // RS_ROS_FOUND

        default:
            RS_ERROR << "Create ODOM transmitter failed. Abort!" << RS_REND;
            exit(-1);
    }
    return transmitter;
}

void SensorManager::localOdomCallback(const odom::OdomMsg &msg) {
    for (auto &cb : odomcbs_)
        cb(msg);
}

/*************************************************/
/***************      LiDAR     ******************/
/*************************************************/
void SensorManager::initLidar(const RsYamlNode &config) {
    RsYamlNode lidars_config;
    if (!rsYamlSubNode(config, "lidar", lidars_config)) {
        RERROR << name() << ": load lidar node failed!";
        RS_THROW("load yaml node failed!");
    }
    RsYamlNode lidar_common_config;
    if (!rsYamlSubNode(lidars_config, "common", lidar_common_config)) {
        RERROR << name() << ": load common node failed!";
        RS_THROW("load yaml node failed!");
    }
    RsYamlNode lidar_config;
    if (!rsYamlSubNode(lidars_config, "lidar", lidar_config)) {
        RERROR << name() << ": load lidar node failed!";
        RS_THROW("load yaml node failed!");
    }
    int msg_source = 0;
    bool send_points_ros = false;
    bool send_packets_ros = false;

    rsYamlRead(lidar_common_config, "msg_source", msg_source);
    rsYamlRead(lidar_common_config, "send_packets_ros", send_packets_ros);
    rsYamlRead(lidar_common_config, "send_points_ros", send_points_ros);

    for (size_t i = 0; i < lidar_config.size(); i++) {
        AdapterBase::Ptr recv_ptr;
        lidar_config[i]["msg_source"] = msg_source;
        /*Receiver*/
        switch (LidarMsgSource(msg_source)) {
            case LidarMsgSource::MSG_FROM_DRIVER:  // use driver
                recv_ptr = createLidarReceiver(lidar_config[i], LidarAdapterType::DriverAdapter);
                point_cloud_receive_adapter_vec_.push_back(recv_ptr);
                if (send_packets_ros) {
                    packet_receive_adapter_vec_.push_back(recv_ptr);
                }
                break;

            case LidarMsgSource::PACKET_FROM_ROS:  // pkt from ros
                send_packets_ros = false;
                point_cloud_receive_adapter_vec_.emplace_back(
                createLidarReceiver(lidar_config[i], LidarAdapterType::DriverAdapter));
                packet_receive_adapter_vec_.emplace_back(
                createLidarReceiver(lidar_config[i], LidarAdapterType::PacketRosAdapter));
                packet_receive_adapter_vec_.back()->regRecvCallback(
                std::bind(&AdapterBase::decodeScan, point_cloud_receive_adapter_vec_[i], std::placeholders::_1));
                packet_receive_adapter_vec_.back()->regRecvCallback(
                std::bind(&AdapterBase::decodePacket, point_cloud_receive_adapter_vec_[i], std::placeholders::_1));
                break;

            case LidarMsgSource::POINT_CLOUD_FROM_ROS:  // points from ros
                send_points_ros = false;
                send_packets_ros = false;
                point_cloud_receive_adapter_vec_.emplace_back(
                createLidarReceiver(lidar_config[i], LidarAdapterType::PointCloudRosAdapter));
                break;

            default:
                return;
        }
        use_lidar_ = true;
        point_cloud_receive_adapter_vec_.back()->regRecvCallback(
        std::bind(&SensorManager::localLidarPointsCallback, this, std::placeholders::_1));
        /*Transmitter*/
        if (send_packets_ros) {
            lidar_config[i]["send_packets_ros"] = true;
            packet_transmit_adapter_vec_.emplace_back(
            createLidarTransmitter(lidar_config[i], LidarAdapterType::PacketRosAdapter));
            packet_receive_adapter_vec_.back()->regRecvCallback(
            [this, i](const lidar::ScanMsg &msg) { packet_transmit_adapter_vec_[i]->send(msg); });
            packet_receive_adapter_vec_.back()->regRecvCallback(
            [this, i](const lidar::PacketMsg &msg) { packet_transmit_adapter_vec_[i]->send(msg); });
        }

        if (send_points_ros) {
            lidar_config[i]["send_points_ros"] = true;
            point_cloud_transmit_adapter_vec_.emplace_back(
            createLidarTransmitter(lidar_config[i], LidarAdapterType::PointCloudRosAdapter));
            point_cloud_receive_adapter_vec_.back()->regRecvCallback(
            [this, i](const lidar::LidarPointCloudMsg &msg) { point_cloud_transmit_adapter_vec_[i]->send(msg); });
        }
    }
}

std::shared_ptr<AdapterBase> SensorManager::createLidarReceiver(const RsYamlNode &config,
                                                                const LidarAdapterType &adapter_type) {
    std::shared_ptr<AdapterBase> receiver;
    switch (adapter_type) {
        case LidarAdapterType::DriverAdapter:
            receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<lidar::DriverAdapter>());
            receiver->init(config);
            break;

        case LidarAdapterType::PacketRosAdapter:
#ifdef RS_ROS_FOUND
            receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<lidar::PacketRosAdapter>());
            receiver->init(config);
            break;
#else  // RS_ROS_FOUND
        RS_ERROR << "ROS not found! Could not use LiDAR ROS functions!" << RS_REND;
        exit(-1);
#endif  // RS_ROS_FOUND

        case LidarAdapterType::PointCloudRosAdapter:
#ifdef RS_ROS_FOUND
            receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<lidar::PointCloudRosAdapter>());
            receiver->init(config);
            break;
#else  // RS_ROS_FOUND
        RS_ERROR << "ROS not found! Could not use LiDAR ROS functions!" << RS_REND;
        exit(-1);
#endif  // RS_ROS_FOUND

        default:
            RS_ERROR << "Create LiDAR receiver failed. Abort!" << RS_REND;
            exit(-1);
    }

    return receiver;
}

std::shared_ptr<AdapterBase> SensorManager::createLidarTransmitter(const RsYamlNode &config,
                                                                   const LidarAdapterType &adapter_type) {
    std::shared_ptr<AdapterBase> transmitter;
    switch (adapter_type) {
        case LidarAdapterType::PacketRosAdapter:
#ifdef RS_ROS_FOUND
            transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<lidar::PacketRosAdapter>());
            transmitter->init(config);
            break;
#else  // RS_ROS_FOUND
        RS_ERROR << "ROS not found! Could not use LiDAR ROS functions!" << RS_REND;
        exit(-1);
#endif  // RS_ROS_FOUND

        case LidarAdapterType::PointCloudRosAdapter:
#ifdef RS_ROS_FOUND
            transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<lidar::PointCloudRosAdapter>());
            transmitter->init(config);
            break;
#else  // RS_ROS_FOUND
        RS_ERROR << "ROS not found! Could not use LiDAR ROS functions!" << RS_REND;
        exit(-1);
#endif  // RS_ROS_FOUND

        default:
            RS_ERROR << "Create LiDAR transmitter failed. Abort!" << RS_REND;
            exit(-1);
    }

    return transmitter;
}

void SensorManager::localLidarPointsCallback(const lidar::LidarPointCloudMsg &msg) {
    for (auto &cb : point_cloud_cbs_)
        cb(msg);
}

void SensorManager::localExceptionCallback(const common::ErrCode &code) {
    for (auto &excb : excbs_)
        excb(code);
}
}  // namespace sensor
}  // namespace robosense
