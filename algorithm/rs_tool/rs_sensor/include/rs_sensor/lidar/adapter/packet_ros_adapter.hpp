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

#include "rs_dependence/rs_dependence_manager.h"
#ifdef RS_ROS_FOUND

#include "rs_common/external/common.h"
#include "rs_sensor/adapter_base/adapter_base.h"
#include "rs_sensor/lidar/msg/ros_msg_translator.h"
#include <ros/ros.h>

namespace robosense {
namespace lidar {
class PacketRosAdapter : public sensor::AdapterBase {
public:
    PacketRosAdapter() {};

    ~PacketRosAdapter() {
        stop();
    }

    void init(const RsYamlNode &config);

    void regRecvCallback(const std::function<void(const ScanMsg &)> callback) override;

    void regRecvCallback(const std::function<void(const PacketMsg &)> callBack) override;

    void send(const ScanMsg &msg);

    void send(const PacketMsg &msg);

private:
    std::string name() {
        return "PacketRosAdapter";
    }

    void localLidarPacketsmsopCallback(const rslidar_msgs::rslidarScan &msg);

    void localLidarPacketsdifopCallback(const rslidar_msgs::rslidarPacket &msg);

private:
    std::unique_ptr<ros::NodeHandle> nh_;
    std::vector<std::function<void(const lidar::ScanMsg &)>> lidar_packets_msop_cbs_;
    std::vector<std::function<void(const lidar::PacketMsg &)>> lidar_packets_difop_cbs_;
    ros::Publisher lidar_packets_msop_pub_;
    ros::Publisher lidar_packets_difop_pub_;
    ros::Subscriber lidar_packets_msop_sub_;
    ros::Subscriber lidar_packets_difop_sub_;
};

inline void PacketRosAdapter::init(const RsYamlNode &config) {
    int msg_source = 0;
    bool send_packets_ros = false;
    RsYamlNode ros_config;
    if (!rsYamlSubNode(config, "ros", ros_config)) {
        RERROR << name() << ": failed to load ros_config node!";
        RS_THROW("failed to load yaml node!");
    }
    nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    std::string ros_recv_topic;
    if (!rsYamlRead(ros_config, "ros_recv_packets_topic", ros_recv_topic)) {
        RERROR << name() << ": failed to load ros_recv_packets_topic node!";
        RS_THROW("failed to load yaml node!");
    }
    std::string ros_send_topic;
    if (!rsYamlRead(ros_config, "ros_send_packets_topic", ros_send_topic)) {
        RERROR << name() << ": failed to load ros_send_packets_topic node!";
        RS_THROW("failed to load yaml node!");
    }
    rsYamlRead(config, "msg_source", msg_source);
    rsYamlRead(config, "send_packets_ros", send_packets_ros);
    if (msg_source == 2) {
        lidar_packets_difop_sub_ =
        nh_->subscribe(ros_recv_topic + "_difop", 1, &PacketRosAdapter::localLidarPacketsdifopCallback, this);
        lidar_packets_msop_sub_ = nh_->subscribe(ros_recv_topic, 1, &PacketRosAdapter::localLidarPacketsmsopCallback,
                                                 this);
    }
    if (send_packets_ros) {
        lidar_packets_difop_pub_ = nh_->advertise<rslidar_msgs::rslidarPacket>(ros_send_topic + "_difop", 10);
        lidar_packets_msop_pub_ = nh_->advertise<rslidar_msgs::rslidarScan>(ros_send_topic, 10);
    }
}

inline void PacketRosAdapter::regRecvCallback(const std::function<void(const ScanMsg &)> callBack) {
    lidar_packets_msop_cbs_.emplace_back(callBack);
}

inline void PacketRosAdapter::regRecvCallback(const std::function<void(const lidar::PacketMsg &)> callBack) {
    lidar_packets_difop_cbs_.emplace_back(callBack);
}

inline void PacketRosAdapter::send(const lidar::ScanMsg &msg)  // Will send NavSatStatus and Odometry
{
    lidar_packets_msop_pub_.publish(toRosMsg(msg));
}

inline void PacketRosAdapter::send(const lidar::PacketMsg &msg)  // Will send NavSatStatus and Odometry
{
    lidar_packets_difop_pub_.publish(toRosMsg(msg));
}

inline void PacketRosAdapter::localLidarPacketsmsopCallback(const rslidar_msgs::rslidarScan &msg) {
    for (auto &cb : lidar_packets_msop_cbs_) {
        cb(toRsMsg(msg));
    }
}

inline void PacketRosAdapter::localLidarPacketsdifopCallback(const rslidar_msgs::rslidarPacket &msg) {
    for (auto &cb : lidar_packets_difop_cbs_) {
        cb(toRsMsg(msg));
    }
}
}  // namespace lidar
}  // namespace robosense
#endif  // RS_ROS_FOUND