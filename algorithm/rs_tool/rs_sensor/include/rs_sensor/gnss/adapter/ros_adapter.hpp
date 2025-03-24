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
#include "rs_sensor/gnss/msg/ros_msg_translator.h"
#include <ros/ros.h>

namespace robosense {
namespace gnss {
class RosAdapter : virtual public sensor::AdapterBase {
public:
    RosAdapter() = default;

    ~RosAdapter() {
        stop();
    }

    void init(const RsYamlNode &config);

    void regRecvCallback(const std::function<void(const GnssMsg &)> callBack);

    void send(const GnssMsg &msg);

private:
    void localgnssCallback(const sensor_msgs::NavSatFix &msg);
    std::string name() {
        return "RosAdapter";
    }

private:
    std::unique_ptr<ros::NodeHandle> nh_;
    std::vector<std::function<void(const GnssMsg &)>> gnss_cb_;
    ros::Publisher gnss_pub_;
    ros::Publisher gnss_odom_pub_;
    ros::Subscriber gnss_sub_;
};

inline void RosAdapter::init(const RsYamlNode &config) {
    using namespace robosense::common;
    int msg_source = 0;
    bool send_msg_ros = false;
    std::string ros_recv_topic;
    std::string ros_send_topic;
    RsYamlNode ros_config;
    if (!rsYamlSubNode(config, "ros", ros_config, name(), false)) {
        RERROR << name() << ": load ros node failed!";
        RS_THROW("load yaml node failed!");
    }
    RsYamlNode common_config;
    if (!rsYamlSubNode(config, "common", common_config, name(), false)) {
        RERROR << name() << ": load common node failed!";
        RS_THROW("load yaml node failed!");
    }
    rsYamlRead(common_config, "msg_source", msg_source);
    rsYamlRead(common_config, "send_msg_ros", send_msg_ros);
    if (!rsYamlRead(ros_config, "ros_recv_topic", ros_recv_topic)) {
        RERROR << name() << ": load ros_recv_topic failed!";
        RS_THROW("load yaml node failed!");
    }
    if (!rsYamlRead(ros_config, "ros_send_topic", ros_send_topic)) {
        RERROR << name() << ": load ros_send_topic failed!";
        RS_THROW("load yaml node failed!");
    }

    nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    if (msg_source == 2) {
        gnss_sub_ = nh_->subscribe(ros_recv_topic, 1, &RosAdapter::localgnssCallback, this);
        send_msg_ros = false;
    }
    if (send_msg_ros) {
        gnss_pub_ = nh_->advertise<sensor_msgs::NavSatFix>(ros_send_topic, 10);
        gnss_odom_pub_ = nh_->advertise<nav_msgs::Odometry>(ros_send_topic + "/odom", 10);
    }
}

inline void RosAdapter::regRecvCallback(const std::function<void(const GnssMsg &)> callBack) {
    gnss_cb_.emplace_back(callBack);
}

inline void RosAdapter::send(const GnssMsg &msg) {
    gnss_pub_.publish(toRosMsg(msg));
    gnss_odom_pub_.publish(toRosMsgOdom(msg));
}

inline void RosAdapter::localgnssCallback(const sensor_msgs::NavSatFix &msg) {
    for (auto &cb : gnss_cb_) {
        cb(toRsMsg(msg));
    }
}
}  // namespace gnss
}  // namespace robosense
#endif  // RS_ROS_FOUND
