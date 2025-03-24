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
#include "rs_sensor/odom/msg/ros_msg_translator.h"

namespace robosense {
namespace odom {
class RosAdapter : virtual public sensor::AdapterBase {
public:
    RosAdapter() = default;

    ~RosAdapter() { stop(); }

    void init(const RsYamlNode &config);


    void regRecvCallback(const std::function<void(const OdomMsg &)> callBack);

    void send(const OdomMsg &msg);

private:
    std::string name() {
        return "RosAdapter";
    }
    void localodomCallback(const nav_msgs::Odometry &msg);

private:
    std::unique_ptr<ros::NodeHandle> nh_;
    std::vector<std::function<void(const OdomMsg &)>> odom_cb_;
    ros::Publisher odom_pub_;
    ros::Subscriber odom_sub_;
};

inline void RosAdapter::init(const RsYamlNode &config) {
    using namespace robosense::common;
    std::string ros_recv_topic;
    std::string ros_send_topic;
    int msg_source = 0;
    bool send_msg_ros = false;
    RsYamlNode ros_config;
    if (!rsYamlSubNode(config, "ros", ros_config)) {
        RERROR << name() << "failed to load ros_config node!";
        RS_THROW("failed to load yaml node!");
    }
    RsYamlNode common_config;
    if (!rsYamlSubNode(config, "common", common_config)) {
        RERROR << name() << "failed to load common_config node!";
        RS_THROW("failed to load yaml node!");
    }
    rsYamlRead(common_config, "msg_source", msg_source);
    rsYamlRead(common_config, "send_msg_ros", send_msg_ros);
    if (!rsYamlRead(ros_config, "ros_recv_topic", ros_recv_topic)) {
        RERROR << name() << "failed to load ros_recv_topic node!";
        RS_THROW("failed to load yaml node!");
    }
    if (!rsYamlRead(ros_config, "ros_send_topic", ros_send_topic)) {
        RERROR << name() << "failed to load ros_send_topic node!";
        RS_THROW("failed to load yaml node!");
    }

    nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    if (msg_source == 2) {
        odom_sub_ = nh_->subscribe(ros_recv_topic, 1, &RosAdapter::localodomCallback, this);
        send_msg_ros = false;
    }
    if (send_msg_ros) {
        odom_pub_ = nh_->advertise<nav_msgs::Odometry>(ros_send_topic, 10);
    }
}

inline void RosAdapter::regRecvCallback(const std::function<void(const OdomMsg &)> callBack) {

    odom_cb_.emplace_back(callBack);
}

inline void RosAdapter::send(const OdomMsg &msg) // Will send NavSatStatus and Odometry
{
    odom_pub_.publish(toRosMsg(msg));
}

inline void RosAdapter::localodomCallback(const nav_msgs::Odometry &msg) {
    for (auto &cb : odom_cb_) {
        cb(toRsMsg(msg));
    }
}
} // namespace sensor
} // namespace robosense
#endif // RS_ROS_FOUND