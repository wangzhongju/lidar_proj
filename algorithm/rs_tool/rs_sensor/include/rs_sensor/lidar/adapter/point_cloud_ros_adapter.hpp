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

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include "rs_sensor/adapter_base/adapter_base.h"
#include "rs_sensor/lidar/msg/ros_msg_translator.h"

namespace robosense {
namespace lidar {
class PointCloudRosAdapter : virtual public sensor::AdapterBase {
public:
    PointCloudRosAdapter() = default;

    ~PointCloudRosAdapter() {
        stop();
    }

    void init(const RsYamlNode &config);

    void regRecvCallback(const std::function<void(const lidar::LidarPointCloudMsg &)> callBack);

    void send(const lidar::LidarPointCloudMsg &msg);

private:
    std::string name() {
        return "PointCloudRosAdapter";
    }
    void localLidarPointsCallback(const pcl::PointCloud<pcl::PointXYZI> &msg);

private:
    std::shared_ptr<ros::NodeHandle> nh_;
    std::vector<std::function<void(const lidar::LidarPointCloudMsg &)>> point_cloud_cbs_;
    ros::Publisher lidar_points_pub_;
    ros::Subscriber lidar_points_sub_;
    std::string frame_id_;
};

inline void PointCloudRosAdapter::init(const RsYamlNode &config) {
    int msg_source = 0;
    bool send_points_ros = true;
    RsYamlNode ros_config;
    if (!rsYamlSubNode(config, "ros", ros_config)) {
        RERROR << name() << ": load ros_config node failed!";
        RS_THROW("load yaml node failed!");
    }
    nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    if (!rsYamlRead(config["driver"], "frame_id", frame_id_)) {
        RERROR << name() << ": load frame_id node failed!";
        RS_THROW("load yaml node failed!");
    }
    std::string ros_recv_topic;
    if (!rsYamlRead(ros_config, "ros_recv_points_topic", ros_recv_topic)) {
        RERROR << name() << ": load ros_recv_points_topic node failed!";
        RS_THROW("load yaml node failed!");
    }
    std::string ros_send_topic;
    if (!rsYamlRead(ros_config, "ros_send_points_topic", ros_send_topic)) {
        RERROR << name() << ": load ros_send_points_topic node failed!";
        RS_THROW("load yaml node failed!");
    }
    rsYamlRead(config, "msg_source", msg_source);
    rsYamlRead(config, "send_points_ros", send_points_ros);
    if (msg_source == 3) {
        lidar_points_sub_ = nh_->subscribe(ros_recv_topic, 1, &PointCloudRosAdapter::localLidarPointsCallback, this);
        send_points_ros = false;
    }
    if (send_points_ros) {
        lidar_points_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10);
    }
}

inline void
PointCloudRosAdapter::regRecvCallback(const std::function<void(const lidar::LidarPointCloudMsg &)> callBack) {
    point_cloud_cbs_.emplace_back(callBack);
}

inline void PointCloudRosAdapter::send(const lidar::LidarPointCloudMsg &msg)  // Will send NavSatStatus and Odometry
{
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(toRosMsg(msg), ros_msg);
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(msg.timestamp);
    lidar_points_pub_.publish(ros_msg);
}

inline void PointCloudRosAdapter::localLidarPointsCallback(const pcl::PointCloud<pcl::PointXYZI> &msg) {
    for (auto &cb : point_cloud_cbs_) {
        cb(toRsMsg(msg, frame_id_));
    }
}
}  // namespace sensor
}  // namespace robosense
#endif  // RS_ROS_FOUND