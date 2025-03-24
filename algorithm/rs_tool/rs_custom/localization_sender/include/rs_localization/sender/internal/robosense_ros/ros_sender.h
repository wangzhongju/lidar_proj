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
#ifndef RS_LOCALIZATION_INTERNAL_ROBOSENSE_ROS_ROS_SENDER_H
#define RS_LOCALIZATION_INTERNAL_ROBOSENSE_ROS_ROS_SENDER_H

#include "rs_dependence/rs_dependence_manager.h"
#ifdef RS_ROS_FOUND

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt32.h>

#include "rs_localization/sender/external/base_sender.h"

namespace robosense {
namespace localization {

class RosLocalizationSender : public BaseLocalizationSender {
public:
    using Ptr = std::shared_ptr<RosLocalizationSender>;

    // load configures from yaml and init the localization info ros sender function.
    // input: yaml node
    // output: void.
    int init(const RsYamlNode& config_node, const std::shared_ptr<LocalizationInterface>& localization_ptr) override;

    // entrance of sending localization info message
    // input: void.
    // output: void. the localization info will be transferred by to ros type and sent by ros publisher.
    void send() override;

    // start the thread and wait for data.
    void start() override;

    // stop the thread.
    void stop() override;

private:
    std::string name() {
        return "RosLocalizationSender";
    }
    std::thread localization_sender_thread_;
    std::thread t_;
    std::shared_ptr<ros::NodeHandle> nh_ptr_;
    ros::Publisher localization_pos_pub_;
    ros::Publisher localization_fix_pub_;
    ros::Publisher localization_grid_map_pub_;
    ros::Publisher localization_pointcloud_map_pub_;
    ros::Publisher localization_path_pub_;
    ros::Publisher localization_status_pub_;
    std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_ptr_;
    nav_msgs::Path car_path_;
    struct Params {
        int localization_freq_ = 30;
        bool start_flag_ = false;
        bool send_pos_ros_ = false;
        std::string send_pos_ros_topic;
        std::string send_fix_ros_topic;
        bool send_map_ros_ = false;
        std::string send_map_ros_topic;
        bool send_path_ros_ = false;
        std::string send_path_ros_topic;

        void load(const RsYamlNode& config_node) {
            rsYamlRead(config_node, "localization_freq_", localization_freq_);
            rsYamlRead(config_node, "send_pos_ros", send_pos_ros_);
            rsYamlRead(config_node, "send_pos_ros_topic", send_pos_ros_topic);
            rsYamlRead(config_node, "send_fix_ros_topic", send_fix_ros_topic);
            rsYamlRead(config_node, "send_map_ros", send_map_ros_);
            rsYamlRead(config_node, "send_map_ros_topic", send_map_ros_topic);
            rsYamlRead(config_node, "send_path_ros", send_path_ros_);
            rsYamlRead(config_node, "send_path_ros_topic", send_path_ros_topic);
        }

        void log(const std::string& name) {
            RINFO << "localization_freq_ " << localization_freq_;
            RINFO << "send_map_ros_ " << send_map_ros_;
            RINFO << "send_map_ros_topic " << send_map_ros_topic;
            RINFO << "send_pos_ros_ " << send_pos_ros_;
        }
    }params;

    void initRosPublisher(const Params& params) {
        nh_ptr_.reset(new ros::NodeHandle);
        tf_broadcaster_ptr_.reset(new tf::TransformBroadcaster);
        localization_status_pub_ = nh_ptr_->advertise<std_msgs::UInt32>("rs_localization_status", 10, false);
        if(params.send_pos_ros_) {
            localization_pos_pub_ = nh_ptr_->advertise<nav_msgs::Odometry>(params.send_pos_ros_topic, 10);
            localization_fix_pub_ = nh_ptr_->advertise<sensor_msgs::NavSatFix>(params.send_fix_ros_topic, 10);
        }
        if(params.send_map_ros_) {
          localization_grid_map_pub_ = nh_ptr_->advertise<nav_msgs::OccupancyGrid>(params.send_map_ros_topic + "_grid", 10, true);
          localization_pointcloud_map_pub_ = nh_ptr_->advertise<sensor_msgs::PointCloud2>(params.send_map_ros_topic + "_pointcloud", 10, true);
        }
        if(params.send_path_ros_) {
            localization_path_pub_ = nh_ptr_->advertise<nav_msgs::Path>(params.send_path_ros_topic, 10);
        }
    }
    void pubLocalizationRos(const VehicleStateMsg &vstate);
};


}
}
#endif  // RS_ROS_FOUND
#endif   // RS_LOCALIZATION_INTERNAL_ROBOSENSE_ROS_ROS_SENDER_H
