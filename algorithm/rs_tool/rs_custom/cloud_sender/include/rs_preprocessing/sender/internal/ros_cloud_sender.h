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

#ifndef RS_PREPROCESSING_SENDER_INTERNAL_ROS_CLOUD_SENDER_H_
#define RS_PREPROCESSING_SENDER_INTERNAL_ROS_CLOUD_SENDER_H_
#include "rs_dependence/rs_dependence_manager.h"
#include "rs_preprocessing/sender/external/base_cloud_sender.h"

#ifdef RS_ROS_FOUND
#include <ros/ros.h>
#include <ros/publisher.h>

namespace robosense {
namespace preprocessing {

class RosCloudSender : public BaseCloudSender {
public:
    using Ptr = std::shared_ptr<RosCloudSender>;

    // load configures from yaml and init the localization cloud ros sender function.
    // input: yaml node
    // output: void.
    void init(const RsYamlNode& config_node, const RsPreprocessing::Ptr& preprocessing) override;

    // entrance of sending localization cloud message
    // input: robosense lidar point cloud message struct.
    // output: void. the robosense point cloud message will be transferred to ros cloud message and sent by ros publisher.
    void send(const lidar::LidarPointCloudMsg::Ptr& lidar_msg) override;

    // entrance of adding data to localization cloud ros sender.
    // input: robosense lidar point cloud message struct.
    // output: void. data added by this function will be delivered to function "send" in line 47.
    void addData(const lidar::LidarPointCloudMsg::Ptr& lidar_msg) override;

    // start the thread and wait for data.
    void start() override;

    // stop the thread.
    void stop() override;

private:
    std::string name() {
        return "RosCloudSender";
    }
    void initRosPublisher();

    ros::NodeHandle nh_;
    ros::Publisher fusion_lidar_pub_;
    PipelineThreadWorker<lidar::LidarPointCloudMsg::Ptr>::Ptr pipeline_thread_worker_ptr_;
    struct Params {
        float send_cloud_freq_ = 10;
        bool send_fusion_lidar = false;
        std::string fusion_lidar_topic = "fusion_lidar_points";
        void load(const RsYamlNode& config_node) {
            rsYamlRead(config_node, "send_cloud_freq_", send_cloud_freq_);
            rsYamlRead(config_node, "send_fusion_lidar", send_fusion_lidar);
            rsYamlRead(config_node, "fusion_lidar_topic", fusion_lidar_topic);
        }

        void log(const std::string& name) {

        }
    }params;

};

}   // namespace preprocessing
}   // namespace robosense
#endif  // RS_ROS_FOUND
#endif  // RS_PREPROCESSING_SENDER_INTERNAL_ROS_CLOUD_SENDER_H_