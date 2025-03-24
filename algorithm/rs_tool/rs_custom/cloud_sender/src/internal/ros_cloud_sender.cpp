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
#include "rs_preprocessing/sender/internal/ros_cloud_sender.h"

#ifdef RS_ROS_FOUND
#include "rs_sensor/lidar/msg/ros_msg_translator.h"
namespace robosense {
namespace preprocessing {

void RosCloudSender::init(const RsYamlNode &config_node, const RsPreprocessing::Ptr &preprocessing) {
    preprocessing_ptr_ = preprocessing;
    params.load(config_node);
    params.log(name());
    initRosPublisher();
}

void RosCloudSender::send(const lidar::LidarPointCloudMsg::Ptr& lidar_msg) {
    if (params.send_fusion_lidar) {
        if (lidar_msg == nullptr) {
            return;
        }
        sensor_msgs::PointCloud2 ros_msg_;
        pcl::toROSMsg(toRosMsg(*lidar_msg), ros_msg_);
        ros_msg_.header.stamp = ros_msg_.header.stamp.fromSec(lidar_msg->timestamp);
        fusion_lidar_pub_.publish(ros_msg_);
    }
}

void RosCloudSender::addData(const lidar::LidarPointCloudMsg::Ptr &lidar_msg) {
    pipeline_thread_worker_ptr_->add(lidar_msg);
}
void RosCloudSender::start() {
    pipeline_thread_worker_ptr_.reset(new PipelineThreadWorker<lidar::LidarPointCloudMsg::Ptr>(name()));
    pipeline_thread_worker_ptr_->bind([&](const lidar::LidarPointCloudMsg::Ptr& msg_ptr){
        this->send(msg_ptr);
        return true;
    });
    pipeline_thread_worker_ptr_->start();
}

void RosCloudSender::stop() {
    pipeline_thread_worker_ptr_->stop();
}

void RosCloudSender::initRosPublisher() {
    if(params.send_fusion_lidar) {
        fusion_lidar_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >(params.fusion_lidar_topic, 10, true);
    }

}
RS_REGISTER_PREPROCESSING_SENDER(RosCloudSender)
}   // namespace preprocessing
}   // namespace robosense
#endif  // RS_ROS_FOUND