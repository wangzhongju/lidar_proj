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

#ifndef RS_PREPROCESSING_SENDER_EXTERNAL_CLOUD_SENDER_H_
#define RS_PREPROCESSING_SENDER_EXTERNAL_CLOUD_SENDER_H_

#include "rs_preprocessing/sender/external/base_cloud_sender.h"
namespace robosense {
namespace preprocessing {

class CloudSender {
public:
    using Ptr = std::shared_ptr<CloudSender>;

    // load configures from yaml and init the localization cloud sender function.
    // input: yaml node
    // output: void.
    void init(const RsYamlNode& config_node, const RsPreprocessing::Ptr& preprocessing);

    // entrance of sending localization cloud message
    // input: robosense lidar point cloud message struct.
    // output: void.
    void send(const lidar::LidarPointCloudMsg::Ptr& lidar_msg);

    // entrance of adding data to localization cloud sender.
    // input: robosense lidar point cloud message struct.
    // output: void. data added by this function will be delivered to function "send" in line 42.
    void addData(const lidar::LidarPointCloudMsg::Ptr& lidar_msg);

    // start the thread and wait for data.
    void start();

    // stop the thread.
    void stop();

private:
    std::string name() {
        return "CLoudSender";
    }
    std::vector<BaseCloudSender::Ptr> impl_vec_;
};
}   // namespace preprocessing
}   // namespace robosense

#endif  // RS_PREPROCESSING_SENDER_EXTERNAL_CLOUD_SENDER_H_