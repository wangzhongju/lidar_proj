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

#ifndef RS_PERCEPTION_COMMUNICATION_INTERNAL_ROBOSENSE_ROS_ROS_RECEIVER_H_
#define RS_PERCEPTION_COMMUNICATION_INTERNAL_ROBOSENSE_ROS_ROS_RECEIVER_H_

#include "rs_perception/communication/external/base_receiver.h"
#include "rs_perception/communication/internal/robosense_ros/ros_basic.h"
#include "rs_perception/custom/robosense_ros/ros_custom_msg.h"
#include "rs_dependence/rs_dependence_manager.h"

namespace robosense {
namespace perception {

#ifdef RS_ROS_FOUND
class RosReceiver : public BaseReceiver {
public:
    using Ptr = std::shared_ptr<RosReceiver>;

    // load configures from yaml and init ros receive function.
    // input: yaml node
    int init(const RsYamlNode &config_node) override;

    // entrance of registering callback function
    // input: a callback function
    // output: void.
    void registerRcvComm(const PerceptReceiveCallback &cb) override;

    // entrance of registering error code callback function
    // input: a callback function
    // output: void.
    void registerErrorComm(const CommunicaterErrorCallback &cb) override;

private:
    std::string name() {
        return "RosReceiver";
    }

    unsigned int frameId_;
    std::mutex precept_recv_mtx_;
    std::vector<PerceptReceiveCallback> percept_recv_cbs_;
    RsRosBasic::Ptr ros_receiver_ptr_;
    RsCommonCustomMsgParams::Ptr custom_params_;
    RsCommunicationParams communication_params;
    RsBaseCustomMsg::Ptr custom_msg;

private:
    void localRecvCallback(
    const perception_ros_msg::RsPerceptionMsg::ConstPtr &msg);

public:
    ros::NodeHandlePtr ros_handle_ptr_ = nullptr;
};
#endif  // RS_ROS_FOUND

}  // namespace perception
}  // namespace robosense


#endif  // RS_PERCEPTION_COMMUNICATION_INTERNAL_ROBOSENSE_ROS_ROS_RECEIVER_H_
