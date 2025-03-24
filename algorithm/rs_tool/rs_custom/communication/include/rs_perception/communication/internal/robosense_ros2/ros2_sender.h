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
#ifndef RS_PERCEPTION_ROBOSENSE_ROS2_ROS2_SENDER_H
#define RS_PERCEPTION_ROBOSENSE_ROS2_ROS2_SENDER_H

#include "rs_perception/communication/external/base_sender.h"
#include "rs_perception/communication/internal/robosense_ros2/ros2_basic.h"
#include "rs_perception/custom/robosense_ros2/ros2_custom_msg.h"
#include "rs_dependence/rs_dependence_manager.h"

namespace robosense {
namespace perception {

#ifdef RS_ROS2_FOUND

class Ros2Sender : public BaseSender {
public:
    using Ptr = std::shared_ptr<Ros2Sender>;

    // load configures from yaml and init ros2 send function.
    // input: yaml node
    int init(const RsYamlNode& config_node) override;

    // entrance of sending perception result by socket
    // input: robosense perception message struct
    // output: a robosense communication error code.
    //         this code can indicate whether the transmission was successful or not.
    COMMUNICATION_ERROR_CODE send(const RsPerceptionMsg::Ptr& msg_ptr) override;

    // entrance of registering error code callback function
    // input: a callback function
    // output: void.
    void registerErrorComm(const CommunicaterErrorCallback& cb) override;

    std::shared_ptr<rclcpp::Node> ros2_handle_ptr_;

private:
    std::string name() {
        return "Ros2Sender";
    }

    Ros2Params custom_params_;
    unsigned int frame_id_;
    unsigned int device_id_;
    RsRos2Basic::Ptr ros2_sender_ptr_;
    RsBaseCustomMsg::Ptr custom_msg_;
};


#endif // RS_ROS2_FOUND
}
}


#endif  // RS_PERCEPTION_ROBOSENSE_ROS2_ROS2_SENDER_H
