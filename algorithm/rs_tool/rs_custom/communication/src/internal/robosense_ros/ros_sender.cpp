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

#include "rs_perception/communication/internal/robosense_ros/ros_sender.h"
#include "rs_dependence/rs_dependence_manager.h"

namespace robosense {
namespace perception {
#ifdef RS_ROS_FOUND
int RosSender::init(const RsYamlNode &config_node_) {
    RsYamlNode config_node;
    rsYamlSubNode(config_node_, "config", config_node);
    RTRACE << name() << "=> config_node: " << config_node;
    RsYamlNode general_node, control_node, ros_node, send_control_node;
    rsYamlSubNode(config_node, "control", control_node);

    // control
    rsYamlRead(control_node, "topic_name", params_.topic_name);

    rsYamlSubNode(control_node, "send_control", send_control_node);
    rsYamlRead(send_control_node, "enable_parallel",
               params_.send_control.ros_send_control_enable_paralle);
    rsYamlRead(send_control_node, "parallel_buffer_size",
               params_.send_control.ros_send_control_buffer_size);

    params_.log(name());

    custom_msg.reset(RsBaseCustomMsgRegisterer::getInstanceByName("RsRosCustomMsg"));
    custom_msg->init(config_node);

    if (ros_handle_ptr_ == nullptr) {
        ros_handle_ptr_.reset(new ros::NodeHandle());
    }

    try {
        ros_sender_ptr_.reset(
        new RsRosBasic(params_.topic_name, ros_handle_ptr_));
    } catch (const std::exception &e) {
        return -1;
    }

    int errCode = ros_sender_ptr_->initSndComm();
    if (errCode != 0) {
        return -2;
    }
    RINFO << "RsCommunication: ros method init success!";
    frameId_ = 0;
    return 0;
}

COMMUNICATION_ERROR_CODE RosSender::send(const RsPerceptionMsg::Ptr &msg_ptr) {
    custom_msg->serialization(msg_ptr);

    perception_ros_msg::RsPerceptionMsgPtr ros_msg_;
    ros_msg_ = custom_msg->any_->AnyCast<RosMsg>()->ros_msg_;

    if (ros_sender_ptr_ != nullptr && ros_msg_!= nullptr) {
        ros_sender_ptr_->send(ros_msg_);
    }
    ++frameId_;
    RINFO << "robosense ros send frame_id = " << frameId_ << " successed !";

    return COMMUNICATION_ERROR_CODE::Success;
}

void RosSender::registerErrorComm(const CommunicaterErrorCallback &cb) {
    if (cb != nullptr) {
        ros_sender_ptr_->registerErrorComm(cb);
    }
}

RS_REGISTER_SENDER(RosSender);
#endif  // RS_ROS_FOUND
}  // namespace perception
}  // namespace robosense

