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

#include "rs_perception/communication/internal/robosense_ros/ros_receiver.h"
#include "rs_dependence/rs_dependence_manager.h"

namespace robosense {
namespace perception {
#ifdef RS_ROS_FOUND
int RosReceiver::init(const RsYamlNode &config_node_) {
    RsYamlNode config_node;
    rsYamlSubNode(config_node_, "config", config_node);
    RTRACE << name() << "=> config_node: " << config_node;
    RsYamlNode general_node, control_node, ros_node, send_control_node;

    rsYamlSubNode(config_node, "control", control_node);

    RosParams params;
    rsYamlRead(control_node, "topic_name", params.topic_name);

    rsYamlSubNode(control_node, "send_control", send_control_node);
    rsYamlRead(send_control_node, "enable_parallel",
               params.send_control.ros_send_control_enable_paralle);
    rsYamlRead(send_control_node, "parallel_buffer_size",
               params.send_control.ros_send_control_buffer_size);

    params.log(name());

    custom_msg.reset(RsBaseCustomMsgRegisterer::getInstanceByName("RsRosCustomMsg"));
    custom_msg->init(config_node);

    if (RosReceiver::ros_handle_ptr_ == nullptr) {
        RosReceiver::ros_handle_ptr_.reset(new ros::NodeHandle());
    }

    try {
        ros_receiver_ptr_.reset(
        new RsRosBasic(params.topic_name, RosReceiver::ros_handle_ptr_));
    } catch (const std::exception &e) {
        return -1;
    }

    int ret = ros_receiver_ptr_->initRcvComm(
    std::bind(&RosReceiver::localRecvCallback, this, std::placeholders::_1));
    if (ret != 0) {
        return -2;
    }

    frameId_ = 0;

    return 0;
}

void RosReceiver::registerRcvComm(const PerceptReceiveCallback &cb) {
    if (cb != nullptr) {
        std::lock_guard<std::mutex> lg(precept_recv_mtx_);
        percept_recv_cbs_.push_back(cb);
    }
}

void RosReceiver::registerErrorComm(const CommunicaterErrorCallback &cb) {
    if (cb != nullptr) {
        ros_receiver_ptr_->registerErrorComm(cb);
    }
}

void RosReceiver::localRecvCallback(const perception_ros_msg::RsPerceptionMsgConstPtr &msg) {
    RsPerceptionMsg::Ptr msg_ptr(new RsPerceptionMsg());
    RosMsg::Ptr ros_ptr( new RosMsg);
    ros_ptr->ros_const_msg_ = msg;
    custom_msg->any_.reset(new Any(*ros_ptr));
    custom_msg->deSerialization(msg_ptr);


    for (auto recv_cb : percept_recv_cbs_) {
        if (recv_cb != nullptr) {
            recv_cb(msg_ptr);
        }
    }
    frameId_++;
    RINFO << "robosense ros receive frame_id = " << frameId_ << " successed !";
}

RS_REGISTER_RECEIVER(RosReceiver);
#endif  // RS_ROS_FOUND
}  // namespace perception
}  // namespace robosense


