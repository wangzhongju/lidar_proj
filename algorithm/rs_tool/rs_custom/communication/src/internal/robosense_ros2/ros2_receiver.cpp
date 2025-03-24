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
#include "rs_perception/communication/internal/robosense_ros2/ros2_receiver.h"

namespace robosense {
namespace perception {
#ifdef RS_ROS2_FOUND

int Ros2Receiver::init(const RsYamlNode &config_node_) {
    RsYamlNode config_node;
    rsYamlSubNode(config_node_, "config", config_node);
    RTRACE << name() << "=> config_node: " << config_node;
    RsYamlNode general_node, control_node, ros_node, send_control_node;

    rsYamlSubNode(config_node, "control", control_node);

    rsYamlRead(control_node, "topic_name", custom_params_.topic_name);

    rsYamlSubNode(control_node, "send_control", send_control_node);
    rsYamlRead(send_control_node, "enable_parallel",
               custom_params_.send_control.ros_send_control_enable_paralle);
    rsYamlRead(send_control_node, "parallel_buffer_size",
               custom_params_.send_control.ros_send_control_buffer_size);

    custom_params_.log(name());

    custom_msg_.reset(RsBaseCustomMsgRegisterer::getInstanceByName("RsRos2CustomMsg"));
    custom_msg_->init(config_node);

    if (Ros2Receiver::ros2_handle_ptr_ == nullptr) {
        Ros2Receiver::ros2_handle_ptr_.reset(new rclcpp::Node("ros2_receive_node"));
    }

    try {
        ros2_receiver_ptr_.reset(
        new RsRos2Basic(custom_params_.topic_name, Ros2Receiver::ros2_handle_ptr_));
    }
    catch (const std::exception &e) {
        return -1;
    }

    int ret = ros2_receiver_ptr_->initRecvComm(
    std::bind(&Ros2Receiver::localRecvCallback, this, std::placeholders::_1));
    if (ret != 0) {
        return -2;
    }

    frame_id_ = 0;

    return 0;
}

void Ros2Receiver::registerRcvComm(const PerceptReceiveCallback &cb) {
    if (cb != nullptr) {
        std::lock_guard<std::mutex> lg(precept_recv_mtx_);
        percept_recv_cbs_.push_back(cb);
    }
}

void Ros2Receiver::registerErrorComm(const CommunicaterErrorCallback &cb) {
    if (cb != nullptr) {
        ros2_receiver_ptr_->registerErrorComm(cb);
    }
}

void Ros2Receiver::localRecvCallback(const perception_ros2_msg::msg::RsPerceptionMsg::SharedPtr msg) {
    RsPerceptionMsg::Ptr msg_ptr(new RsPerceptionMsg());
    Ros2Msg::Ptr ros2_ptr(new Ros2Msg);
    ros2_ptr->ros2_const_msg_ = msg;
    custom_msg_->any_.reset(new Any(*ros2_ptr));
    custom_msg_->deSerialization(msg_ptr);

//    for (auto& recv_cb : percept_recv_cbs_) {
//        if (recv_cb != nullptr) {
//            recv_cb(msg_ptr);
//        }
//    }

    frame_id_++;
    RINFO << "robosense ros2 receive frame_id = " << frame_id_ << " successed !";
}
RS_REGISTER_RECEIVER(Ros2Receiver);
#endif  // RS_ROS2_FOUND
}  // namespace perception
}  // namespace robosense