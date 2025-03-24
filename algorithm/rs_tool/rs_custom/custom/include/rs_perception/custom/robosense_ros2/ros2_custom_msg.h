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
#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS2_ROS2_CUSTOM_MSG_H
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS2_ROS2_CUSTOM_MSG_H


#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_dependence/rs_dependence_manager.h"
#include "rs_perception/custom/robosense_ros2/ros2_custom_transformer.h"

namespace robosense {
namespace perception {
#ifdef RS_ROS2_FOUND

class Ros2Msg {
public:
    using Ptr = std::shared_ptr<Ros2Msg>;
    perception_ros2_msg::msg::RsPerceptionMsg::SharedPtr ros2_msg_;
    perception_ros2_msg::msg::RsPerceptionMsg::ConstSharedPtr ros2_const_msg_;
};


class RsRos2CustomMsg : public RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsRos2CustomMsg>;
    
    RsRos2CustomMsg() {
        ros2_ptr_.reset(new Ros2Msg);
        msg_.reset(new RsPerceptionMsg);
    }
    
    // load configures from yaml and init ros2 perception message serialize function.
    // input: yaml node
    void init(const RsYamlNode &config_node_) override {
        custom_params_.reset(new RsCommonCustomMsgParams);
        RsYamlNode general_node, ros_node;
        rsYamlSubNode(config_node_, "general", general_node);
        rsYamlSubNode(config_node_, "ros", ros_node);

        // general
        rsYamlRead(general_node, "device_id", custom_params_->device_id);

        // ros
        rsYamlRead(ros_node, "send_point_cloud", custom_params_->send_point_cloud);
        rsYamlRead(ros_node, "send_attention_objects", custom_params_->send_attention_objects);
        rsYamlRead(ros_node, "send_freespace", custom_params_->send_freespace);
        rsYamlRead(ros_node, "send_lane", custom_params_->send_lane);
        rsYamlRead(ros_node, "send_roadedge", custom_params_->send_roadedge);
        rsYamlRead(ros_node, "send_sematic_indices", custom_params_->send_sematic);

        custom_params_->log(name());
    }

    // entrance of perception message serialize.
    // input: robosense perception message.
    // output: void. the result of serialization will be recorded in an internal variable.
    void serialization(const RsPerceptionMsg::Ptr &msg) override {
         msg_ = msg;
         if (custom_params_) {
             Ros2::transform(msg_, ros2_ptr_->ros2_msg_, *custom_params_);
             any_.reset(new Any(*ros2_ptr_));
         }
    }

    // entrance of perception message deserialize.
    // input: robosense perception message.
    // output: void. the result of deserialization will be recorded in an internal variable.
    void deSerialization(const RsPerceptionMsg::Ptr &msg) override {
         if (custom_params_) {
             auto ptr = any_->AnyCast<Ros2Msg>();
             ros2_ptr_->ros2_const_msg_ = ptr->ros2_const_msg_;
             Ros2::transform(ros2_ptr_->ros2_const_msg_, msg_, *custom_params_);
         }
         auto& result_ptr = msg->rs_lidar_result_ptr;
         result_ptr = msg_->rs_lidar_result_ptr;
    }

    // entrance of perception message deserialize.
    // input: robosense perception message and message buffer.
    // output: void.
    void deSerialization(const std::string& msg, std::vector<RsPerceptionMsg::Ptr>& res) override {
        (void)(msg);
        (void)(res);
    }

    // entrance of getting the result of serialization.
    // input: a vector that used to save the buffers.
    // output: void. the result of serialization is added into the vector with a message header.
    //               for ros2 communication method, this function is empty.
    int getSerializeMessage(std::vector<RsCharBufferPtr>& msgs) override {
        (void)(msgs);
        return 0;
    }

    std::string name() {
        return "Ros2Parameters";
    }

    Ros2Msg::Ptr ros2_ptr_;
    RsCommonCustomMsgParams::Ptr custom_params_;
    
};

#endif //  RS_ROS2_FOUND
}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS2_ROS2_CUSTOM_MSG_H
