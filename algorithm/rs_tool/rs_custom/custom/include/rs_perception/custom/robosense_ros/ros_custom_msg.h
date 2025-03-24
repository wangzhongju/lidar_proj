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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS_ROS_CUSTOM_MSG_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS_ROS_CUSTOM_MSG_H_

#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND
#include "perception_ros_msg/RsPerceptionMsg.h"
#include "rs_perception/custom/robosense_ros/ros_custom_transformer.h"
#endif  // RS_ROS_FOUND

namespace robosense {
namespace perception {
#ifdef RS_ROS_FOUND
// 自定义数据
class RosMsg {
public:
    using Ptr = std::shared_ptr<RosMsg>;
    perception_ros_msg::RsPerceptionMsgPtr ros_msg_;
    perception_ros_msg::RsPerceptionMsgConstPtr ros_const_msg_;
};

class RsRosCustomMsg : public RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsRosCustomMsg>;

    RsRosCustomMsg() {
        ros_ptr.reset(new RosMsg);
        msg_.reset(new RsPerceptionMsg);
    }

    // load configures from yaml and init ros perception message serialize function.
    // input: yaml node
    void init(const RsYamlNode &config_node_) override {
        customParams.reset(new RsCommonCustomMsgParams);
        RsYamlNode general_node, ros_node;
        rsYamlSubNode(config_node_, "general", general_node);
        rsYamlSubNode(config_node_, "ros", ros_node);

        // general
        rsYamlRead(general_node, "device_id", customParams->device_id);

        // ros
        rsYamlRead(ros_node, "send_point_cloud", customParams->send_point_cloud);
        rsYamlRead(ros_node, "send_attention_objects", customParams->send_attention_objects);
        rsYamlRead(ros_node, "send_freespace", customParams->send_freespace);
        rsYamlRead(ros_node, "send_lane", customParams->send_lane);
        rsYamlRead(ros_node, "send_roadedge", customParams->send_roadedge);
        rsYamlRead(ros_node, "send_sematic_indices", customParams->send_sematic);

        customParams->log(name());
    }

    // entrance of perception message serialize.
    // input: robosense perception message.
    // output: void. the result of serialization will be recorded in an internal variable.
    void serialization(const RsPerceptionMsg::Ptr &msg) override {
        msg_ = msg;
        if (customParams) {
            Ros::transform(msg_, ros_ptr->ros_msg_, *customParams);
            any_.reset(new Any(*ros_ptr));
        }
    }

    // entrance of perception message deserialize.
    // input: robosense perception message.
    // output: void. the result of deserialization will be recorded in an internal variable.
    void deSerialization(const RsPerceptionMsg::Ptr &msg) override {
        if (customParams) {
            auto ptr = any_->AnyCast<RosMsg>();
            ros_ptr->ros_const_msg_ = ptr->ros_const_msg_;
            Ros::transform(ros_ptr->ros_const_msg_, msg_, *customParams);
        }
        auto &result_ptr = msg->rs_lidar_result_ptr;
        result_ptr = msg_->rs_lidar_result_ptr;
    }

    // entrance of perception message deserialize.
    // input: robosense perception message and message buffer.
    // output: void.
    void deSerialization(const std::string& msg, std::vector<RsPerceptionMsg::Ptr>& res) override{
        (void)(msg);
        (void)(res);
    }

    // entrance of getting the result of serialization.
    // input: a vector that used to save the buffers.
    // output: void. the result of serialization is added into the vector with a message header.
    //               for ros communication method, this function is empty.
    int getSerializeMessage(std::vector<RsCharBufferPtr>& msgs) override {
        (void)(msgs);
    }

    std::string name() {
        return "RosParameters";
    }

    RosMsg::Ptr ros_ptr;
    RsCommonCustomMsgParams::Ptr customParams;
};
#endif  // RS_ROS_FOUND
}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS_ROS_CUSTOM_MSG_H_
