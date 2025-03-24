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
#ifndef RS_PERCEPTION_ROBOSENSE_ROS2_ROS2_BASIC_H
#define RS_PERCEPTION_ROBOSENSE_ROS2_ROS2_BASIC_H

#include <mutex>
#include "rs_perception/communication/external/common/basic_type.h"
#include "rs_perception/communication/external/common/ros_params.h"
#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include "perception_ros2_msg/msg/rs_perception_msg.hpp"
#endif // RS_ROS2_FOUND

namespace robosense {
namespace perception {

#ifdef RS_ROS2_FOUND
class RsRos2Basic : public std::enable_shared_from_this<RsRos2Basic> {

private:
    using FuncRos2RecvCallback = std::function<void(const
    perception_ros2_msg::msg::RsPerceptionMsg::SharedPtr )>;

public:
    using Ptr = std::shared_ptr<RsRos2Basic>;
    using ConstPtr = std::shared_ptr<const RsRos2Basic>;

    RsRos2Basic(const std::string &topicName, const std::shared_ptr<rclcpp::Node> &handlePtr)
    : topic_name_(topicName), ros2_handle_ptr_(handlePtr) {}

    ~RsRos2Basic() {}

    void registerErrorComm(const CommunicaterErrorCallback &cb) {
        if (cb != nullptr) {
            std::lock_guard<std::mutex> lg(ros2_error_mtx_);
            ros2_error_cbs_.push_back(cb);
        }
    }

    int initSendComm() {
        if (ros2_handle_ptr_ == nullptr) {
            return -1;
        }

        try {
            ros2_pub_ =
                ros2_handle_ptr_->create_publisher<perception_ros2_msg::msg::RsPerceptionMsg>(
                                  topic_name_, 2);
        }
        catch (const std::exception &e) {
            return -2;
        }
        return 0;
    }

    int initRecvComm(const FuncRos2RecvCallback &cb) {
        if (cb != nullptr) {
            std::lock_guard<std::mutex> lg(ros2_recv_mtx_);
            ros2_recv_cbs_.push_back(cb);
        }

        if (ros2_handle_ptr_ == nullptr) {
            return -1;
        }

        try {
            ros2_sub_ =
            ros2_handle_ptr_->create_subscription<perception_ros2_msg::msg::RsPerceptionMsg>(
            topic_name_,2,std::bind(&RsRos2Basic::localRecvCallback, shared_from_this(), std::placeholders::_1));
        }
        catch(const std::exception &e) {
            return -2;
        }

        return 0;
    }

    int send(const perception_ros2_msg::msg::RsPerceptionMsg::SharedPtr msg) {
        if (msg != nullptr) {
            ros2_pub_->publish(*msg);
        }
        else {
            return -1;
        }
        return 0;
    }

    void localRecvCallback(const perception_ros2_msg::msg::RsPerceptionMsg::SharedPtr msg) {
        if (msg != nullptr) {
            for (const auto& ros2_recv_cb: ros2_recv_cbs_) {
                if (ros2_recv_cb != nullptr) {
                    ros2_recv_cb(msg);
                }
            }
        }
    }


private:

    std::string topic_name_;
    std::shared_ptr<rclcpp::Node> ros2_handle_ptr_;
    rclcpp::Publisher<perception_ros2_msg::msg::RsPerceptionMsg>::SharedPtr ros2_pub_;
    rclcpp::Subscription<perception_ros2_msg::msg::RsPerceptionMsg>::SharedPtr ros2_sub_;
    std::mutex ros2_recv_mtx_;


    std::vector<FuncRos2RecvCallback> ros2_recv_cbs_;

    std::mutex ros2_error_mtx_;
    std::vector<CommunicaterErrorCallback> ros2_error_cbs_;
};

#endif // RS_ROS2_FOUND
}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_ROBOSENSE_ROS2_ROS2_BASIC_H
