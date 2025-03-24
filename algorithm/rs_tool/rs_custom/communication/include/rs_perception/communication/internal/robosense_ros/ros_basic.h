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

#ifndef RS_PERCEPTION_ROBOSENSE_ROS_ROS_BASIC_H_
#define RS_PERCEPTION_ROBOSENSE_ROS_ROS_BASIC_H_

#include <mutex>
#include "rs_perception/communication/external/common/basic_type.h"
#include "rs_perception/communication/external/common/ros_params.h"
#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND
#include "perception_ros_msg/RsPerceptionMsg.h"
#endif  // RS_ROS_FOUND
namespace robosense {
namespace perception {

#ifdef RS_ROS_FOUND
class RsRosBasic : public std::enable_shared_from_this<RsRosBasic> {
 public:
  using Ptr = std::shared_ptr<RsRosBasic>;
  using ConstPtr = std::shared_ptr<const RsRosBasic>;

  void registerErrorComm(const CommunicaterErrorCallback &cb) {
    if (cb != nullptr) {
      std::lock_guard<std::mutex> lg(ros_error_mtx_);
      ros_error_cbs_.push_back(cb);
    }
  }
  RsRosBasic(const std::string &topicName, const ros::NodeHandlePtr &handlePtr)
      : topic_name_(topicName), ros_handle_ptr_(handlePtr) {}

  ~RsRosBasic() {}

 private:
  using FuncRosRecvCallback = std::function<void(
      const perception_ros_msg::RsPerceptionMsg::ConstPtr &)>;

 public:
  int initSndComm() {
    if (ros_handle_ptr_ == nullptr) {
      return -1;
    }

    try {
      ros_pub_ =
          ros_handle_ptr_->advertise<perception_ros_msg::RsPerceptionMsg>(
              topic_name_, 1);
    } catch (const std::exception &e) {
      return -2;
    }
    return 0;
  }

  int initRcvComm(const FuncRosRecvCallback &cb) {
    if (cb != nullptr) {
      std::lock_guard<std::mutex> lg(ros_recv_mtx_);
      ros_recv_cbs_.push_back(cb);
    }

    if (ros_handle_ptr_ == nullptr) {
      return -1;
    }

    try {
      ros_sub_ =
          ros_handle_ptr_->subscribe<perception_ros_msg::RsPerceptionMsg>(
              topic_name_, 1,
              std::bind(&RsRosBasic::localRecvCallback, this,
                        std::placeholders::_1));
    } catch (const std::exception &e) {
      return -2;
    }

    return 0;
  }

  int send(const perception_ros_msg::RsPerceptionMsg::ConstPtr &msg) {
    if (msg != nullptr) {
      ros_pub_.publish(msg);
    } else {
      return -1;
    }

    return 0;
  }

 public:
  void localRecvCallback(
      const perception_ros_msg::RsPerceptionMsg::ConstPtr &msg) {
    if (msg != nullptr) {
      for (auto ros_recv_cb : ros_recv_cbs_) {
        if (ros_recv_cb != nullptr) {
          ros_recv_cb(msg);
        }
      }
    }
  }

 private:
  std::string topic_name_;
  ros::Publisher ros_pub_;
  ros::Subscriber ros_sub_;
  ros::NodeHandlePtr ros_handle_ptr_;
  std::mutex ros_recv_mtx_;
  std::vector<FuncRosRecvCallback> ros_recv_cbs_;

  std::mutex ros_error_mtx_;
  std::vector<CommunicaterErrorCallback> ros_error_cbs_;
};
#endif  // RS_ROS_FOUND
}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_ROBOSENSE_ROS_ROS_BASIC_H_
