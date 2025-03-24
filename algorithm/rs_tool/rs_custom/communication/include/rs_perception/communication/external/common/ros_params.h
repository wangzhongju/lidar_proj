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

#ifndef RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_ROS_PARAMS_H_
#define RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_ROS_PARAMS_H_

#include <functional>
#include <memory>
#include <sstream>

#include "rs_perception/common/external/rs_config_manager.h"

namespace robosense {
namespace perception {

struct RosSendControl {
  bool ros_send_control_enable_paralle = false;
  unsigned int ros_send_control_buffer_size = 2;
};

struct RosParams {
  std::string topic_name;
  RosSendControl send_control;

  void log(const std::string &name) {
    std::stringstream ss;
    ss << name << ": RosParams topic_name " << topic_name << std::endl;
    ss << name << ": RosParams RosSendControl ros_send_control_enable_paralle "
       << send_control.ros_send_control_enable_paralle << std::endl;
    ss << name << ": RosParams RosSendControl ros_send_control_buffer_size "
       << send_control.ros_send_control_buffer_size << std::endl;

    RsConfigManager().append(ss.str());
  }
};

struct Ros2Params {
    std::string topic_name;
    RosSendControl send_control;

    void log(const std::string &name) {
        std::stringstream ss;
        ss << name << ": Ros2Params topic_name " << topic_name << std::endl;
        ss << name << ": Ros2Params Ros2SendControl ros_send_control_enable_paralle "
        << send_control.ros_send_control_enable_paralle << std::endl;
        ss << name << ": Ros2Params Ros2SendControl ros_send_control_buffer_size "
        << send_control.ros_send_control_buffer_size << std::endl;

        RsConfigManager().append(ss.str());
    }
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_ROS_PARAMS_H_
