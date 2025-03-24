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
#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS2_ROS2_CUSTOM_TRANSFORMER_H
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS2_ROS2_CUSTOM_TRANSFORMER_H

#include "rs_perception/custom/robosense_ros2/ros2_custom_transformer_util.h"


namespace robosense {
namespace perception {

#ifdef RS_ROS2_FOUND
namespace Ros2{

// RsPerceptionMsg -> ros2 msg
void transform(const RsPerceptionMsg::Ptr& rs_msg, perception_ros2_msg::msg::RsPerceptionMsg::SharedPtr& ros2_msg,
               const RsCommonCustomMsgParams& custom_params);

// ros2 msg -> RsPerceptionMsg
void transform(const perception_ros2_msg::msg::RsPerceptionMsg::ConstSharedPtr& ros2_msg, RsPerceptionMsg::Ptr& rs_msg,
               const RsCommonCustomMsgParams& custom_params);
}
#endif  // RS_ROS2_FOUND
}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS2_ROS2_CUSTOM_TRANSFORMER_H
