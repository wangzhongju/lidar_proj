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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS_ROS_CUSTOM_TRANSFORMER_TRANSFORMAER_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS_ROS_CUSTOM_TRANSFORMER_TRANSFORMAER_H_

#include "rs_perception/custom/robosense_ros/ros_custom_transformer_util.h"
#include "rs_dependence/rs_dependence_manager.h"

namespace robosense {
namespace perception {
#ifdef RS_ROS_FOUND
namespace Ros {
// ros msg to perception msg
void transform(const perception_ros_msg::RsPerceptionMsgConstPtr& ros_msg_, RsPerceptionMsg::Ptr& rs_msg,
               const RsCommonCustomMsgParams& params);

// perception msg to ros msg
void transform(const RsPerceptionMsg::Ptr & rs_msg, perception_ros_msg::RsPerceptionMsgPtr& ros_msg,
               const RsCommonCustomMsgParams& params);

}  // namespace Ros
#endif  // RS_ROS_FOUND
}  // namespace perception
}  // namespace robosense



#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_ROS_ROS_CUSTOM_TRANSFORMER_TRANSFORMAER_H_
