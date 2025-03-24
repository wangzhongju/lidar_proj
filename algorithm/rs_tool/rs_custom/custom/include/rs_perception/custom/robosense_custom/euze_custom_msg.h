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
#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_EUZE_CUSTOM_MSG_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_EUZE_CUSTOM_MSG_H_

#include "rs_perception/custom/common/base_custom_msg.h"

namespace robosense {
namespace perception {

class EuzeMsg {
 public:
  using Ptr = std::shared_ptr<EuzeMsg>;
  // 用户自定义数据
};
class RsEuzeCustomMsg : public RsBaseCustomMsg {
 public:
    using Ptr = std::shared_ptr<RsEuzeCustomMsg>;

  void init(const RsYamlNode &config_node_) override {
    customParams_.reset(new RsCustomMsgParams);
    RsYamlNode general_node, custom_node;

    rsYamlSubNode(config_node_, "general", general_node);
    rsYamlSubNode(config_node_, "custom", custom_node);

    // general
    rsYamlRead(general_node, "device_id", customParams_->device_id);

    // custom
    rsYamlRead(custom_node, "send_point_cloud", customParams_->send_point_cloud);
    rsYamlRead(custom_node, "send_attention_objects", customParams_->send_attention_objects);
    rsYamlRead(custom_node, "send_freespace", customParams_->send_freespace);
    rsYamlRead(custom_node, "send_lane", customParams_->send_lane);
    rsYamlRead(custom_node, "send_roadedge", customParams_->send_roadedge);
    rsYamlRead(custom_node, "send_sematic_indices", customParams_->send_sematic);
  }

  void serialization(const RsPerceptionMsg::Ptr& msg) override {
    msg_ = msg;
    // 自行定义转换函数
  }
  void deSerialization(const RsPerceptionMsg::Ptr& msg) override {
    // 自行定义转换函数
      auto& res_ptr = msg->rs_lidar_result_ptr;
      res_ptr = msg_->rs_lidar_result_ptr;
  }

  void deSerialization(const std::string& msg, std::vector<RsPerceptionMsg::Ptr>& res) override{

  }

  int getSerializeMessage(std::vector<RsCharBufferPtr>& msgs) override {

  }

  template <class Archive>
  void serialize(Archive& archive) {
    // extra_infos
    // archive(params_.device_id);
  }

  EuzeMsg::Ptr custom_msg_;
  RsCustomMsgParams::Ptr customParams_;
};

}  // namespace perception
}  // namespace robosense

CEREAL_REGISTER_TYPE(robosense::perception::RsEuzeCustomMsg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(robosense::perception::RsBaseCustomMsg,
                                     robosense::perception::RsEuzeCustomMsg)

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_EUZE_CUSTOM_MSG_H_
