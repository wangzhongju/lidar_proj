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

#ifndef RS_PERCEPTION_CUSTOM_COMMON_BASE_CUSTOM_MSG_H_
#define RS_PERCEPTION_CUSTOM_COMMON_BASE_CUSTOM_MSG_H_

#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/deque.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include "rs_common/external/register/register.h"
#include "rs_perception/common/external/msg/rs_perception_msg.h"
#include "rs_perception/common/external/rs_config_manager.h"
#include "rs_perception/custom/common/base_custom_params.h"
#include "rs_perception/communication/external/common/basic_type.h"

namespace robosense {
namespace perception {

class RsBaseCustomMsg {
public:
    using Ptr = std::shared_ptr<RsBaseCustomMsg>;

    virtual ~RsBaseCustomMsg() = default;

    virtual void init(const RsYamlNode &config_node_) = 0;

    virtual void serialization(const RsPerceptionMsg::Ptr &msg) = 0;

    virtual void deSerialization(const RsPerceptionMsg::Ptr &msg) = 0;

    virtual void deSerialization(const std::string& msg, std::vector<RsPerceptionMsg::Ptr>& res) = 0;

    virtual int getSerializeMessage(std::vector<RsCharBufferPtr>& msgs) = 0;

    RsPerceptionMsg::Ptr msg_;
    Any::Ptr any_;

    RsCommunicationParams params_;
};

RS_REGISTER_REGISTERER(RsBaseCustomMsg);
#define RS_REGISTER_CUSTOM_MSG(name) RS_REGISTER_CLASS(RsBaseCustomMsg, name)

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_COMMON_BASE_CUSTOM_MSG_H_
