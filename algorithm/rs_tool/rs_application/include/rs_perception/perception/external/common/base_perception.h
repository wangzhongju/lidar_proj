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

#ifndef RS_PERCEPTION_PERCEPTION_EXTERNAL_COMMON_BASE_PERCEPTION_H_
#define RS_PERCEPTION_PERCEPTION_EXTERNAL_COMMON_BASE_PERCEPTION_H_

#include "rs_common/external/common.h"
#include "rs_perception/common/external/common.h"

namespace robosense {
namespace perception {

class BasePerception{
public:
    using Ptr = std::shared_ptr<BasePerception>;

    virtual ~BasePerception() = default;

    virtual void init(const RsYamlNode& config_node) = 0;

    virtual void perception(const RsPerceptionMsg::Ptr& msg_ptr) = 0;

    virtual void start() = 0;

    virtual void stop() = 0;

    virtual void addData(const RsPerceptionMsg::Ptr& msg_ptr) = 0;

    virtual void regPerceptionCallback(const std::function<void(const RsPerceptionMsg::Ptr& msg_ptr)>& cb) = 0;

    virtual std::string name() = 0;
};

RS_REGISTER_REGISTERER(BasePerception);
#define RS_REGISTER_PERCEPTION(name)  \
RS_REGISTER_CLASS(BasePerception, name)

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_PERCEPTION_EXTERNAL_COMMON_BASE_PERCEPTION_H_
