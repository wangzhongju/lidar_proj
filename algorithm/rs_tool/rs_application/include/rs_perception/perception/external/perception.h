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

#ifndef RS_PERCEPTION_PERCEPTION_EXTERNAL_PERCEPTION_H_
#define RS_PERCEPTION_PERCEPTION_EXTERNAL_PERCEPTION_H_

#include "rs_perception/perception/external/common/base_perception.h"

namespace robosense {
namespace perception {

class Perception:public BasePerception {
public:
    using Ptr = std::shared_ptr<Perception>;

    ~Perception() {
        stop();
    }

    void init(const RsYamlNode& config_node) override;

    void perception(const RsPerceptionMsg::Ptr& msg_ptr) override;

    void start() override;

    void stop() override;

    void addData(const RsPerceptionMsg::Ptr& msg_ptr) override;

    void regPerceptionCallback(const std::function<void(const RsPerceptionMsg::Ptr& msg_ptr)>& cb) override;

private:
    inline std::string name() override {
        return "Perception";
    }

    BasePerception::Ptr impl_ptr_;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_PERCEPTION_EXTERNAL_PERCEPTION_H_
