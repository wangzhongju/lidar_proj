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

#include "rs_perception/perception/external/perception.h"

namespace robosense {
namespace perception {

void Perception::init(const RsYamlNode& config_node) {
    RsYamlNode general_node;
    rsYamlSubNode(config_node, "general", general_node);
    std::string application;
    rsYamlRead(general_node, "_application_", application);
    std::string strategy = application + "Perception";

    if (!BasePerceptionRegisterer::IsValid(strategy)) {
        RERROR << name() <<": invalid application " << application;
        RS_THROW("invalid application!");
    }
    impl_ptr_.reset(BasePerceptionRegisterer::getInstanceByName(strategy));
    impl_ptr_->init(config_node);
    RsConfigManager().append(name() + ": application " + strategy);
}

void Perception::perception(const RsPerceptionMsg::Ptr& msg_ptr) {
    impl_ptr_->perception(msg_ptr);
}

void Perception::start() {
    impl_ptr_->start();
}

void Perception::stop() {
    impl_ptr_->stop();
}

void Perception::addData(const RsPerceptionMsg::Ptr& msg_ptr) {
    impl_ptr_->addData(msg_ptr);
}

void Perception::regPerceptionCallback(const std::function<void(const RsPerceptionMsg::Ptr& msg_ptr)>& cb) {
    impl_ptr_->regPerceptionCallback(cb);
}

}  // namespace perception
}  // namespace robosense
