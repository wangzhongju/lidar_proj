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

#include "rs_application/version.h"
#include "rs_application/application.h"

namespace robosense {

void RsApplication::init(const RsYamlNode& config) {
    RsYamlNode general_node;
    rsYamlSubNode(config, "general", general_node);
    std::string application;
    rsYamlRead(general_node, "application", application);

    std::string strategy = application + "Application";

    if (!BaseRsApplicationRegisterer::IsValid(strategy)) {
        RERROR << name() <<": invalid application " << application;
        RS_THROW("invalid application!");
    }
    impl_ptr_.reset(BaseRsApplicationRegisterer::getInstanceByName(strategy));
    impl_ptr_->init(config);
    RINFO << name() << ": version " << RS_SDK_VERSION;
    RINFO << name() << ": ready!";
}

void RsApplication::start() {
    impl_ptr_->start();
}

void RsApplication::stop() {
    impl_ptr_->stop();
}

}  // namespace robosense
