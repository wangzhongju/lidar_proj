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
#include "rs_localization/sender/external/sender.h"

namespace robosense {
namespace localization {
int LocalizationSender::init(const RsYamlNode &config_node,
                             const std::shared_ptr<localization::LocalizationInterface> &localization_ptr) {
    RsYamlNode localization_node = config_node;

    impl_ptrs_.clear();
    std::string method;
    for (size_t i = 0; i < localization_node.size(); ++i) {
        rsYamlRead(localization_node[i], "method", method);
        std::string strategy = method + "LocalizationSender";
        if (!BaseLocalizationSenderRegisterer::IsValid(strategy)) {
            RERROR << name() << ": init strategy " << strategy << " failed!";
            RS_THROW("init register class error!");
        }
        BaseLocalizationSender::Ptr impl_ptr(BaseLocalizationSenderRegisterer::getInstanceByName(strategy));
        int ret = impl_ptr->init(localization_node[i], localization_ptr);
        if (ret != 0) {
            RERROR << name() << ": init " << strategy
                   << " sender failed: index = " << i;
            RS_THROW("init sender error !");
        }
        impl_ptrs_.push_back(impl_ptr);
    }
    return 0;
}

void LocalizationSender::send() {
    for (auto impl_ptr : impl_ptrs_) {
        if (impl_ptr != nullptr) {
            impl_ptr->send();
        }
    }
}

void LocalizationSender::start() {
    for (auto impl_ptr : impl_ptrs_) {
        if (impl_ptr != nullptr) {
            impl_ptr->start();
        }
    }
}

void LocalizationSender::stop() {
    for (auto impl_ptr : impl_ptrs_) {
        if (impl_ptr != nullptr) {
            impl_ptr->stop();
        }
    }
}

}   // namespace localization
}   // namespace robosense