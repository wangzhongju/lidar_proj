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
#ifndef RS_SDK_SENDER_H
#define RS_SDK_SENDER_H

#include "rs_localization/sender/external/base_sender.h"

namespace robosense {
namespace localization {

class LocalizationSender {
public:
    using Ptr = std::shared_ptr<LocalizationSender>;

    // load configures from yaml and init the localization info sender function.
    // input: yaml node
    // output: void.
    int init(const RsYamlNode& config_node, const std::shared_ptr<localization::LocalizationInterface>&);

    // entrance of sending localization cloud message
    // input: void.
    // output: void.
    void send();

    // start the thread and wait for data.
    void start();

    // stop the thread.
    void stop();

protected:
    std::string name() {
        return "LocalizationSender";
    }
    std::vector<BaseLocalizationSender::Ptr> impl_ptrs_;

};
}   // namespace localization
}   // namespace robosense

#endif //RS_SDK_SENDER_H
