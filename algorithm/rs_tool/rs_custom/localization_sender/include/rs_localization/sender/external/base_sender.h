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

#ifndef RS_LOCALIZATION_SENDER_EXTERNAL_BASE_SENDER_H_
#define RS_LOCALIZATION_SENDER_EXTERNAL_BASE_SENDER_H_

#include "rs_common/external/common.h"
#include "rs_localization/external/localization_interface.h"
#include "rs_perception/communication/external/common/socket_udp_sender.h"

namespace robosense {
namespace localization {

class BaseLocalizationSender {
public:
    using Ptr = std::shared_ptr<BaseLocalizationSender>;
    virtual ~BaseLocalizationSender() = default;

    virtual int init(const RsYamlNode& config_node, const std::shared_ptr<localization::LocalizationInterface>&) = 0;

    virtual void send() = 0;

    virtual void start() = 0;

    virtual void stop() = 0;

protected:
    perception::RsSocketUdpSender::Ptr socket_sender_ptr_;
    perception::SocketParams socket_params_;
    std::shared_ptr<LocalizationInterface> localization_ptr_;
};

RS_REGISTER_REGISTERER(BaseLocalizationSender);
#define RS_REGISTER_LOCALIZATION_SENDER(name) RS_REGISTER_CLASS(BaseLocalizationSender, name)

}  // namespace localization
}  // namespace robosense

#endif  // RS_LOCALIZATION_SENDER_EXTERNAL_BASE_SENDER_H_