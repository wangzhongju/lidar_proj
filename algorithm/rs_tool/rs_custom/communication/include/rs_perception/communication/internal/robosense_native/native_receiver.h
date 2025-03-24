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

#ifndef RS_PERCEPTION_COMMUNICATION_INTERNAL_ROBOSENSE_NATIVE_NATIVE_RECEIVER_H_
#define RS_PERCEPTION_COMMUNICATION_INTERNAL_ROBOSENSE_NATIVE_NATIVE_RECEIVER_H_

#include "rs_perception/communication/external/base_receiver.h"
#include "rs_perception/custom/common/base_custom_params.h"
#include "rs_perception/custom/common/base_custom_msg.h"

namespace robosense {
namespace perception {

class NativeReceiver : BaseReceiver {
public:
    using Ptr = std::shared_ptr<NativeReceiver>;

    // load configures from yaml and init native udp receive function.
    // input: yaml node
    int init(const RsYamlNode &config_node) override;

    // entrance of registering callback function
    // input: a callback function
    // output: void.
    void registerRcvComm(const PerceptReceiveCallback &cb) override;

    // entrance of registering error code callback function
    // input: a callback function
    // output: void.
    void registerErrorComm(const CommunicaterErrorCallback &cb) override;

private:
    std::string name() { return "NativeNewReceiver"; }
    void localRecvCallback(const std::string &msg);

    std::mutex precept_recv_mtx_;

    unsigned int frameId_;
    unsigned int deviceId_;
    std::string communicate_method_;

    std::vector<PerceptReceiveCallback> percept_recv_cbs_;

    SocketParams params_;
    RsBaseCustomMsg::Ptr custom_msg;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_COMMUNICATION_INTERNAL_ROBOSENSE_NATIVE_NATIVE_RECEIVER_H_
