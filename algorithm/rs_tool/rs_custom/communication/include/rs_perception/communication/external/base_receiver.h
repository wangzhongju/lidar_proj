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

#ifndef RS_PERCEPTION_COMMUNICATION_EXTERNAL_BASE_RECEIVER_H_
#define RS_PERCEPTION_COMMUNICATION_EXTERNAL_BASE_RECEIVER_H_

#include "rs_common/external/common.h"
#include "rs_perception/common/external/common.h"
#include "rs_perception/communication/external/common/socket_udp_receiver.h"
#include "rs_perception/communication/external/common/socket_tcp_receiver_client.h"
#include "rs_perception/communication/external/common/socket_tcp_receiver_server.h"

namespace robosense {
namespace perception {

class BaseReceiver {
 public:
  using Ptr = std::shared_ptr<BaseReceiver>;

  virtual ~BaseReceiver() = default;

  virtual int init(const RsYamlNode &config_node) = 0;

  virtual void registerRcvComm(
      const PerceptReceiveCallback &cb) = 0;

  virtual void registerErrorComm(
      const CommunicaterErrorCallback &cb) = 0;

 protected:
  RsSocketUdpReceiver::Ptr socket_receiver_ptr_;
  RsSocketTcpReceiverServer::Ptr socket_tcp_server_receiver_ptr_;
  RsSocketTcpReceiverClient::Ptr socket_tcp_client_receiver_ptr_;
};

RS_REGISTER_REGISTERER(BaseReceiver);
#define RS_REGISTER_RECEIVER(name) RS_REGISTER_CLASS(BaseReceiver, name)

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_COMMUNICATION_EXTERNAL_BASE_RECEIVER_H_
