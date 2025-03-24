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

#ifndef RS_PERCEPTION_COMMUNICATION_EXTERNAL_BASE_SENDER_H_
#define RS_PERCEPTION_COMMUNICATION_EXTERNAL_BASE_SENDER_H_

#include "rs_common/external/common.h"
#include "rs_perception/common/external/common.h"
#include "rs_perception/communication/external/common/socket_udp_sender.h"
#include "rs_perception/communication/external/common/socket_tcp_sender_client.h"
#include "rs_perception/communication/external/common/socket_tcp_sender_server.h"

namespace robosense {
namespace perception {

class BaseSender {
 public:
  using Ptr = std::shared_ptr<BaseSender>;
  virtual ~BaseSender() = default;

  virtual int init(const RsYamlNode& config_node) = 0;

  virtual COMMUNICATION_ERROR_CODE send(
      const RsPerceptionMsg::Ptr& msg_ptr) = 0;

  virtual void registerErrorComm(const CommunicaterErrorCallback& cb) = 0;

 protected:
  RsSocketUdpSender::Ptr socket_sender_ptr_;
  RsSocketTcpSenderClient::Ptr socket_tcp_client_sender_ptr_;
  RsSocketTcpSenderServer::Ptr socket_tcp_server_sender_ptr_;
};

RS_REGISTER_REGISTERER(BaseSender);
#define RS_REGISTER_SENDER(name) RS_REGISTER_CLASS(BaseSender, name)

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_COMMUNICATION_EXTERNAL_BASE_SENDER_H_
