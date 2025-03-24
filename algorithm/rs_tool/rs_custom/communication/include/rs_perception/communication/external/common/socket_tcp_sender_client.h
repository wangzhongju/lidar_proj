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

#ifndef RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_TCP_SENDER_CLIENT_H_
#define RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_TCP_SENDER_CLIENT_H_

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "rs_perception/communication/external/common/basic_type.h"
#include "rs_perception/communication/external/common/error_code.h"
#include "rs_perception/communication/external/common/socket_params.h"

namespace robosense {
namespace perception {
class RsSocketTcpSenderClient {
 public:
  using Ptr = std::shared_ptr<RsSocketTcpSenderClient>;
  using ConstPtr = std::shared_ptr<const RsSocketTcpSenderClient>;

 public:
  explicit RsSocketTcpSenderClient(const SocketParams &params) {
    params_ = params;
  }

  ~RsSocketTcpSenderClient() {
    if (connect_thread_ptr_) {
      connect_is_stop_ = true;
      RINFO << name() << ": exit tcp client socket connect thread ... ";
      if (connect_thread_ptr_->joinable()) {
        connect_thread_ptr_->join();
      }
    }

    if (send_thread_ptr_) {
      send_is_stop_ = true;
      send_msg_cond_.notify_all();
      RINFO << name() << ": exit tcp client socket send thread ...";
      if (send_thread_ptr_->joinable()) {
        send_thread_ptr_->join();
      }
    }
  }

  COMMUNICATION_ERROR_CODE initSocket(
      const std::vector<CommunicaterErrorCallback> &cbs);

  COMMUNICATION_ERROR_CODE send(const std::vector<char> &buffer);

  void registerErrorComm(const CommunicaterErrorCallback &cb);

 private:
  void closeSocket();

  COMMUNICATION_ERROR_CODE initThread();

  COMMUNICATION_ERROR_CODE initSocket();

  void connect();

  void core();

 private:
  std::string name() { return "RsSocketTcpSenderClient"; }

 private:
  std::mutex socket_mtx_;
  bool socket_is_connect_;
  struct sockaddr_in ser_addr_;

  SocketParams params_;
  struct timeval timeout_;
  int socket_fd_;
  bool connect_is_stop_;
  std::shared_ptr<std::thread> connect_thread_ptr_;

  std::mutex send_msg_mtx_;
  bool send_is_stop_;
  std::condition_variable send_msg_cond_;
  std::queue<std::vector<char>> send_msgs_;
  std::shared_ptr<std::thread> send_thread_ptr_;

  std::mutex mx_er_cb_;
  std::vector<CommunicaterErrorCallback> er_cb_list_;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_TCP_SENDER_CLIENT_H_
