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

#ifndef RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_TCP_SENDER_SERVER_H_
#define RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_TCP_SENDER_SERVER_H_

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
class RsSocketTcpSenderServer {
 public:
  using Ptr = std::shared_ptr<RsSocketTcpSenderServer>;
  using ConstPtr = std::shared_ptr<const RsSocketTcpSenderServer>;

 public:
  explicit RsSocketTcpSenderServer(const SocketParams &params) {
    params_ = params;
  }

  ~RsSocketTcpSenderServer() {
    if (listen_thread_ptr_) {
      closeListenSocket();
      listen_is_stop_ = true;
      RINFO << name() << ": exit tcp client socket listen thread ... ";
      if (listen_thread_ptr_->joinable()) {
        listen_thread_ptr_->join();
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
  void closeSessionSocket(int session_fd);

  void closeListenSocket();

  COMMUNICATION_ERROR_CODE initThread();

  COMMUNICATION_ERROR_CODE initSessionSocket(int session_fd);

  COMMUNICATION_ERROR_CODE initListenSocket();

  void listen();

  void core();

 private:
  std::string name() { return "RsSocketTcpSenderServer"; }

 private:
  SocketParams params_;
  struct timeval timeout_;

  std::mutex listen_socket_mtx_;
  bool listen_is_stop_;
  struct sockaddr_in listen_ser_addr_;
  int listen_socket_fd_;
  std::shared_ptr<std::thread> listen_thread_ptr_;

  std::mutex session_socket_mtx_;
  std::vector<int> session_socket_fds_;

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

#endif  // RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_TCP_SENDER_SERVER_H_
