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

#ifndef RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_UDP_SENDER_H_
#define RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_UDP_SENDER_H_

#include <sys/time.h>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "rs_common/external/rs_exceptions.h"
#include "rs_perception/communication/external/common/basic_type.h"
#include "rs_perception/communication/external/common/error_code.h"
#include "rs_perception/communication/external/common/socket_params.h"

namespace robosense {
namespace perception {

class RsSocketUdpSender {
 public:
  using Ptr = std::shared_ptr<RsSocketUdpSender>;

  explicit RsSocketUdpSender(const SocketParams& params) { params_ = params; }

  ~RsSocketUdpSender() {
    if (thread_worker_ptr_ != nullptr) {
      run_flag_ = false;
      send_msg_cond_.notify_all();
      if (thread_worker_ptr_->joinable()) {
        thread_worker_ptr_->join();
      }
    }
  }

  COMMUNICATION_ERROR_CODE initSocket(
      const std::vector<CommunicaterErrorCallback>& cbs) {
    for (auto cb : cbs) {
      if (cb != nullptr) {
        registerErrorComm(cb);
      }
    }

    COMMUNICATION_ERROR_CODE errCode;

    errCode = initSocket();
    if (errCode != COMMUNICATION_ERROR_CODE::Success) {
      return errCode;
    }

    errCode = initThread();
    if (errCode != COMMUNICATION_ERROR_CODE::Success) {
      return errCode;
    }

    return COMMUNICATION_ERROR_CODE::Success;
  }

  COMMUNICATION_ERROR_CODE send(const std::vector<char>& buffer) {
    if (buffer.size() > max_dgram_size_) {
      return COMMUNICATION_ERROR_CODE::SocketDgramOutSizeError;
    }

    if (socket_fd_ < 0) {
      return COMMUNICATION_ERROR_CODE::SocketClose;
    } else {
      std::lock_guard<std::mutex> lg(send_msg_mtx_);
      send_msgs_.push(buffer);
      send_msg_cond_.notify_one();
    }
    return COMMUNICATION_ERROR_CODE::Success;
  }

  void registerErrorComm(const CommunicaterErrorCallback& cb) {
    std::lock_guard<std::mutex> lg(mx_er_cb_);
    er_cb_list_.emplace_back(cb);
  }

 private:
  std::string name() { return "RsSocketUdpSender"; }
  COMMUNICATION_ERROR_CODE initSocket();
  COMMUNICATION_ERROR_CODE initThread();
  void core();

 private:
  int socket_fd_;
  struct sockaddr_in ser_addr_;
  SocketParams params_;
  std::unique_ptr<std::thread> thread_worker_ptr_;
  std::mutex send_msg_mtx_;
  std::condition_variable send_msg_cond_;
  std::queue<std::vector<char>> send_msgs_;
  bool run_flag_;
  std::mutex mx_er_cb_;
  std::vector<CommunicaterErrorCallback> er_cb_list_;
  unsigned int send_msg_size_;

 private:
  const unsigned int max_dgram_size_ = std::numeric_limits<uint16_t>::max() - 1;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_UDP_SENDER_H_
