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

#ifndef RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_TCP_RECEIVER_SERVER_H_
#define RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_TCP_RECEIVER_SERVER_H_

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <list>

#include "rs_common/external/rs_exceptions.h"
#include "rs_perception/communication/external/common/basic_type.h"
#include "rs_perception/communication/external/common/error_code.h"
#include "rs_perception/communication/external/common/socket_params.h"

namespace robosense {
namespace perception {

class RsSocketTcpReceiverServer {
 public:
  using Ptr = std::shared_ptr<RsSocketTcpReceiverServer>;
  using ConstPtr = std::shared_ptr<const RsSocketTcpReceiverServer>;

 public:
  explicit RsSocketTcpReceiverServer(const SocketParams &params) {
    params_ = params;
  }

  ~RsSocketTcpReceiverServer() {
    run_flag_ = false;
    for (auto thread_ptr : thread_ptrs_) {
      if (thread_ptr != nullptr) {
        if (thread_ptr->joinable()) {
          thread_ptr->join();
        }
      }
    }

    if (thread2_ptr_ != nullptr) {
      buffer_cond_.notify_all();
      if (thread2_ptr_->joinable()) {
        thread2_ptr_->join();
      }
    }
  }

  COMMUNICATION_ERROR_CODE initSocket(
      const std::vector<SocketReceiveCallback> cbs,
      const std::vector<CommunicaterErrorCallback> &exceptCbs) {
    COMMUNICATION_ERROR_CODE errCode;
    
    for (auto cb : cbs) {
      if (cb != nullptr) {
        registerRcvComm(cb);
      }
    }

    for (auto cb : exceptCbs) {
      if (cb != nullptr) {
        registerErrorComm(cb);
      }
    }
    
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

  void registerRcvComm(const SocketReceiveCallback &cb) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    cb_list_.emplace_back(cb);
  }

  void registerErrorComm(const CommunicaterErrorCallback &cb) {
    std::lock_guard<std::mutex> lg(mx_er_cb_);
    er_cb_list_.emplace_back(cb);
  }

 private:
  std::string name() { return "RsSocketTcpReceiverServer"; }

  COMMUNICATION_ERROR_CODE initSocket();

  COMMUNICATION_ERROR_CODE initThread();
  void core();
  void core2();

 private:
  std::mutex socket_mtx_;
  int socket_fd_=-1;
  int accept_socket_fd_ = -1;
  SocketParams params_;
  struct timeval timeout_;

  std::mutex buffer_mtx_;
  std::list<std::string> rcv_buffer_;

  std::vector<std::shared_ptr<std::thread>> thread_ptrs_;
  std::unique_ptr<std::thread> thread2_ptr_;

  std::mutex mx_cb_, mx_er_cb_;
  std::vector<CommunicaterErrorCallback> er_cb_list_;
  std::vector<SocketReceiveCallback> cb_list_;
  std::condition_variable buffer_cond_;
 private:
  bool run_flag_ = false;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_TCP_RECEIVER_SERVER_H_
