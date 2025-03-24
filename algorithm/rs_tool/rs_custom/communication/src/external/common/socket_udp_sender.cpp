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

#include "rs_perception/communication/external/common/socket_udp_sender.h"

#include <cstring>

#include "rs_common/external/common.h"

namespace robosense {
namespace perception {

COMMUNICATION_ERROR_CODE RsSocketUdpSender::initSocket() {
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    RERROR << name() << ": create socket failed => errno = " << errno << ", errmsg = " << strerror(errno);;
    return COMMUNICATION_ERROR_CODE::SocketCreateError;
  }

  memset(&ser_addr_, 0, sizeof(ser_addr_));
  ser_addr_.sin_family = AF_INET;
  ser_addr_.sin_addr.s_addr = inet_addr(params_.remote_ip.c_str());
  ser_addr_.sin_port = htons(params_.remote_port);

  int ret = 0;

  {  // set reuse
    int opt = 1;
    ret = setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR,
                     static_cast<const void*>(&opt), sizeof(opt));
    if (ret != 0) {
      RERROR << name()
             << ": set socket_buffer_size failed => errno = " << errno << ", errmsg = " << strerror(errno);;
      return COMMUNICATION_ERROR_CODE::SocketOptReUseAddrError;
    }
  }
  {  // set buffer_size
    ret = setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF,
                     static_cast<const void*>(&(params_.socket_buffer_size)),
                     sizeof(params_.socket_buffer_size));
    if (ret != 0) {
      RERROR << name()
             << ": set socket_buffer_size failed => errno = " << errno << ", errmsg = " << strerror(errno);;
      return COMMUNICATION_ERROR_CODE::SocketOptSndBufError;
    }
  }
  {  // set block & non-block
    if (params_.timeout_ms != 0) {
      struct timeval timeout;
      timeout.tv_sec = params_.timeout_ms / 1000;            // ms => s
      timeout.tv_usec = (params_.timeout_ms % 1000) * 1000;  // ms => us
      ret =
          setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO,
                     reinterpret_cast<const char*>(&timeout), sizeof(timeout));
      if (ret != 0) {
        RERROR << name() << ": socket Sndtimo failed => errno = " << errno << ", errmsg = " << strerror(errno);;
        return COMMUNICATION_ERROR_CODE::SocketOptTimeoutError;
      }
    }
  }
  send_msg_size_ = 0;
  return COMMUNICATION_ERROR_CODE::Success;
}

COMMUNICATION_ERROR_CODE RsSocketUdpSender::initThread() {
  try {
    run_flag_ = true;
    thread_worker_ptr_.reset(new std::thread(&RsSocketUdpSender::core, this));
  } catch (const std::exception& e) {
    RERROR << name() << ": create socket udp sender thread failed!";
    return COMMUNICATION_ERROR_CODE::SocketWorkThreadError;
  }
  return COMMUNICATION_ERROR_CODE::Success;
}

void RsSocketUdpSender::core() {
  while (run_flag_) {
    std::vector<char> send_msg;
    {
      std::unique_lock<std::mutex> lg(send_msg_mtx_);
      send_msg_cond_.wait(
          lg, [this] { return (!send_msgs_.empty() || !run_flag_); });
      if (!run_flag_) {
        break;
      }
      send_msg = send_msgs_.front();
      send_msgs_.pop();
    }

    socklen_t len = sizeof(ser_addr_);
    auto ret = sendto(socket_fd_, static_cast<const void*>(send_msg.data()),
                      send_msg.size(), 0,
                      reinterpret_cast<sockaddr*>(&ser_addr_), len);
    if (ret < 0) {
      if (errno == EINTR) {
        RWARNING << name() << ": send failed: errno = " << errno
                 << ", message = system interrupt!";
        std::lock_guard<std::mutex> lg(send_msg_mtx_);
        send_msgs_.push(send_msg);
        ::usleep(100);
      } else if (errno == EAGAIN) {
        RWARNING << name() << ": send failed: errno = " << errno
                 << ", message = system try again!";
        std::lock_guard<std::mutex> lg(send_msg_mtx_);
        send_msgs_.push(send_msg);
        ::usleep(100);
      } else if (errno == EBUSY) {
        RWARNING << name() << ": send failed: errno = " << errno
                 << ", message = device busy!";
        std::lock_guard<std::mutex> lg(send_msg_mtx_);
        send_msgs_.push(send_msg);
        ::usleep(100);
      } else {
        RERROR << name() << ": send failed: errno = " << errno
               << ", message: error";
        close(socket_fd_);
        socket_fd_ = -1;
        for (auto cb : er_cb_list_) {
          if (cb != nullptr) {
            cb(COMMUNICATION_ERROR_CODE::SocketSndError);
          }
        }
      }
    } else {
      RTRACE << "Send Successed: ret = " << ret;
      if (params_.send_control.send_control_enable) {
        send_msg_size_ += ret;
        if (send_msg_size_ / params_.send_control.send_control_thres > 0) {
          RTRACE << "Trigger Send Control";
          ::usleep(1000 * params_.send_control.send_control_ms);
          send_msg_size_ -= params_.send_control.send_control_thres;
        }
      }
    }
  }
}

}  // namespace perception
}  // namespace robosense
