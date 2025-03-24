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

#include "rs_perception/communication/external/common/socket_tcp_receiver_server.h"

#include <cstring>

#include "rs_common/external/common.h"

namespace robosense {
namespace perception {

COMMUNICATION_ERROR_CODE RsSocketTcpReceiverServer::initSocket() {
  
  socket_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  int backlog = 10;

  if (socket_fd_ < 0) {
     RERROR << name() << ": create socket failed => errno = " << errno
           << ", errmsg = " << strerror(errno);
     return COMMUNICATION_ERROR_CODE::SocketCreateError;
  }

  struct sockaddr_in ser_addr;
  memset(&ser_addr, 0, sizeof(ser_addr));
  ser_addr.sin_family = AF_INET;
  ser_addr.sin_addr.s_addr = inet_addr(params_.remote_ip.c_str());
  ser_addr.sin_port = htons(params_.remote_port);
 
  int ret = 0;
  {  // set connect syn count
    int synCnt = 3;
    ret = setsockopt(socket_fd_, IPPROTO_TCP, TCP_SYNCNT, &synCnt,
                     sizeof(synCnt));

    if (ret != 0) {
      RERROR << name() << ": socket SynCnt failed => errno = " << errno
             << ", errMsg = " << strerror(errno);
    }
  }

  {  // set reuseaddr
    int opt = 1;
    ret = setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR,
                     static_cast<const void *>(&opt), sizeof(int));
    if (ret != 0) {
      RERROR << name() << ": socket Reuseaddr failed => errno = " << errno
             << ", errmsg = " << strerror(errno);
      return COMMUNICATION_ERROR_CODE::SocketOptReUseAddrError;
    }
  }

  {
    // set reuseport
    int opt = 1;
    ret = setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEPORT,
                     static_cast<const void *>(&opt), sizeof(int));
    if (ret != 0) {
      RERROR << name() << ": socket Reuseport failed => errno = " << errno
             << ", errmsg = " << strerror(errno);
      return COMMUNICATION_ERROR_CODE::SocketOptReUsePortError;
    }
  }

  {  // set buffer_size
    ret = setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF,
                     static_cast<const void *>(&params_.socket_buffer_size),
                     sizeof(unsigned int));
    if (ret != 0) {
      RERROR << name() << ": socket Recbuf failed => errno = " << errno
             << ", errmsg = " << strerror(errno);
      return COMMUNICATION_ERROR_CODE::SocketOptRcvBufError;
    }
  }

  // {  // set block / non-block
  //   if (params_.timeout_ms != 0) {
  //     timeout_.tv_sec = params_.timeout_ms / 1000;            // ms => s
  //     timeout_.tv_usec = (params_.timeout_ms % 1000) * 1000;  // ms => us
  //     ret = setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO,
  //                      static_cast<const void *>(&timeout_), sizeof(timeout_));
  //     if (ret != 0) {
  //       RERROR << name() << ": socket Sndtimo failed => errno = " << errno
  //              << ", errmsg = " << strerror(errno);
  //       return COMMUNICATION_ERROR_CODE::SocketOptTimeoutError;
  //     }
  //   }
  // }

  {  // bind
    ret = bind(socket_fd_,
               reinterpret_cast<const sockaddr *>(&ser_addr),
               sizeof(ser_addr));

    if (ret != 0) {
      RERROR << name() << ": socket bind failed => errno = " << errno
             << ", errmsg = " << strerror(errno);
      return COMMUNICATION_ERROR_CODE::SocketOptBindError;
    }
  }

  ret = ::listen(socket_fd_, backlog);

  // struct sockaddr_in addr_client;
  // int addr_client_len = sizeof(struct sockaddr_in);
  // int session_fd = ::accept(socket_fd_, reinterpret_cast<sockaddr *>(&addr_client),
  //                reinterpret_cast<socklen_t *>(&addr_client_len));
  
  return COMMUNICATION_ERROR_CODE::Success;
}


COMMUNICATION_ERROR_CODE RsSocketTcpReceiverServer::initThread() {
  run_flag_ = true;
  try {
    if (params_.receive_control.receive_io_parallel_enable &&
        params_.receive_control.receive_io_parallel_degree > 1) {
      for (size_t i = 0; i < params_.receive_control.receive_io_parallel_degree;
           ++i) {
        std::shared_ptr<std::thread> thread_ptr(
            new std::thread(&RsSocketTcpReceiverServer::core, this));
        thread_ptrs_.push_back(thread_ptr);
      }
    } else {
      if ((params_.receive_control.receive_io_parallel_enable == true &&
           params_.receive_control.receive_io_parallel_degree < 2) ||
          (params_.receive_control.receive_io_parallel_enable == false &&
           params_.receive_control.receive_io_parallel_degree > 1)) {
        RWARNING << name()
                 << ": receive io parallel configure not resonable, force no "
                    "receive io parallel !";
      }
      params_.receive_control.receive_io_parallel_enable = false;
      params_.receive_control.receive_io_parallel_degree = 1;
      std::shared_ptr<std::thread> thread_ptr(
          new std::thread(&RsSocketTcpReceiverServer::core, this));
      thread_ptrs_.push_back(thread_ptr);
    }
  } catch (const std::exception &e) {
    RERROR << name() << ": create socket tcp client receive core thread failed!";
    run_flag_ = false;
    return COMMUNICATION_ERROR_CODE::SocketWorkThreadError;
  }

  try {
    if (params_.receive_control.receive_process_parallel_enable) {
      thread2_ptr_.reset(new std::thread(&RsSocketTcpReceiverServer::core2, this));
    } else {
      thread2_ptr_.reset();
    }
  } catch (const std::exception &e) {
    RERROR << name() << ": create socket tcp client receive core2 thread failed!";
    run_flag_ = false;
    return COMMUNICATION_ERROR_CODE::SocketWorkThreadError;
  }

  return COMMUNICATION_ERROR_CODE::Success;
}

void RsSocketTcpReceiverServer::core() {
  std::shared_ptr<char> buffer(new char[params_.max_msg_size],
                               std::default_delete<char[]>());
  socklen_t len;
  int count;

  struct sockaddr_in clent_addr;
  int addr_client_len = sizeof(clent_addr);
  
  accept_socket_fd_ = ::accept(socket_fd_, reinterpret_cast<sockaddr *>(&clent_addr),
                 reinterpret_cast<socklen_t *>(&addr_client_len));

  if (accept_socket_fd_<0) {
    return;
  }

  while (run_flag_) {
    memset(buffer.get(), 0, params_.max_msg_size);
    len = sizeof(clent_addr);
    if (params_.receive_control.receive_io_parallel_enable) {
      std::lock_guard<std::mutex> lg(socket_mtx_);
      // count = recvfrom(accept_socket_fd_, buffer.get(), params_.max_msg_size, 0,
      //                  (struct sockaddr *)&clent_addr, &len);
      count = recv(accept_socket_fd_, buffer.get(), params_.max_msg_size, 0);
    } else {
      // count = recvfrom(accept_socket_fd_, buffer.get(), params_.max_msg_size, 0,
      //                  (struct sockaddr *)&clent_addr, &len);
      count = recv(accept_socket_fd_, buffer.get(), params_.max_msg_size, 0);
    }

    if (count < 0) {
      if (errno == EINTR) {
        RWARNING << name() << ": receive failed: errno = " << errno
                 << ", message = system interrupt!";
        ::usleep(100);
      } else if (errno == EAGAIN) {
        RWARNING << name() << ": receive failed: errno = " << errno
                 << ", message = system try again!";
        ::usleep(100);
      } else if (errno == EBUSY) {
        RWARNING << name() << ": receive failed: errno = " << errno
                 << ", message = device busy!";
        ::usleep(100);
      } else {
        RERROR << name() << ": receive failed: errno = " << errno
               << ", message = error";
        close(socket_fd_);
        socket_fd_ = -1;

        close(accept_socket_fd_);
        accept_socket_fd_ = -1;
        for (auto cb : er_cb_list_) {
          cb(COMMUNICATION_ERROR_CODE::SocketRcvError);
        }
      }
    } else if (count == 0) {
      RWARNING << name() << ": receive nothing !";
      ::usleep(100);
    } else {
      RTRACE << name()
             << ": recevice data succeed => recevice msg size: " << count;
      if (params_.receive_control.receive_process_parallel_enable) {
        std::lock_guard<std::mutex> lg(buffer_mtx_);
        rcv_buffer_.emplace_back(buffer.get(), count);
        buffer_cond_.notify_one();
      } else {
        std::string msg(buffer.get(), count);
        for (size_t i = 0; i < cb_list_.size(); ++i) {
          if (cb_list_[i] != nullptr) {
            cb_list_[i](msg);
          }
        }
      }
    }
  }
}

void RsSocketTcpReceiverServer::core2() {
  while (run_flag_) {
    std::string msg;
    {
      std::unique_lock<std::mutex> lg(buffer_mtx_);
      buffer_cond_.wait(
          lg, [this] { return (rcv_buffer_.size() > 0) || !run_flag_; });

      if (!run_flag_) {
        break;
      }
      msg = *(rcv_buffer_.begin());
      rcv_buffer_.erase(rcv_buffer_.begin());
    }

    for (size_t i = 0; i < cb_list_.size(); ++i) {
      if (cb_list_[i] != nullptr) {
        cb_list_[i](msg);
      }
    }
  }
}

}  // namespace perception
}  // namespace robosense