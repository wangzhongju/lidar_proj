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

#include "rs_perception/communication/external/common/socket_tcp_sender_client.h"

#include <cstring>

#include "rs_common/external/common.h"

namespace robosense {
namespace perception {

COMMUNICATION_ERROR_CODE RsSocketTcpSenderClient::initSocket(
    const std::vector<CommunicaterErrorCallback> &cbs) {
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

COMMUNICATION_ERROR_CODE RsSocketTcpSenderClient::send(
    const std::vector<char> &buffer) {
  if (buffer.size() == 0) {
    return COMMUNICATION_ERROR_CODE::Success;
  }

  if (socket_is_connect_ == false) {
    return COMMUNICATION_ERROR_CODE::SocketClose;
  } else {
    RINFO << "tcp socket client sender data size = " << buffer.size(); 
    std::lock_guard<std::mutex> lg(send_msg_mtx_);
    send_msgs_.push(buffer);
    send_msg_cond_.notify_one();
  }

  return COMMUNICATION_ERROR_CODE::Success;
}

void RsSocketTcpSenderClient::registerErrorComm(
    const CommunicaterErrorCallback &cb) {
  std::lock_guard<std::mutex> lg(mx_er_cb_);
  er_cb_list_.emplace_back(cb);
}

void RsSocketTcpSenderClient::closeSocket() {
  std::lock_guard<std::mutex> lg(socket_mtx_);
  if (socket_fd_ >= 0) {
    shutdown(socket_fd_, SHUT_RDWR);
    close(socket_fd_);
  }
  socket_fd_ = -1;
  socket_is_connect_ = false;
}

COMMUNICATION_ERROR_CODE RsSocketTcpSenderClient::initThread() {
  connect_is_stop_ = true;
  try {
    connect_is_stop_ = false;
    connect_thread_ptr_.reset(
        new std::thread(&RsSocketTcpSenderClient::connect, this));

  } catch (const std::exception &e) {
    RERROR << name() << ", create socket connect work thread failed: errmsg = "
           << e.what();
    return COMMUNICATION_ERROR_CODE::SocketWorkThreadError;
  }

  send_is_stop_ = true;
  try {
    send_is_stop_ = false;
    send_thread_ptr_.reset(
        new std::thread(&RsSocketTcpSenderClient::core, this));
  } catch (const std::exception &e) {
    RERROR << name()
           << ", create socket send work thread failed: errmsg = " << e.what();
    return COMMUNICATION_ERROR_CODE::SocketWorkThreadError;
  }

  return COMMUNICATION_ERROR_CODE::Success;
}

COMMUNICATION_ERROR_CODE RsSocketTcpSenderClient::initSocket() {
  int ret = 0;

  std::lock_guard<std::mutex> lg(socket_mtx_);
  socket_fd_ = -1;
  socket_is_connect_ = false;

  socket_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
//  RINFO << "socket_fd_ = " << socket_fd_;
  if (socket_fd_ == -1) {
    RERROR << name() << ": create tcp client socket failed: errno = " << errno
           << ", errMsg = " << strerror(errno);
    return COMMUNICATION_ERROR_CODE::SocketCreateError;
  }
//  socket_is_connect_ = true;

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
    ret = setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF,
                     static_cast<const void *>(&params_.socket_buffer_size),
                     sizeof(unsigned int));
    if (ret != 0) {
      RERROR << name() << ": socket Recbuf failed => errno = " << errno
             << ", errmsg = " << strerror(errno);
      return COMMUNICATION_ERROR_CODE::SocketOptRcvBufError;
    }
  }
  {  // set block / non-block
    if (params_.timeout_ms != 0) {
      timeout_.tv_sec = params_.timeout_ms / 1000;            // ms => s
      timeout_.tv_usec = (params_.timeout_ms % 1000) * 1000;  // ms => us
      ret = setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO,
                       static_cast<const void *>(&timeout_), sizeof(timeout_));
      if (ret != 0) {
        RERROR << name() << ": socket Sndtimo failed => errno = " << errno
               << ", errmsg = " << strerror(errno);
        return COMMUNICATION_ERROR_CODE::SocketOptTimeoutError;
      }
    }
  }

  // 服务器地址
  memset(&ser_addr_, 0, sizeof(ser_addr_));
  ser_addr_.sin_family = AF_INET;
  ser_addr_.sin_addr.s_addr = inet_addr(params_.remote_ip.c_str());
  ser_addr_.sin_port = htons(params_.remote_port);

  return COMMUNICATION_ERROR_CODE::Success;
}

void RsSocketTcpSenderClient::connect() {
  int retryCnt = 0;
  while (connect_is_stop_ == false) {
    if (socket_is_connect_ == true) {
      ::usleep(1000 * 100);  // 100ms
      continue;
    }

    if (connect_is_stop_ == true) {
      break;
    }

    if (socket_fd_ == -1) {
      COMMUNICATION_ERROR_CODE errCode = initSocket();

      if (errCode != COMMUNICATION_ERROR_CODE::Success) {
        retryCnt++;
        ::usleep(1000 * 100);  // 100ms
      }

      if (retryCnt >= 5) {
        RERROR << name()
               << ": socket tcp client re-try count >= 5, tcp sender failed !";
        break;
      }
    }

    retryCnt = 0;

    int ret = 0;
    ret = ::connect(socket_fd_, reinterpret_cast<const sockaddr *>(&ser_addr_),
                    sizeof(sockaddr));
//    RINFO << "299 ret = " << ret;
    if (ret == 0) {
      std::lock_guard<std::mutex> lg(socket_mtx_);
      socket_is_connect_ = true;
    }
    else {
      closeSocket();
    }
  }
}

void RsSocketTcpSenderClient::core() {
  while (send_is_stop_ == false) {
    std::vector<char> send_msg;
    {
      std::unique_lock<std::mutex> lg(send_msg_mtx_);
      send_msg_cond_.wait(
          lg, [this] { return (!send_msgs_.empty() || send_is_stop_); });
      if (send_is_stop_) {
        break;
      }
      send_msg = send_msgs_.front();
      send_msgs_.pop();
    }

    int offset = 0;
    while (static_cast<size_t>(offset) < send_msg.size()) {
      int ret = ::send(socket_fd_,
                       static_cast<const void *>(send_msg.data() + offset),
                       send_msg.size() - offset, 0);
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
          closeSocket();
          for (auto cb : er_cb_list_) {
            if (cb != nullptr) {
              cb(COMMUNICATION_ERROR_CODE::SocketSndError);
            }
          }
        }
      } else {
        offset += ret;
      }
    }
    RINFO << "robosense socket tcp client send success: ret = " << offset; 
  }
}

}  // namespace perception
}  // namespace robosense