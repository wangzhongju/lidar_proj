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

#include "rs_perception/communication/external/common/socket_tcp_sender_server.h"

#include <cstring>

#include "rs_common/external/common.h"

namespace robosense {
namespace perception {

COMMUNICATION_ERROR_CODE RsSocketTcpSenderServer::initSocket(
    const std::vector<CommunicaterErrorCallback> &cbs) {
  for (auto cb : cbs) {
    if (cb != nullptr) {
      registerErrorComm(cb);
    }
  }

  COMMUNICATION_ERROR_CODE errCode;

  errCode = initListenSocket();
  if (errCode != COMMUNICATION_ERROR_CODE::Success) {
    return errCode;
  }

  errCode = initThread();

  if (errCode != COMMUNICATION_ERROR_CODE::Success) {
    return errCode;
  }

  return COMMUNICATION_ERROR_CODE::Success;
}

COMMUNICATION_ERROR_CODE RsSocketTcpSenderServer::send(const std::vector<char> &buffer) {
  if (buffer.size() == 0 || session_socket_fds_.size() == 0) {
    return COMMUNICATION_ERROR_CODE::Success;
  }

  if (listen_socket_fd_ == -1) {
    return COMMUNICATION_ERROR_CODE::SocketClose;
  } else {
    std::lock_guard<std::mutex> lg(send_msg_mtx_);
    send_msgs_.push(buffer);
    send_msg_cond_.notify_one();
  }

  return COMMUNICATION_ERROR_CODE::Success;
}

void RsSocketTcpSenderServer::registerErrorComm(
    const CommunicaterErrorCallback &cb) {
  std::lock_guard<std::mutex> lg(mx_er_cb_);
  er_cb_list_.emplace_back(cb);
}

void RsSocketTcpSenderServer::closeSessionSocket(int session_fd) {
  std::lock_guard<std::mutex> lg(session_socket_mtx_);
  if (session_fd >= 0) {
    shutdown(session_fd, SHUT_RDWR);
    close(session_fd);
  }

  for (size_t i = 0; i < session_socket_fds_.size(); ++i) {
    if (session_socket_fds_[i] == session_fd) {
      session_socket_fds_.erase(session_socket_fds_.begin() + i);
      break;
    }
  }
}

void RsSocketTcpSenderServer::closeListenSocket() {
  std::lock_guard<std::mutex> lg(listen_socket_mtx_);
  if (listen_socket_fd_ >= 0) {
    shutdown(listen_socket_fd_, SHUT_RDWR);
    close(listen_socket_fd_);
  }
  listen_socket_fd_ = -1;
}

COMMUNICATION_ERROR_CODE RsSocketTcpSenderServer::initThread() {
  listen_is_stop_ = true;
  try {
    listen_is_stop_ = false;
    listen_thread_ptr_.reset(
        new std::thread(&RsSocketTcpSenderServer::listen, this));

  } catch (const std::exception &e) {
    RERROR << name() << ", create socket listen work thread failed: errmsg = "
           << e.what();
    return COMMUNICATION_ERROR_CODE::SocketWorkThreadError;
  }

  send_is_stop_ = true;
  try {
    send_is_stop_ = false;
    send_thread_ptr_.reset(
        new std::thread(&RsSocketTcpSenderServer::core, this));
  } catch (const std::exception &e) {
    RERROR << name()
           << ", create socket send work thread failed: errmsg = " << e.what();
    return COMMUNICATION_ERROR_CODE::SocketWorkThreadError;
  }

  return COMMUNICATION_ERROR_CODE::Success;
}

COMMUNICATION_ERROR_CODE RsSocketTcpSenderServer::initListenSocket() {
  int ret = 0;

  std::lock_guard<std::mutex> lg(listen_socket_mtx_);
  listen_socket_fd_ = -1;

  listen_socket_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (listen_socket_fd_ == -1) {
    RERROR << name() << ": create tcp client socket failed: errno = " << errno
           << ", errMsg = " << strerror(errno);
    return COMMUNICATION_ERROR_CODE::SocketCreateError;
  }

  {  // set connect syn count
    int synCnt = 3;
    ret = setsockopt(listen_socket_fd_, IPPROTO_TCP, TCP_SYNCNT, &synCnt,
                     sizeof(synCnt));

    if (ret != 0) {
      RERROR << name() << ": socket SynCnt failed => errno = " << errno
             << ", errMsg = " << strerror(errno);
    }
  }

  {  // set reuseaddr
    int opt = 1;
    ret = setsockopt(listen_socket_fd_, SOL_SOCKET, SO_REUSEADDR,
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
    ret = setsockopt(listen_socket_fd_, SOL_SOCKET, SO_REUSEPORT,
                     static_cast<const void *>(&opt), sizeof(int));
    if (ret != 0) {
      RERROR << name() << ": socket Reuseport failed => errno = " << errno
             << ", errmsg = " << strerror(errno);
      return COMMUNICATION_ERROR_CODE::SocketOptReUsePortError;
    }
  }

  {  // bind
    memset(&listen_ser_addr_, 0, sizeof(listen_ser_addr_));
    listen_ser_addr_.sin_family = AF_INET;
    listen_ser_addr_.sin_addr.s_addr = INADDR_ANY;
    listen_ser_addr_.sin_port = htons(params_.remote_port);

    ret = bind(listen_socket_fd_,
               reinterpret_cast<const sockaddr *>(&listen_ser_addr_),
               sizeof(listen_ser_addr_));

    if (ret != 0) {
      RERROR << name() << ": socket bind failed => errno = " << errno
             << ", errmsg = " << strerror(errno);
      return COMMUNICATION_ERROR_CODE::SocketOptBindError;
    }
  }

  {  // listen
    ret = ::listen(listen_socket_fd_, 5);
    if (ret != 0) {
      RERROR << name() << ": listen socket failed => errno = " << errno
             << ", errmsg = " << strerror(errno);
      return COMMUNICATION_ERROR_CODE::SocketListenError;
    }
  }

  return COMMUNICATION_ERROR_CODE::Success;
}

COMMUNICATION_ERROR_CODE RsSocketTcpSenderServer::initSessionSocket(
    int session_fd) {
  int ret = 0;

  {  // set buffer_size
    ret = setsockopt(session_fd, SOL_SOCKET, SO_SNDBUF,
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
      ret = setsockopt(session_fd, SOL_SOCKET, SO_SNDTIMEO,
                       static_cast<const void *>(&timeout_), sizeof(timeout_));
      if (ret != 0) {
        RERROR << name() << ": socket Sndtimo failed => errno = " << errno
               << ", errmsg = " << strerror(errno);
        return COMMUNICATION_ERROR_CODE::SocketOptTimeoutError;
      }
    }else {
      timeout_.tv_sec = 0; 
      timeout_.tv_usec = 100000; // 100ms   
    }
  }

  {
    std::lock_guard<std::mutex> lg(session_socket_mtx_);
    session_socket_fds_.push_back(session_fd);
  }

  return COMMUNICATION_ERROR_CODE::Success;
}

void RsSocketTcpSenderServer::listen() {
  int retryCnt = 0;
  while (listen_is_stop_ == false) {
    if (listen_is_stop_ == true) {
      break;
    }

    if (listen_socket_fd_ == -1) {
      COMMUNICATION_ERROR_CODE errCode = initListenSocket();

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
    struct sockaddr_in addr_client;
    int addr_client_len = sizeof(struct sockaddr_in);
    int session_fd =
        ::accept(listen_socket_fd_, reinterpret_cast<sockaddr *>(&addr_client),
                 reinterpret_cast<socklen_t *>(&addr_client_len));
    if (session_fd < 0) {
      closeListenSocket();
    } else {
      COMMUNICATION_ERROR_CODE errCode = initSessionSocket(session_fd);
      if (errCode != COMMUNICATION_ERROR_CODE::Success) {
        closeSessionSocket(session_fd);
      }
    }
  }
}

void RsSocketTcpSenderServer::core() {
  std::vector<int> session_socket_fds;
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

    fd_set writeset, exceptset;
    int max_fd = 0;
    FD_ZERO(&writeset);
    FD_ZERO(&exceptset);
    {
      std::lock_guard<std::mutex> lg(session_socket_mtx_);
      for (size_t i = 0; i < session_socket_fds_.size(); ++i) {
        if (session_socket_fds_[i] > max_fd) {
          max_fd = session_socket_fds_[i];
          FD_SET(session_socket_fds_[i], &writeset); 
        }
      }
    }
    exceptset = writeset; 
    int selectRet = select(max_fd + 1, NULL, &writeset, &exceptset, &timeout_);
    if (selectRet < 0) {
      RERROR << name() << ", socket tcp server select failed, errno = " << errno
             << ", errmsg = " << strerror(errno);
      continue;
    } else if (selectRet == 0) {
      RWARNING << name()
               << ", socket tcp server select timeout, errno = " << errno
               << ", errmsg = " << strerror(errno);
      continue;
    } else {
      {
        session_socket_fds.clear();
        std::lock_guard<std::mutex> lg(session_socket_mtx_);
        session_socket_fds = session_socket_fds_;
      }

      for (size_t i = 0; i < session_socket_fds.size(); ++i) {
        if (FD_ISSET(session_socket_fds[i], &exceptset)) {
          closeSessionSocket(session_socket_fds[i]);
        }
      }

      {
        session_socket_fds.clear();
        std::lock_guard<std::mutex> lg(session_socket_mtx_);
        session_socket_fds = session_socket_fds_;
      }

      {
        for (size_t i = 0; i < session_socket_fds.size(); ++i) {
          if (FD_ISSET(session_socket_fds[i], &writeset)) {
            int offset = 0;
            while (static_cast<size_t>(offset) < send_msg.size()) {
              int ret =
                  ::send(session_socket_fds[i],
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
                  closeSessionSocket(session_socket_fds[i]);
                  for (auto cb : er_cb_list_) {
                    if (cb != nullptr) {
                      cb(COMMUNICATION_ERROR_CODE::SocketSndError);
                    }
                  }
                  break;
                }
              } else {
                offset += ret;
              }
            }
          }
        }
      }
    }
  }
}

}  // namespace perception
}  // namespace robosense