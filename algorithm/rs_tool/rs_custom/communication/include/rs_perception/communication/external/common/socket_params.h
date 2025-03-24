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

#ifndef RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_PARAMS_H_
#define RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_PARAMS_H_

#include <arpa/inet.h>
#include <memory.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <unistd.h>

#include <functional>
#include <memory>
#include <sstream>

#include "rs_perception/common/external/rs_config_manager.h"

namespace robosense {
namespace perception {

struct SocketSndControl {
  bool send_control_enable = true;
  unsigned int send_control_thres = 131072;
  unsigned int send_control_ms = 1;
  bool send_control_compress_enable = false;
};

struct SocketRcvControl {
  bool receive_io_parallel_enable = true;
  unsigned int receive_io_parallel_degree = 4;
  bool receive_process_parallel_enable = true;
};

struct SocketParams {
  std::string remote_ip;
  unsigned int remote_port;
  unsigned int socket_buffer_size = 4194304;  // bytes, default is 4MB
  unsigned int max_msg_size = 15360;          // bytes, default is ~15KB
  unsigned int timeout_ms = 0;  // ms, default is 0, synchronization & block,
                                // otherwise synchronization & non-block
  SocketSndControl send_control;
  SocketRcvControl receive_control;

  void log(const std::string &name) {
    std::stringstream ss;
    ss << name << ": SocketParams remote_ip " << remote_ip << std::endl;
    ss << name << ": SocketParams remote_port " << remote_port << std::endl;
    ss << name << ": SocketParams socket_buffer_size " << socket_buffer_size
       << std::endl;
    ss << name << ": SocketParams max_msg_size " << max_msg_size << std::endl;
    ss << name << ": SocketParams timeout_ms " << timeout_ms << std::endl;
    ss << name << ": SocketParams send_control.send_control_enable "
       << send_control.send_control_enable << std::endl;
    ss << name << ": SocketParams send_control.send_control_thres "
       << send_control.send_control_thres << std::endl;
    ss << name << ": SocketParams send_control.send_control_ms "
       << send_control.send_control_ms << std::endl;
    ss << name << ": SocketParams send_control.send_control_compress_enable "
       << send_control.send_control_compress_enable << std::endl;
    ss << name << ": SocketParams receive_control.receive_io_parallel_enable "
       << receive_control.receive_io_parallel_enable << std::endl;
    ss << name << ": SocketParams receive_control.receive_io_parallel_degree "
       << receive_control.receive_io_parallel_degree << std::endl;
    ss << name
       << ": SocketParams receive_control.receive_process_parallel_enable "
       << receive_control.receive_process_parallel_enable << std::endl;
    RsConfigManager().append(ss.str());
  }
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_SOCKET_PARAMS_H_
