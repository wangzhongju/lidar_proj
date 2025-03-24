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

#include "rs_perception/communication/external/receiver.h"

namespace robosense {
namespace perception {

void Receiver::init(const RsYamlNode &config_node) {
  if (config_node.size() == 0) {
    RERROR << name() << ": no any receive configure file, please check again !";
    RS_THROW("no any receive configure file error !");
  } else {
    RINFO << name()
          << ": receiver configure file count = " << config_node.size();
  }

  impl_ptrs_.clear();
  std::string method;
  for (size_t i = 0; i < config_node.size(); ++i) {
    rsYamlRead(config_node[i], "method", method);
    if (method != "Ros" && method != "Websocket" && method != "Ros2") {
        method = "Native";
    }
    std::string strategy = method + "Receiver";
    if (!BaseReceiverRegisterer::IsValid(strategy)) {
      RERROR << name() << ": init strategy " << strategy << " failed!";
      RS_THROW("init register class error!");
    }
    BaseReceiver::Ptr impl_ptr(BaseReceiverRegisterer::getInstanceByName(strategy));
    int ret = impl_ptr->init(config_node[i]/*["config"]*/);
    if (ret != 0) {
      RERROR << name() << ": init " << strategy << " receive failed !";
      RS_THROW("init receiver error !");
    }
    impl_ptrs_.push_back(impl_ptr);
  }
}

void Receiver::registerRcvComm(const PerceptReceiveCallback &cb) {
  if (cb != nullptr) {
    for (auto impl_ptr : impl_ptrs_) {
      if (impl_ptr != nullptr) {
        impl_ptr->registerRcvComm(cb);
      }
    }
  }
}

void Receiver::registerErrorComm(const CommunicaterErrorCallback &cb) {
  if (cb != nullptr) {
    for (auto impl_ptr : impl_ptrs_) {
      if (impl_ptr != nullptr) {
        impl_ptr->registerErrorComm(cb);
      }
    }
  }
}

}  // namespace perception
}  // namespace robosense
