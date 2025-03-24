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

#ifndef RS_PERCEPTION_COMMUNICATION_EXTERNAL_RECEIVER_H_
#define RS_PERCEPTION_COMMUNICATION_EXTERNAL_RECEIVER_H_

#include "rs_perception/communication/external/base_receiver.h"

namespace robosense {
namespace perception {

class Receiver {
 public:
  using Ptr = std::shared_ptr<Receiver>;

  // load configures from yaml and init communication receive function.
  // input: yaml node
  void init(const RsYamlNode &config_node);

  // entrance of registering callback function
  // input: a callback function
  // output: void.
  void registerRcvComm(const PerceptReceiveCallback &cb);

  // entrance of registering error code callback function
  // input: a callback function
  // output: void.
  void registerErrorComm(const CommunicaterErrorCallback &cb);

 protected:
  std::string name() { return "Receiver"; }
  std::vector<BaseReceiver::Ptr> impl_ptrs_;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_COMMUNICATION_EXTERNAL_RECEIVER_H_
