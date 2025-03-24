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

#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_perception/communication/internal/robosense_websocket/websocket_receiver.h"

#ifdef ROBOSENSE_WEBSOCKET_FOUND


namespace robosense {
namespace perception {

int WebsocketReceiver::init(const RsYamlNode &config_node) {
  RERROR << "SDK 3.1 Not Support Robosense Websocket Data Receiver";
  (void)(config_node);
  return -1;
}

void WebsocketReceiver::registerRcvComm(const PerceptReceiveCallback &cb) {
  (void)(cb);
}

void WebsocketReceiver::registerErrorComm(const CommunicaterErrorCallback &cb) {
  (void)(cb);
}

void WebsocketReceiver::localRecvCallback(const std::string &msg) {
  (void)(msg);
}

RS_REGISTER_RECEIVER(WebsocketReceiver);


}  // namespace perception
}  // namespace robosense

#endif  // ROBOSENSE_WEBSOCKET_FOUND