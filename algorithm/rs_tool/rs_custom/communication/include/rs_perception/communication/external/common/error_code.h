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

#ifndef RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_ERROR_CODE_H_
#define RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_ERROR_CODE_H_

namespace robosense {
namespace perception {

enum class COMMUNICATION_ERROR_CODE {
  Success = 0x00,
  // Socket Error Code
  SocketCreateError = 0x01,
  SocketOptBindError = 0x02,
  SocketOptSndBufError = 0x03,
  SocketOptRcvBufError = 0x04,
  SocketOptReUseAddrError = 0x05,
  SocketOptTimeoutError = 0x06,
  SocketSndError = 0x07,
  SocketRcvError = 0x08,
  SocketSndTimeoutError = 0x09,
  SocketRcvTimeoutError = 0x0A,
  SocketDgramOutSizeError = 0x0B,
  SocketOptReUsePortError = 0x0C,
  SocketListenError = 0x0D,

  SocketWorkThreadError = 0xF0,
  SocketClose = 0xFF,
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_ERROR_CODE_H_
