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

#ifndef RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_BASIC_TYPE_H_
#define RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_BASIC_TYPE_H_

#include <memory.h>

#include <functional>
#include <map>
#include <memory>

#include "rs_perception/common/external/msg/rs_perception_msg.h"
#include "rs_perception/communication/external/common/error_code.h"

namespace robosense {
namespace perception {

enum class CommunicationMethod : int {
  NATIVE = 0,
  NATIVE_BYTES,
  ROBOSENSE_V2R,
  ROBOSENSE_ROS,
  ROBOSENSE_ROS2,
};

/**
 * CommunicationMethod and mapping
 */
const std::map<CommunicationMethod, std::string> kCommunicationMethod2NameMap =
    {{CommunicationMethod::NATIVE, "Native"},
     {CommunicationMethod::NATIVE_BYTES, "NativeBytes"},
     {CommunicationMethod::ROBOSENSE_V2R, "V2R"},
     {CommunicationMethod::ROBOSENSE_ROS, "Ros"},
     {CommunicationMethod::ROBOSENSE_ROS2, "Ros2"}};

const std::map<std::string, CommunicationMethod>
    kCommunicationMethodName2TypeMap = {
        {"Native", CommunicationMethod::NATIVE},
        {"NativeBytes", CommunicationMethod::NATIVE_BYTES},
        {"V2R", CommunicationMethod::ROBOSENSE_V2R},
        {"Ros", CommunicationMethod::ROBOSENSE_ROS},
        {"Ros2", CommunicationMethod::ROBOSENSE_ROS2},
};

using CommunicaterErrorCallback =
    std::function<void(const COMMUNICATION_ERROR_CODE &)>;
using PerceptReceiveCallback =
    std::function<void(const RsPerceptionMsg::Ptr &)>;

using SocketReceiveCallback = std::function<void(const std::string &)>;

enum class RS_DATA_ENDIAN_TYPE : int {
  RS_DATA_BIG_ENDIAN = 0,
  RS_DATA_LITTLE_ENDIAN
};

template <typename T>
class RSEndian {
 public:
  typedef typename std::shared_ptr<RSEndian> Ptr;
  typedef typename std::shared_ptr<const RSEndian> ConstPtr;

 public:
  RSEndian() {
    unsigned char *pChar = (unsigned char *)(&magicData);

    if (*pChar == magicBigEndian[0] && (*(pChar + 1)) == magicBigEndian[1] &&
        (*(pChar + 2)) == magicBigEndian[2] &&
        (*(pChar + 3)) == magicBigEndian[3]) {
      m_hostEndianType = RS_DATA_ENDIAN_TYPE::RS_DATA_BIG_ENDIAN;
    } else if (*pChar == magicLittleEndian[0] &&
               (*(pChar + 1)) == magicLittleEndian[1] &&
               (*(pChar + 2)) == magicLittleEndian[2] &&
               (*(pChar + 3)) == magicLittleEndian[3]) {
      m_hostEndianType = RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN;
    }
  }

 public:
  RS_DATA_ENDIAN_TYPE getHostEndian() { return m_hostEndianType; }

  int toTargetEndianArray(T t, void *pArray, unsigned int maxSize,
                          RS_DATA_ENDIAN_TYPE dstEndianType) {
    unsigned int tSize = sizeof(T);

    if (maxSize < tSize || pArray == nullptr) {
      return -1;
    }

    // 保存副本
    T val = t;

    if (m_hostEndianType != dstEndianType) {
      char *pData = reinterpret_cast<char *>(&val);
      unsigned int tSize_h = tSize / 2;

      for (unsigned int idx = 0; idx < tSize_h; ++idx) {
        char tmp = pData[idx];
        unsigned int swapIdx = tSize - idx - 1;
        pData[idx] = pData[swapIdx];
        pData[swapIdx] = tmp;
      }
    }

    memcpy(pArray, &val, tSize);

    return 0;
  }

  int toHostEndianValue(T &t, const void *pArray, unsigned int maxSize,
                        RS_DATA_ENDIAN_TYPE srcEndianType) {
    unsigned int tSize = sizeof(T);
    if (maxSize < tSize || pArray == nullptr || tSize > 128) {
      return -1;
    }

    if (srcEndianType != m_hostEndianType) {
      // 保存副本
      memcpy(byteBuffer, pArray, tSize);

      char *pData = static_cast<char *>(byteBuffer);
      unsigned int tSize_h = tSize / 2;

      for (unsigned int idx = 0; idx < tSize_h; ++idx) {
        char tmp = pData[idx];
        unsigned int swapIdx = tSize - idx - 1;
        pData[idx] = pData[swapIdx];
        pData[swapIdx] = tmp;
      }

      memcpy(&t, byteBuffer, tSize);
    } else {
      memcpy(&t, pArray, tSize);
    }

    return 0;
  }

 private:
  RS_DATA_ENDIAN_TYPE m_hostEndianType;

 private:
  const unsigned int magicData = 0xA1B2C3D4;
  const unsigned char magicBigEndian[4] = {
      (unsigned char)0xA1, (unsigned char)0xB2, (unsigned char)0xC3,
      (unsigned char)0xD4};
  const unsigned char magicLittleEndian[4] = {
      (unsigned char)0xD4, (unsigned char)0xC3, (unsigned char)0xB2,
      (unsigned char)0xA1};
  // 对于内建类型足够大
  char byteBuffer[128];
};

namespace native_sdk_2_x {

/**
 * ROBO_COMM_PROTOCOL_TYPE:
 * 对应消息头的msgVersion字段，其中低位字节表示版本，高位字节表示通信方法(ROS/ROS2/NATIVE/WEBSOCKET...)
 *
 */
enum class ROBO_COMM_PROTOCOL_TYPE : int {
  ROBO_CONFIG_PROTOCOL = 0x0001,  // 不进行版本检查版本，以配置为准版本
  ROBO_NATIVE_PROTOCOL = 0x0101,  //
  ROBO_PROTO_PROTOCOL = 0x0201,   //
};

enum class ROBO_MSG_TYPE : int {
  ROBO_MSG_OBJECT = 0,
  ROBO_MSG_OBJECT_EMPTY,
  ROBO_MSG_ATT_OBJECT,
  ROBO_MSG_ATT_OBJECT_EMPTY,
  ROBO_MSG_FREESPACE,
  ROBO_MSG_FREESPACE_EMPTY,
  ROBO_MSG_LANE,
  ROBO_MSG_LANE_EMPTY,
  ROBO_MSG_CURB,
  ROBO_MSG_CURB_EMPTY,
  ROBO_MSG_NON_GD_IDX,
  ROBO_MSG_NON_GD_IDX_EMPTY,
  ROBO_MSG_GD_IDX,
  ROBO_MSG_GD_IDX_EMPTY,
  ROBO_MSG_BG_IDX,
  ROBO_MSG_BG_IDX_EMPTY,
  ROBO_MSG_POINT,
  ROBO_MSG_POINT_EMPTY,
  ROBO_MSG_POSE,
  ROBO_MSG_POSE_EMPTY,
  ROBO_MSG_ROAD,  // ROBO_MSG_ROAD: ROBO_MSG_LANE + ROBO_MSG_CURB
  ROBO_MSG_ROAD_EMPTY,
};

/**
 * @brief The ROBO_INDICES_TYPE enum
 *
 * ROBO_INDICES_XXXX: indices Type
 *
 */
enum class ROBO_INDICES_TYPE : int {
  ROBO_INDICES_NON_GD = 0,
  ROBO_INDICES_GD,
  ROBO_INDICES_BG,
};

/**
 * @brief The st_RoboMsgHeader struct
 *
 * msgVersion : robosense message version, default is 0x01
 * msgCheck16 : check-16 sum, Not Used Now, default is 0xFFFF
 * msgType    : robosense message type
 * msgLocalLen: single message length
 * msgFrameId : message frame id, the id is from 0 when sdk just run
 * msgLocalCnt: message frame id count
 * msgtotalCnt : message total id
 * msgTimestampMs: In fact is Second
 * msgRes     : Not Used Now
 *
 */

struct alignas(8) st_RoboMsgHeader {
  double msgTimestampMs;
  uint16_t msgVersion;
  uint16_t msgType;
  unsigned int deviceId;
  unsigned int msgFrameId;
  unsigned int msgTotalCnt;
  unsigned int msgLocalCnt;
  uint16_t msgLocalLen;
  uint16_t msgCheck16;
  unsigned char msgRes1[8];

  st_RoboMsgHeader() { reset(); }

  int toTargetEndianArray(unsigned char *data, const int maxLen,
                          RS_DATA_ENDIAN_TYPE targetType =
                              RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
    if ((size_t)(maxLen) < sizeof(st_RoboMsgHeader)) {
      return -1;
    }

    int offset = 0;

    RSEndian<uint16_t> uint16Endian;
    RSEndian<unsigned int> uint32Endian;
    RSEndian<double> doubleEndian;

    doubleEndian.toTargetEndianArray(msgTimestampMs, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(double);

    uint16Endian.toTargetEndianArray(msgVersion, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgType, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    uint32Endian.toTargetEndianArray(deviceId, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(unsigned int);

    uint32Endian.toTargetEndianArray(msgFrameId, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(unsigned int);

    uint32Endian.toTargetEndianArray(msgTotalCnt, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(unsigned int);

    uint32Endian.toTargetEndianArray(msgLocalCnt, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(unsigned int);

    uint16Endian.toTargetEndianArray(msgLocalLen, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgCheck16, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    memset(data + offset, 0, sizeof(msgRes1));

    return offset;
  }

  int toHostEndianValue(const char *data, const int dataLen,
                        RS_DATA_ENDIAN_TYPE srcType =
                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN) {
    if ((size_t)(dataLen) < sizeof(st_RoboMsgHeader)) {
      return -1;
    }

    // Re-initial
    reset();

    int offset = 0;

    RSEndian<uint16_t> uint16Endian;
    RSEndian<unsigned int> uint32Endian;
    RSEndian<double> doubleEndian;

    doubleEndian.toHostEndianValue(msgTimestampMs, data + offset,
                                   dataLen - offset, srcType);
    offset += sizeof(double);

    uint16Endian.toHostEndianValue(msgVersion, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgType, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint32Endian.toHostEndianValue(deviceId, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    uint32Endian.toHostEndianValue(msgFrameId, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    uint32Endian.toHostEndianValue(msgTotalCnt, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    uint32Endian.toHostEndianValue(msgLocalCnt, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    uint16Endian.toHostEndianValue(msgLocalLen, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgCheck16, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    // Now Not Used
    memset(&msgRes1, 0, sizeof(msgRes1));

    return 0;
  }

  void reset() {
    msgVersion =
        static_cast<uint16_t>(ROBO_COMM_PROTOCOL_TYPE::ROBO_CONFIG_PROTOCOL);
    msgType = 0xFFFF;
    deviceId = 0x00000000;
    msgTimestampMs = 0.0;
    msgFrameId = 0x000000000;
    msgTotalCnt = 0x000000000;
    msgLocalCnt = 0x000000000;
    msgLocalLen = 0x0000;
    msgCheck16 = 0xFFFF;
    memset(msgRes1, 0, sizeof(msgRes1));
  }

  void print() {
    std::cout << "msgVersion     = " << (unsigned int)(msgVersion) << std::endl;
    std::cout << "msgType        = " << (unsigned int)(msgType) << std::endl;
    std::cout << "deviceId       = " << deviceId << std::endl;
    std::cout << "msgTimestampMs = " << msgTimestampMs << std::endl;
    std::cout << "msgFrameId     = " << msgFrameId << std::endl;
    std::cout << "msgTotalCnt    = " << msgTotalCnt << std::endl;
    std::cout << "msgLocalCnt    = " << msgLocalCnt << std::endl;
    std::cout << "msgLocalLen    = " << msgLocalLen << std::endl;
    std::cout << "msgCheck16     = " << msgCheck16 << std::endl;
  }
};

/**
 * @brief The RSSerializeBuffer struct
 *
 * types  : represent buffer data type
 * lengths: represent data length
 * buffers: buffer data
 *
 */
class RSSerializeBuffer {
 public:
  std::vector<ROBO_MSG_TYPE> types;
  std::vector<int> offsets;
  std::vector<int> lengths;
  std::vector<char> buffers;
  std::map<ROBO_MSG_TYPE, int> msgCntMap;

 public:
  RSSerializeBuffer() { reset(); }

  void clearInfo() {
    types.clear();
    offsets.clear();
    lengths.clear();
    msgCntMap.clear();
  }

  void reset() {
    clearInfo();
    buffers.resize(32 * 1024 * 1024);  // 32M,防止点数过多造成的内存越界
  }
};

/**
 * @brief The ROBO_MSG_RECV_TYPE enum
 *
 * ROBO_MSG_LOSS        : Means Message Translate But Loss
 * ROBO_MSG_INCOMMPLETE : Means Message Translate Partly
 * ROBO_MSG_SUCCESS     : Means Message Translate And Receive Success
 * ROBO_MSG_EMPTY       : Means Message Is Empty
 * ROBO_MSG_NO_TRANSLATE: Means Message Not Translate
 *
 */
enum class ROBO_MSG_RECV_STATUS : int {
  ROBO_MSG_LOSS = -2,         // ISSUES
  ROBO_MSG_INCOMMPLETE = -1,  // ISSUES
  ROBO_MSG_SUCCESS = 0,       // OK
  ROBO_MSG_EMPTY = 1,         // OK
  ROBO_MSG_NO_TRANSLATE = 2,  // OK
};

/**
 * @brief The st_RoboMsgRecorder struct
 *
 * totalCnt:
 * receivedCnt:
 *
 */
struct st_RoboMsgRecorder {
  int totalCnt;
  int receivedCnt;

  st_RoboMsgRecorder() {
    totalCnt = 0;
    receivedCnt = 0;
  }
};

}  // namespace native_sdk_2_x

namespace native_sdk_3_0 {
/**
 * Robosenen Message Type
 *
 * ROBO_MSG_XXX_EMPTY : Robosense Empty                  Type, Used To Fill When
 * No Any Data Need Transform ROBO_MSG_OBJECT    : Robosense Object Type
 * ROBO_MSG_ATT_OBJECT: Robosense Attention Object       Type
 * ROBO_MSG_FREESPACE : Robosense Freespace              Type
 * ROBO_MSG_LANE      : Robosense Lane                   Type
 * ROBO_MSG_CURB      : Robosense Curb                   Type
 * ROBO_MSG_POINT     : Robosense Point                  Type
 *
 * ROBO_MSG_WEBSOCEKT_METADATA: Robosenes Websocket Metadata Type
 * ROBO_MSG_WEBSOCKET_MESSAGE : Robosense Websocket Message Data Type
 *
 * ROBO_MSG_ROBO_V2R_DATAMESSAGE: Robosense V2R Message Data Type
 * ROBO_MSG_ROBO_V2R_STATUSMESSAGE: Robosense V2R Status Data Type
 *
 */
enum class ROBO_MSG_TYPE : int {
  ROBO_MSG_OBJECT = 0,
  ROBO_MSG_OBJECT_EMPTY,
  ROBO_MSG_ATT_OBJECT,
  ROBO_MSG_ATT_OBJECT_EMPTY,
  ROBO_MSG_FREESPACE,
  ROBO_MSG_FREESPACE_EMPTY,
  ROBO_MSG_LANE,
  ROBO_MSG_LANE_EMPTY,
  ROBO_MSG_CURB,
  ROBO_MSG_CURB_EMPTY,
  ROBO_MSG_POINT,
  ROBO_MSG_POINT_EMPTY,
  ROBO_MSG_AXISSTATUS,
  ROBO_MSG_AXISSTATUS_EMPTY,
  ROBO_MSG_GLOBALCAR_POSE,
  ROBO_MSG_GLOBALCAR_POSE_EMPTY,
  ROBO_MSG_AXISLIDAR_POSE,
  ROBO_MSG_AXISLIDAR_POSE_EMPTY,
  ROBO_MSG_WEBSOCEKT_METADATA,
  ROBO_MSG_WEBSOCKET_MESSAGE,
  ROBO_MSG_ROBO_V2R_DATAMESSAGE,
  ROBO_MSG_ROBO_V2R_STATUSMESSAGE,
  ROBO_MSG_GE_OBJECT,
  ROBO_MSG_GE_LANE,
  ROBO_MSG_GE_EDGE,
  ROBO_MSG_GE_FREESPACE,
  ROBO_MSG_GE_ROADSIGN,
  ROBO_MSG_GE_POINTCLOUD,
  ROBO_MSG_CR_OBSTACLE = 0,
  ROBO_MSG_CR_LANE = 1,
  ROBO_MSG_CR_CURB = 2,
  ROBO_MSG_CR_FREESPACE = 3,
};

/**
 * ROBO_COMM_PROTOCOL_TYPE:
 * 对应消息头的msgVersion字段，其中低位字节表示通信主版本，
 * 高位字节表示通信方法(ROS/ROS2/NATIVE/WEBSOCKET...)
 *
 */
enum class ROBO_COMM_PROTOCOL_TYPE : int {
  ROBO_CONFIG_PROTOCOL = 0x0003,  // 不进行版本检查版本，以配置为准版本
  // Robosense
  ROBO_NATIVE_PROTOCOL = 0x0103,  //
  ROBO_PROTO_PROTOCOL = 0x0203,   //
  ROBO_V2R_PROTOCOL = 0x0303,     //

  // Other
  ROBO_PLC_PROTOCOL = 0x1003,   // PLC 通信协议
  ROBO_DTGH_PROTOCOL = 0x1103,  // DTGH 通信协议
};

/**
 * Robosense Serialize Buffer
 *
 * types  : represent buffer data type
 * lengths: represent data length
 * buffers: buffer data
 *
 */
class RSSerializeBuffer {
 public:
  std::vector<ROBO_MSG_TYPE> types;
  std::vector<int> offsets;
  std::vector<int> lengths;
  std::vector<char> buffers;
  std::map<ROBO_MSG_TYPE, int> msgCntMap;

 public:
  RSSerializeBuffer() { reset(); }

  inline void clearInfo() {
    types.clear();
    offsets.clear();
    lengths.clear();
    msgCntMap.clear();
  }

  inline void reset() {
    clearInfo();
    buffers.resize(32 * 1024 * 1024);  // 32M,防止点数过多造成的内存越界
  }
};

/**
 * @brief The ROBO_MSG_RECV_TYPE enum
 *
 * ROBO_MSG_LOSS        : Means Message Translate But Loss
 * ROBO_MSG_INCOMMPLETE : Means Message Translate Partly
 * ROBO_MSG_SUCCESS     : Means Message Translate And Receive Success
 * ROBO_MSG_EMPTY       : Means Message Is Empty
 * ROBO_MSG_NO_TRANSLATE: Means Message Not Translate
 *
 */
enum class ROBO_MSG_RECV_STATUS : int {
  ROBO_MSG_LOSS = -2,         // ISSUES
  ROBO_MSG_INCOMMPLETE = -1,  // ISSUES
  ROBO_MSG_SUCCESS = 0,       // OK
  ROBO_MSG_EMPTY = 1,         // OK
  ROBO_MSG_NO_TRANSLATE = 2,  // OK
};

enum class RS_INDEX_TYPE : int {
  RS_INDEX_ALG_UNTRACKER = -1,
  RS_INDEX_GD_INDEX = -1000000,
  RS_INDEX_NGD_INDEX = -1000001,
  RS_INDEX_BG_INDEX = -1000002,
  RS_INDEX_VALID_INDEX = -1000003,
};

struct RSCommPoint {
  float x;
  float y;
  float z;
  float intensity;

  int label;
};

/**
 * @brief The st_RoboMsgRecorder struct
 *
 * totalCnt:
 * receivedCnt:
 *
 */
struct st_RoboMsgRecorder {
  int totalCnt;
  int receivedCnt;

  st_RoboMsgRecorder() {
    totalCnt = 0;
    receivedCnt = 0;
  }
};

/**
 * Robosense Message Data Header
 *
 * msgTimestampMs: In fact is Second
 * msgVersion : robosense message version, default is 0x01
 * msgType    : robosense message type
 * msgFrameId : message frame id, the id is from 0 when sdk just run
 * msgtotalCnt: message total id
 * msgLocalCnt: message frame id count
 * msgLocalLen: single message length
 * msgIndex   : single message index, from 0 start for per frame
 * msgTotalFragment: message total fragment count
 * msgFragmentIndex: message fragment Index
 * msgCheck16   : check-16 sum, Not Used Now, default is 0xFFFF
 * msgRes0      : Not Used Now
 *
 *
 */
struct alignas(8) st_RoboMsgHeader {
 public:
  double msgTimestampMs;
  uint16_t msgVersion;
  uint16_t msgType;
  unsigned int deviceId;
  unsigned int msgFrameId;
  unsigned int msgTotalCnt;
  unsigned int msgLocalCnt;
  uint16_t msgLocalLen;
  uint16_t msgIndex;  // 消息索引，用于分割消息的归属标识
  uint16_t msgTotalFragment;  // 如果取值 > 0, 表示一条消息被分割
  uint16_t msgFragmentIndex;  // 被分割的消息的编号索引
  uint16_t msgRes0;
  uint16_t msgCheck16;

 public:
  st_RoboMsgHeader() { reset(); }

  int toTargetEndianArray(unsigned char *data, const int maxLen,
                          RS_DATA_ENDIAN_TYPE targetType) {
    if ((size_t)(maxLen) < sizeof(st_RoboMsgHeader)) {
      return -1;
    }

    int offset = 0;

    RSEndian<uint16_t> uint16Endian;
    RSEndian<unsigned int> uint32Endian;
    RSEndian<double> doubleEndian;

    doubleEndian.toTargetEndianArray(msgTimestampMs, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(double);

    // RINFO_SHORT << "Send msgVersion = " << msgVersion;

    uint16Endian.toTargetEndianArray(msgVersion, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgType, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    uint32Endian.toTargetEndianArray(deviceId, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(unsigned int);

    uint32Endian.toTargetEndianArray(msgFrameId, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(unsigned int);

    uint32Endian.toTargetEndianArray(msgTotalCnt, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(unsigned int);

    uint32Endian.toTargetEndianArray(msgLocalCnt, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(unsigned int);

    uint16Endian.toTargetEndianArray(msgLocalLen, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgIndex, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgTotalFragment, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgFragmentIndex, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgRes0, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgCheck16, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    return offset;
  }

  int toHostEndianValue(const char *data, const int dataLen,
                        RS_DATA_ENDIAN_TYPE srcType) {
    if ((size_t)(dataLen) < sizeof(st_RoboMsgHeader)) {
      return -1;
    }

    // Re-initial
    reset();

    int offset = 0;

    RSEndian<uint16_t> uint16Endian;
    RSEndian<unsigned int> uint32Endian;
    RSEndian<double> doubleEndian;

    doubleEndian.toHostEndianValue(msgTimestampMs, data + offset,
                                   dataLen - offset, srcType);
    offset += sizeof(double);

    uint16Endian.toHostEndianValue(msgVersion, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    // RINFO_SHORT << "Send msgVersion = " << msgVersion;

    uint16Endian.toHostEndianValue(msgType, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint32Endian.toHostEndianValue(deviceId, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    uint32Endian.toHostEndianValue(msgFrameId, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    uint32Endian.toHostEndianValue(msgTotalCnt, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    uint32Endian.toHostEndianValue(msgLocalCnt, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    uint16Endian.toHostEndianValue(msgLocalLen, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgIndex, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgTotalFragment, data + offset,
                                   dataLen - offset, srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgFragmentIndex, data + offset,
                                   dataLen - offset, srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgRes0, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgCheck16, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    return 0;
  }

  void reset() {
    msgTimestampMs = 0.0;
    msgVersion =
        static_cast<uint16_t>(ROBO_COMM_PROTOCOL_TYPE::ROBO_CONFIG_PROTOCOL);
    msgType = 0xFFFF;
    deviceId = 0x00000000;
    msgFrameId = 0x000000000;
    msgTotalCnt = 0x000000000;
    msgLocalCnt = 0x000000000;
    msgLocalLen = 0x0000;
    msgIndex = 0x0000;
    msgTotalFragment = 0x0000;
    msgFragmentIndex = 0x0000;
    msgRes0 = 0x0000;
    msgCheck16 = 0xFFFF;
  }

  void printInfo(std::ostream &ofstr) {
    ofstr << "Robosense Message Header ======> " << std::endl;
    ofstr << "msgTimestampMs   = " << msgTimestampMs << std::endl;
    ofstr << "msgVersion       = " << (unsigned int)(msgVersion) << std::endl;
    ofstr << "msgType          = " << (unsigned int)(msgType) << std::endl;
    ofstr << "deviceId         = " << deviceId << std::endl;
    ofstr << "msgFrameId       = " << msgFrameId << std::endl;
    ofstr << "msgTotalCnt      = " << msgTotalCnt << std::endl;
    ofstr << "msgLocalCnt      = " << msgLocalCnt << std::endl;
    ofstr << "msgLocalLen      = " << msgLocalLen << std::endl;
    ofstr << "msgIndex         = " << msgIndex << std::endl;
    ofstr << "msgTotalFragment = " << msgTotalFragment << std::endl;
    ofstr << "msgFragmentIndex = " << msgFragmentIndex << std::endl;
    ofstr << "msgRes0          = " << msgRes0 << std::endl;
    ofstr << "msgCheck16       = " << msgCheck16 << std::endl;
  }
};

}  // namespace native_sdk_3_0

namespace native_sdk_3_1 {

enum class ROBO_MSG_TYPE : int {
    ROBO_MSG_V2R_DATA_MESSAGE = 0x0FFE,
    ROBO_MSG_V2R_STATUS_MESSAGE = 0x0FFF,

    ROBO_MSG_WEBSOCEKT_METADATA = 0x00FE,
    ROBO_MSG_WEBSOCKET_MESSAGE = 0x00FF,

    ROBO_MSG_CRREAL_NATIVE = 0xFFFE,

    ROBO_MSG_DTGH_OBJECT = 0xFFFF,
    ROBO_MSG_DTGH_EVENT = 0xFFFE,
    ROBO_MSG_DTGH_TRAFFIC = 0xFFFD,
};


enum class RS_INDEX_TYPE : int {
    RS_INDEX_ALG_UNTRACKER = -2,
    RS_INDEX_GD_INDEX = -1000000,
    RS_INDEX_NGD_INDEX = -1000001,
    RS_INDEX_BG_INDEX = -1000002,
    RS_INDEX_VALID_INDEX = -1000003,
    };
enum class ROBO_DATA_TYPE : int {
    ROBO_MSG_OBJECT = 0,
    ROBO_MSG_OBJECT_EMPTY,
    ROBO_MSG_ATT_OBJECT,
    ROBO_MSG_ATT_OBJECT_EMPTY,
    ROBO_MSG_FREESPACE,
    ROBO_MSG_FREESPACE_EMPTY,
    ROBO_MSG_LANE,
    ROBO_MSG_LANE_EMPTY,
    ROBO_MSG_CURB,
    ROBO_MSG_CURB_EMPTY,
    ROBO_MSG_POINT,
    ROBO_MSG_POINT_EMPTY,
    ROBO_MSG_AXISSTATUS,            // status
    ROBO_MSG_AXISSTATUS_EMPTY,
    ROBO_MSG_GLOBALCAR_POSE,        // global_pose_ptr
    ROBO_MSG_GLOBALCAR_POSE_EMPTY,
    ROBO_MSG_AXISLIDAR_POSE,        // status_pose_map
    ROBO_MSG_AXISLIDAR_POSE_EMPTY,
    ROBO_MSG_NON_GD_IDX,
    ROBO_MSG_NON_GD_IDX_EMPTY,
    ROBO_MSG_GD_IDX,
    ROBO_MSG_GD_IDX_EMPTY,
    ROBO_MSG_BG_IDX,
    ROBO_MSG_BG_IDX_EMPTY,
    ROBO_MSG_TIMESTAMP,
    ROBO_MSG_GPS_ORIGIN,
    ROBO_MSG_VALID_INDICES,
    ROBO_MSG_VALID_INDICES_EMPTY,
    };

enum class ROBO_PROTO_DATA_TYPE : int {
    ROBO_PROTO_DATA_OBJECT = 0,
    ROBO_PROTO_DATA_OBJECT_EMPTY,
    ROBO_PROTO_DATA_ATT_OBJECT,
    ROBO_PROTO_DATA_ATT_OBJECT_EMPTY,
    ROBO_PROTO_DATA_FREESPACE,
    ROBO_PROTO_DATA_FREESPACE_EMPTY,
    ROBO_PROTO_DATA_LANE,
    ROBO_PROTO_DATA_LANE_EMPTY,
    ROBO_PROTO_DATA_CURB,
    ROBO_PROTO_DATA_CURB_EMPTY,
    ROBO_PROTO_DATA_POINT,
    ROBO_PROTO_DATA_POINT_EMPTY,
    ROBO_PROTO_DATA_AXISSTATUS,
    ROBO_PROTO_DATA_AXISSTATUS_EMPTY,
    ROBO_PROTO_DATA_GLOBALCAR_POSE,
    ROBO_PROTO_DATA_GLOBALCAR_POSE_EMPTY,
    ROBO_PROTO_DATA_AXISLIDAR_POSE,
    ROBO_PROTO_DATA_AXISLIDAR_POSE_EMPTY,
    ROBO_PROTO_DATA_WEBSOCEKT_METADATA,
    ROBO_PROTO_DATA_WEBSOCKET_MESSAGE,
    ROBO_PROTO_DATA_ROBO_V2R_DATAMESSAGE,
    ROBO_PROTO_DATA_ROBO_V2R_STATUSMESSAGE,
    ROBO_PROTO_DATA_GE_OBJECT,
    ROBO_PROTO_DATA_GE_LANE,
    ROBO_PROTO_DATA_GE_EDGE,
    ROBO_PROTO_DATA_GE_FREESPACE,
    ROBO_PROTO_DATA_GE_ROADSIGN,
    ROBO_PROTO_DATA_GE_POINTCLOUD,
    ROBO_PROTO_DATA_LOC_STATUS,
    ROBO_PROTO_DATA_LOC_CUR_STATE,
    };
/**
 * ROBO_COMM_PROTOCOL_TYPE:
 * 对应消息头的msgVersion字段，其中低位字节表示通信主版本，
 * 高位字节表示通信方法(ROS/ROS2/NATIVE/WEBSOCKET...)
 *
 */
enum class ROBO_COMM_PROTOCOL_TYPE : int {
    ROBO_CONFIG_PROTOCOL = 0x0003,  // 不进行版本检查版本，以配置为准版本
    // Robosense
    ROBO_NATIVE_PROTOCOL = 0x0103,  //
    ROBO_PROTO_PROTOCOL = 0x0203,   //
    ROBO_V2R_PROTOCOL = 0x0303,     //

    // Other
    ROBO_PLC_PROTOCOL = 0x1003,   // PLC 通信协议
    ROBO_DTGH_PROTOCOL = 0x1103,  // DTGH 通信协议
};

class RSSerializeBuffer_ {
public:
    std::vector<ROBO_MSG_TYPE> types;
    std::vector<int> offsets;
    std::vector<int> lengths;
    std::vector<unsigned char> buffers;
    std::map<ROBO_MSG_TYPE, int> msgCntMap;

public:
    RSSerializeBuffer_() { reset(); }

    void clearInfo() {
        types.clear();
        offsets.clear();
        lengths.clear();
        msgCntMap.clear();
    }

    void reset() {
        clearInfo();
        buffers.resize(8 * 1024 * 1024);  // 8M
    }
};

/**
 * @brief The RSSerializeBuffer struct
 *
 * types  : represent buffer data type
 * lengths: represent data length
 * buffers: buffer data
 *
 */
class RSSerializeBuffer {
public:
    std::vector<ROBO_MSG_TYPE> types;
    std::vector<int> offsets;
    std::vector<int> lengths;
    std::vector<char> buffers;
    std::map<ROBO_MSG_TYPE, int> msgCntMap;

public:
    RSSerializeBuffer() { reset(); }

    void clearInfo() {
        types.clear();
        offsets.clear();
        lengths.clear();
        msgCntMap.clear();
    }

    void reset() {
        clearInfo();
        buffers.resize(32 * 1024 * 1024);  // 32M,防止点数过多造成的内存越界
    }
};

class RSBytes3_1SerializeBuffer {
public:
    std::vector<ROBO_DATA_TYPE> types;
    std::vector<int> offsets;
    std::vector<int> lengths;
    std::vector<char> buffers;
    std::map<ROBO_DATA_TYPE, int> msgCntMap;

public:
    RSBytes3_1SerializeBuffer() { reset(); }

    void clearInfo() {
        types.clear();
        offsets.clear();
        lengths.clear();
        msgCntMap.clear();
    }

    void reset() {
        clearInfo();
        buffers.resize(32 * 1024 * 1024);  // 32M,防止点数过多造成的内存越界
    }
};



class RSProtoSerializeBuffer {
public:
    std::vector<ROBO_PROTO_DATA_TYPE> types;
    std::vector<int> offsets;
    std::vector<int> lengths;
    std::vector<char> buffers;
    std::map<ROBO_PROTO_DATA_TYPE, int> msgCntMap;
    std::map<int, int> msgPointCntMap; //<对应offsets/types/lengths的索引号， 点的个数>

public:
    RSProtoSerializeBuffer() { reset(); }

    void clearInfo() {
        types.clear();
        offsets.clear();
        lengths.clear();
        msgCntMap.clear();
    }

    void reset() {
        clearInfo();
        buffers.resize(32 * 1024 * 1024);  // 32M,防止点数过多造成的内存越界
    }
};

/**
 * @brief The ROBO_MSG_RECV_TYPE enum
 *
 * ROBO_MSG_LOSS        : Means Message Translate But Loss
 * ROBO_MSG_INCOMMPLETE : Means Message Translate Partly
 * ROBO_MSG_SUCCESS     : Means Message Translate And Receive Success
 * ROBO_MSG_EMPTY       : Means Message Is Empty
 * ROBO_MSG_NO_TRANSLATE: Means Message Not Translate
 *
 */
enum class ROBO_MSG_DATA_RECV_STATUS : int {
    ROBO_MSG_DATA_LOSS = -2,         // ISSUES
    ROBO_MSG_DATA_INCOMMPLETE = -1,  // ISSUES
    ROBO_MSG_DATA_SUCCESS = 0,       // OK
    ROBO_MSG_DATA_EMPTY = 1,         // OK
    ROBO_MSG_DATA_NO_TRANSLATE = 2,  // OK
};

struct RSCommPoint {
    float x;
    float y;
    float z;
    float intensity;

    int label;
};

/**
 * @brief The st_RoboMsgRecorder struct
 *
 * totalCnt:
 * receivedCnt:
 *
 */
struct st_RoboMsgRecorder {
    int totalCnt;
    int receivedCnt;

    st_RoboMsgRecorder() {
        totalCnt = 0;
        receivedCnt = 0;
    }
};

/**
 * Robosense Message Data Header
 *
 * msgTimestampS:
 * msgVersion : robosense message version, default is 0x7E 0x8E
 * msgType    : robosense message type
 * msgFrameId : message frame id, the id is from 0 when sdk just run
 * msgtotalCnt: message total id
 * msgLocalCnt: message frame id count
 * msgLocalLen: single message length
 * msgIndex   : single message index, from 0 start for per frame
 * msgTotalFragment: message total fragment count
 * msgFragmentIndex: message fragment Index
 * msgCheck16   : check-16 sum, Not Used Now, default is 0xFFFF
 * msgRes0      : Not Used Now
 *
 *
 */
class alignas(8) st_RoboMsgHeader {
 public:
  uint16_t msgVersion;
  uint16_t msgType;
  unsigned int deviceId;
  double msgTimestampS;
  unsigned int msgFrameId;
  unsigned int msgTotalCnt;
  unsigned int msgTotalLen;  // 压缩/非压缩的发送的总的数据长度
  unsigned int msgTotalUncompressLen;  // 压缩时，为原始数据的长度,非压缩时为0
  uint16_t msgLocalCnt;
  uint16_t msgLocalLen;
  uint16_t msgIndex;  // 消息索引，用于分割消息的归属标识
  uint16_t msgTotalFragment;  // 如果取值 > 0, 表示一条消息被分割
  uint16_t msgFragmentIndex;  // 被分割的消息的编号索引
  uint16_t msgCheck16;
  unsigned int msgRes0;

 public:
  st_RoboMsgHeader() { reset(); }

  int toTargetEndianArray(unsigned char *data, const int maxLen,
                          RS_DATA_ENDIAN_TYPE targetType) {
    if ((size_t)(maxLen) < sizeof(st_RoboMsgHeader)) {
      return -1;
    }

    int offset = 0;

    RSEndian<uint16_t> uint16Endian;
    RSEndian<unsigned int> uint32Endian;
    RSEndian<double> doubleEndian;

    uint16Endian.toTargetEndianArray(msgVersion, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgType, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    uint32Endian.toTargetEndianArray(deviceId, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(unsigned int);

    doubleEndian.toTargetEndianArray(msgTimestampS, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(double);

    uint32Endian.toTargetEndianArray(msgFrameId, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(unsigned int);

    uint32Endian.toTargetEndianArray(msgTotalCnt, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(unsigned int);

    uint32Endian.toTargetEndianArray(msgTotalLen, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(unsigned int);

    uint32Endian.toTargetEndianArray(msgTotalUncompressLen, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(unsigned int);

    uint32Endian.toTargetEndianArray(msgLocalCnt, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgLocalLen, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgIndex, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgTotalFragment, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgFragmentIndex, data + offset,
                                     maxLen - offset, targetType);
    offset += sizeof(uint16_t);

    uint16Endian.toTargetEndianArray(msgCheck16, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(uint16_t);

    uint32Endian.toTargetEndianArray(msgRes0, data + offset, maxLen - offset,
                                     targetType);
    offset += sizeof(unsigned int);

    return offset;
  }

  int toHostEndianValue(const char *data, const int dataLen, RS_DATA_ENDIAN_TYPE srcType) {
    if ((size_t)(dataLen) < sizeof(st_RoboMsgHeader)) {
      return -1;
    }

    // Re-initial
    reset();

    int offset = 0;

    RSEndian<uint16_t> uint16Endian;
    RSEndian<unsigned int> uint32Endian;
    RSEndian<double> doubleEndian;

    uint16Endian.toHostEndianValue(msgVersion, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgType, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint32Endian.toHostEndianValue(deviceId, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    doubleEndian.toHostEndianValue(msgTimestampS, data + offset,
                                   dataLen - offset, srcType);
    offset += sizeof(double);

    uint32Endian.toHostEndianValue(msgFrameId, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    uint32Endian.toHostEndianValue(msgTotalCnt, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    uint32Endian.toHostEndianValue(msgTotalLen, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    uint32Endian.toHostEndianValue(msgTotalUncompressLen, data + offset,
                                   dataLen - offset, srcType);
    offset += sizeof(unsigned int);

    uint16Endian.toHostEndianValue(msgLocalCnt, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgLocalLen, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgIndex, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgTotalFragment, data + offset,
                                   dataLen - offset, srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgFragmentIndex, data + offset,
                                   dataLen - offset, srcType);
    offset += sizeof(uint16_t);

    uint16Endian.toHostEndianValue(msgCheck16, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(uint16_t);

    uint32Endian.toHostEndianValue(msgRes0, data + offset, dataLen - offset,
                                   srcType);
    offset += sizeof(unsigned int);

    return 0;
  }

  void reset() {
    msgVersion = 0x7E8E;
    msgType = 0xFFFF;
    deviceId = 0x00000000;
    msgTimestampS = 0.0;
    msgFrameId = 0x00000000;
    msgTotalCnt = 0x00000000;
    msgTotalLen = 0x00000000;
    msgTotalUncompressLen = 0x00000000;
    msgLocalCnt = 0x0000;
    msgLocalLen = 0x0000;
    msgIndex = 0x0000;
    msgTotalFragment = 0x0000;
    msgFragmentIndex = 0x0000;
    msgCheck16 = 0xFFFF;
    msgRes0 = 0x00000000;
  }
};

}  // namespace native_sdk_3_1

// 实现CRC16计算
class SimpleCRC16 {
public:
    unsigned short CRC16_CCITT(unsigned char *puchMsg, unsigned int usDataLen) {
        unsigned short wCRCin = 0x0000;
        unsigned short wCPoly = 0x1021;
        unsigned char wChar = 0;

        while (usDataLen--) {
            wChar = *(puchMsg++);
            InvertUint8(&wChar, &wChar);
            wCRCin ^= (wChar << 8);

            for (int i = 0; i < 8; i++) {
                if (wCRCin & 0x8000) {
                    wCRCin = (wCRCin << 1) ^ wCPoly;
                } else {
                    wCRCin = wCRCin << 1;
                }
            }
        }
        InvertUint16(&wCRCin, &wCRCin);
        return (wCRCin);
    }

    unsigned short CRC16_CCITT_FALSE(unsigned char *puchMsg,
                                     unsigned int usDataLen) {
        unsigned short wCRCin = 0xFFFF;
        unsigned short wCPoly = 0x1021;
        unsigned char wChar = 0;

        while (usDataLen--) {
            wChar = *(puchMsg++);
            wCRCin ^= (wChar << 8);

            for (int i = 0; i < 8; i++) {
                if (wCRCin & 0x8000) {
                    wCRCin = (wCRCin << 1) ^ wCPoly;
                } else {
                    wCRCin = wCRCin << 1;
                }
            }
        }
        return (wCRCin);
    }

    unsigned short CRC16_XMODEM(unsigned char *puchMsg, unsigned int usDataLen) {
        unsigned short wCRCin = 0x0000;
        unsigned short wCPoly = 0x1021;
        unsigned char wChar = 0;

        while (usDataLen--) {
            wChar = *(puchMsg++);
            wCRCin ^= (wChar << 8);

            for (int i = 0; i < 8; i++) {
                if (wCRCin & 0x8000) {
                    wCRCin = (wCRCin << 1) ^ wCPoly;
                } else {
                    wCRCin = wCRCin << 1;
                }
            }
        }
        return (wCRCin);
    }

    unsigned short CRC16_X25(unsigned char *puchMsg, unsigned int usDataLen) {
        unsigned short wCRCin = 0xFFFF;
        unsigned short wCPoly = 0x1021;
        unsigned char wChar = 0;

        while (usDataLen--) {
            wChar = *(puchMsg++);
            InvertUint8(&wChar, &wChar);
            wCRCin ^= (wChar << 8);

            for (int i = 0; i < 8; i++) {
                if (wCRCin & 0x8000) {
                    wCRCin = (wCRCin << 1) ^ wCPoly;
                } else {
                    wCRCin = wCRCin << 1;
                }
            }
        }
        InvertUint16(&wCRCin, &wCRCin);
        return (wCRCin ^ 0xFFFF);
    }

    unsigned short CRC16_MODBUS(unsigned char *puchMsg, unsigned int usDataLen) {
        unsigned short wCRCin = 0xFFFF;
        unsigned short wCPoly = 0x8005;
        unsigned char wChar = 0;

        while (usDataLen--) {
            wChar = *(puchMsg++);
            InvertUint8(&wChar, &wChar);
            wCRCin ^= (wChar << 8);

            for (int i = 0; i < 8; i++) {
                if (wCRCin & 0x8000) {
                    wCRCin = (wCRCin << 1) ^ wCPoly;
                } else {
                    wCRCin = wCRCin << 1;
                }
            }
        }
        InvertUint16(&wCRCin, &wCRCin);
        return (wCRCin);
    }

    unsigned short CRC16_IBM(unsigned char *puchMsg, unsigned int usDataLen) {
        unsigned short wCRCin = 0x0000;
        unsigned short wCPoly = 0x8005;
        unsigned char wChar = 0;

        while (usDataLen--) {
            wChar = *(puchMsg++);
            InvertUint8(&wChar, &wChar);
            wCRCin ^= (wChar << 8);

            for (int i = 0; i < 8; i++) {
                if (wCRCin & 0x8000) {
                    wCRCin = (wCRCin << 1) ^ wCPoly;
                } else {
                    wCRCin = wCRCin << 1;
                }
            }
        }
        InvertUint16(&wCRCin, &wCRCin);
        return (wCRCin);
    }

    unsigned short CRC16_MAXIM(unsigned char *puchMsg, unsigned int usDataLen) {
        unsigned short wCRCin = 0x0000;
        unsigned short wCPoly = 0x8005;
        unsigned char wChar = 0;

        while (usDataLen--) {
            wChar = *(puchMsg++);
            InvertUint8(&wChar, &wChar);
            wCRCin ^= (wChar << 8);

            for (int i = 0; i < 8; i++) {
                if (wCRCin & 0x8000) {
                    wCRCin = (wCRCin << 1) ^ wCPoly;
                } else {
                    wCRCin = wCRCin << 1;
                }
            }
        }
        InvertUint16(&wCRCin, &wCRCin);
        return (wCRCin ^ 0xFFFF);
    }

    unsigned short CRC16_USB(unsigned char *puchMsg, unsigned int usDataLen) {
        unsigned short wCRCin = 0xFFFF;
        unsigned short wCPoly = 0x8005;
        unsigned char wChar = 0;

        while (usDataLen--) {
            wChar = *(puchMsg++);
            InvertUint8(&wChar, &wChar);
            wCRCin ^= (wChar << 8);

            for (int i = 0; i < 8; i++) {
                if (wCRCin & 0x8000) {
                    wCRCin = (wCRCin << 1) ^ wCPoly;
                } else {
                    wCRCin = wCRCin << 1;
                }
            }
        }
        InvertUint16(&wCRCin, &wCRCin);
        return (wCRCin ^ 0xFFFF);
    }

private:
    void InvertUint8(unsigned char *DesBuf, unsigned char *SrcBuf) {
        int i;
        unsigned char temp = 0;

        for (i = 0; i < 8; i++) {
            if (SrcBuf[0] & (1 << i)) {
                temp |= 1 << (7 - i);
            }
        }
        DesBuf[0] = temp;
    }

    void InvertUint16(unsigned short *DesBuf, unsigned short *SrcBuf) {
        int i;
        unsigned short temp = 0;

        for (i = 0; i < 16; i++) {
            if (SrcBuf[0] & (1 << i)) {
                temp |= 1 << (15 - i);
            }
        }
        DesBuf[0] = temp;
    }
};

class RSSimpleCRC16 {
public:
    typedef std::shared_ptr<RSSimpleCRC16> Ptr;
    typedef std::shared_ptr<const RSSimpleCRC16> ConstPtr;

public:
    RSSimpleCRC16(const unsigned short poly, const unsigned short init,
                  const bool refIn, const bool refOut,
                  const unsigned short xorOut)
                  : m_poly(poly),
                  m_init(init),
                  m_refIn(refIn),
                  m_refOut(refOut),
                  m_xorOut(xorOut) {
        // TODO...
    }

    ~RSSimpleCRC16() {
        // TODO...
    }

public:
    int init() {
        try {
            m_pSharedCrc16.reset(new SimpleCRC16());
        } catch (const std::exception &e) {
            return -1;
        }

        return 0;
    }

    unsigned short calcuCheckSum(unsigned char *pData, unsigned int dataLen,
                                 bool isReset = true) {
        (void)isReset;
        unsigned short checkSum = m_pSharedCrc16->CRC16_X25(pData, dataLen);
        return checkSum;
    }

    bool compareCheckSum(unsigned char *pData, unsigned int dataLen,
                         const unsigned short checkSum, bool isReset = true) {
        unsigned short currentCheckSum = calcuCheckSum(pData, dataLen, isReset);

        return (checkSum == currentCheckSum);
    }

private:
    // 计算CheckSum 相关的设置
    unsigned short m_poly;
    unsigned short m_init;
    bool m_refIn;
    bool m_refOut;
    unsigned short m_xorOut;
    std::shared_ptr<SimpleCRC16> m_pSharedCrc16;
};

struct xvizGLTF {
public:
    typedef std::shared_ptr<xvizGLTF> Ptr;
    typedef std::shared_ptr<const xvizGLTF> ConstPtr;

public:
    enum GLTF_OFFSET : int {
        GLTF_GLTF_MAGIC = 0,
        GLTF_MAGIC_VERSION = 1,
        GLTF_FILE_SIZE = 2,
        GLTF_JSON_SIZE = 3,
        GLTF_JSON_MAGIC = 4,

        GLTF_BINARY_SIZE = 0,
        GLTF_BINARY_MAGIC = 1,

        GLTF_GLTF_HEADER_SIZE = 20,
        GLTF_BINARY_HEADER_SIZE = 8,

        GLTF_FIXED_HEADER_SIZE = 28,
        };

public:
    xvizGLTF() {
        gltfHeader.resize(5, 0);
        binaryHeader.resize(2, 0);

        gltfHeader[GLTF_GLTF_MAGIC] = 0x46546c67;
        gltfHeader[GLTF_MAGIC_VERSION] = 0x00000002;
        gltfHeader[GLTF_JSON_MAGIC] = 0x4e4f534a;
        binaryHeader[GLTF_BINARY_MAGIC] = 0x004e4942;
    }

    int getGltfSize() {
        return (jsonData.size() + jsonPadding.size() + binaryData.size() +
        GLTF_FIXED_HEADER_SIZE);
    }

public:
    std::vector<unsigned int> gltfHeader;
    std::vector<char> jsonData;
    std::vector<char> jsonPadding;
    std::vector<unsigned int> binaryHeader;
    std::vector<char> binaryData;

public:
    static const int GLTF_MAGIC = 0;
    static const int JSON_MAGIC = 0;
    static const int BINARY_MAGIC = 0;
};

using RsCharBuffer = std::vector<char>;
using RsCharBufferPtr = std::shared_ptr<RsCharBuffer>;
using RsCharBufferConstPtr = std::shared_ptr<const RsCharBuffer>;

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_COMMUNICATION_EXTERNAL_COMMON_BASIC_TYPE_H_
