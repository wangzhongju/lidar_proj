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

#ifndef RS_PERCEPTION_CUSTOM_COMMON_BASE_CUSTOM_PARAMS_H_
#define RS_PERCEPTION_CUSTOM_COMMON_BASE_CUSTOM_PARAMS_H_

#include "rs_common/external/rs_any.h"
#include "rs_perception/common/external/rs_config_manager.h"
#include "rs_common/external/util/yaml_read.h"

namespace robosense {
namespace perception {
const std::map<std::string, std::string> kCustomMsgMap{
    {"Native", "RsNativeCustomMsg"},
    {"NativeBytes", "RsNativeBytesCustomMsg"},
    {"V2R", "RsRobosenseV2RCustomMsg"},
    {"Ros", "RsRosCustomMsg"},
    {"Websocket", "RsWebsocketCustomMsg"}, 
};
// 通信接口参数
class RsCommunicationParams {
 public:
  using Ptr = std::shared_ptr<RsCommunicationParams>;
  using ConstPtr = std::shared_ptr<const RsCommunicationParams>;
  // 使用any map存储不同通信方式参数
  std::map<std::string, Any::Ptr> params_map;
};
class RsBaseCustomMsgParams {
 public:
  using Ptr = std::shared_ptr<RsBaseCustomMsgParams>;
  using ConstPtr = std::shared_ptr<const RsBaseCustomMsgParams>;

 public:
  virtual void log(const std::string& name) = 0;
};

class RsCommonCustomMsgParams : public RsBaseCustomMsgParams {
 public:
  using Ptr = std::shared_ptr<RsCommonCustomMsgParams>;
  using ConstPtr = std::shared_ptr<const RsCommonCustomMsgParams>;

 public:
  int device_id = 0;
  bool send_point_cloud = false;
  bool send_attention_objects = false;
  bool send_freespace = false;
  bool send_lane = false;
  bool send_roadedge = false;
  bool send_sematic = false;
  bool compress_enable = false;
  unsigned int max_msg_size = 1;

  void log(const std::string& name) {
    std::stringstream ss;
    ss << name << "RsCommonCustomMsgParams: device_id " << device_id
       << std::endl;
    ss << name << "RsCommonCustomMsgParams: send_point_cloud "
       << send_point_cloud << std::endl;
    ss << name << "RsCommonCustomMsgParams: send_attention_objects "
       << send_attention_objects << std::endl;
    ss << name << "RsCommonCustomMsgParams: send_freespace " << send_freespace
       << std::endl;
    ss << name << "RsCommonCustomMsgParams: send_lane " << send_lane
       << std::endl;
    ss << name << "RsCommonCustomMsgParams: send_roadedge " << send_roadedge
       << std::endl;
    ss << name << "RsCommonCustomMsgParams: send_sematic " << send_sematic
       << std::endl;
    RsConfigManager().append(ss.str());
  }
};

class RsCommonBytesCustomMsgParams : public RsBaseCustomMsgParams {
public:
    using Ptr = std::shared_ptr<RsCommonBytesCustomMsgParams>;
    using ConstPtr = std::shared_ptr<const RsCommonBytesCustomMsgParams>;

public:
    int device_id = 0;
    bool send_timestamp = true;
    bool send_global_pose = true;
    bool send_gps_origin = true;
    bool send_status_pose_map = true;
    bool send_status = true;
    bool send_valid_indices = false;
    bool send_object = true;

    bool send_object_supplement = false;
    bool send_point_cloud = false;
    bool send_attention_objects = false;
    bool send_freespace = false;
    bool send_lane = false;
    bool send_roadedge = false;
    bool send_sematic = false;
    bool send_non_ground_indices = false;
    bool send_ground_indices = false;
    bool send_background_indices = false;
    bool compress_enable = false;
    unsigned int max_msg_size = 1;

    void log(const std::string& name) {
        std::stringstream ss;
        ss << name << "RsCommonBytesCustomMsgParams: device_id " << device_id << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_timestamp " << send_timestamp << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_global_pose " << send_global_pose << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_gps_origin " << send_gps_origin << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_status_pose_map " << send_status_pose_map << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_status " << send_status << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_valid_indices " << send_valid_indices << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_object " << send_object << std::endl;

        ss << name << "RsCommonBytesCustomMsgParams: send_object_supplement " << send_object_supplement << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_point_cloud " << send_point_cloud << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_attention_objects " << send_attention_objects << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_freespace " << send_freespace << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_lane " << send_lane << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_roadedge " << send_roadedge << std::endl;
        ss << name << "RsCommonBytesCustomMsgParams: send_sematic " << send_sematic << std::endl;
        RsConfigManager().append(ss.str());
    }
};

class RsCustomMsgParams {
 public:
  using Ptr = std::shared_ptr<RsCustomMsgParams>;
  using ConstPtr = std::shared_ptr<const RsCustomMsgParams>;

 public:
  int device_id = 0;
  bool send_point_cloud = false;
  bool send_attention_objects = false;
  bool send_freespace = false;
  bool send_lane = false;
  bool send_roadedge = false;
  bool send_sematic = false;
  std::string custom_method;

  void log(const std::string& name) {
    std::stringstream ss;
    ss << name << "RsCustomMsgParams: device_id " << device_id << std::endl;
    ss << name << "RsCustomMsgParams: send_point_cloud " << send_point_cloud
       << std::endl;
    ss << name << "RsCustomMsgParams: send_attention_objects "
       << send_attention_objects << std::endl;
    ss << name << "RsCustomMsgParams: send_freespace " << send_freespace
       << std::endl;
    ss << name << "RsCustomMsgParams: send_lane " << send_lane << std::endl;
    ss << name << "RsCustomMsgParams: send_roadedge " << send_roadedge
       << std::endl;
    ss << name << "RsCustomMsgParams: send_sematic " << send_sematic
       << std::endl;
    ss << name << "RsCustomMsgParams: custom_method " << custom_method
       << std::endl;
    RsConfigManager().append(ss.str());
  }
};

class RsDtghCustomMsgParams {
public:
    using Ptr = std::shared_ptr<RsDtghCustomMsgParams>;
    using ConstPtr = std::shared_ptr<const RsDtghCustomMsgParams>;

public:
    int device_id = 0;
    bool enable_upload = false;
    bool enable_lidar_object = false;
    bool enable_wgs84_object = false;
    int upload_time_ms = 100;
    int lidar_work_time_ms = 100;
    int keep_alive_time_ms = 5000;
    int device_time_status_ms = 5000;
    bool allow_keep_alive = true;
    bool allow_device_status = true;
    std::string device_no = "dt_device_no";
    std::string mec_no = "dt_mec_no";
    std::string custom_method;

    void log(const std::string &name) {
        std::stringstream ss;
        ss << name << "RsDtghCustomMsgParams: device_id " << device_id << std::endl;
        ss << name << "RsDtghCustomMsgParams: enable_upload " << enable_upload
        << std::endl;
        ss << name << "RsDtghCustomMsgParams: enable_lidar_object "
        << enable_lidar_object << std::endl;
        ss << name << "RsDtghCustomMsgParams: enable_wgs84_object " << enable_wgs84_object
        << std::endl;
        ss << name << "RsDtghCustomMsgParams: upload_time_ms " << upload_time_ms << std::endl;
        ss << name << "RsDtghCustomMsgParams: lidar_work_time_ms " << lidar_work_time_ms << std::endl;
        ss << name << "RsDtghCustomMsgParams: keep_alive_time_ms " << keep_alive_time_ms
        << std::endl;
        ss << name << "RsDtghCustomMsgParams: device_time_status_ms " << device_time_status_ms
        << std::endl;
        ss << name << "RsDtghCustomMsgParams: device_no " << device_no << std::endl;
        ss << name << "RsDtghCustomMsgParams: mec_no " << mec_no << std::endl;
        ss << name << "RsDtghCustomMsgParams: custom_method " << custom_method << std::endl;
        RsConfigManager().append(ss.str());
    }
};

class RsDtgh_1CustomMsgParams {
public:
    using Ptr = std::shared_ptr<RsDtgh_1CustomMsgParams>;
    using ConstPtr = std::shared_ptr<const RsDtgh_1CustomMsgParams>;

public:
    int device_id = 0;
    bool send_object = true;
    bool send_event = true;
    bool send_traffic = true;
    std::string custom_method;

    void log(const std::string &name) {
        std::stringstream ss;
        ss << name << "RsDtgh_1CustomMsgParams: device_id " << device_id << std::endl;
        ss << name << "RsDtgh_1CustomMsgParams: send_object " << send_object << std::endl;
        ss << name << "RsDtgh_1CustomMsgParams: send_event " << send_event << std::endl;
        ss << name << "RsDtgh_1CustomMsgParams: send_traffic " << send_traffic << std::endl;
        ss << name << "RsDtgh_1CustomMsgParams: custom_method " << custom_method << std::endl;
        RsConfigManager().append(ss.str());
    }
};

class RsXingyunCustomMsgParams {
public:
    using Ptr = std::shared_ptr<RsXingyunCustomMsgParams>;
    using ConstPtr = std::shared_ptr<const RsXingyunCustomMsgParams>;

public:
    int device_id = 0;
    bool send_object = true;
    bool send_event = true;
    bool send_traffic = true;
    bool enable_upload = false;
    bool enable_lidar_object = false;
    bool enable_wgs84_object = false;
    int upload_time_ms = 100;
    int lidar_work_time_ms = 100;
    int keep_alive_time_ms = 30000;
    std::string device_no = "dt_device_no";
    std::string mec_no = "dt_mec_no";
    std::string custom_method;

    void log(const std::string &name) {
        std::stringstream ss;
        ss << name << "RsXingyunCustomMsgParams: device_id " << device_id << std::endl;
        ss << name << "RsXingyunCustomMsgParams: enable_upload " << enable_upload
        << std::endl;
        ss << name << "RsXingyunCustomMsgParams: enable_lidar_object "
        << enable_lidar_object << std::endl;
        ss << name << "RsXingyunCustomMsgParams: enable_wgs84_object " << enable_wgs84_object
        << std::endl;
        ss << name << "RsXingyunCustomMsgParams: upload_time_ms " << upload_time_ms << std::endl;
        ss << name << "RsXingyunCustomMsgParams: lidar_work_time_ms " << lidar_work_time_ms << std::endl;
        ss << name << "RsXingyunCustomMsgParams: keep_alive_time_ms " << keep_alive_time_ms
        << std::endl;
        ss << name << "RsXingyunCustomMsgParams: device_no " << device_no << std::endl;
        ss << name << "RsXingyunCustomMsgParams: mec_no " << mec_no << std::endl;
        ss << name << "RsXingyunCustomMsgParams: custom_method " << custom_method << std::endl;
        RsConfigManager().append(ss.str());
    }
};

class RsCustomNativeBytesSDK2_XMsgParams : public RsBaseCustomMsgParams {
 public:
  using Ptr = std::shared_ptr<RsCustomNativeBytesSDK2_XMsgParams>;
  using ConstPtr = std::shared_ptr<const RsCustomNativeBytesSDK2_XMsgParams>;

 public:
//  bool enable = false;
  bool send_objects = true;
  bool send_attention_objects = false;
  bool send_object_supplements = false;
  bool send_freespace = false;
  bool send_lanes = false;
  bool send_curbs = false;
  bool send_non_ground_indices = false;
  bool send_ground_indices = false;
  bool send_background_indices = false;
  bool send_pointcloud = false;
  bool send_pose = false;
  int device_id = 0;
  int max_msg_size = 32768;

  void log(const std::string& name) override {
    std::stringstream ss;

//    ss << name << ": RsCustomNativeBytesSDK2_XMsgParams: enable " << enable
//       << std::endl;
    ss << name << ": RsCustomNativeBytesSDK2_XMsgParams: send_objects "
       << send_objects << std::endl;
    ss << name
       << ": RsCustomNativeBytesSDK2_XMsgParams: send_attention_objects "
       << send_attention_objects << std::endl;
    ss << name
       << ": RsCustomNativeBytesSDK2_XMsgParams: send_object_supplements "
       << send_object_supplements << std::endl;
    ss << name << ": RsCustomNativeBytesSDK2_XMsgParams: send_freespace "
       << send_freespace << std::endl;
    ss << name << ": RsCustomNativeBytesSDK2_XMsgParams: send_lanes "
       << send_lanes << std::endl;
    ss << name << ": RsCustomNativeBytesSDK2_XMsgParams: send_curbs "
       << send_curbs << std::endl;
    ss << name << ": RsCustomNativeBytesSDK2_XMsgParams: send_pointcloud "
       << send_pointcloud << std::endl;
    ss << name << ": RsCustomNativeBytesSDK2_XMsgParams: send_pose "
       << send_pose << std::endl;
    ss << name << ": RsCustomNativeBytesSDK2_XMsgParams: device_id "
       << device_id << std::endl;
    ss << name << ": RsCustomNativeBytesSDK2_XMsgParams: max_msg_size "
       << max_msg_size << std::endl;

    RsConfigManager().append(ss.str());
  }
};

class RsCustomNativeBytesSDK3_0MsgParams : public RsBaseCustomMsgParams {
public:
    using Ptr = std::shared_ptr<RsCustomNativeBytesSDK3_0MsgParams>;
    using ConstPtr = std::shared_ptr<const RsCustomNativeBytesSDK3_0MsgParams>;

public:
    bool enable = false;
    bool send_objects = true;
    bool send_attention_objects = false;
    bool send_object_supplements = false;
    bool send_freespace = false;
    bool send_lanes = false;
    bool send_curbs = false;
    bool send_axis_lidar_pose = false;
    bool send_pointcloud = false;
    bool send_global_car_pose = false;
    // send_axisstatus 不提供配置，由 (enableObject || enableAttObject) 确定
    bool send_axisstatus = false;
    int device_id = 0;
    int max_msg_size = 32768;

    void log(const std::string& name) {
        std::stringstream ss;
        ss << name << ": RsCustomNativeBytesSDK3_0MsgParams: enable " << enable
           << std::endl;
        ss << name << ": RsCustomNativeBytesSDK3_0MsgParams: send_objects "
           << send_objects << std::endl;
        ss << name
           << ": RsCustomNativeBytesSDK3_0MsgParams: send_attention_objects "
           << send_attention_objects << std::endl;
        ss << name
           << ": RsCustomNativeBytesSDK3_0MsgParams: send_object_supplements "
           << send_object_supplements << std::endl;
        ss << name << ": RsCustomNativeBytesSDK3_0MsgParams: send_freespace "
           << send_freespace << std::endl;
        ss << name << ": RsCustomNativeBytesSDK3_0MsgParams: send_lanes "
           << send_lanes << std::endl;
        ss << name << ": RsCustomNativeBytesSDK3_0MsgParams: send_curbs "
           << send_curbs << std::endl;
        ss << name << ": RsCustomNativeBytesSDK3_0MsgParams: send_axis_lidar_pose "
           << send_axis_lidar_pose << std::endl;
        ss << name << ": RsCustomNativeBytesSDK3_0MsgParams: send_pointcloud "
           << send_pointcloud << std::endl;
        ss << name << ": RsCustomNativeBytesSDK3_0MsgParams: send_global_car_pose "
           << send_global_car_pose << std::endl;
        ss << name << ": RsCustomNativeBytesSDK3_0MsgParams: device_id "
           << device_id << std::endl;
        ss << name << ": RsCustomNativeBytesSDK3_0MsgParams: max_msg_size "
           << max_msg_size << std::endl;

        RsConfigManager().append(ss.str());
    }
};

class RsProtoCustomMsgParams : public RsBaseCustomMsgParams {
public:
    using Ptr = std::shared_ptr<RsProtoCustomMsgParams>;
    using ConstPtr = std::shared_ptr<RsProtoCustomMsgParams>;

public:
    bool send_objects = false;
    bool send_attention_objects = false;
    bool send_object_supplement = false;
    bool send_freespace = false;
    bool send_lane = false;
    bool send_roadedge = false;
    bool send_pose = false;
    bool send_point_cloud = false;
    int device_id = 0;
    int max_msg_size = 32768;

    void log(const std::string& name) {
        std::stringstream ss;

        ss << name << ": RsProtoCustomMsgParams: send_objects " << send_objects << std::endl;
        ss << name << ": RsProtoCustomMsgParams: send_attention_objects " << send_attention_objects << std::endl;
        ss << name << ": RsProtoCustomMsgParams: send_object_supplement " << send_object_supplement << std::endl;
        ss << name << ": RsProtoCustomMsgParams: send_freespace " << send_freespace << std::endl;
        ss << name << ": RsProtoCustomMsgParams: send_lane " << send_lane << std::endl;
        ss << name << ": RsProtoCustomMsgParams: send_roadedge " << send_roadedge << std::endl;
        ss << name << ": RsProtoCustomMsgParams: send_pose " << send_pose << std::endl;
        ss << name << ": RsProtoCustomMsgParams: send_point_cloud " << send_point_cloud << std::endl;


        RsConfigManager().append(ss.str());
    }
};

enum class ROBOSENSE_V2R_METHOD : int {
  ROBOSENSE_V2R_UDP_SENDER = 0,
  ROBOSENSE_V2R_TCP_CLIENT_SENDER,
  ROBOSENSE_V2R_TCP_SERVER_SENDER,
};

enum class ROBOSENSE_V2R_VERSION : int {
  ROBOSENSE_V2R_V1_4,
  ROBOSENSE_V2R_V1_5,
  ROBOSENSE_V2R_V1_6,
};

enum class ROBOSENSE_V2R_CENTER_TYPE : int {
  ROBOSENSE_V2R_CENTER,
  ROBOSENSE_V2R_ANCHOR,
};

class RsRobosenseV2RThConfig {
 public:
  float velocityHeadingTh = 0.5;       // m/s
  float accelerationHeadingTh = 0.05;  // m/s^2
  bool accelerationValid = false;
};

class RsCustomRobosenseV2RMsgParams : public RsBaseCustomMsgParams {
 public:
  using Ptr = std::shared_ptr<RsCustomRobosenseV2RMsgParams>;
  using ConstPtr = std::shared_ptr<const RsCustomRobosenseV2RMsgParams>;

 public:
  int device_id = 0;
  std::string sMethod;
  ROBOSENSE_V2R_METHOD method;
  std::string sVersion;
  ROBOSENSE_V2R_VERSION version;
  std::string sCenter;
  ROBOSENSE_V2R_CENTER_TYPE center;
  RsRobosenseV2RThConfig headingConfig;

  void log(const std::string& name) {
    std::stringstream ss;
    ss << name << ": RsCustomRobosenseV2RMsgParams: device_id " << device_id
       << std::endl;
    ss << name << ": RsCustomRobosenseV2RMsgParams: method " << sMethod
       << std::endl;
    ss << name << ": RsCustomRobosenseV2RMsgParams: version " << sVersion
       << std::endl;
    ss << name << ": RsCustomRobosenseV2RMsgParams: heading.velocityHeadingTh "
       << headingConfig.velocityHeadingTh << std::endl;
    ss << name
       << ": RsCustomRobosenseV2RMsgParams: heading.accelerationHeadingTh "
       << headingConfig.accelerationHeadingTh << std::endl;
    ss << name << ": RsCustomRobosenseV2RMsgParams: heading.accelerationValid "
       << headingConfig.accelerationValid << std::endl;

    RsConfigManager().append(ss.str());
  }

  void updateFromString() {
    method = fromStringMethod(sMethod);
    version = fromStringVersion(sVersion);
    center = fromStringCenter(sCenter);
  }

  ROBOSENSE_V2R_METHOD fromStringMethod(const std::string& sMethod) {
    if (sMethod == "UDP_SENDER") {
      return ROBOSENSE_V2R_METHOD::ROBOSENSE_V2R_UDP_SENDER;
    } else if (sMethod == "TCP_CLIENT") {
      return ROBOSENSE_V2R_METHOD::ROBOSENSE_V2R_TCP_CLIENT_SENDER;
    } else if (sMethod == "TCP_SERVER") {
      return ROBOSENSE_V2R_METHOD::ROBOSENSE_V2R_TCP_SERVER_SENDER;
    } else {
      return ROBOSENSE_V2R_METHOD::ROBOSENSE_V2R_UDP_SENDER;
    }
  }

  std::string toStringMethod(const ROBOSENSE_V2R_METHOD method) {
    switch (method) {
      case ROBOSENSE_V2R_METHOD::ROBOSENSE_V2R_UDP_SENDER: {
        return "UDP_SENDER";
      }
      case ROBOSENSE_V2R_METHOD::ROBOSENSE_V2R_TCP_CLIENT_SENDER: {
        return "TCP_CLIENT";
      }
      case ROBOSENSE_V2R_METHOD::ROBOSENSE_V2R_TCP_SERVER_SENDER: {
        return "TCP_SERVER";
      }
      default: {
        return "UDP_SENDER";
      }
    }
  }

  ROBOSENSE_V2R_VERSION fromStringVersion(const std::string& sVersion) {
    if (sVersion == "ROBOSENSE_V2R_V1.4") {
      return ROBOSENSE_V2R_VERSION::ROBOSENSE_V2R_V1_4;
    } else if (sVersion == "ROBOSENSE_V2R_V1.5") {
      return ROBOSENSE_V2R_VERSION::ROBOSENSE_V2R_V1_5;
    } else if (sVersion == "ROBOSENSE_V2R_V1.6") {
      return ROBOSENSE_V2R_VERSION::ROBOSENSE_V2R_V1_6;
    } else {
      return ROBOSENSE_V2R_VERSION::ROBOSENSE_V2R_V1_6;
    }
  }

  std::string toStringVersion(const ROBOSENSE_V2R_VERSION version) {
    switch (version) {
      case ROBOSENSE_V2R_VERSION::ROBOSENSE_V2R_V1_4: {
        return "ROBOSENSE_V2R_V1.4";
      }
      case ROBOSENSE_V2R_VERSION::ROBOSENSE_V2R_V1_5: {
        return "ROBOSENSE_V2R_V1.5";
      }
      case ROBOSENSE_V2R_VERSION::ROBOSENSE_V2R_V1_6: {
        return "ROBOSENSE_V2R_V1.6";
      }
      default: {
        return "ROBOSENSE_V2R_V1.6";
      }
    }
  }

  ROBOSENSE_V2R_CENTER_TYPE fromStringCenter(const std::string& sCenter) {
    if (sCenter == "CENTER") {
      return ROBOSENSE_V2R_CENTER_TYPE::ROBOSENSE_V2R_CENTER;
    } else if (sCenter == "ANCHOR") {
      return ROBOSENSE_V2R_CENTER_TYPE::ROBOSENSE_V2R_ANCHOR;
    } else {
      return ROBOSENSE_V2R_CENTER_TYPE::ROBOSENSE_V2R_CENTER;
    }
  }

  std::string toStringCenter(const ROBOSENSE_V2R_CENTER_TYPE center) {
    switch (center) {
      case ROBOSENSE_V2R_CENTER_TYPE::ROBOSENSE_V2R_CENTER: {
        return "CENTER";
      }
      case ROBOSENSE_V2R_CENTER_TYPE::ROBOSENSE_V2R_ANCHOR: {
        return "ANCHOR";
      }
      default: {
        return "CENTER";
      }
    }
  }
};

class RsFlagConvert {
 public:
  static std::string to_yes_no(bool isBool) { return isBool ? "Yes" : "No"; }
};

class RsXvizCommConfig {
 public:
  unsigned int buffer_size;
  unsigned short int port;
  unsigned short int web_update_frame_gap;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Common Configure ======> " << std::endl;
    ss << "========> buffer_size: " << buffer_size << std::endl;
    ss << "========> port     : " << port << std::endl;
    ss << "========> web_update_frame_gap: " << web_update_frame_gap
       << std::endl;
  }
};

class RsXvizColorConfig {
 public:
  std::vector<unsigned char> colors;

 public:
  RsXvizColorConfig() { reset(); }

  void reset() { colors.resize(4, 0x00); }

  void printInfo(const std::string& prefix, std::stringstream& ss) {
    ss << prefix << "====> colors: [" << (unsigned int)(colors[0]) << ","
       << (unsigned int)(colors[1]) << "," << (unsigned int)(colors[2]) << ","
       << (unsigned int)(colors[3]) << "] <==>" << fromVecToColorString()
       << std::endl;
  }

  // from std::vector<unsigned char> to std::string
  std::string fromVecToColorString() {
    std::string str = "#";
    int colorsCnt = colors.size();
    // for (int i = 0; i < colors.size(); ++i)
    for (int i = 0; i < colorsCnt; ++i) {
      unsigned char color = colors[i];

      // First Char
      unsigned int ch = color / 16;
      if (ch <= 9) {
        str.push_back((char)(ch + '0'));
      } else {
        str.push_back((char)(ch - 10 + 'A'));
      }

      // Second Char
      ch = color % 16;
      if (ch <= 9) {
        str.push_back((char)(ch + '0'));
      } else {
        str.push_back((char)(ch - 10 + 'A'));
      }
    }
    return str;
  }
};

class RsXvizObjectAttachConfig {
 public:
  bool attach_object;
  RsXvizColorConfig color;
  int text_size;

 public:
  RsXvizObjectAttachConfig() { reset(); }

  void reset() {
    attach_object = false;
    color.reset();
    text_size = 12;
  }

  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Object Attach Configure =====> " << std::endl;
    ss << "=====> attach_object " << RsFlagConvert::to_yes_no(attach_object)
       << std::endl;
    color.printInfo("=====> color ", ss);
    ss << "=====> text_size " << text_size << std::endl;
  }
};

class RsXvizLidarGridConfig {
 public:
  std::vector<float> grid_circle_ranges;
  RsXvizColorConfig grid_circle_color;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Lidar Grid Configure ======> " << std::endl;
    int totalGridCircleRangesSize = grid_circle_ranges.size();
    for (int i = 0; i < totalGridCircleRangesSize; ++i) {
      ss << "====> grid_circle_ranges[" << i << "] = " << grid_circle_ranges[i]
         << std::endl;
    }
    grid_circle_color.printInfo("====> grid_circle_color ", ss);
  }
};

class RsXvizObjectEnablesConfig {
 public:
  bool object_enable_cube;
  bool object_enable_box;
  bool object_enable_polygon;
  bool object_enable_polyhedral;
  bool object_enable_velocitydir;
  bool object_enable_trackingpoint;
  bool object_enable_trajectory;
  bool object_enable_boxinfo;
  bool object_enable_labelinfo;
  bool object_enable_trackinfo;
  bool object_enable_gpsinfo;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Object Enable Configure ======> " << std::endl;
    ss << "====> object_enable_cube          : "
       << RsFlagConvert::to_yes_no(object_enable_cube) << std::endl;
    ss << "====> object_enable_box           : "
       << RsFlagConvert::to_yes_no(object_enable_box) << std::endl;
    ss << "====> object_enable_polygon       : "
       << RsFlagConvert::to_yes_no(object_enable_polygon) << std::endl;
    ss << "====> object_enable_polyhedral    : "
       << RsFlagConvert::to_yes_no(object_enable_polyhedral) << std::endl;
    ss << "====> object_enable_velocitydir   : "
       << RsFlagConvert::to_yes_no(object_enable_velocitydir) << std::endl;
    ss << "====> object_enable_trackingpoint : "
       << RsFlagConvert::to_yes_no(object_enable_trackingpoint) << std::endl;
    ss << "====> object_enable_trajectory    : "
       << RsFlagConvert::to_yes_no(object_enable_trajectory) << std::endl;
    ss << "====> object_enable_boxinfo       : "
       << RsFlagConvert::to_yes_no(object_enable_boxinfo) << std::endl;
    ss << "====> object_enable_labelinfo     : "
       << RsFlagConvert::to_yes_no(object_enable_labelinfo) << std::endl;
    ss << "====> object_enable_trackinfo     : "
       << RsFlagConvert::to_yes_no(object_enable_trackinfo) << std::endl;
    ss << "====> object_enable_gpsinfo       : "
       << RsFlagConvert::to_yes_no(object_enable_gpsinfo) << std::endl;
  }
};

class RsXvizAttentionEnablesConfig {
 public:
  bool attention_enable_polygon;
  bool attention_enable_polyhedral;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Attention Object Enable Configure =======> "
       << std::endl;
    ss << "====> attention_enable_polygon    : "
       << RsFlagConvert::to_yes_no(attention_enable_polygon) << std::endl;
    ss << "====> attention_enable_polyhedral : "
       << RsFlagConvert::to_yes_no(attention_enable_polyhedral) << std::endl;
  }
};

class RsXvizLaneEnablesConfig {
 public:
  bool lanes_enable;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Lane Enable Configure ======> " << std::endl;
    ss << "====> lanes_enable : " << RsFlagConvert::to_yes_no(lanes_enable)
       << std::endl;
  }
};

class RsXvizCurbEnablesConfig {
 public:
  bool curbs_enable;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Curbs Enable Configure ======> " << std::endl;
    ss << "====> curbs_enable : " << RsFlagConvert::to_yes_no(curbs_enable)
       << std::endl;
  }
};

class RsXvizFreespaceEnablesConfig {
 public:
  bool freespaces_enable;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Freespace Enable Configure ======> " << std::endl;
    ss << "====> freespaces_enable : "
       << RsFlagConvert::to_yes_no(freespaces_enable) << std::endl;
  }
};

class RsXvizPointCloudEnablesConfig {
 public:
  bool pointcloud_enable;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ PointCloud Enable Configure ======> " << std::endl;
    ss << "====> pointcloud_enable : "
       << RsFlagConvert::to_yes_no(pointcloud_enable) << std::endl;
  }
};

class RsXvizMapEnablesConfig {
 public:
  bool maps_enable;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Maps Enable Configure ======> " << std::endl;
    ss << "====> maps_enable : " << RsFlagConvert::to_yes_no(maps_enable)
       << std::endl;
  }
};

class RsXvizImageEnablesConfig {
 public:
  bool images_enable;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Images Enable Configure ======> " << std::endl;
    ss << "====> images_enable : " << RsFlagConvert::to_yes_no(images_enable)
       << std::endl;
  }
};

class RsXvizRoiEnableConfig {
 public:
  bool rois_enable;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Rois Enable Configure ======> " << std::endl;
    ss << "====> rois_enable : " << RsFlagConvert::to_yes_no(rois_enable)
       << std::endl;
  }
};

class RsXvizEnablesConfig {
 public:
  RsXvizObjectEnablesConfig objectEnables;
  RsXvizAttentionEnablesConfig attentionEnables;
  RsXvizLaneEnablesConfig lanesEnables;
  RsXvizCurbEnablesConfig curbsEnables;
  RsXvizFreespaceEnablesConfig freespacesEnables;
  RsXvizPointCloudEnablesConfig pointCloudEnables;
  RsXvizMapEnablesConfig mapEnables;
  RsXvizImageEnablesConfig imageEnables;
  RsXvizRoiEnableConfig roiEnables;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Enable Configure ======> " << std::endl;
    objectEnables.printInfo("", ss);
    attentionEnables.printInfo("", ss);
    lanesEnables.printInfo("", ss);
    curbsEnables.printInfo("", ss);
    freespacesEnables.printInfo("", ss);
    pointCloudEnables.printInfo("", ss);
    mapEnables.printInfo("", ss);
    imageEnables.printInfo("", ss);
    roiEnables.printInfo("", ss);
  }
};

class RsXvizFillMapsConfig {
 public:
  std::vector<std::string> maps;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Fill Maps Configure ======> " << std::endl;
    for (size_t i = 0; i < maps.size(); ++i) {
      ss << "====> Maps[" << i + 1 << "] = " << maps[i] << std::endl;
    }
  }
};

class RsXvizRoiInfo {
 public:
  int index;
  int roi_type;
  int filter_type;
  std::vector<RsVector3f> anchors;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Roi Configure ======> " << std::endl;
    ss << "====> index       : " << index << std::endl;
    ss << "====> ros_type    : " << roi_type << std::endl;
    ss << "====> filter_type : " << filter_type << std::endl;
    ss << "====> anchors[";
    for (size_t i = 0; i < anchors.size(); ++i) {
      if (i != anchors.size() - 1) {
        ss << "[" << anchors[i].x << ", " << anchors[i].y << ", "
           << anchors[i].z << "]," << std::endl;
      } else {
        ss << "[" << anchors[i].x << ", " << anchors[i].y << ", "
           << anchors[i].z << "]";
      }
    }
    ss << "]" << std::endl;
  }
};

struct st_roiColorMapConfig {
 public:
  int roi_type;
  int filter_type;
  RsXvizColorConfig color;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Roi Color Configure ======> " << std::endl;
    ss << "====> roi_type    : " << roi_type << std::endl;
    ss << "====> filter_type : " << filter_type << std::endl;
    color.printInfo("====> color: ", ss);
  }
};

struct st_roiColorMapperConfig {
 public:
  std::vector<std::vector<st_roiColorMapConfig>> roiMapperColors;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": Roi Color Mapper Configure ======> " << std::endl;
    for (size_t i = 0; i < roiMapperColors.size(); ++i) {
      for (size_t j = 0; j < roiMapperColors[i].size(); ++j) {
        ss << "====> roiMapperColors[" << i + 1 << "][" << j + 1 << "] ====> "
           << std::endl;
        roiMapperColors[i][j].printInfo("====> ", ss);
      }
    }
  }
};

class RsXvizFillRoisConfig {
 public:
  std::vector<std::string> rois;
  std::vector<RsXvizRoiInfo> roiInfos;
  std::vector<RsXvizColorConfig> roiColors;

 public:
  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Fill Rois Configure ======> " << std::endl;
    ss << "====> Rois Configure ======> " << std::endl;
    for (size_t i = 0; i < rois.size(); ++i) {
      ss << "========> rois[" << i + 1 << "] : " << rois[i] << std::endl;
      ss << "========> roiInfos[" << i + 1 << "] ===> " << std::endl;
      roiInfos[i].printInfo("=========>", ss);
      ss << "========> roiColors[" << i + 1 << "] ===> " << std::endl;
      roiColors[i].printInfo("=========>", ss);
    }
  }

  int loadAllRoiInfos(const st_roiColorMapperConfig& roiColorMapperConfig) {
    int ret;

    roiInfos.clear();
    for (size_t i = 0; i < rois.size(); ++i) {
      // Load ROI Data
      std::vector<RsXvizRoiInfo> fileRoiInfos;
      ret = loadRoiInfos(rois[i], fileRoiInfos);
      if (ret != 0) {
        return -1;
      }

      // Match ROI Color
      const std::vector<st_roiColorMapConfig>& roiColorMapConfig =
          roiColorMapperConfig.roiMapperColors[i];
      std::vector<RsXvizColorConfig> roiColorMaps;
      for (size_t j = 0; j < fileRoiInfos.size(); ++j) {
        const RsXvizRoiInfo& roiInfo = fileRoiInfos[j];

        for (size_t k = 0; k < roiColorMapConfig.size(); ++k) {
          const st_roiColorMapConfig& colorMapConfig = roiColorMapConfig[k];

          if (roiInfo.roi_type == colorMapConfig.roi_type &&
              roiInfo.filter_type == colorMapConfig.filter_type) {
            roiColorMaps.push_back(colorMapConfig.color);
            break;
          }
        }
      }
      roiColors.insert(roiColors.end(), roiColorMaps.begin(),
                       roiColorMaps.end());
      roiInfos.insert(roiInfos.end(), fileRoiInfos.begin(), fileRoiInfos.end());
    }
    return 0;
  }

  int loadRoiInfos(const std::string& roiPath,
                   std::vector<RsXvizRoiInfo>& roiInfos) {
    roiInfos.clear();

    RsYamlNode roiInfoNode;
    try {
      // roiInfoNode = YAML::LoadFile(roiPath);
      rsLoadYamlFile(roiPath, roiInfoNode);
    } catch (const std::exception& e) {
      std::cerr << e.what() << '\n';
      return -1;
    }

    RsYamlNode roisNode = roiInfoNode["rois"];
    size_t roisCnt = roisNode.size();
    for (size_t i = 0; i < roisCnt; ++i) {
      RsYamlNode singleRoiNode = roisNode[i];

      RsXvizRoiInfo roiInfo;

      roiInfo.index = singleRoiNode["index"].as<int>();
      roiInfo.roi_type = singleRoiNode["roi_type"].as<int>();
      roiInfo.filter_type = singleRoiNode["filter_type"].as<int>();

      roiInfo.anchors.clear();
      RsYamlNode anchorsNode = singleRoiNode["anchors"];
      for (size_t j = 0; j < anchorsNode.size(); ++j) {
        // Eigen::Vector3f anchor;
        RsVector3f anchor;

        anchor.x = anchorsNode[j][0].as<float>();
        anchor.y = anchorsNode[j][1].as<float>();
        anchor.z = anchorsNode[j][2].as<float>();

        roiInfo.anchors.push_back(anchor);
      }
      roiInfos.push_back(roiInfo);
    }
    return 0;
  }
};

typedef RsXvizObjectAttachConfig RsXvizTrackPointConfig;
typedef RsXvizObjectAttachConfig RsXvizTrajectorConfig;
typedef RsXvizObjectAttachConfig RsXvizlabelTextConfig;
typedef RsXvizObjectAttachConfig RsXvizTrackTextConfig;
typedef RsXvizObjectAttachConfig RsXvizBoxTextConfig;
typedef RsXvizObjectAttachConfig RsXvizGpsTextConfig;
typedef RsXvizObjectAttachConfig RsXvizVelocityDirConfig;
typedef RsXvizObjectAttachConfig RsXvizAccelerateDirConfig;

class RsXvizTransformConfig {
 public:
  std::vector<float> matrix;

  RsXvizTransformConfig() { reset(); }

  void reset() {
    matrix.resize(16, 0);

    matrix[0] = 1.0;
    matrix[1] = 0.0;
    matrix[2] = 0.0;
    matrix[3] = 0.0;

    matrix[4] = 0.0;
    matrix[5] = 1.0;
    matrix[6] = 0.0;
    matrix[7] = 0.0;

    matrix[8] = 0.0;
    matrix[9] = 0.0;
    matrix[10] = 1.0;
    matrix[11] = 0.0;

    matrix[12] = 0.0;
    matrix[13] = 0.0;
    matrix[14] = 0.0;
    matrix[15] = 1.0;
  }

  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Transform Configure ======> " << std::endl;
    for (int i = 0; i < 4; ++i) {
      ss << matrix[i + 0] << ", " << matrix[i + 1] << ", " << matrix[i + 2]
         << ", " << matrix[i + 3] << std::endl;
    }
  }
};

class RsXvizObjectConfig {
 public:
  std::string type;
  int typeID;
  RsXvizColorConfig color;

  RsXvizObjectConfig() { reset(); }

  void reset() {
    type = "-";
    typeID = -1;
    color.reset();
  }

  void printInfo(const std::string& name, std::stringstream& ss) {
    ss << name << ": XVIZ Object Type & Color Configure ======> " << std::endl;
    ss << "====> type   : " << type << std::endl;
    ss << "====> typeID : " << typeID << std::endl;
    color.printInfo("====> color : ", ss);
  }
};

enum class RS_COLOR_MAP_FACTOR_INDEX : int {
  RS_COLOR_MAP_FACTOR_X = 0,
  RS_COLOR_MAP_FACTOR_Y,
  RS_COLOR_MAP_FACTOR_Z,
  RS_COLOR_MAP_FACTOR_INTENSITY,
};

enum class RS_COLOR_MAP_TYPE : int {
  RS_POINTCLOUD_COLOR_MAP_AUTUMN_TYPE = 0,
  RS_POINTCLOUD_COLOR_MAP_BONE_TYPE,
  RS_POINTCLOUD_COLOR_MAP_JET_TYPE,
  RS_POINTCLOUD_COLOR_MAP_WINTER_TYPE,
  RS_POINTCLOUD_COLOR_MAP_RAINBOW_TYPE,
  RS_POINTCLOUD_COLOR_MAP_OCEAN_TYPE,
  RS_POINTCLOUD_COLOR_MAP_SUMMER_TYPE,
  RS_POINTCLOUD_COLOR_MAP_SPRING_TYPE,
  RS_POINTCLOUD_COLOR_MAP_COOL_TYPE,
  RS_POINTCLOUD_COLOR_MAP_HSV_TYPE,
  RS_POINTCLOUD_COLOR_MAP_PINK_TYPE,
  RS_POINTCLOUD_COLOR_MAP_HOT_TYPE,
  RS_POINTCLOUD_COLOR_MAP_PARULA_TYPE,
  RS_POINTCLOUD_COLOR_MAP_RVIZ_TYPE,
  //
  RS_POINTCLOUD_COLOR_MAP_TOTAL,
};

class RsXvizMapOrigin {
 public:
  float lat;
  float lon;
  float alt;

  RsXvizMapOrigin() {
    lat = 0.0f;
    lon = 0.0f;
    alt = 0.0f;
  }
};

// PointCloud Color Mapper Configure
class RsPointCloudColorMapperConfig {
 public:
  bool colorMapperEnable;
  std::vector<unsigned char> defaultColor;
  RS_COLOR_MAP_TYPE colorMapperType;
  std::vector<bool> factorsEnable;
  std::vector<float> factorsLow;
  std::vector<float> factorsUp;
  bool enableLabelPointColor;
  bool enableColorMapInvert;
  std::vector<float> factorsAlphas;
  float pointSize;

  RsPointCloudColorMapperConfig() {
    colorMapperEnable = false;
    defaultColor.resize(4, 0xFF);
    factorsEnable.resize(4, false);
    factorsLow.resize(4);
    factorsUp.resize(4);
    factorsAlphas.resize(4, 0.25);
    enableLabelPointColor = false;
    enableColorMapInvert = false;

    // 默认值
    factorsLow[0] = -125.0;
    factorsLow[1] = -125.0;
    factorsLow[2] = -5.0;
    factorsLow[3] = 0.0;

    factorsUp[0] = 125.0;
    factorsUp[1] = 125.0;
    factorsUp[2] = 5.0;
    factorsUp[3] = 255.0;

    pointSize = 1.0;
  }
};

// Map Color Mapper Configure
typedef RsPointCloudColorMapperConfig RsMapColorMapperConfig;

class RsCustomWebsocketMsgParams : public RsBaseCustomMsgParams {
 public:
  using Ptr = std::shared_ptr<RsCustomWebsocketMsgParams>;
  using ConstPtr = std::shared_ptr<const RsCustomWebsocketMsgParams>;

 public:
  RsXvizCommConfig commConfig;
  std::vector<RsXvizObjectConfig> objectConfigs;
  RsXvizColorConfig attentionObjectConfigs;
  RsXvizColorConfig laneConfigs;
  RsXvizColorConfig curbConfigs;
  RsXvizColorConfig freespaceConfigs;
  RsXvizTrackPointConfig trackPointConfigs;
  RsXvizTrajectorConfig trajectoryConfigs;
  RsXvizlabelTextConfig labelTextConfigs;
  RsXvizTrackTextConfig trackTextConfigs;
  RsXvizVelocityDirConfig velocityDirConfig;
  RsXvizAccelerateDirConfig accelerateDirConfig;
  RsXvizBoxTextConfig boxTextConfigs;
  RsXvizGpsTextConfig gpsTextConfigs;
  RsXvizTransformConfig transformConfig;
  RsXvizLidarGridConfig lidarGridConfig;
  RsPointCloudColorMapperConfig colorMapConfig;
  RsMapColorMapperConfig mapColorMapConfig;
  st_roiColorMapperConfig roiColorMapConfig;
  RsXvizMapOrigin mapOriginConfig;
  RsXvizEnablesConfig enablesConfig;
  RsXvizFillMapsConfig fillMapsConfig;
  RsXvizFillRoisConfig fillRoisConfig;

  void log(const std::string& name) {
    std::stringstream ss;
    ss << name << ": XVIZ Configure =======> " << std::endl;
    commConfig.printInfo("====>", ss);

    int totalObjectCnt = objectConfigs.size();
    for (int i = 0; i < totalObjectCnt; ++i) {
      ss << "Object[" << i << "] ===> " << std::endl;
      objectConfigs[i].printInfo("====>", ss);
    }

    ss << "attention object ===> " << std::endl;
    attentionObjectConfigs.printInfo("====>", ss);

    ss << "Object => trackPoint===> " << std::endl;
    trackPointConfigs.printInfo("====>", ss);

    ss << "Object => trajectory===> " << std::endl;
    trajectoryConfigs.printInfo("====>", ss);

    ss << "Object => labelText ===> " << std::endl;
    labelTextConfigs.printInfo("====>", ss);

    ss << "Object => trackText ===> " << std::endl;
    trackTextConfigs.printInfo("====>", ss);

    ss << "Object => boxText   ===> " << std::endl;
    boxTextConfigs.printInfo("====>", ss);

    ss << "Object => gpsText ==> " << std::endl;
    gpsTextConfigs.printInfo("====>", ss);

    ss << "Object => velocity_dir ==> " << std::endl;
    velocityDirConfig.printInfo("====>", ss);

    ss << "Object => accelerate_dir ==> " << std::endl;
    accelerateDirConfig.printInfo("====>", ss);

    ss << "Lane                ===> " << std::endl;
    laneConfigs.printInfo("====>", ss);

    ss << "Curb                ===> " << std::endl;
    curbConfigs.printInfo("====>", ss);

    ss << "Freespace           ===> " << std::endl;
    freespaceConfigs.printInfo("====>", ss);

    ss << "transform           ===> " << std::endl;
    transformConfig.printInfo("====>", ss);

    ss << "lidar grid circle   ===> " << std::endl;
    lidarGridConfig.printInfo("====>", ss);
  }

  int parserWebsocketConfig(const RsYamlNode& configNode) {
    int ret = 0;
    ret = parseXvizCommonConfig(configNode["common"]);

    if (ret != 0) {
      return -1;
    }

    ret = parseXvizEnablesConfig(configNode["enables"]);
    if (ret != 0) {
      return -2;
    }

    ret = parseXvizObjectsConfig(configNode["objects"]);

    if (ret != 0) {
      return -3;
    }

    ret = parseAttentionObjectConfig(configNode["attention_objects"]);
    if (ret != 0) {
      return -4;
    }

    ret = parseXvizObjectAttanchsConfig(configNode["object_attachs"]);

    if (ret != 0) {
      return -5;
    }

    ret = parseXvizLanesConfig(configNode["lanes"]);

    if (ret != 0) {
      return -6;
    }

    ret = parseXvizCurbsConfig(configNode["curbs"]);
    if (ret != 0) {
      return -7;
    }

    ret = parseXvizFreespacesConfig(configNode["freespaces"]);
    if (ret != 0) {
      return -8;
    }

    ret = parseXvizTransfromConfig(configNode["transform"]);
    if (ret != 0) {
      return -9;
    }

    ret = parseXvizLidarGridsConfig(configNode["lidar_grids"]);
    if (ret != 0) {
      return -10;
    }

    ret = parseXvizColorMapperConfig(configNode["pointcloud_color_map"], colorMapConfig);
    if (ret != 0) {
      return -11;
    }

    ret = parseXvizColorMapperConfig(configNode["map_color_map"],
                                     mapColorMapConfig);
    if (ret != 0) {
      return -12;
    }

    ret = parseXvizColorMapperConfig(configNode["roi_color_map"],
                                     roiColorMapConfig.roiMapperColors);

    if (ret != 0) {
      return -13;
    }

    ret = parseXvizMapOriginConfig(configNode["map_origin"]);

    if (ret != 0) {
      return -14;
    }

    // 获取填充数据的
    ret = parseXvizFillConfig(configNode["fill_maps"], fillMapsConfig.maps);

    if (ret != 0) {
      return -15;
    }

    ret = parseXvizFillConfig(configNode["fill_rois"], fillRoisConfig.rois);

    if (ret != 0) {
      return -16;
    }

    // 进行ROI解析
    ret = fillRoisConfig.loadAllRoiInfos(roiColorMapConfig);
    if (ret != 0) {
      return -17;
    }

    return 0;
  }

  // Websocket Common Node
  int parseXvizCommonConfig(const RsYamlNode& commonNode) {
    commConfig.buffer_size = commonNode["socket_parallel_bufer_size"].as<int>();
    commConfig.port = commonNode["socket_listen_port"].as<unsigned int>();
    commConfig.web_update_frame_gap =
        commonNode["web_update_frame_gap"].as<unsigned int>();

    return 0;
  }

  // websocket Enables Node
  int parseXvizEnablesConfig(const RsYamlNode& enablesNode) {
    RsYamlNode objectEnablesNode = enablesNode["object"];
    RsYamlNode attentionEnablesNode = enablesNode["attention_object"];
    RsYamlNode laneEnablesNode = enablesNode["lane"];
    RsYamlNode curbEnablesNode = enablesNode["curb"];
    RsYamlNode freespaceEnablesNode = enablesNode["freespace"];
    RsYamlNode pointCloudEnablesNode = enablesNode["pointcloud"];
    RsYamlNode mapEnablesNode = enablesNode["map"];
    RsYamlNode imageEnablesNode = enablesNode["image"];
    RsYamlNode roiEnablesNode = enablesNode["roi"];

    // object
    {
      enablesConfig.objectEnables.object_enable_cube =
          objectEnablesNode["cube"].as<bool>();
      enablesConfig.objectEnables.object_enable_box =
          objectEnablesNode["box"].as<bool>();
      enablesConfig.objectEnables.object_enable_polygon =
          objectEnablesNode["polygon"].as<bool>();
      enablesConfig.objectEnables.object_enable_polyhedral =
          objectEnablesNode["polyhedral"].as<bool>();
      enablesConfig.objectEnables.object_enable_trackingpoint =
          objectEnablesNode["tracking_point"].as<bool>();
      enablesConfig.objectEnables.object_enable_trajectory =
          objectEnablesNode["trajectory"].as<bool>();
      enablesConfig.objectEnables.object_enable_velocitydir =
          objectEnablesNode["velocity_dir"].as<bool>();
      enablesConfig.objectEnables.object_enable_boxinfo =
          objectEnablesNode["box_info"].as<bool>();
      enablesConfig.objectEnables.object_enable_labelinfo =
          objectEnablesNode["label_info"].as<bool>();
      enablesConfig.objectEnables.object_enable_trackinfo =
          objectEnablesNode["track_info"].as<bool>();
      enablesConfig.objectEnables.object_enable_gpsinfo =
          objectEnablesNode["gps_info"].as<bool>();
    }

    // attention object
    {
      enablesConfig.attentionEnables.attention_enable_polygon =
          attentionEnablesNode["polygon"].as<bool>();
      enablesConfig.attentionEnables.attention_enable_polyhedral =
          attentionEnablesNode["polyhedral"].as<bool>();
    }

    // lane
    {
      enablesConfig.lanesEnables.lanes_enable =
          laneEnablesNode["lanes"].as<bool>();
    }

    // freespace
    {
      enablesConfig.freespacesEnables.freespaces_enable =
          freespaceEnablesNode["freespaces"].as<bool>();
    }

    // curbs
    {
      enablesConfig.curbsEnables.curbs_enable =
          curbEnablesNode["curbs"].as<bool>();
    }

    // pointcloud
    {
      enablesConfig.pointCloudEnables.pointcloud_enable =
          pointCloudEnablesNode["pointclouds"].as<bool>();
    }

    // map
    {
      enablesConfig.mapEnables.maps_enable = mapEnablesNode["maps"].as<bool>();
    }

    // roi
    {
      enablesConfig.roiEnables.rois_enable = roiEnablesNode["rois"].as<bool>();
    }

    return 0;
  }

  // websocket objects Node
  int parseXvizObjectsConfig(const RsYamlNode& objectsNode) {
    objectConfigs.clear();
    for (size_t i = 0; i < objectsNode.size(); ++i) {
      const RsYamlNode& objectNode = objectsNode[i];

      RsXvizObjectConfig objectConfig;
      objectConfig.type = objectNode["type"].as<std::string>();
      objectConfig.typeID = objectNode["typeid"].as<int>();
      RsYamlNode colorNode = objectNode["color"];
      parseArrayNode(colorNode, objectConfig.color);

      objectConfigs.push_back(objectConfig);
    }
    return 0;
  }

  // websocket attention_objects Node
  int parseAttentionObjectConfig(const RsYamlNode& attentionNode) {
    RsYamlNode attentionObjectsColorNode = attentionNode["color"];
    parseArrayNode(attentionObjectsColorNode, attentionObjectConfigs);

    return 0;
  }

  // websocket object_attachs Node
  int parseXvizObjectAttanchsConfig(const RsYamlNode& objectAttsNode) {
    // track_points
    {
      const RsYamlNode& configNode = objectAttsNode["track_points"];
      bool isAttachObject = configNode["attach_object"].as<bool>();
      RsXvizColorConfig attachObjectColor;
      parseArrayNode(configNode["color"], attachObjectColor);

      trackPointConfigs.attach_object = isAttachObject;
      trackPointConfigs.color = attachObjectColor;
    }
    // trajectory
    {
      const RsYamlNode& configNode = objectAttsNode["trajectory"];
      bool isAttachObject = configNode["attach_object"].as<bool>();
      RsXvizColorConfig attachObjectColor;
      parseArrayNode(configNode["color"], attachObjectColor);

      trajectoryConfigs.attach_object = isAttachObject;
      trajectoryConfigs.color = attachObjectColor;
    }
    // label_text
    {
      const RsYamlNode& configNode = objectAttsNode["label_text"];
      bool isAttachObject = configNode["attach_object"].as<bool>();
      RsXvizColorConfig attachObjectColor;
      parseArrayNode(configNode["color"], attachObjectColor);
      int text_size = configNode["text_size"].as<int>();

      labelTextConfigs.attach_object = isAttachObject;
      labelTextConfigs.color = attachObjectColor;
      labelTextConfigs.text_size = text_size;
    }
    // track_text
    {
      const RsYamlNode& configNode = objectAttsNode["track_text"];
      bool isAttachObject = configNode["attach_object"].as<bool>();
      RsXvizColorConfig attachObjectColor;
      parseArrayNode(configNode["color"], attachObjectColor);
      int text_size = configNode["text_size"].as<int>();

      trackTextConfigs.attach_object = isAttachObject;
      trackTextConfigs.color = attachObjectColor;
      trackTextConfigs.text_size = text_size;
    }
    // box_text
    {
      const RsYamlNode& configNode = objectAttsNode["box_text"];
      bool isAttachObject = configNode["attach_object"].as<bool>();
      RsXvizColorConfig attachObjectColor;
      parseArrayNode(configNode["color"], attachObjectColor);
      int text_size = configNode["text_size"].as<int>();

      boxTextConfigs.attach_object = isAttachObject;
      boxTextConfigs.color = attachObjectColor;
      boxTextConfigs.text_size = text_size;
    }
    // gps_text
    {
      const RsYamlNode& configNode = objectAttsNode["gps_text"];
      bool isAttachObject = configNode["attach_object"].as<bool>();
      RsXvizColorConfig attachObjectColor;
      parseArrayNode(configNode["color"], attachObjectColor);
      int text_size = configNode["text_size"].as<int>();

      gpsTextConfigs.attach_object = isAttachObject;
      gpsTextConfigs.color = attachObjectColor;
      gpsTextConfigs.text_size = text_size;
    }
    // velocity_dir
    {
      const RsYamlNode& configNode = objectAttsNode["velocity_dir"];
      bool isAttachObject = configNode["attach_object"].as<bool>();
      RsXvizColorConfig attachObjectColor;
      parseArrayNode(configNode["color"], attachObjectColor);

      velocityDirConfig.attach_object = isAttachObject;
      velocityDirConfig.color = attachObjectColor;
    }
    // acc_dir
    {
      const RsYamlNode& configNode = objectAttsNode["acc_dir"];
      bool isAttachObject = configNode["attach_object"].as<bool>();
      RsXvizColorConfig attachObjectColor;
      parseArrayNode(configNode["color"], attachObjectColor);

      accelerateDirConfig.attach_object = isAttachObject;
      accelerateDirConfig.color = attachObjectColor;
    }

    return 0;
  }

  // websocket lanes Node
  int parseXvizLanesConfig(const RsYamlNode& lanesNode) {
    RsYamlNode laneColorNode = lanesNode["color"];
    parseArrayNode(laneColorNode, laneConfigs);

    return 0;
  }

  // websocket curbs Node
  int parseXvizCurbsConfig(const RsYamlNode& curbsNode) {
    RsYamlNode curbColorNode = curbsNode["color"];
    parseArrayNode(curbColorNode, curbConfigs);

    return 0;
  }

  // websocket freespaces Node
  int parseXvizFreespacesConfig(const RsYamlNode& freespacesNode) {
    RsYamlNode freespaceColorNode = freespacesNode["color"];
    parseArrayNode(freespaceColorNode, freespaceConfigs);
    return 0;
  }

  // websocket transform Node
  int parseXvizTransfromConfig(const RsYamlNode& transformNode) {
    RsYamlNode matrixNode = transformNode["matrix"];
    parseArrayNode(matrixNode, transformConfig);

    return 0;
  }

  // websocket lidar_grids Node
  int parseXvizLidarGridsConfig(const RsYamlNode& lidarGridNode) {
    parseArrayNode(lidarGridNode["grid_circle_ranges"], "grid_circle_ranges",
                   lidarGridConfig);
    parseArrayNode(lidarGridNode["grid_circle_color"],
                   lidarGridConfig.grid_circle_color);
    return 0;
  }

  // websocket color_map / websocket map_color_map / roi_color_map Node
  int parseXvizColorMapperConfig(
      const RsYamlNode& colorMapperNode,
      RsPointCloudColorMapperConfig& colorMapConfig) {
    std::string colorMapType =
        colorMapperNode["color_map_type"].as<std::string>();
    colorMapConfig.colorMapperType = fromNameToType(colorMapType);
    colorMapConfig.colorMapperEnable =
        colorMapperNode["color_map_enable"].as<bool>();
    RsYamlNode colorMapFactors = colorMapperNode["color_map_factors"];
    RsYamlNode colorMapLows = colorMapperNode["color_map_lows"];
    RsYamlNode colorMapHighs = colorMapperNode["color_map_highs"];
    RsYamlNode colorMapAlphas = colorMapperNode["color_map_alphas"];
    RsYamlNode defaultColors = colorMapperNode["default_color"];

    int colorMapFactorsSize = colorMapFactors.size();
    // for (int i = 0; i < colorMapFactors.size(); ++i)
    for (int i = 0; i < colorMapFactorsSize; ++i) {
      std::string factor = colorMapFactors[i].as<std::string>();
      float low = colorMapLows[i].as<float>();
      float high = colorMapHighs[i].as<float>();
      float alpha = colorMapAlphas[i].as<float>();

      int index = fromFactorToIndex(factor);
      if (index >= 0) {
        colorMapConfig.factorsEnable[index] = true;
        colorMapConfig.factorsLow[index] = low;
        colorMapConfig.factorsUp[index] = high;
        colorMapConfig.factorsAlphas[index] = alpha;
      }
    }
    colorMapConfig.enableColorMapInvert =
        colorMapperNode["color_map_invert"].as<bool>();
    colorMapConfig.enableLabelPointColor =
        colorMapperNode["color_map_label"].as<bool>();

    int defaultColorsSize = defaultColors.size();
    for (int i = 0; i < defaultColorsSize; ++i) {
      colorMapConfig.defaultColor[i] =
          static_cast<unsigned char>(defaultColors[i].as<int>());
    }
    colorMapConfig.pointSize = colorMapperNode["point_size"].as<float>();

    return 0;
  }

  // websocket roi_color_map Node
  int parseXvizColorMapperConfig(
      const RsYamlNode& roiColorMapperNode,
      std::vector<std::vector<st_roiColorMapConfig>>& roiMapperColors) {
    roiMapperColors.clear();
    for (size_t i = 0; i < roiColorMapperNode.size(); ++i) {
      RsYamlNode roiColorMapNodes = roiColorMapperNode[i];
      std::vector<st_roiColorMapConfig> roiColorMapConfigs;
      for (size_t j = 0; j < roiColorMapNodes.size(); ++j) {
        RsYamlNode roiColorNode = roiColorMapNodes[j];

        st_roiColorMapConfig config;
        config.roi_type = roiColorNode["roi_type"].as<int>();
        config.filter_type = roiColorNode["filter_type"].as<int>();
        parseArrayNode(roiColorNode["color"], config.color);

        roiColorMapConfigs.push_back(config);
      }
      roiMapperColors.push_back(roiColorMapConfigs);
    }
    return 0;
  }

  // websocket map_origin Node
  int parseXvizMapOriginConfig(const RsYamlNode& mapOriginNode) {
    mapOriginConfig.lat = mapOriginNode["lat"].as<float>();
    mapOriginConfig.lon = mapOriginNode["lon"].as<float>();
    mapOriginConfig.alt = mapOriginNode["alt"].as<float>();

    return 0;
  }

  // websocket fill_images/fill_maps/fill_rois Node
  int parseXvizFillConfig(const RsYamlNode& fillDatasNode,
                          std::vector<std::string>& fillDataPaths) {
    fillDataPaths.clear();

    size_t fillDatasCnt = fillDatasNode.size();
    for (size_t iter = 0; iter < fillDatasCnt; ++iter) {
      const std::string value = fillDatasNode[iter].as<std::string>();
      fillDataPaths.push_back(value);
    }

    return 0;
  }

  void parseArrayNode(const RsYamlNode& colorNode,
                      RsXvizColorConfig& colorConfig) {
    int colorNodeCnt = colorNode.size();
    // for (int i = 0; i < colorNode.size(); ++i)
    for (int i = 0; i < colorNodeCnt; ++i) {
      colorConfig.colors[i] =
          static_cast<unsigned char>(colorNode[i].as<unsigned int>());
    }
  }

  void parseArrayNode(const RsYamlNode& transformNode,
                      RsXvizTransformConfig& transformConfig) {
    int transformNodeCnt = transformNode.size();
    // for (int i = 0; i < transformNode.size(); ++i)
    for (int i = 0; i < transformNodeCnt; ++i) {
      transformConfig.matrix[i] = transformNode[i].as<float>();
    }
  }

  void parseArrayNode(const RsYamlNode& lidarGridNode,
                      const std::string& keyWord,
                      RsXvizLidarGridConfig& lidarGridConfig) {
    int lidarGridNodeCnt = lidarGridNode.size();
    if (keyWord == "grid_circle_ranges") {
      // for (int i = 0; i < lidarGridNode.size(); ++i)
      for (int i = 0; i < lidarGridNodeCnt; ++i) {
        lidarGridConfig.grid_circle_ranges.push_back(
            lidarGridNode[i].as<float>());
      }
    }
  }

  RS_COLOR_MAP_TYPE fromNameToType(const std::string& name) {
    RS_COLOR_MAP_TYPE colorType =
        RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_HSV_TYPE;

    if (name == "autumn") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_AUTUMN_TYPE;
    } else if (name == "bone") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_BONE_TYPE;
    } else if (name == "jet") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_JET_TYPE;
    } else if (name == "winter") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_WINTER_TYPE;
    } else if (name == "rainbow") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_RAINBOW_TYPE;
    } else if (name == "ocean") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_OCEAN_TYPE;
    } else if (name == "summer") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_SUMMER_TYPE;
    } else if (name == "spring") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_SPRING_TYPE;
    } else if (name == "cool") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_COOL_TYPE;
    } else if (name == "hsv") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_HSV_TYPE;
    } else if (name == "pink") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_PINK_TYPE;
    } else if (name == "hot") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_HOT_TYPE;
    } else if (name == "parula") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_PARULA_TYPE;
    } else if (name == "rviz") {
      colorType = RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_RVIZ_TYPE;
    }

    return colorType;
  }

  int fromFactorToIndex(const std::string& factor) {
    int index = -1;
    if (factor == "x") {
      index =
          static_cast<int>(RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_X);
    } else if (factor == "y") {
      index =
          static_cast<int>(RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_Y);
    } else if (factor == "z") {
      index =
          static_cast<int>(RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_Z);
    } else if (factor == "intensity") {
      index = static_cast<int>(
          RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_INTENSITY);
    }

    return index;
  }
};



}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_COMMON_CUSTOM_PARAMS_H_
