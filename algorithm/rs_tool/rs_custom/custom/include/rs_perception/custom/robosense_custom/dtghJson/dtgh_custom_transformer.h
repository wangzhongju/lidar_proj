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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_DTGH_CUSTOM_TRANSFORMER_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_DTGH_CUSTOM_TRANSFORMER_H_

#include "rs_perception/custom/robosense_custom/dtghJson/dtgh_custom_transformer_utils.h"


namespace robosense {
namespace perception {
namespace Dtgh {

const std::map<lidar::EventType, int> kEvtType2TypeIDMap = { // sequence from pdf
{lidar::EventType::Unknown, 0},
{lidar::EventType::VehicleReverse, 1},
{lidar::EventType::Jam, 2},
{lidar::EventType::UnknownLoss, 3},
{lidar::EventType::AbnormalStop, 4},
{lidar::EventType::LimiteVel_Bot, 11},
{lidar::EventType::AbnormalSwitch, 13},
{lidar::EventType::LimiteVel_Up, 14},
{lidar::EventType::AbnormalOccupy, 15},
{lidar::EventType::PedestrianInvade, 16},
{lidar::EventType::PedestrianCross, 18},
{lidar::EventType::AbnormalSwitchTwice, 13},
{lidar::EventType::Statistical, 998},
{lidar::EventType::LimiteVelocity, 999},
};

const std::map<int, lidar::EventType> kTypeID2EvtTypeMap = {
{0, lidar::EventType::Unknown},
{1, lidar::EventType::VehicleReverse},
{2, lidar::EventType::Jam},
{3, lidar::EventType::UnknownLoss},
{4, lidar::EventType::AbnormalStop},
{11, lidar::EventType::LimiteVel_Bot},
{13, lidar::EventType::AbnormalSwitch},
{14, lidar::EventType::LimiteVel_Up},
{15, lidar::EventType::AbnormalOccupy},
{16, lidar::EventType::PedestrianInvade},
{18, lidar::EventType::PedestrianCross},
{20, lidar::EventType::AbnormalSwitchTwice},
{998, lidar::EventType::Statistical},
{999, lidar::EventType::LimiteVelocity},
};

class EventConvert {
public:
    using Ptr  = std::shared_ptr<EventConvert>;
    EventConvert() = default;

    void init() {
        lunzhuan_num = 0;
        lunzhuan_evt_num = 0;
        obj_id_map.clear();
        tmp_obj_id_map.clear();

        Jam_id_map.clear();
        tmp_Jam_id_map.clear();
        jam_his_status_map.clear();

        evt_id_map.clear();
        tmp_evt_id_map.clear();
        evt_his_status_map.clear();
    }

    void convert(const RsPerceptionMsg::Ptr& msg_ptr, Dtgh::EventMsg::Ptr& result_ptr, const bool& hd_map_disable);


    std::string transTostring(int num) {
        char buf[32] = {0};
        sprintf(buf, "%.3d", num);
        return std::string((const char *)buf);
    }

    struct ObjectInfo {
        std::string object_id;
        int time_loss = 0;
    };

    struct EventInfo {
        std::string event_id;
        int time_loss = 0;
    };

    int loss_threshold = 36000; // 物体/事件丢失了1小时

    // obj 2011
    int lunzhuan_num = 0;
    std::map<int, ObjectInfo> obj_id_map;
    std::map<int, std::string> tmp_obj_id_map;

    // evt 2012
    int lunzhuan_evt_num = 0;
    std::map<int, std::string> Jam_id_map;
    std::map<int, std::string> tmp_Jam_id_map;
    std::map<int,EventList> jam_his_status_map;

    std::map<std::string, EventInfo> evt_id_map;
    std::map<std::string, std::string> tmp_evt_id_map;
    std::map<std::string, EventList> evt_his_status_map;

    int lidar_frame = 1;
    int evt_id = 1;

};

// 获取设备状态相关的信息
class RSDeviceStatusUtil {
public:
    static int getDeviceStatusInfo(st_HardwareInfo &hardwareInfo);

private:
    static int getCpuUseInfo(st_HardwareInfo &hardwareInfo);

    static int getMemoryUseInfo(st_HardwareInfo &hardwareInfo);

    static int getDiskUseInfo(st_HardwareInfo &hardwareInfo);

    static int getDeviceTempture(st_HardwareInfo &hardwareInfo);

    static int getGpuUseInfo(st_HardwareInfo &hardwareInfo);

    static int splitString(std::string &content, std::vector<std::string> &elements, const char spliter = ' ', const bool keepEmpty = false);
};

// 序列化成json的方法
class RSCustomSerialize {
public:
    // 配置响应　
    static int serialize(const st_UploadInfo &info, std::string &encodeData);

    // 保活
    static int serialize(const st_KeepAliveInfo &info, std::string &encodeData);

    // 设备状态
    static int serialize(const st_DeviceStatusInfo &info, std::string &encodeData);
    static int serialize(const st_HardwareInfo &info, Json::Value &jsonValue);

    // Object List
    static int serialize(const ObjectList &info, const std::string& deviceNo, Json::Value &jsonValue);
    static int serialize(const ObjectSend &info, const std::string& deviceNo, std::string &encodeData);

    // Event List
    static int serialize(const EventList &info, const std::string& deviceNo, Json::Value &jsonValue);
    static int serialize(const EventSend &info, const std::string& deviceNo, std::string &encodeData);

    static RS_DTGH_OBJECTTYPE fromRoboToDTGH(const ObjectType &type);

};

// 心跳信息相关
class RSCustomKeepAlive {
public:
    void init(RsDtghCustomMsgParams& custom_params) {
        params_ = custom_params;
        auto func = [this]() {
            while (params_.allow_keep_alive) {
                this->startKeepAliveTimer();
            }
        };
        keep_alive_thread_ = std::thread (func);
        keep_alive_thread_.detach();
    }

    void startKeepAliveTimer() {
//        if (timestamp_.empty()) RINFO << "keep alive timestamp is empty";
//        else RINFO << "keep alive timestamp: " << timestamp_;

        std::this_thread::sleep_for(std::chrono::milliseconds(params_.keep_alive_time_ms));
        genKeepAliveData();
    }

    int genKeepAliveData() {
        // 生成心跳信息
        st_KeepAliveInfo info;

        info.MsgType = static_cast<int>(RS_DTGH_MSGTYPE::RS_DTGH_KEEPALIVE);
        info.DevNo = params_.device_no;
        info.MecNo = params_.mec_no;

//        RINFO << "----> 202 obtain KEEP ALIVE timestamp begin";
        double timestamp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count() * 1.0 / 1000;
        std::string sTimestamp = RSDtghCommUtil::Timestamp2time1(timestamp);
//        RINFO << "----> 205 obtain KEEP ALIVE timestamp end";

        info.Timestamp = sTimestamp;

//        RINFO << "----> 209 serialize KEEP ALIVE begin";
        std::string encodeData;
        int ret = RSCustomSerialize::serialize(info, encodeData);
//        RINFO << "----> 212 serialize KEEP ALIVE end";

        // 把结果往外写，加锁防止读写冲突
//        RINFO << "----> 215 write KEEP ALIVE begin";
        std::lock_guard<std::mutex> lg(keep_alive_mutex_);
        timestamp_.clear();
        timestamp_ = sTimestamp; // RINFO << "TIMESTAMP FINISH";
        encodeData_.clear();
        encodeData_ = encodeData; // RINFO << "ENCODEDATA FINISH";
//        RINFO << "----> 221 write KEEP ALIVE end";

        if (ret != 0) {
            return -1;
        }

        return 0;
    }

    std::vector<std::string> getTimestampAndRes() {
        std::lock_guard<std::mutex> lg(keep_alive_mutex_);
        std::vector<std::string> res;
        res.resize(2);
        res[0] = timestamp_;
        res[1] = encodeData_;
//        RINFO << "----> 236 get keep alive timestamp and res end";
        return res;
    }

    std::string encodeData_, timestamp_;
    RsDtghCustomMsgParams params_;
    std::mutex keep_alive_mutex_;
    std::thread keep_alive_thread_;

};

// 设备状态相关
class RSCustomDeviceStatus{
public:
    void init(RsDtghCustomMsgParams& custom_params) {
        params_ = custom_params;
        auto func = [this]() {
            while (params_.allow_device_status) {
                this->startDeviceStatusTimer();
            }
        };
        device_status_thread_ = std::thread (func);
        device_status_thread_.detach();
    }

    void startDeviceStatusTimer() {
//        if (timestamp_.empty()) RINFO << "device status timestamp is empty";
//        else RINFO << "device status timestamp: " << timestamp_;

        std::this_thread::sleep_for(std::chrono::milliseconds(params_.device_time_status_ms));
        genDeviceStatusData();
    }

    int genDeviceStatusData() {
        // 生成设备状态信息
        st_DeviceStatusInfo info;

        info.MsgType = static_cast<int>(RS_DTGH_MSGTYPE::RS_DTGH_DEVICESTATUE);
        info.DevNo = params_.device_no;
        info.MecNo = params_.mec_no;

//        RINFO << "====> 277 obtain DEVICE STATUS timestamp begin";
        double timestamp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count() * 1.0 / 1000;
        std::string sTimestamp = RSDtghCommUtil::Timestamp2time1(timestamp);
//        RINFO << "====> 280 obtain DEVICE STATUS timestamp end";

        info.Timestamp = sTimestamp;

        // 获取硬件信息
//        RINFO << "====> 285 obtain DEVICE STATUS info begin";
        st_HardwareInfo hardwareInfo;
        int ret = RSDeviceStatusUtil::getDeviceStatusInfo(hardwareInfo);
        if (ret != 0) {
            return -1;
        }

        info.hardwareInfos.push_back(hardwareInfo);
//        RINFO << "====> 293 obtain DEVICE STATUS info end";

//        RINFO << "====> 295 serialize DEVICE STATUS begin";
        std::string encodeData;
        ret = RSCustomSerialize::serialize(info, encodeData);
//        RINFO << "====> 298 serialize DEVICE STATUS end";

        // 把结果往外写，加锁防止读写冲突
//        RINFO << "====> 301 write DEVICE STATUS begin";
        std::lock_guard<std::mutex> lg(device_status_mutex_);
        timestamp_.clear();
        timestamp_ = sTimestamp; // RINFO << "TIMESTAMP FINISH !";
        encodeData_.clear();
        encodeData_ = encodeData; // RINFO << "ENCODEDATA FINISH !";
//        RINFO << "====> 307 write DEVICE STATUS end";
        if (ret != 0) {
            return -2;
        }

        return 0;
    }

    std::vector<std::string> getTimestampAndRes() {
        std::lock_guard<std::mutex> lg(device_status_mutex_);
        std::vector<std::string> res;
        res.resize(2);
        res[0] = timestamp_;
        res[1] = encodeData_;
//        RINFO << "----> 321 get device status timestamp and res end";
        return res;
    }

    std::string encodeData_, timestamp_;
    RsDtghCustomMsgParams params_;
    std::mutex device_status_mutex_;
    std::thread device_status_thread_;

};

}
}
}

#endif //RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_DTGH_CUSTOM_TRANSFORMER_H_
