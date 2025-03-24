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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_DTGH_CUSTOM_TRANSFORMER_UTILS_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_DTGH_CUSTOM_TRANSFORMER_UTILS_H_

#include "json/json.h"
#include "rs_common/external/common.h"
#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_perception/lidar/map_filter/external/common/rs_hd_map.h"
#include "rs_perception/lidar/map_filter/external/base_map_filter.h"

namespace robosense {
namespace perception {
namespace Dtgh {

const std::map<int, std::string> kTypeID2PtcTypeMap = {
{0,"Unknown"},
{1,"Car"},
{2,"Bicycle"},
{3,"Pedestrian"},
};

const std::map<int, std::string> kTypeID2EvtTypeStringMap = {
{0,"Unknown"},
{1,"VehicleReverse"},
{2,"Jam"},
{3,"UnknownLoss"},
{4,"AbnormalStop"},
{11,"LimitVel_Bot"},
{13,"AbnormalSwitch"},
{14, "LimitVel_Up"},
{15,"AbnormalOccupy"},
{16,"PedestrianInvade"},
{18,"PedestrianCross"},
{20,"AbnormalSwitchTwice"},
{998,"Statistical"},
{999,"LimitVelocity"},
};

const std::map<int, std::string> kTypeID2EvtStatusStringMap = {
{0,"Event begin"},
{1,"Event happening"},
{2,"Event end"},
};

struct ObjectList {
    std::string ID = "";
    int PtcType = 0;
    double VehL = -1.0;
    double VehW = -1.0;
    double VehH = -1.0;
    double PtcLon = -1.0;
    double PtcLat = -1.0;
    double PtcEle = -1.0;
    float XPos = -1000.0;
    float YPos = -1000.0;
    double PtcSpeed = -1.0;
    double PtcHeading = -1.0;
    int VehType;
};

class ObjectSend {
public:
    using Ptr = std::shared_ptr<ObjectSend>;
    int MsgType = 2011;
    std::string DevNo;
    std::string MecNo;
    std::string Timestamp;
    std::vector<ObjectList> Obj_List;

};

struct EventList {
    std::string ObjId = "-1";
    std::string ID = "-1";
    int tracker_id = -1;
    int EvtStatus = -1;
    int EvtType = 0;
    double Lon = -1.0;
    double Lat = -1.0;
    double Ele = -1.0;
    float XPos = -1000.0;
    float YPos = -1000.0;
    double VehL = -1.0;
    double VehW = -1.0;
    double VehH = -1.0;
    int laneIdx = -1;
    int vel_num = 0;
    int direction = 0;
    int roiIdx = -1;
    int statisticRoiIdx = -1;
};

class EventSend {
public:
    using Ptr = std::shared_ptr<EventSend>;
    int MsgType = 2012;
    std::string DevNo;
    std::string MecNo;
    std::string Timestamp;
    std::vector<EventList> Evt_List;

};

class EventMsg {
public:
    using Ptr = std::shared_ptr<EventMsg>;
    EventMsg(){
        obj_send_ptr.reset(new ObjectSend);
        event_send_ptr.reset(new EventSend);
    }

    ObjectSend::Ptr obj_send_ptr;
    EventSend::Ptr event_send_ptr;
    bool event_detect = true;
};

// 消息类型
enum class RS_DTGH_MSGTYPE : int {
    RS_DTHG_UPLOADINFO = 1001,
    RS_DTGH_LIDAROBJECT = 2011,
    RS_DTGH_WGS84OBJECT = 2012,
    RS_DTGH_KEEPALIVE = 5001,
    RS_DTGH_DEVICESTATUE = 5002,
    };

// 交通参与者类型
enum class RS_DTGH_OBJECTTYPE : int{
    RS_DTGH_UNKNOWN = 0,
    RS_DTGH_VEHICLE,
    RS_DTGH_BIKE,
    RS_DTGH_PED,
    };

// 车辆类型
enum class RS_DTGH_VEHICLETYPE : int {
    RS_DTGH_VEHICLE_UNKNOW = 0,
    RS_DTGH_VEHICLE_AMB,
    RS_DTGH_VEHICLE_FIRE,
    RS_DTGH_VEHICLE_POLICE,
    };

// 上传配置信息
struct st_UploadInfo {
    int MsgType;
    std::string DevNo;
    int Ack;
};

// 心跳数据信息
struct st_KeepAliveInfo {
    int MsgType;
    std::string DevNo;
    std::string MecNo;
    std::string Timestamp;
};

// 设备状态信息
struct st_HardwareInfo {
    std::string SoftwareVersion;
    std::string CpuDegree;
    std::string CpuRate;
    std::string RamRate;
    std::string SataFreeSpace;
    std::string GpuDegree;
    std::string GpuRate;
    std::string GpuSpaceRate;
    std::string ErrorCode;
};

struct st_DeviceStatusInfo {
    int MsgType;
    std::string DevNo;
    std::string MecNo;
    std::string Timestamp;
    std::vector<st_HardwareInfo> hardwareInfos;
};

class RSDtghCommUtil {
public:
    template <typename T>
    static std::string num2str(const T num, int precision) {
        std::stringstream ss;
        ss.setf(std::ios::fixed, std::ios::floatfield);
        ss.precision(precision + 1);
        std::string st;
        ss << num;
        ss >> st;
        return st.substr(0, st.size() - 1);
    }

    template <typename T>
    static T str2num(const std::string &str) {
        std::stringstream ss;
        T value;
        ss << str;
        ss >> value;
        return value;
    }

    static std::string Timestamp2time1(double timestamp) {
        std::string sTimestamp;

        unsigned long long int second = (unsigned long long int)(std::floor(timestamp));
        unsigned int millisecond = (unsigned int)(std::floor((timestamp - second) * 1000));

        // Step1: 将秒 => 年-月-日 hh-mm-ss, 注意是东8区
        std::string sSecond;

        auto FORMATDATA = [](struct tm *ttm) -> std::string {
            char buf[32] = {0};
            sprintf(buf, "%.4d-%.2d-%.2d %.2d:%.2d:%.2d", ttm->tm_year + 1900,
                    ttm->tm_mon + 1, ttm->tm_mday, ttm->tm_hour, ttm->tm_min,
                    ttm->tm_sec);

            return std::string((const char *)buf); // strdup(buf);
        };

        // 把UTC时间转换为 tm 结构体, 注意不是本地时间，是世界统一时间，全时间的时间都一样.
        struct tm utcTM = *gmtime((std::time_t *)(&second));
        if (utcTM.tm_hour + 8 > 23) {
            utcTM.tm_hour = utcTM.tm_hour + 8 - 24;
            utcTM.tm_mday++;
        }
        else {
            utcTM.tm_hour += 8;
        }
        sSecond = FORMATDATA(&utcTM);

        // std::cout << "sSecond = " << sSecond << std::endl;
        // 确保填充为三位有效数字
        std::string sMiliSecond = std::to_string(millisecond);

        // std::cout << "sMiliSecond1 = " << sMiliSecond << std::endl;
        while (sMiliSecond.size() < 3) {
            sMiliSecond = std::string("0") + sMiliSecond;
        }

        if (sMiliSecond.size() > 3) {
            sMiliSecond.resize(3);
        }
        // std::cout << "sMiliSecond2 = " << sMiliSecond << std::endl;
        // Step2: 年-月-日 hh-mm-ss.毫秒　
        sTimestamp = sSecond + "." + sMiliSecond;
        // std::cout << "sTimestamp = " << sTimestamp << std::endl;
        return sTimestamp;
    }

    static std::string Timestamp2time2(double timestamp) {
        std::string sTimestamp;

        unsigned long long int second = (unsigned long long int)(std::floor(timestamp));
        unsigned int millisecond = (unsigned int)(std::floor((timestamp - second) * 1000));

        // Step1: 将秒 => 年-月-日 hh-mm-ss, 注意是东8区
        std::string sSecond;

        auto FORMATDATA = [](struct tm *ttm) -> std::string {
            char buf[32] = {0};
            sprintf(buf, "%.4d%.2d%.2d%.2d%.2d%.2d", ttm->tm_year + 1900,
                    ttm->tm_mon + 1, ttm->tm_mday, ttm->tm_hour, ttm->tm_min,
                    ttm->tm_sec);

            return std::string((const char *)buf); // strdup(buf);
        };

        // 把UTC时间转换为 tm 结构体, 注意不是本地时间，是世界统一时间，全时间的时间都一样.
        struct tm utcTM = *gmtime((std::time_t *)(&second));
        if (utcTM.tm_hour + 8 > 23) {
            utcTM.tm_hour = utcTM.tm_hour + 8 - 24;
            utcTM.tm_mday++;
        }
        else {
            utcTM.tm_hour += 8;
        }
        sSecond = FORMATDATA(&utcTM);

        // std::cout << "sSecond = " << sSecond << std::endl;
        // 确保填充为三位有效数字
        std::string sMiliSecond = std::to_string(millisecond);

        // std::cout << "sMiliSecond1 = " << sMiliSecond << std::endl;
        while (sMiliSecond.size() < 3) {
            sMiliSecond = std::string("0") + sMiliSecond;
        }

        if (sMiliSecond.size() > 3) {
            sMiliSecond.resize(3);
        }
        // std::cout << "sMiliSecond2 = " << sMiliSecond << std::endl;
        // Step2: 年-月-日 hh-mm-ss.毫秒　
        sTimestamp = sSecond + sMiliSecond;
        // std::cout << "sTimestamp = " << sTimestamp << std::endl;
        return sTimestamp;
    }

    // 将以正东为参考，逆时针旋转为正 => 以正北为参考，顺时针为正
    static float eastYaw2NorthYaw(float eastYaw) {
        float northYaw = (90.0f - eastYaw > 1e-3) ? (90.0f - eastYaw) : (90.0f - eastYaw + 360.f);
        return northYaw;
    }

    // 将以正北为参考，顺时针为正　=> 正东为参考，逆时针为正
    static float northYaw2EastYaw(float northYaw){
        float eastYaw = (90.0f - northYaw > 1e-3) ? (90.0f - northYaw) : (90.0f - northYaw + 360.f);
        return eastYaw;
    }
};

}
}
}

#endif //RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_DTGH_CUSTOM_TRANSFORMER_UTILS_H_