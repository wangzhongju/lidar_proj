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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_XINGYUN_XINGYUN_CUSTOM_TRANSFORMER_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_XINGYUN_XINGYUN_CUSTOM_TRANSFORMER_H_

#include "rs_perception/custom/robosense_custom/xingyunCustom/xingyun_custom_transformer_utils.h"
#include "rs_perception/communication/external/common/basic_type.h"

#ifdef RS_PROTO_FOUND
#include "Proto_msg.PerceptXingyun.pb.h"
#include <netinet/in.h>

namespace robosense {
namespace perception {
namespace Xingyun {

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

    void convert(const RsPerceptionMsg::Ptr& msg_ptr, Xingyun::EventMsg::Ptr& result_ptr, const bool& hd_map_disable);

    int convertToProto(char* data,EventMsg::Ptr& result_ptr,RsXingyunCustomMsgParams& custom_params_,int frame_id_,const RsPerceptionMsg::Ptr &msg_ptr);

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


// 心跳信息相关
class RSCustomKeepAlive {
public:
    // void keepAliveCallback(RsXingyunCustomMsgParams custom_params) {
    //     genKeepAliveData(custom_params);
    //     startKeepAliveTimer(custom_params);
    // }

    void startKeepAliveTimer(RsXingyunCustomMsgParams custom_params) {
//        if (timestamp_.empty()) RINFO << "keep alive timestamp is empty";
//        else RINFO << "keep alive timestamp: " << timestamp_;
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(custom_params.keep_alive_time_ms));
            genKeepAliveData(custom_params);
        }
        
    }

    int genKeepAliveData(RsXingyunCustomMsgParams custom_params) {
        st_KeepAliveInfo info;

        info.MsgType = static_cast<int>(RS_XINGYUN_MSGTYPE::RS_XINGYUN_KEEPALIVE);
        info.DevNo = custom_params.device_no;
        info.MecNo = custom_params.mec_no;

        double timestamp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count() * 1.0 / 1000;
        std::string sTimestamp = RSXingyunCommUtil::Timestamp2time1(timestamp);
        unsigned char send2fusion[10] = {0};

        send2fusion[0] = 0xDA;
        send2fusion[1] = 0xDB;
        send2fusion[2] = 0xDC;
        send2fusion[3] = 0xDD;
        send2fusion[4] = 0x02;
        send2fusion[5] = 0x04;
        *((unsigned short *)(send2fusion + 6)) = htons(1);

        send2fusion[8] = 0x02;

        info.Timestamp = sTimestamp;
        std::lock_guard<std::mutex> lg(keep_alive_mutex_);
        timestamp_.clear();
        timestamp_ = sTimestamp;

        encodeData_.clear();

        for (int i=0;i<9;i++) {
            encodeData_ += send2fusion[i];
        }

        //        std::cout << "KeepAlive encodeData = " << encodeData << std::endl;
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


    std::mutex keep_alive_mutex_;
    std::string encodeData_, timestamp_;
};

}
}
}
#endif  // RS_PROTO_FOUND
#endif //RS_PERCEPTION_CUSTOM_ROBOSENSE_XINGYUN_XINGYUN_CUSTOM_TRANSFORMER_H_
