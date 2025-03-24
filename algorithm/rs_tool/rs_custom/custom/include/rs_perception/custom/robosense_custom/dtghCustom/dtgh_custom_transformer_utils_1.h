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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_CUSTOM_TRANSFORMER_UTILS_1_H
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_CUSTOM_TRANSFORMER_UTILS_1_H

#include "rs_common/external/common.h"
#include "rs_perception/communication/external/common/basic_type.h"
#include "rs_perception/custom/common/base_custom_msg.h"

namespace robosense {
namespace perception {
namespace Dtgh1{


const std::map<std::string, char> kEventTypeString2CharIdMap = {
{"AbnormalStop", 0x00},
{"VehicleReverse", 0x01},
{"AbnormalOccupy", 0x02},
{"LimiteVel_Up", 0x03},
{"LimiteVel_Bot", 0x04},
{"AbnormalSwitchTwice", 0x06},
{"AbnormalSwitch", 0x07},
{"PedestrianInvade", 0x08},
{"UnknownLoss", 0x0b},
};

const std::map<int, char> kEventStatusInt2CharIdMap = {
{0, 0x00},
{1, 0x01},
};

const std::map<int, char> kLaneIDInt2CharIdMap = {
{1, 0x01},
{2, 0x02},
{3, 0x03},
{4, 0x04},
{5, 0x05},
{6, 0x06},
{7, 0x07},
{8, 0x08},
{9, 0x09},
{10, 0x0A},
{11, 0x0B},
{12, 0x0C},
{13, 0x0D},
{14, 0x0E},
{15, 0x0F},
{16, 0x10},
{17, 0x11},
{18, 0x12},
{19, 0x13},
{20, 0x14},
{21, 0x15},
{22, 0x16},
{23, 0x17},
{24, 0x18},
{25, 0x19},
{26, 0x1A},
{27, 0x1B},
{28, 0x1C},
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

struct EventList {
    int tracker_id = -1; // 用于辨别物体
    int event_type = 0;
    int event_status = -1;
    double longitude = 0;
    double latitude = 0;
    double timestamp = 0;
};

class EventSend {
public:
    using Ptr = std::shared_ptr<EventSend>;
    std::vector<EventList> Evt_list;
};

class EventMsg {
public:
    using Ptr = std::shared_ptr<EventMsg>;
    EventMsg(){
        event_send_ptr.reset(new EventSend);
    }

    EventSend::Ptr event_send_ptr;
};

}
}
}

#endif //RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_CUSTOM_TRANSFORMER_UTILS_1_H
