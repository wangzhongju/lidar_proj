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
#ifndef RS_PERCEPTION_LIDAR_MAP_FILTER_INTERNAL_MOLE_RS_HD_MAP_H
#define RS_PERCEPTION_LIDAR_MAP_FILTER_INTERNAL_MOLE_RS_HD_MAP_H

#include <vector>
#include <memory>
#include <algorithm>
#include "rs_common/external/basic_type/rs_vector.h"
#include "rs_perception/common/external/basic_type/rs_range.h"
#include "rs_perception/common/external/common.h"
#include "rs_common/external/rs_logger.h"

namespace robosense {
namespace perception {
namespace lidar {

enum class ReferPoint{
    CENTER = 0,
    CORNER = 1,
};

enum class FilterType{
    BOX_KEEP = 0,
    BOX_DISCARD = 1,
};

enum class AttributeType{
    UNKNOWN = 0,    // 未知,默认
    ROADWAY = 1,    // 车行道
    SIDEWALK = 2,   // 人行道
    CROSSWALK = 3,   // 人行横道,斑马线
    PED_ISLAND = 4,  // 行人安全岛
    GUARDRAIL = 5,   // 护栏
    GREEN_AREA = 6   // 绿化带
};

enum class EventType {
    AbnormalSwitch = 0,
    LimiteVelocity = 1,
    VehicleReverse = 2,
    Jam = 3,
    AbnormalOccupy = 4,
    AbnormalStop = 5,
    PedestrianCross = 6,
    PedestrianInvade = 7,
    UnknownLoss = 8,
    Statistical = 9,
    LimiteVel_Up = 10,
    LimiteVel_Bot = 11,
    Unknown = 12,
    AbnormalSwitchTwice = 13,
};

const std::map<int, std::array<int, 2>> kConfigFilter2InteFilter = {
{0, {0,0}}, // {center, box_keep}
{1, {0,1}}, // {center, box_discard}
{2, {1,0}}, // {corner, box_keep}
{3, {1,1}}, // {corner, box_discard}
};

const std::map<EventType, std::string> kEventType2NameMap = {
{EventType::AbnormalSwitch, "AbnormalSwitch"},
{EventType::LimiteVelocity, "LimiteVelocity"},
{EventType::VehicleReverse, "VehicleReverse"},
{EventType::Jam, "Jam"},
{EventType::AbnormalOccupy, "AbnormalOccupy"},
{EventType::AbnormalStop, "AbnormalStop"},
{EventType::PedestrianCross, "PedestrianCross"},
{EventType::PedestrianInvade, "PedestrianInvade"},
{EventType::UnknownLoss, "UnknownLoss"},
{EventType::Statistical, "Statistical"},
{EventType::Unknown, "Unknown"},
{EventType::LimiteVel_Up, "LimiteVel_Up"},
{EventType::LimiteVel_Bot, "LimiteVel_Bot"},
{EventType::AbnormalSwitchTwice, "AbnormalSwitchTwice"}
};

const std::map<std::string, EventType> kEventName2TypeMap = {
{"AbnormalSwitch", EventType::AbnormalSwitch},
{"LimiteVelocity", EventType::LimiteVelocity},
{"VehicleReverse", EventType::VehicleReverse},
{"Jam", EventType::Jam},
{"AbnormalOccupy", EventType::AbnormalOccupy},
{"AbnormalStop", EventType::AbnormalStop},
{"PedestrianInvade", EventType::PedestrianInvade},
{"UnknownLoss", EventType::UnknownLoss},
{"Statistical", EventType::Statistical},
{"Unknown", EventType::Unknown},
{"LimiteVel_Up", EventType::LimiteVel_Up},
{"LimiteVel_Bot", EventType::LimiteVel_Bot},
{"AbnormalSwitchTwice", EventType::AbnormalSwitchTwice},
};

const std::map<ReferPoint, std::string> kReferPointType2NameMap = {
{ReferPoint::CENTER, "center"},
{ReferPoint::CORNER, "corner"},
};

const std::map<std::string, ReferPoint> kReferPointName2TypeMap = {
{"center", ReferPoint::CENTER},
{"corner", ReferPoint::CORNER},
};

struct MOD_PUBLIC StaticParams {
    int roiIdx = -1;
    int laneIdx = -1;
    int direction = -1;
    std::vector<RsVector3f> pts;
};

struct MOD_PUBLIC EventParams {
    std::map<int, int> neighborLaneType;
    float timeTh = 0.;
    float percentTh = 0.;
    float velocityTh = 0.;
    int velType = -1;
    float velocityThUpper = 0.;
    float timeThUpper = 0.;
    float velocityThBottom = 0.;
    float timeThBottom = 0.;
    float degreeTh = 0.;
    int vehicleNumTh = -1;
    std::vector<RsVector3f> directionPoints;
    std::map<int, StaticParams> statistic_map;
};

class MOD_PUBLIC RsROI {
public:
    using Ptr = std::shared_ptr<RsROI>;
    int id = -1;
    int priority = 0;
    ReferPoint refer_point = ReferPoint::CENTER;
    FilterType filter_type = FilterType::BOX_KEEP;
    AttributeType attri_type = AttributeType::UNKNOWN;
    std::vector<RsVector3f> polygon;
    std::map<EventType, EventParams> event_map;
};

class MOD_PUBLIC RsHdMap {
public:
    using Ptr = std::shared_ptr<RsHdMap>;
    std::map<int, RsROI::Ptr> rois_map;
    bool keep_flag = false;  // 标识符：表示这堆ROI中是否含有keep属性的ROI
    bool discard_flag = false;  // 标识符：表示这堆ROI中是否含有discard属性的ROI

    std::string name() {
        return "RsHdMap";
    }

    // load configures from yaml and init properties of roi.
    // input: yaml node
    void loadRois(const RsYamlNode& rois_node){
        rois_map.clear();
        RsCollectAttribute attr;
        attr.axis = AxisStatus::GLOBAL_AXIS;
        for (size_t i = 0; i < rois_node.size(); i++) {
            RsROI::Ptr tmp_roi_ptr;
            tmp_roi_ptr.reset(new RsROI);
            tmp_roi_ptr->event_map.clear();
            rsYamlRead(rois_node[i], "index", tmp_roi_ptr->id);
            int temp = 0;
            rsYamlRead(rois_node[i], "filter_type", temp);
//            tmp_roi_ptr->filter_type = FilterType(temp);
            const auto& filter_config = kConfigFilter2InteFilter.at(temp);
            tmp_roi_ptr->refer_point = ReferPoint(filter_config.at(0));
            tmp_roi_ptr->filter_type = FilterType(filter_config.at(1));

            if (tmp_roi_ptr->filter_type == FilterType::BOX_KEEP) {
                keep_flag = true;
            }
            else {
                discard_flag = true;
            }

            attr.type_map[tmp_roi_ptr->id] = temp;

            rsYamlRead(rois_node[i], "roi_type", temp);
            tmp_roi_ptr->attri_type = AttributeType(temp);

            RsYamlNode polygon_node;
            rsYamlSubNode(rois_node[i], "anchors", polygon_node);
            if (polygon_node.size() < 3) {
                continue;
            }

            for (size_t j = 0; j < polygon_node.size(); j++) {
                RsVector3f vertex;
                vertex.x = polygon_node[j][0].as<float>();
                vertex.y = polygon_node[j][1].as<float>();
                vertex.z = polygon_node[j][2].as<float>();
                tmp_roi_ptr->polygon.emplace_back(vertex);
            }
            attr.pts_map[tmp_roi_ptr->id] = tmp_roi_ptr->polygon;

            // event init
            RsYamlNode events_node;
            rsYamlSubNode(rois_node[i], "events", events_node);
            initRoisEvent(events_node,tmp_roi_ptr);

            rois_map[tmp_roi_ptr->id] = tmp_roi_ptr;
        }
        RsConfigManager().addCollect("roi", attr);
    }

    // load configures from yaml and init event properties of roi.
    // input: yaml node and robosense roi struct
    void initRoisEvent(const RsYamlNode& events_node, RsROI::Ptr& tmp_roi_ptr){
        bool enable_ = false;
        // 限速
        rsYamlRead(events_node["LimiteVelocity"], "enable", enable_);
        const auto vel_node = events_node["LimiteVelocity"];
        if (enable_) {
            EventParams tmp_event;
            rsYamlRead(vel_node, "velType", tmp_event.velType);
            rsYamlRead(vel_node, "velocityThUpper", tmp_event.velocityThUpper);
            rsYamlRead(vel_node, "timeThUpper", tmp_event.timeThUpper);
            rsYamlRead(vel_node, "velocityThBottom", tmp_event.velocityThBottom);
            rsYamlRead(vel_node, "timeThBottom", tmp_event.timeThBottom);
            tmp_roi_ptr->event_map[EventType::LimiteVelocity] = tmp_event;
            enable_ = false;
        }

        // 异常停车
        rsYamlRead(events_node["AbnormalStop"], "enable", enable_);
        const auto stop_node = events_node["AbnormalStop"];
        if (enable_) {
            EventParams tmp_event;
            rsYamlRead(stop_node, "velocityTh", tmp_event.velocityTh);
            rsYamlRead(stop_node, "timeTh", tmp_event.timeTh);
            tmp_roi_ptr->event_map[EventType::AbnormalStop] = tmp_event;
            enable_ = false;
        }

        // 异常变道
        rsYamlRead(events_node["AbnormalSwitch"], "enable", enable_);
        const auto switch_node = events_node["AbnormalSwitch"];
        if (enable_) {
            EventParams tmp_event;
            rsYamlRead(switch_node, "timeTh", tmp_event.timeTh);
            RsYamlNode neighborRoi_node;
            rsYamlSubNode(switch_node, "neighborRoiIds", neighborRoi_node);
            for (size_t j = 0; j < neighborRoi_node.size(); ++j) {
                int tmp_id;
                rsYamlRead(neighborRoi_node[j], "id", tmp_id);
                rsYamlRead(neighborRoi_node[j], "style", tmp_event.neighborLaneType[tmp_id]);
            }
            tmp_roi_ptr->event_map[EventType::AbnormalSwitch] = tmp_event;
            enable_ = false;
        }

        // 非法占用
        rsYamlRead(events_node["AbnormlOccupy"], "enable", enable_);
        const auto occupy_node = events_node["AbnormlOccupy"];
        if (enable_) {
            EventParams tmp_event;
            rsYamlRead(occupy_node, "timeTh", tmp_event.timeTh);
            tmp_roi_ptr->event_map[EventType::AbnormalOccupy] = tmp_event;
            enable_ = false;
        }

        // 车辆逆行
        rsYamlRead(events_node["VehicleReverse"], "enable", enable_);
        const auto reverse_node = events_node["VehicleReverse"];
        if (enable_) {
            EventParams tmp_event;
            rsYamlRead(reverse_node, "degreeTh", tmp_event.degreeTh);
            RsYamlNode anchor_node;
            rsYamlSubNode(reverse_node, "directionPoints", anchor_node);
            for (size_t j = 0; j < anchor_node.size(); j++) {
                RsVector3f vertex;
                vertex.x = anchor_node[j][0].as<float>();
                vertex.y = anchor_node[j][1].as<float>();
                vertex.z = anchor_node[j][2].as<float>();
                tmp_event.directionPoints.emplace_back(vertex);
            }
            tmp_roi_ptr->event_map[EventType::VehicleReverse] = tmp_event;
            enable_ = false;
        }

        // 行人横穿
        rsYamlRead(events_node["PedestrianCross"], "enable", enable_);
        const auto cross_node = events_node["PedestrianCross"];
        if (enable_) {
            EventParams tmp_event;
            RsYamlNode anchor_node;
            rsYamlSubNode(cross_node, "directionPoints", anchor_node);
            for (size_t j = 0; j < anchor_node.size(); j++) {
                RsVector3f vertex;
                vertex.x = anchor_node[j][0].as<float>();
                vertex.y = anchor_node[j][1].as<float>();
                vertex.z = anchor_node[j][2].as<float>();
                tmp_event.directionPoints.emplace_back(vertex);
            }
            rsYamlRead(cross_node, "timeTh", tmp_event.timeTh);
            tmp_roi_ptr->event_map[EventType::PedestrianCross] = tmp_event;
            enable_ = false;
        }

        // 行人闯入
        rsYamlRead(events_node["PedestrianInvade"], "enable", enable_);
        const auto invade_node = events_node["PedestrianInvade"];
        if (enable_) {
            EventParams tmp_event;
            rsYamlRead(invade_node, "timeTh", tmp_event.timeTh);
            tmp_roi_ptr->event_map[EventType::PedestrianInvade] = tmp_event;
        }

        // 未知物体遗撒
        rsYamlRead(events_node["UnknownLoss"], "enable", enable_);
        const auto loss_node = events_node["UnknownLoss"];
        if (enable_) {
            EventParams tmp_event;
            rsYamlRead(loss_node, "timeTh", tmp_event.timeTh);
            tmp_roi_ptr->event_map[EventType::UnknownLoss] = tmp_event;
        }

        // 交通拥堵
        rsYamlRead(events_node["Jam"], "enable", enable_);
        const auto jam_node = events_node["Jam"];
        if (enable_) {
            EventParams tmp_event;
            rsYamlRead(jam_node, "percentTh", tmp_event.percentTh);
            rsYamlRead(jam_node, "velocityTh", tmp_event.velocityTh);
            rsYamlRead(jam_node, "vehicleNumTh", tmp_event.vehicleNumTh);
            tmp_roi_ptr->event_map[EventType::Jam] = tmp_event;
            enable_ = false;
        }

        // 车流统计
        rsYamlRead(events_node["Statistical"], "enable", enable_);
        const auto sta_node = events_node["Statistical"];
        if (enable_) {
            EventParams tmp_event;
            rsYamlRead(sta_node, "timeTh", tmp_event.timeTh);
            rsYamlRead(sta_node, "velocityTh", tmp_event.velocityTh);
            RsYamlNode lane_node;
            rsYamlSubNode(sta_node, "statLanes", lane_node);
            for (size_t j = 0; j < lane_node.size(); j++) {
                StaticParams sta_params;
                rsYamlRead(lane_node[j], "roiIdx", sta_params.roiIdx);
                rsYamlRead(lane_node[j], "laneIdx", sta_params.laneIdx);
                rsYamlRead(lane_node[j], "direction", sta_params.direction);
                tmp_event.statistic_map[sta_params.roiIdx] = sta_params;
            }
            tmp_roi_ptr->event_map[EventType::Statistical] = tmp_event;
            std::sort(tmp_roi_ptr->polygon.begin(), tmp_roi_ptr->polygon.end(),
                      [](const RsVector3f& a, const RsVector3f& b) {
                if (a.x == b.x) {
                    return a.y < b.y;
                }
                else {
                    return a.x < b.x;
                }
            });
        }
    }
};
}   // namespace lidar
}   // namespace perception
}   // namespace robosense
#endif  // RS_PERCEPTION_LIDAR_MAP_FILTER_INTERNAL_MOLE_RS_HD_MAP_H
