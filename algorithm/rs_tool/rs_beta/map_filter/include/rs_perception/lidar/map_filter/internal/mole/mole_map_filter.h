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
#ifndef RS_PERCEPTION_LIDAR_MAP_FILTER_INTERNAL_MOLE_MOLE_MAP_FILTER_H
#define RS_PERCEPTION_LIDAR_MAP_FILTER_INTERNAL_MOLE_MOLE_MAP_FILTER_H
#include <queue>
#include "rs_perception/lidar/map_filter/external/base_map_filter.h"
#include "rs_perception/lidar/map_filter/external/common/rs_hd_map.h"
#include "rs_common/external/rs_configure_manager.h"

// #include "rs_beta/map_filter/include/rs_perception/lidar/map_filter/external/base_map_filter.h"
// #include "rs_beta/map_filter/include/rs_perception/lidar/map_filter/external/common/rs_hd_map.h"
namespace robosense {
namespace perception {
namespace lidar {
class MoleMapFilter : public BaseMapFilter {
public:
    using Ptr = std::shared_ptr<MoleMapFilter>;
    MoleMapFilter() {
        res_ptr_.reset(new MapFilterMsg);
    }

    // entrance of initialize of map filter
    // input: yaml node
    void init(const RsYamlNode& config_node) override;

    // entrance of map filter
    // input: robosense perception message struct.
    // output: void. the result will be recorded in "msg_ptr".
    void perception(const RsLidarFrameMsg::Ptr& msg_ptr) override;

    // load configures from yaml and init event properties of roi.
    // input: yaml node and robosense roi struct
    void getResult(Any::Ptr& any) override;

private:
    std::string name() {
        return "MoleMapFilter";
    }

    // entrance of object filter function.
    // this function can perform specified processing on the objects in the ROI.
    // input: robosense perception message struct (only object in it is useful)
    // output: void. objects that match the ROI filtering rules will be kept.
    void objFilter(const RsLidarFrameMsg::Ptr& msg_ptr);

    // entrance of event detect function
    // input: robosense perception message struct (only object in it is useful)
    // output: void. a collection containing the event content will be generated.
    void eventDetect(const RsLidarFrameMsg::Ptr& msg_ptr);

    // entrance of event collect function.
    // this function will collect the result from eventDetect function.
    // input: robosense perception message struct (only object in it is useful)
    // output: void. a collection of objects paired with events will be generated.
    void eventCollect(const RsLidarFrameMsg::Ptr& msg_ptr);

    // calculate the angle between two vectors.
    // input: two vectors.
    // output: the angle.
    float angCalculate(const RsVector3f& p1, const RsVector3f& p2) {
        return std::atan2((p1.y - p2.y), (p1.x - p2.x));
    }

    struct Params {
        bool filter_obj = false;
        bool event_detect = false;
        RsHdMap::Ptr hd_map_ptr;

        void load(const RsYamlNode& config_node) {
            rsYamlRead(config_node, "filter_obj", filter_obj);
            rsYamlRead(config_node, "event_detect", event_detect);

            hd_map_ptr.reset(new RsHdMap);
            std::string refer_point_ = "center";
            rsYamlRead(config_node, "refer_point", refer_point_);
//            hd_map_ptr->refer_point_ = kReferPointName2TypeMap.at(refer_point_);
            RsYamlNode rois_node;
            rsYamlSubNode(config_node, "rois", rois_node);
            rsYamlSubNode(rois_node, "rois", rois_node);
            hd_map_ptr->loadRois(rois_node);
        }

        void log(std::string name) {
            RsYamlNode node;
            node["filter_obj"] = filter_obj;
            node["event_detect"] = event_detect;
            const auto rois_map = hd_map_ptr->rois_map;

            for (auto itr = rois_map.begin(); itr != rois_map.end(); itr++) {
                const auto evt_map = itr->second->event_map;
                for (auto itr_e = evt_map.begin(); itr_e != evt_map.end(); ++itr_e) {
                    auto event_type = kEventType2NameMap.at(itr_e->first);
                    node["events"][itr->first][event_type] = true;
                }
            }
            RsYamlNode up_node;
            up_node[name] = node;
            std::stringstream ss;
            up_node.serialize(ss);
            ss << std::endl;
            RsConfigManager().append(ss.str());
        }
    }params;

    struct EventObject {
        using Ptr = std::shared_ptr<EventObject>;
        int track_id = -1;
        // 超速
        float timeThUpper = 0.0;
        bool LimiteVel_Up = false;
        // 低速
        float timeThBottom = 0.0;
        bool LimiteVel_Bot = false;
        // 变道
        float timeTh_switch = 0.0;
        float time_miss = 0.0;
        std::queue<int> roi_id;
        bool AbnormalSwitch = false;
        bool AbnormalSwitchTwice = false;
        // 占用
        double timeTh_occupy = 0.0;
        bool AbnormalOccupy = false;
        // 停车
        float timeTh_stop = 0.0;
        bool AbnormalStop = false;
        // 逆行
        bool VehicleReverse = false;
        // 横穿
        bool PedestrianCross = false;
        double timeTh_cross = 0.0;
        // 行人闯入
        bool PedestrianInvade = false;
        double timeTh_invade = 0.0;
        // 道路遗撒
        bool UnknownLoss = false;
        double timeTh_loss = 0.0;
    };

    struct EventResult {
        // roi内每个obj的事件map
        std::map<int, std::map<int, EventObject::Ptr>> roi_evt_map;
        // 变道map结果，与obj相关
        std::map<int, EventObject::Ptr> switch_map;
        // 交通拥堵map，与roi相关
        std::map<int, bool> jam_map;
        // 每个roi内统计计数
        std::map<int, std::map<int, StaticMsg>> count_map;
        std::map<int, double> count_time;
        std::map<int, RsVector3f> obj_last_map;
        bool timer = false;
        void init(const Params& params) {
            roi_evt_map.clear();
            switch_map.clear();
            jam_map.clear();
            count_map.clear();
            obj_last_map.clear();
            count_time.clear();
            const auto& rois_map = params.hd_map_ptr->rois_map;

            // 初始化roi的obj事件map和统计ROI map。
            for (auto itr = rois_map.begin(); itr != rois_map.end(); itr++) {
                std::map<int, EventObject::Ptr> obj_map;
                roi_evt_map[itr->first] = obj_map;

                auto roi = itr->second;

                if (roi->event_map.find(EventType::Statistical) != roi->event_map.end()) {
                    std::map<int, StaticMsg> count_map_;
                    count_map[itr->first] = count_map_;
                }
            }
        }
        void clean() {
            jam_map.clear();
        }
    } evt_res;

    //=========================================
    // conponent_cost related
    //=========================================
    struct Cost {
        int count = 0;
        double obj_filter;
        double event_detect;
        double event_collect;
        double all = 0.;
        double timer, tot_timer;
        void reset() {
            obj_filter = 0.;
            event_detect = 0.;
            event_collect = 0.;
            all = 0.;
            count = 0;
        }
        void print(const std::string& name) {
            RTRACE << name << ": ***filter_obj avr cost " << obj_filter / (count + 1.e-6) << " ms.";
            RTRACE << name << ": ***event_detect avr cost " << event_detect / (count + 1.e-6) << " ms.";
            RTRACE << name << ": ***event_collect avr cost " << event_collect / (count + 1.e-6) << " ms.";
            RTRACE << name << ": ***all avr cost " << all / (count + 1.e-6) << " ms.";
            if (count > 10000) {
                reset();
            }
        }
    }component_cost;

    // determine whether the speed of the object is too high or too low.
    // determine whether the object has a parking violation.
    // input: object track id, object velocity, event name, event attribute, EventObject pointer(for saving result)
    // output: void.
    void velocityJudgeEvent(int id, const RsVector3f &vel, const EventType &event_name,
                            const EventParams &event_attr, EventObject::Ptr &event_obj);

    // determine whether the direction of movement of the object is normal.
    // input: direction refer points, object velocity, direction threshold, bool res(for saving result), object track id
    // output: void.
    void directionJudgeEvent(const std::vector<RsVector3f> &dir_points, const RsVector3f &vel,
                             const float &threshold, bool &res, int& id);

    // determine whether the object has intrusion behavior.
    // input: real invade time, threshold time, bool res(for saving result)
    // output: void.
    void invadeJudgeEvent(double &invade_time, const float &threshold ,bool &res);

    // statistics of traffic flow over a period of time.
    // input: object track id, object center, refer points of the area, roi section id
    // output: void.
    void trafficStatistics(int id, const RsVector3f& center, RsVector3f& p_1, RsVector3f& p_2, int section_id,
                           std::map<int, RsVector3f>& obj_last_map, std::map<int, StaticMsg>& vehicle_count_map_);

    // determine whether the object has illegal lane change behavior.
    // input: object track id, object history status.
    // output: void.
    void switchJudgeEvent(int id, EventObject::Ptr &event_obj, const RsROI::Ptr &roi);

    void ticCount() {
        component_cost.timer = getTime();
        component_cost.tot_timer = getTime();
        if (RsConfigManager().getTictocAverage()) {
            component_cost.count++;
        }
    }
    void tic(const std::string& sub_name, double& cost) {
        double tmp_cost = (getTime() - component_cost.timer) * 1000.;
        component_cost.timer = getTime();
        RDEBUG << name() << ": " << sub_name << " cost " << tmp_cost << " ms.";
        if (RsConfigManager().getTictocAverage()) {
            cost += tmp_cost;
        }
    }
    void ticAll() {
        double tmp_cost = (getTime() - component_cost.tot_timer) * 1000.;
        RDEBUG << name() << ": tot cost " << tmp_cost << " ms";
        if (RsConfigManager().getTictocAverage()) {
            component_cost.all += tmp_cost;
            component_cost.print(name());
        }
    }
    //=========================================
};
}   // namespace lidar
}   // namespace perception
}   // namespace robosense
#endif  // RS_PERCEPTION_LIDAR_MAP_FILTER_INTERNAL_MOLE_MOLE_MAP_FILTER_H
