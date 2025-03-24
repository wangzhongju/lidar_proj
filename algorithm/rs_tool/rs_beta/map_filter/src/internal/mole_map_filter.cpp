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
#include "rs_perception/lidar/map_filter/internal/mole/mole_map_filter.h"
#include "rs_perception/common/external/rs_util.h"
#include "rs_perception/common/external/basic_type/rs_bbox.h"

namespace robosense {
namespace perception {
namespace lidar {
void MoleMapFilter::init(const RsYamlNode &config_node) {
    params.load(config_node);
    params.log(name());
    evt_res.init(params);
    res_ptr_->filter_obj = params.filter_obj;
    res_ptr_->event_detect = params.event_detect;
}

void MoleMapFilter::perception(const RsLidarFrameMsg::Ptr &msg_ptr) {
    clean();
    if (!params.filter_obj && !params.event_detect) {
        return;
    }
    ticCount();
    msg_ptr->transAxis(AxisStatus::GLOBAL_AXIS, name());
    if (params.filter_obj) {
        objFilter(msg_ptr);
        tic("obj_filter", component_cost.obj_filter);
    }
    if (params.event_detect) {
        eventDetect(msg_ptr);
        tic("evt_detect", component_cost.event_detect);
        eventCollect(msg_ptr);
        tic("evt_collect", component_cost.event_collect);
    }
    ticAll();
}

void MoleMapFilter::objFilter(const RsLidarFrameMsg::Ptr &msg_ptr) {
    const auto& rois_map = params.hd_map_ptr->rois_map;
    const auto& object_ptr_vec = msg_ptr->objects;
    auto& temp_obj_vec = res_ptr_->objects;
    temp_obj_vec.reserve(object_ptr_vec.size());

    for (const auto& obj: object_ptr_vec) {
        const auto& obj_center = obj->core_infos_.center;
        RsBBox box(obj_center, obj->core_infos_.size, obj->core_infos_.direction);
        std::vector<RsVector3f> corners;
        box.corners(corners);
        VecInt tmp_vec(5,0);

        bool keep_flag_check = !(params.hd_map_ptr->keep_flag);
        bool discard_flag_check = !(params.hd_map_ptr->discard_flag);
        for (const auto& item: rois_map) {
            const auto& roi_id = item.first;
            const auto& roi = item.second;
            if (roi->filter_type == FilterType::BOX_DISCARD) {
                if (roi->refer_point == ReferPoint::CENTER) {
                    if (inPolygon(roi->polygon, obj_center)) {
                        discard_flag_check = false;
                        break;
                    }
                    else {
                        discard_flag_check = true;
                    }
                }
                else { // roi->refer_point == ReferPoint::CORNER
                    tmp_vec[0] = 1;
                    for (int k = 1; k < 5; k++) {
                        if (inPolygon(roi->polygon, corners[k-1])) {
                            tmp_vec[k] = -1;
                        }
                    }
                }
            }
            else {  // roi->filter_type == FilterType::BOX_KEEP
                if (roi->refer_point == ReferPoint::CENTER) {
                    if (inPolygon(roi->polygon, obj_center)) {
                        keep_flag_check = true;
                    }
                    else if (keep_flag_check) {
                        continue;
                    }
                    else {
                        keep_flag_check = false;
                    }
                }
                else {  // roi->refer_point == ReferPoint::CORNER
                    tmp_vec[0] = 1;
                    for (int k = 1; k < 5; k++) {
                        if (inPolygon(roi->polygon, corners[k-1]) && tmp_vec[k] != -1) {
                            tmp_vec[k] = 1;
                        }
                    }
                }
            }

            if (tmp_vec[0] == 1) {
                int judge = tmp_vec[1] + tmp_vec[2] + tmp_vec[3] + tmp_vec[4];
                if (judge > 0) {
                    keep_flag_check = true;
                    discard_flag_check = true;
                }
                else if (judge < 0) {
                    discard_flag_check = false;
                }
                else {
                    if (params.hd_map_ptr->discard_flag && !params.hd_map_ptr->keep_flag) {
                        keep_flag_check = true;
                        discard_flag_check = true;
                    }
                    else if (params.hd_map_ptr->keep_flag && !params.hd_map_ptr->discard_flag) {
                        keep_flag_check = false;
                    }
                    else if (params.hd_map_ptr->keep_flag && params.hd_map_ptr->discard_flag) {
//                      keep_flag_check = false;
                        discard_flag_check = keep_flag_check;
                    }
                }
            }
            
            if (keep_flag_check && discard_flag_check) {
                obj->supplement_infos_.in_roi = true;
                obj->supplement_infos_.roi_id = roi_id;
                temp_obj_vec.emplace_back(obj);
                break;
            }

        }
        
    }
}

void MoleMapFilter::eventDetect(const RsLidarFrameMsg::Ptr &msg_ptr) {
    const auto rois_map = params.hd_map_ptr->rois_map;
    const auto& object_ptr_vec = msg_ptr->objects;
    if (rois_map.empty()) {
        RDEBUG << name() <<": roi nums is 0!";
        return;
    }

    evt_res.jam_map.clear();

    for (auto itr = rois_map.begin(); itr != rois_map.end(); itr++) {
        auto& obj_map_ = evt_res.roi_evt_map[itr->first];
        const auto roi = itr->second;
        int jam_num = 0;
        float vel = 0.0;

        std::map<int, EventObject::Ptr> obj_map;

        bool is_statistical = (roi->event_map.find(EventType::Statistical) != roi->event_map.end());
        std::map<int, StaticMsg> vehicle_count_map_;
        std::map<int, RsVector3f> obj_last_map;
        if (is_statistical) {
            vehicle_count_map_ = evt_res.count_map[itr->first];
            if (vehicle_count_map_.find(itr->first) == vehicle_count_map_.end()) {
                vehicle_count_map_[itr->first].vel_num = 0;
            }
            auto& sta_param = roi->event_map[EventType::Statistical];
            for (auto itr_s = sta_param.statistic_map.begin(); itr_s != sta_param.statistic_map.end(); itr_s++) {
                if (vehicle_count_map_.find(itr_s->first) == vehicle_count_map_.end()) {
                    vehicle_count_map_[itr_s->first].vel_num = 0;
                    vehicle_count_map_[itr_s->first].laneIdx = itr_s->second.laneIdx;
                    vehicle_count_map_[itr_s->first].direction = itr_s->second.direction;
                }
            }
        }

        for (auto obj: object_ptr_vec) {
            const auto& obj_center = obj->core_infos_.center;

            if (is_statistical) {
                trafficStatistics(obj->core_infos_.tracker_id, obj_center, roi->polygon[0],
                                  roi->polygon[2], itr->first, obj_last_map, vehicle_count_map_);
                continue;
            }

            if (!inPolygon(roi->polygon, obj_center)) {
                continue;
            }
            EventObject::Ptr event_obj;
            if (obj_map_.find(obj->core_infos_.tracker_id) != obj_map_.end()) {
                event_obj = obj_map_[obj->core_infos_.tracker_id];
                obj_map[obj->core_infos_.tracker_id] = event_obj;
            }
            else {
                event_obj.reset(new EventObject);
                event_obj->track_id = obj->core_infos_.tracker_id;
                obj_map[obj->core_infos_.tracker_id] = event_obj;
            }

            for (auto & iter : roi->event_map) {
                const auto event_name = iter.first; // 管辖的事件名称
                const auto event_attr = iter.second; // 管辖的事件的属性

                RDEBUG << "Event type: " << kEventType2NameMap.at(event_name);
                if (event_name == EventType::LimiteVelocity || event_name == EventType::AbnormalStop) {
                    if (obj->core_infos_.type == ObjectType::CAR || obj->core_infos_.type == ObjectType::TRUCK_BUS) {
                        velocityJudgeEvent(obj->core_infos_.tracker_id, obj->core_infos_.velocity,
                                           event_name, event_attr,event_obj);
                    }
                }
                else if (event_name == EventType::VehicleReverse) {
                    if (obj->core_infos_.type == ObjectType::CAR || obj->core_infos_.type == ObjectType::TRUCK_BUS) {
                        directionJudgeEvent(event_attr.directionPoints, obj->core_infos_.velocity,
                                            event_attr.degreeTh, event_obj->VehicleReverse, obj->core_infos_.tracker_id);
                        RTRACE << "obj id: " << obj->core_infos_.tracker_id <<
                        " VehicleReverse: " <<  event_obj->VehicleReverse;
                    }
                }
                else if (event_name == EventType::AbnormalOccupy || event_name == EventType::PedestrianInvade
                        || event_name == EventType::UnknownLoss || event_name == EventType::PedestrianCross) {

                    if (event_name == EventType::PedestrianInvade) {
                        invadeJudgeEvent(event_obj->timeTh_invade, event_attr.timeTh,event_obj->PedestrianInvade);
                        RTRACE << "obj id: " << obj->core_infos_.tracker_id <<
                        " PedestrianInvade: " <<  event_obj->PedestrianInvade;
                    }
                    else if (event_name == EventType::AbnormalOccupy
                    && (obj->core_infos_.type == ObjectType::CAR || obj->core_infos_.type == ObjectType::TRUCK_BUS)) {
                        invadeJudgeEvent(event_obj->timeTh_occupy, event_attr.timeTh, event_obj->AbnormalOccupy);
                        RTRACE << "obj id: " << obj->core_infos_.tracker_id <<
                        " AbnormalOccupy: " <<  event_obj->AbnormalOccupy;
                    }
                    else if (event_name == EventType::UnknownLoss && obj->core_infos_.type == ObjectType::UNKNOW) {
                        invadeJudgeEvent(event_obj->timeTh_loss, event_attr.timeTh, event_obj->UnknownLoss);
                        RTRACE << "obj id: " << obj->core_infos_.tracker_id <<
                        " UnknownLoss: " <<  event_obj->UnknownLoss;
                    }
                    else if (event_name == EventType::PedestrianCross && obj->core_infos_.type == ObjectType::PED) {
                        invadeJudgeEvent(event_obj->timeTh_cross, event_attr.timeTh, event_obj->PedestrianCross);
                        RTRACE << "obj id: " << obj->core_infos_.tracker_id <<
                        " PedestrianCross: " << event_obj->PedestrianCross;
                    }
                }
                else if (event_name == EventType::Jam) {
                    if (obj->core_infos_.type == ObjectType::CAR || obj->core_infos_.type == ObjectType::TRUCK_BUS) {
                        vel += obj->core_infos_.velocity.norm();
                        jam_num++;
                    }
                }
                else { // even_name == EventType::AbnormalSwitch || even_name == EventType::Statistical
                    continue;
                }
            }
        }

        if (is_statistical) {
            evt_res.obj_last_map = obj_last_map;
            evt_res.count_map[itr->first] = vehicle_count_map_;
        }
        else {
            obj_map_ = obj_map;
            // 拥堵事件
            if (jam_num != 0 && jam_num >= roi->event_map[EventType::Jam].vehicleNumTh) {
                if (static_cast<float >(vel / jam_num) <
                roi->event_map[EventType::Jam].velocityTh * roi->event_map[EventType::Jam].percentTh) {
                    // 交通拥堵
                    evt_res.jam_map[roi->id] = true;
                }
            }
        }
    }

    auto& switch_map_ = evt_res.switch_map;
    std::map<int, EventObject::Ptr> switch_map;
    int cache_obj_num = 0;
    for (auto obj : object_ptr_vec) {
        cache_obj_num++;
        if (obj->core_infos_.type != ObjectType::CAR && obj->core_infos_.type != ObjectType::TRUCK_BUS) {
            continue;
        }
        const auto& center = obj->core_infos_.center;
        EventObject::Ptr event_obj;
        if (switch_map_.find(obj->core_infos_.tracker_id) != switch_map_.end()) {
            event_obj = switch_map_[obj->core_infos_.tracker_id];
        }
        else {
            event_obj.reset(new EventObject);
        }
        for (auto itr = rois_map.begin(); itr != rois_map.end(); itr++) {
            const auto roi = itr->second;
            if (inPolygon(roi->polygon, center)) {
                event_obj->time_miss = 0.0;
                if (roi->event_map.find(EventType::AbnormalSwitch) != roi->event_map.end()) {
                    RDEBUG << "Event type: " << kEventType2NameMap.at(EventType::AbnormalSwitch);
                    switchJudgeEvent(obj->core_infos_.tracker_id, event_obj, roi);
                }
            }
            else {
                event_obj->time_miss += 0.1;
            }
        }
        if (event_obj->time_miss < 1.5) {
            switch_map[obj->core_infos_.tracker_id] = event_obj;
        }
    }
    switch_map_ = switch_map;
}

void MoleMapFilter::eventCollect(const RsLidarFrameMsg::Ptr &msg_ptr) {
    const auto& rois_map = params.hd_map_ptr->rois_map;
    auto& map_msp_ = res_ptr_;

    for (size_t i = 0; i < msg_ptr->objects.size(); ++i) {
        const auto obj = msg_ptr->objects[i];
        // 常规事件,与roi和obj相关
        std::vector<EventType> tmp_evt_vec;
        for (auto itr = rois_map.begin(); itr != rois_map.end(); itr++) {
            auto obj_map = evt_res.roi_evt_map.at(itr->first);
            if (obj_map.find(obj->core_infos_.tracker_id) != obj_map.end()) {
                auto obj_evt = obj_map[obj->core_infos_.tracker_id];
                if (obj_evt->LimiteVel_Up) {
                    tmp_evt_vec.emplace_back(EventType::LimiteVel_Up);
                }
                if (obj_evt->LimiteVel_Bot) {
                    tmp_evt_vec.emplace_back(EventType::LimiteVel_Bot);
                }
                if (obj_evt->AbnormalOccupy) {
                    tmp_evt_vec.emplace_back(EventType::AbnormalOccupy);
                }
                if (obj_evt->AbnormalStop) {
                    tmp_evt_vec.emplace_back(EventType::AbnormalStop);
                }
                if (obj_evt->VehicleReverse) {
                    tmp_evt_vec.emplace_back(EventType::VehicleReverse);
                }
                if (obj_evt->PedestrianCross) {
                    tmp_evt_vec.emplace_back(EventType::PedestrianCross);
                }
                if (obj_evt->PedestrianInvade) {
                    tmp_evt_vec.emplace_back(EventType::PedestrianInvade);
                }
                if (obj_evt->UnknownLoss) {
                    tmp_evt_vec.emplace_back(EventType::UnknownLoss);
                }
            }
        }
        map_msp_->obj_evt_map[obj->core_infos_.tracker_id] = tmp_evt_vec;
        // 异常变道 只与目标相关
        if (evt_res.switch_map.find(obj->core_infos_.tracker_id) != evt_res.switch_map.end()) {
            if (evt_res.switch_map[obj->core_infos_.tracker_id]->AbnormalSwitch) {
                map_msp_->obj_evt_map[obj->core_infos_.tracker_id].emplace_back(EventType::AbnormalSwitch);
            }
            if (evt_res.switch_map[obj->core_infos_.tracker_id]->AbnormalSwitchTwice) {
                map_msp_->obj_evt_map[obj->core_infos_.tracker_id].emplace_back(EventType::AbnormalSwitchTwice);
            }
        }
    }

    // 堵塞事件,与目标无关
    map_msp_->jam_map = evt_res.jam_map;

    // 收集规定时间内统计结果
    if (!evt_res.timer) {
        evt_res.timer = true;
        for (auto itr = rois_map.begin(); itr != rois_map.end(); itr++) {
            if (itr->second->event_map.find(EventType::Statistical) == itr->second->event_map.end()) {
                continue;
            }
            if (evt_res.count_time.find(itr->first) == evt_res.count_time.end()) {
                evt_res.count_time[itr->first] = msg_ptr->timestamp;
            }
//            evt_res.count_time[itr->first] = msg_ptr->timestamp;
            // 记录流量统计的开始时间
            auto& tmp_roi = evt_res.count_map[itr->first];
            auto& statisticRoi = tmp_roi[itr->first];
            statisticRoi.start_time = msg_ptr->timestamp;
        }
    }
    for (auto itr = rois_map.begin(); itr != rois_map.end(); itr++) {
        if (itr->second->event_map.find(EventType::Statistical) == itr->second->event_map.end()) {
            continue;
        }
        if (static_cast<double> (msg_ptr->timestamp - evt_res.count_time.at(itr->first)) >
            itr->second->event_map.at(EventType::Statistical).timeTh) {
            // 记录流量统计的结束时间
            auto& tmp_roi = evt_res.count_map[itr->first];
            auto& statisticRoi = tmp_roi[itr->first];
            statisticRoi.end_time = msg_ptr->timestamp;

            map_msp_->count_map[itr->first] = evt_res.count_map.at(itr->first);
            evt_res.count_time.at(itr->first) = msg_ptr->timestamp;
            evt_res.count_map.at(itr->first).clear();
            evt_res.timer = false;
        }
    }
}
void MoleMapFilter::getResult(Any::Ptr &any) {
    any.reset(new Any(*res_ptr_));
}

void MoleMapFilter::velocityJudgeEvent(int id, const RsVector3f &vel, const EventType &event_name,
                                       const EventParams &event_attr, EventObject::Ptr &event_obj) {
    switch (event_name) {
        case EventType::LimiteVelocity:{
            if (vel.norm() < event_attr.velocityThBottom) {
                // 报低速
                event_obj->timeThBottom += 0.1;
                if (event_obj->timeThBottom > event_attr.timeThBottom && event_attr.velType != 0) {
                    event_obj->LimiteVel_Bot = true;
                }
            }
            else if (vel.norm() > event_attr.velocityThUpper) {
                // 报超速
                event_obj->timeThUpper += 0.1;
                if (event_obj->timeThUpper > event_attr.timeThUpper && event_attr.velType != 1) {
                    event_obj->LimiteVel_Up = true;
                }
            }
            else {
                event_obj->timeThUpper = 0;
                event_obj->timeThBottom = 0;
                event_obj->LimiteVel_Up = false;
                event_obj->LimiteVel_Bot = false;
            }
            RTRACE << "obj id: " << id << " LimiteVel_Bot: " <<  event_obj->LimiteVel_Bot;
            RTRACE << "obj id: " << id << " LimiteVel_Up: " <<  event_obj->LimiteVel_Up;
            break;
        }
        case EventType::AbnormalStop: {
            if (vel.norm() * 3.6 < event_attr.velocityTh) {
                event_obj->timeTh_stop += 0.1;
                if (event_obj->timeTh_stop > event_attr.timeTh) {
                    event_obj->AbnormalStop = true;
                }
            }
            else {
                event_obj->timeTh_stop = 0;
                event_obj->AbnormalStop = false;
            }
            RTRACE << "obj id: " << id << " AbnormalStop: " <<  event_obj->AbnormalStop;
            break;
        }
        default:
            break;
    }
}

void MoleMapFilter::directionJudgeEvent(const std::vector<RsVector3f> &dir_points, const RsVector3f &vel,
                                        const float &threshold, bool &res, int& id) {
    const auto size_ = dir_points.size();
    float vel_norm = vel.norm();
    auto x_d = dir_points[size_-1].x - dir_points[0].x;
    auto y_d = dir_points[size_-1].y - dir_points[0].y;
//    float ang_d = std::abs(std::atan2(y_d,x_d) - std::atan2(vel.y, vel.x));
    auto fac1 = x_d * vel.x + y_d * vel.y;
    auto fac2 = sqrt((x_d * x_d + y_d * y_d) * (vel.x * vel.x + vel.y * vel.y));
    double ang_d = acos(fac1 / fac2);

    if (vel_norm >= 2 && ang_d > threshold) {
        res = true;
    }
    else {
        res = false;
    }
//    if (res) {
//        RINFO << "ID: " << id << " vel.x = " << vel.x << " vel.y = " << vel.y << " arccos = " << ang_d << " threshold = " << threshold;
//    }

}

void MoleMapFilter::invadeJudgeEvent(double &invade_time, const float &threshold, bool &res) {
    invade_time += 0.1;
    if (invade_time > threshold) {
        res = true;
    }
    else {
        res = false;
    }
}

void MoleMapFilter::trafficStatistics(int id, const RsVector3f &center, RsVector3f &p_1, RsVector3f &p_2, int section_id,
                                      std::map<int, RsVector3f> &obj_last_map,
                                      std::map<int, StaticMsg> &vehicle_count_map_) {

    auto theta1 = static_cast<float> (angCalculate(center, p_1) + RS_M_PI);
    auto theta2 = static_cast<float> (angCalculate(p_2, p_1) + RS_M_PI);
    auto theta3 = static_cast<float> (angCalculate(center, p_2) + RS_M_PI);
    auto theta4 = static_cast<float> (angCalculate(p_1, p_2) + RS_M_PI);

    if (abs(theta1- theta2) * 2 > RS_M_PI || abs(theta3- theta4) * 2 > RS_M_PI) {
        return;
    }

    if (evt_res.obj_last_map.find(id) == evt_res.obj_last_map.end()) {
        obj_last_map[id] = center;
        return;
    }

    auto center_last = evt_res.obj_last_map[id];
    if (p_1.x == p_2.x) {
        p_1.x += 0.01;
    }

    float delta_y1 = center_last.y - p_1.y - (p_2.y - p_1.y)*(center_last.x - p_1.x)/(p_2.x - p_1.x);
    float delta_y2 = center.y - p_1.y - (p_2.y - p_1.y)*(center.x - p_1.x)/(p_2.x - p_1.x);

    const auto rois_map = params.hd_map_ptr->rois_map;
    auto sta_map = rois_map.at(section_id)->event_map[EventType::Statistical].statistic_map;
    if (delta_y1 * delta_y2 <= 0) {
        for (auto itr = sta_map.begin(); itr != sta_map.end(); itr++) {
            if (inPolygon(rois_map.at(itr->first)->polygon, center)) {
                vehicle_count_map_[section_id].vel_num++;// 截面计数
                vehicle_count_map_[itr->first].vel_num++;// lane计数
            }
        }
    }
    obj_last_map[id] = center;
}

void MoleMapFilter::switchJudgeEvent(int id, EventObject::Ptr &event_obj, const RsROI::Ptr &roi) {
    if (event_obj->roi_id.empty()) {
        event_obj->roi_id.push(roi->id);
        event_obj->timeTh_switch = 0;
    }
    else {
        if (roi->id != event_obj->roi_id.back()) { // 已经发生变道：事件包含关系：连续变道可包含实线变道，反之不一定。
            // 先判断是不是连续变道
            if (event_obj->roi_id.size() > 1) {
                if (event_obj->timeTh_switch < roi->event_map[EventType::AbnormalSwitch].timeTh) {
                    event_obj->AbnormalSwitchTwice = true; // 改为单独输出连续变道
                }
                else {
                    event_obj->AbnormalSwitchTwice = false;
                }
                event_obj->roi_id.push(roi->id);
                event_obj->roi_id.pop();
            }
            else {
                event_obj->roi_id.push(roi->id);
            }
            event_obj->timeTh_switch = 0;

            // 再判断是不是跨实线
            if (roi->event_map[EventType::AbnormalSwitch].neighborLaneType[event_obj->roi_id.front()] == 0) {
                event_obj->AbnormalSwitch = true;
            }
        }
        else {
            event_obj->timeTh_switch += 0.1;
        }
        RTRACE << "obj id: " << id << " AbnormalSwitch: " << event_obj->AbnormalSwitch;
        RTRACE << "obj id: " << id << " AbnormalSwitch: " << event_obj->AbnormalSwitchTwice;
    }
}

RS_REGISTER_MAP_FILTER(MoleMapFilter);
}   // namespace lidar
}   // namespace perception
}   // namespace robosense
