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

#include "rs_perception/custom/robosense_custom/xingyunCustom/xingyun_custom_transformer.h"

#ifdef RS_PROTO_FOUND

namespace robosense {
namespace perception {
namespace Xingyun {

// ============================================================================================ //
// ====================== 感知结果转换为定制的结果形式（实现的是协议中2011和2012） ==================== //
// ============================================================================================ //

void EventConvert::convert(const RsPerceptionMsg::Ptr &msg_ptr, Xingyun::EventMsg::Ptr& result_ptr, const bool& hd_map_disable) {
    const auto& msg_ptr_ = msg_ptr->rs_lidar_result_ptr;
    result_ptr->obj_send_ptr.reset(new ObjectSend);
    auto& obj_s_ptr = result_ptr->obj_send_ptr;
    obj_s_ptr->MsgType = 2011;
    obj_s_ptr->Timestamp = RSXingyunCommUtil::Timestamp2time1(msg_ptr_->timestamp);
    ObjectList obj_list;
    tmp_obj_id_map.clear();
    for (size_t i = 0; i < msg_ptr_->objects.size(); ++i) {
        const auto obj = msg_ptr_->objects[i];
        if (obj_id_map.find(obj->core_infos_.tracker_id) != obj_id_map.end()) {
            obj_list.ID = obj_id_map[obj->core_infos_.tracker_id].object_id;
            obj_id_map[obj->core_infos_.tracker_id].time_loss = 0;
        }
        else {
            obj_list.ID = RSXingyunCommUtil::Timestamp2time2(msg_ptr_->timestamp) +"#" + transTostring(lunzhuan_num);
            obj_id_map[obj->core_infos_.tracker_id].object_id = obj_list.ID;
            obj_id_map[obj->core_infos_.tracker_id].time_loss = 0;
            lunzhuan_num++;
        }
        if (lunzhuan_num > 999) {
            lunzhuan_num = 0;
        }
        tmp_obj_id_map[obj->core_infos_.tracker_id] = obj_list.ID;
        if (obj->core_infos_.type == ObjectType::CAR || obj->core_infos_.type == ObjectType::TRUCK_BUS || obj->core_infos_.type == ObjectType::ULTRA_VEHICLE) {
            /*
            if(obj->core_infos_.type == ObjectType::CAR){
                obj_list.PtcType = 3;
            } else if (obj->core_infos_.type == ObjectType::TRUCK_BUS || obj->core_infos_.type == ObjectType::ULTRA_VEHICLE) {
                obj_list.PtcType = 4;
            }
            */
            obj_list.PtcType = 1;
        } else if (obj->core_infos_.type == ObjectType::BIC) {
            obj_list.PtcType = 2;
        } else if (obj->core_infos_.type == ObjectType::PED) {
            obj_list.PtcType = 3;
        } else {
            obj_list.PtcType = 0;
        }
        obj_list.VehH = static_cast<double>(obj->core_infos_.size.z);
        obj_list.VehL = static_cast<double>(obj->core_infos_.size.x);
        obj_list.VehW = static_cast<double>(obj->core_infos_.size.y);
        obj_list.PtcID = obj->core_infos_.tracker_id;
        obj_list.PtcLon = obj->supplement_infos_.gps_longtitude;
        obj_list.PtcLat = obj->supplement_infos_.gps_latitude;
        obj_list.PtcEle = obj->supplement_infos_.gps_altitude;
        obj_list.XPos = obj->core_infos_.center.x;
        obj_list.YPos = obj->core_infos_.center.y;
        obj_list.PtcSpeed = static_cast<double>(obj->core_infos_.velocity.norm() * 3.6);
        float eastYaw = atan2(obj->core_infos_.velocity.y, obj->core_infos_.velocity.x)/RS_M_PI *180;
        obj_list.PtcHeading = RSXingyunCommUtil::eastYaw2NorthYaw(eastYaw);
        obj_list.VehType = 0;
        obj_s_ptr->Obj_List.emplace_back(obj_list);
    }
    // 历史帧中已经消失的目标清除
    for(auto itr = obj_id_map.begin(); itr != obj_id_map.end();) {
        if (tmp_obj_id_map.find(itr->first) != tmp_obj_id_map.end()) {
            itr++;
            continue;
        }
        else {
            if (itr->second.time_loss > loss_threshold) {
                itr = obj_id_map.erase(itr);
            }
            else {
                itr->second.time_loss += 1;
                itr++;
            }
        }
    }

    // event
    if (hd_map_disable){
        RDEBUG << "74 hd_map is turned off!";
        return;
    }

    Any::Ptr any_ptr_event_detect = msg_ptr_->any_map.at("event_detect");
    auto event_detect = any_ptr_event_detect->AnyCast<bool>();
    result_ptr->event_detect = *event_detect;
    if (!result_ptr->event_detect){
        return;
    }

    result_ptr->event_send_ptr.reset(new EventSend);
    auto& evt_s_ptr = result_ptr->event_send_ptr;
    evt_s_ptr->MsgType = 2012;
    evt_s_ptr->Timestamp = RSXingyunCommUtil::Timestamp2time1(msg_ptr_->timestamp);

    Any::Ptr any_ptr_obj_evt_map = msg_ptr_->any_map.at("obj_evt_map");
    Any::Ptr any_ptr_jam_map = msg_ptr_->any_map.at("jam_map");
//    Any::Ptr any_ptr_count_map = msg_ptr_->any_map.at("count_map");

    auto jam_map = any_ptr_jam_map->AnyCast<std::map<int, bool>>();
    auto obj_evt_map = any_ptr_obj_evt_map->AnyCast<std::map<int, std::vector<lidar::EventType>>>();
//    auto count_map = any_ptr_count_map->AnyCast<std::map<int, std::map<int, lidar::StaticMsg>>>();


    // 流量统计与具体物体无关，单独统计
//    for (auto& itr: *count_map) {
//        EventList evt_list;
//        evt_list.EvtType = 998;
//        evt_list.statisticRoiIdx = itr.first;
//        for (auto& itrs: itr.second) {
//            evt_list.roiIdx = itrs.first;
//            evt_list.laneIdx = itrs.second.laneIdx;
//            evt_list.vel_num = itrs.second.vel_num;
//            evt_list.direction = itrs.second.direction;
//            evt_s_ptr->Evt_List.emplace_back(evt_list);
//            RTRACE << evt_list.roiIdx <<" " << evt_list.laneIdx << " " << evt_list.vel_num << " " << evt_list.direction;
//        }
//    }

    // 交通堵塞统计
    tmp_Jam_id_map.clear();
    for (auto & itr : *jam_map) {
        EventList evt_list;
        if (Jam_id_map.find(itr.first) != Jam_id_map.end()) {
            evt_list.ID = Jam_id_map[itr.first];
            evt_list.EvtStatus = 1;
        }
        else {
            evt_list.ID = RSXingyunCommUtil::Timestamp2time2(msg_ptr_->timestamp) +"#" + transTostring(lunzhuan_evt_num);
            evt_list.EvtStatus = 0;
            Jam_id_map[itr.first] = evt_list.ID;
            lunzhuan_evt_num++;
        }
        if (lunzhuan_evt_num > 999) {
            lunzhuan_evt_num = 0;
        }
        tmp_Jam_id_map[itr.first] = evt_list.ID;
        evt_list.EvtType = 2;
        evt_list.roiIdx = itr.first;
        // 存储历史状态
        if (jam_his_status_map.find(itr.first) != jam_his_status_map.end()) {
            jam_his_status_map.at(itr.first) = evt_list;
        }
        else {
            jam_his_status_map[itr.first] = evt_list;
        }
        evt_s_ptr->Evt_List.emplace_back(evt_list);
    }
    // 删除结束状态的事件
    if (!jam_his_status_map.empty()) {
        for (auto itr = jam_his_status_map.begin(); itr != jam_his_status_map.end(); ) {
            if (tmp_Jam_id_map.find(itr->first) != tmp_Jam_id_map.end()) {
                itr++;
                continue;
            } else {
                auto& evt_list_ = jam_his_status_map.at(itr->first);
                evt_list_.EvtStatus = 2;
                evt_s_ptr->Evt_List.emplace_back(evt_list_);
                itr = jam_his_status_map.erase(itr);
            }
        }
    }

    // 其他事件统计
    tmp_evt_id_map.clear();
    for (auto obj : msg_ptr_->objects) {
        if (obj_evt_map->find(obj->core_infos_.tracker_id) != obj_evt_map->end()) {
            auto obj_evt_vec = obj_evt_map->at(obj->core_infos_.tracker_id);
            EventList evt_list;
            evt_list.ObjId = obj_id_map.at(obj->core_infos_.tracker_id).object_id;
            evt_list.tracker_id = obj->core_infos_.tracker_id;
            evt_list.Lon = obj->supplement_infos_.gps_longtitude;
            evt_list.Lat = obj->supplement_infos_.gps_latitude;
            evt_list.Ele = obj->supplement_infos_.gps_altitude;
            evt_list.VehL = obj->core_infos_.size.x;
            evt_list.VehW = obj->core_infos_.size.y;
            evt_list.VehH = obj->core_infos_.size.z;

            for (auto evt_type : obj_evt_vec) {
                int type = kEvtType2TypeIDMap.at(evt_type);
                evt_list.EvtType = type;
                std::string obj_evt_id = std::to_string(obj->core_infos_.tracker_id)+ std::to_string(type);
                if (evt_id_map.find(obj_evt_id) != evt_id_map.end()) {
                    evt_list.ID = evt_id_map[obj_evt_id].event_id;
                    evt_id_map[obj_evt_id].time_loss = 0;
                    if (evt_his_status_map.find(obj_evt_id) != evt_his_status_map.end()) {
                        evt_list.EvtStatus = 1;
                    }
                    else {
                        evt_list.EvtStatus = 0;
                    }
                }
                else {
                    evt_list.ID = RSXingyunCommUtil::Timestamp2time2(msg_ptr_->timestamp) +"#" + transTostring(lunzhuan_evt_num);
                    evt_list.EvtStatus = 0;
                    evt_id_map[obj_evt_id].event_id = evt_list.ID;
                    evt_id_map[obj_evt_id].time_loss = 0;
                    lunzhuan_evt_num++;
                }
                if (lunzhuan_evt_num > 999) {
                    lunzhuan_evt_num = 0;
                }
                tmp_evt_id_map[obj_evt_id] = evt_list.ID;
                if (evt_his_status_map.find(obj_evt_id) != evt_his_status_map.end()) {
                    evt_his_status_map.at(obj_evt_id) = evt_list;
                }
                else {
                    evt_his_status_map[obj_evt_id] = evt_list;
                }

                evt_s_ptr->Evt_List.emplace_back(evt_list);
                RTRACE <<"id " << evt_list.ObjId << " tracker_id "
                << evt_list.tracker_id <<" evt type " << lidar::kEventType2NameMap.at(evt_type);
            }
        }
    }


    if (evt_his_status_map.empty()) {
        for (auto itr = evt_id_map.begin(); itr != evt_id_map.end();) {
            if (tmp_evt_id_map.find(itr->first) != tmp_evt_id_map.end()) {
                itr++;
                continue;
            }
            else {
                if (itr->second.time_loss > loss_threshold) {
                    itr = evt_id_map.erase(itr);
                }
                else {
                    itr->second.time_loss += 1;
                    itr++;
                }
            }
        }
        return;
    }
    // 历史帧中已经消失的事件清除
    for(auto itr = evt_his_status_map.begin(); itr != evt_his_status_map.end();) {
        if (tmp_evt_id_map.find(itr->first) != tmp_evt_id_map.end()) {
            itr++;
            continue;
        }
        else {
            auto& evt_list_ = evt_his_status_map.at(itr->first);
            evt_list_.EvtStatus = 2;
            evt_s_ptr->Evt_List.emplace_back(evt_list_);
            itr = evt_his_status_map.erase(itr);
        }
    }

    // 总事件map中对发生完的事件进行计时
    for (auto itr = evt_id_map.begin(); itr != evt_id_map.end();) {
        if (tmp_evt_id_map.find(itr->first) != tmp_evt_id_map.end()) {
            itr++;
            continue;
        }
        else {
            if (itr->second.time_loss > loss_threshold) {
                itr = evt_id_map.erase(itr);
            }
            else {
                itr->second.time_loss += 1;
                itr++;
            }
        }
    }
}

int EventConvert::convertToProto(char* data,Xingyun::EventMsg::Ptr& result_ptr,RsXingyunCustomMsgParams& custom_params_,int frame_id_,const RsPerceptionMsg::Ptr &msg_ptr) {

    nebulalink::perceptron3::PerceptronSet temp_perceptronset;

    temp_perceptronset.set_devide_id(std::to_string(custom_params_.device_id));
    temp_perceptronset.set_time_stamp(std::floor(msg_ptr->rs_lidar_result_ptr->timestamp*1000));
    temp_perceptronset.set_number_frame(frame_id_);

    nebulalink::perceptron3::PointGPS *temp_perception_gps_ptr = temp_perceptronset.mutable_perception_gps();   
    temp_perception_gps_ptr->set_object_longitude(msg_ptr->rs_lidar_result_ptr->gps_origin.x);
    temp_perception_gps_ptr->set_object_latitude(msg_ptr->rs_lidar_result_ptr->gps_origin.y);
    temp_perception_gps_ptr->set_object_elevation(msg_ptr->rs_lidar_result_ptr->gps_origin.z);

    Xingyun::ObjectSend info = *(result_ptr->obj_send_ptr);

    int obj_num = 0;

    for (int i = 0; i<info.Obj_List.size();i++) {
        nebulalink::perceptron3::Perceptron *proto_perceptron_ptr = temp_perceptronset.add_perceptron();
        proto_perceptron_ptr->set_object_class_type(info.Obj_List[i].PtcType);
        proto_perceptron_ptr->set_object_speed(info.Obj_List[i].PtcSpeed);
        proto_perceptron_ptr->set_object_direction(info.Obj_List[i].PtcHeading);
        
        nebulalink::perceptron3::Point3 *proto_point3f_ptr = proto_perceptron_ptr->mutable_point3f();
        proto_point3f_ptr->set_x(info.Obj_List[i].XPos);
        proto_point3f_ptr->set_y(info.Obj_List[i].YPos);
        proto_point3f_ptr->set_z(0.0);

        nebulalink::perceptron3::TargetSize *proto_target_size_ptr = proto_perceptron_ptr->mutable_target_size();
        proto_target_size_ptr->set_object_width(info.Obj_List[i].VehW);
        proto_target_size_ptr->set_object_length(info.Obj_List[i].VehL);
        proto_target_size_ptr->set_object_height(info.Obj_List[i].VehH);

        nebulalink::perceptron3::PointGPS *proto_point_gps_ptr = proto_perceptron_ptr->mutable_point_gps();
        proto_point_gps_ptr->set_object_longitude(info.Obj_List[i].PtcLon);
        proto_point_gps_ptr->set_object_latitude(info.Obj_List[i].PtcLat);
        proto_point_gps_ptr->set_object_elevation(info.Obj_List[i].PtcEle);

        obj_num++;
    }
    
    std::cout<<"obj_num:"<<obj_num<<std::endl;

    Xingyun::EventSend info_event = *(result_ptr->event_send_ptr);

    for (int i = 0;i<info_event.Evt_List.size();i++) {
        nebulalink::perceptron3::LaneJamSenseParams *proto_lane_jam_sense_params_ptr = temp_perceptronset.add_lane_jam_sense_params();
        proto_lane_jam_sense_params_ptr->set_lane_id(std::to_string(info_event.Evt_List[i].roiIdx));
        proto_lane_jam_sense_params_ptr->set_lane_veh_num(info_event.Evt_List[i].vel_num);
    }

    temp_perceptronset.SerializeToArray(data, temp_perceptronset.ByteSize());

    std::string proto_string;
    temp_perceptronset.SerializeToString(&proto_string);

   std::cout<<"content:"<<proto_string<<std::endl;

    return  proto_string.length();
}

}
}
}
#endif  // RS_PROTO_FOUND
