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

#include "rs_perception/custom/robosense_custom/dtghJson/dtgh_custom_transformer.h"

namespace robosense {
namespace perception {
namespace Dtgh {

// ============================================================================================ //
// ====================== 感知结果转换为定制的结果形式（实现的是协议中2011和2012） ==================== //
// ============================================================================================ //

void EventConvert::convert(const RsPerceptionMsg::Ptr &msg_ptr, Dtgh::EventMsg::Ptr& result_ptr, const bool& hd_map_disable) {
    const auto& msg_ptr_ = msg_ptr->rs_lidar_result_ptr;
    result_ptr->obj_send_ptr.reset(new ObjectSend);
    auto& obj_s_ptr = result_ptr->obj_send_ptr;
    obj_s_ptr->MsgType = 2011;
    obj_s_ptr->Timestamp = RSDtghCommUtil::Timestamp2time1(msg_ptr_->timestamp);
    ObjectList obj_list;
    tmp_obj_id_map.clear();
    for (size_t i = 0; i < msg_ptr_->objects.size(); ++i) {
        const auto obj = msg_ptr_->objects[i];
        if (obj_id_map.find(obj->core_infos_.tracker_id) != obj_id_map.end()) {
            obj_list.ID = obj_id_map[obj->core_infos_.tracker_id].object_id;
            obj_id_map[obj->core_infos_.tracker_id].time_loss = 0;
        }
        else {
            obj_list.ID = RSDtghCommUtil::Timestamp2time2(msg_ptr_->timestamp) +"#" + transTostring(lunzhuan_num);
            obj_id_map[obj->core_infos_.tracker_id].object_id = obj_list.ID;
            obj_id_map[obj->core_infos_.tracker_id].time_loss = 0;
            lunzhuan_num++;
        }
        if (lunzhuan_num > 999) {
            lunzhuan_num = 0;
        }
        tmp_obj_id_map[obj->core_infos_.tracker_id] = obj_list.ID;
        if (obj->core_infos_.type == ObjectType::CAR || obj->core_infos_.type == ObjectType::TRUCK_BUS || obj->core_infos_.type == ObjectType::ULTRA_VEHICLE) {
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
        obj_list.PtcLon = obj->supplement_infos_.gps_longtitude;
        obj_list.PtcLat = obj->supplement_infos_.gps_latitude;
        obj_list.PtcEle = obj->supplement_infos_.gps_altitude;
        obj_list.XPos = obj->core_infos_.center.x;
        obj_list.YPos = obj->core_infos_.center.y;
        obj_list.PtcSpeed = static_cast<double>(obj->core_infos_.velocity.norm() * 3.6);
        float eastYaw = atan2(obj->core_infos_.velocity.y, obj->core_infos_.velocity.x)/RS_M_PI *180;
        obj_list.PtcHeading = RSDtghCommUtil::eastYaw2NorthYaw(eastYaw);
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
    evt_s_ptr->Timestamp = RSDtghCommUtil::Timestamp2time1(msg_ptr_->timestamp);

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
            evt_list.ID = RSDtghCommUtil::Timestamp2time2(msg_ptr_->timestamp) +"#" + transTostring(lunzhuan_evt_num);
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
                    evt_list.ID = RSDtghCommUtil::Timestamp2time2(msg_ptr_->timestamp) +"#" + transTostring(lunzhuan_evt_num);
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


// ====================================================================== //
// ====================== 获取设备信息（针对5002的工具）===================== //
// ====================================================================== //
int RSDeviceStatusUtil::getDeviceStatusInfo(st_HardwareInfo &hardwareInfo) {
    int ret;
    ret = getCpuUseInfo(hardwareInfo);
    if (ret != 0) {
        return -1;
    }

    ret = getMemoryUseInfo(hardwareInfo);
    if (ret != 0) {
        return -2;
    }

    ret = getDiskUseInfo(hardwareInfo);
    if (ret != 0) {
        return -3;
    }

    ret = getDeviceTempture(hardwareInfo);
    if (ret != 0) {
        return -4;
    }

    ret = getGpuUseInfo(hardwareInfo);
    if (ret != 0) {
        return -5;
    }

    return 0;
}

int RSDeviceStatusUtil::getCpuUseInfo(st_HardwareInfo &hardwareInfo) {
    const std::string filePath = "/tmp/.proc_stat_xxxxxxxx.txt";
    const std::string command = "cat /proc/stat > " + filePath;
    std::ifstream ifstr;
    int ret;
    ret = system(command.c_str());
    if (ret == -1) {
        return -1;
    }

    ifstr.open(filePath, std::ios_base::in | std::ios_base::binary);
    if (ifstr.is_open() == false) {
        return -2;
    }

    std::string lineContent1, lineContent2;
    while (std::getline(ifstr, lineContent1)) {
        if (lineContent1.size() > 0) {
            if (lineContent1.substr(0, 4) == std::string("cpu ")) {
                break;
            }
            else {
                lineContent1.clear();
            }
        }
    }
    ifstr.close();

    // std::cout << "cpu info start: " << lineContent1 << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    ret = system(command.c_str());
    if (ret == -1) {
        return -3;
    }

    ifstr.open(filePath, std::ios_base::in | std::ios_base::binary);
    if (ifstr.is_open() == false) {
        return -4;
    }

    while (std::getline(ifstr, lineContent2)) {
        if (lineContent2.size() > 0) {
            if (lineContent2.substr(0, 4) == std::string("cpu ")) {
                break;
            }
            else {
                lineContent2.clear();
            }
        }
    }
    ifstr.close();

    // std::cout << "cpu info end: " << lineContent2 << std::endl;

    if (lineContent1.size() > 0 && lineContent2.size() > 0) {
        std::vector<std::string> elements1, elements2;
        splitString(lineContent1, elements1, ' ', false);
        splitString(lineContent2, elements2, ' ', false);
        if (elements1.size() != elements2.size() || elements1.size() != 11) {
            return -5;
        }

        float totalTime1 = 0.0;
        float totalTime2 = 0.0;

        float totalIdealTime1 = RSDtghCommUtil::str2num<int>(elements1[4]);
        float totalIdealTime2 = RSDtghCommUtil::str2num<int>(elements2[4]);

        for (size_t i = 1; i < 11; ++i) {
            totalTime1 += RSDtghCommUtil::str2num<int>(elements1[i]);
            totalTime2 += RSDtghCommUtil::str2num<int>(elements2[i]);
        }

        hardwareInfo.CpuRate = RSDtghCommUtil::num2str<float>(((totalTime2 - totalTime1) - (totalIdealTime2 - totalIdealTime1)) / (totalTime2 - totalTime1) * 100, 2);
        // std::cout << "hardwareInfo.CpuRate = " << hardwareInfo.CpuRate << std::endl;
    }
    else {
        return -6;
    }

    return 0;
}

int RSDeviceStatusUtil::getMemoryUseInfo(st_HardwareInfo &hardwareInfo) {
    const std::string filePath = "/tmp/.proc_meminfo_xxxxxxxx.txt";
    const std::string command = "cat /proc/meminfo > " + filePath;
    int ret = system(command.c_str());

    if (ret == -1) {
        return -1;
    }

    std::ifstream ifstr(filePath, std::ios_base::in | std::ios_base::binary);

    if (!ifstr.is_open()) {
        return -2;
    }
    std::string lineContent;

    float mem_total = 0.0f;
    float mem_active = 0.0f;
    float mem_free = 0.0f;
    float mem_buffer = 0.0f;
    float mem_cache = 0.0f;
    float vmem_total = 0.0f;
    float vmem_free = 0.0f;

    const std::string mem_total_key = std::string("MemTotal:");
    const std::string mem_active_key = std::string("Active:");
    const std::string mem_free_key = std::string("MemFree:");
    const std::string mem_buffer_key = std::string("Buffers:");
    const std::string mem_cached_key = std::string("Cached:");
    const std::string mem_swaped_key = std::string("SwapTotal:");
    const std::string mem_swapfree_key = std::string("SwapFree:");

    while (std::getline(ifstr, lineContent)) {
        if (lineContent.substr(0, mem_total_key.size()) == mem_total_key) {
            mem_total = RSDtghCommUtil::str2num<float>(lineContent.substr(mem_total_key.size(), lineContent.size() - mem_total_key.size() - 2));
        }
        else if (lineContent.substr(0, mem_free_key.size()) == mem_free_key) {
            mem_free = RSDtghCommUtil::str2num<float>(lineContent.substr(mem_free_key.size(), lineContent.size() - mem_free_key.size() - 2));
        }
        else if (lineContent.substr(0, mem_active_key.size()) == mem_active_key) {
            mem_active = RSDtghCommUtil::str2num<float>(lineContent.substr(mem_active_key.size(), lineContent.size() - mem_active_key.size() - 2));
        }
        else if (lineContent.substr(0, mem_buffer_key.size()) == mem_buffer_key) {
            mem_buffer = RSDtghCommUtil::str2num<float>(lineContent.substr(mem_buffer_key.size(), lineContent.size() - mem_buffer_key.size() - 2));
        }
        else if (lineContent.substr(0, mem_cached_key.size()) == mem_cached_key) {
            mem_cache = RSDtghCommUtil::str2num<float>(lineContent.substr(mem_cached_key.size(), lineContent.size() - mem_cached_key.size() - 2));
        }
        else if (lineContent.substr(0, mem_swaped_key.size()) == mem_swaped_key) {
            vmem_total = RSDtghCommUtil::str2num<float>(lineContent.substr(mem_swaped_key.size(), lineContent.size() - mem_swaped_key.size() - 2));
        }
        else if (lineContent.substr(0, mem_swapfree_key.size()) == mem_swapfree_key) {
            vmem_free = RSDtghCommUtil::str2num<float>(lineContent.substr(mem_swapfree_key.size(), lineContent.size() - mem_swapfree_key.size() - 2));
        }
        // std::cout << "mem_total = " << mem_total << ", mem_free = " << mem_free << ", mem_active ＝ " << mem_active << ", mem_buffer = " << mem_buffer << ", mem_cache = " << mem_cache << ", vmem_total = " << vmem_total << ", vmem_free = " << vmem_free << std::endl;
    }
    ifstr.close();

    if (mem_total > 0.01) {
        // hardwareInfo.RamRate = RSDtghCommUtil::num2str<float>((mem_total - (mem_free + mem_buffer + mem_cache + vmem_total + vmem_free)) / mem_total * 100, 2);
        hardwareInfo.RamRate = RSDtghCommUtil::num2str<float>(mem_active / mem_total * 100, 2);
    }
    else {
        return -2;
    }

    return 0;
}

int RSDeviceStatusUtil::getDiskUseInfo(st_HardwareInfo &hardwareInfo) {
    const std::string filePath = "/tmp/.proc_hd_xxxxxxx.txt";
    std::string command = "df -hl > " + filePath;

    int ret = system(command.c_str());
    if (ret == -1) {
        return -1;
    }
    std::ifstream ifstr(filePath, std::ios_base::in | std::ios_base::binary);

    if (!ifstr.is_open()) {
        return -2;
    }

    std::vector<std::string> lineContents;
    std::string lineContent;

    const std::string sd_key = "/dev/sd";
    while (std::getline(ifstr, lineContent)) {
        if (lineContent.size() > 0) {
            if (lineContent.substr(0, sd_key.size()) == sd_key) {
                lineContents.push_back(lineContent);
            }
        }
        lineContent.clear();
    }

    ifstr.close();

    hardwareInfo.SataFreeSpace = "-";
    if (lineContents.size() == 0) {
        return -3;
    }

    float totalSataFreeSpace = 0.0f;
    for (size_t i = 0; i < lineContents.size(); ++i) {
        std::vector<std::string> elements;
        splitString(lineContents[i], elements, ' ', false);
        totalSataFreeSpace += RSDtghCommUtil::str2num<float>(elements[3]);
    }

    hardwareInfo.SataFreeSpace = RSDtghCommUtil::num2str<float>(totalSataFreeSpace, 2);

    return 0;
}

int RSDeviceStatusUtil::getDeviceTempture(st_HardwareInfo &hardwareInfo) {
    const std::string filePath = "/tmp/.sensor_xxxxxxxx.txt";
    std::string command = "sensors > " + filePath;

    int ret = system(command.c_str());

    if (ret == -1) {
        return -1;
    }

    std::ifstream ifstr(filePath, std::ios_base::in | std::ios_base::binary);

    if (!ifstr.is_open()) {
        return -2;
    }

    std::string lineContent;

    const std::string core_key = "coretemp";
    const std::string core_key2 = "Package id ";
    bool isFindCore = false;

    float cpuDegree = 0.0f;
    while (std::getline(ifstr, lineContent)) {
        if (lineContent.size() > 0) {
            if (lineContent.substr(0, core_key.size()) == core_key) {
                isFindCore = true;
            }

            // 处于core信息字段时　
            if (isFindCore == true) {
                if (lineContent.substr(0, core_key2.size()) == core_key2) {
                    size_t pos = lineContent.find(':');

                    if (pos == std::string::npos) {
                        continue;
                    }

                    float degree = RSDtghCommUtil::str2num<float>(lineContent.substr(pos + 1));

                    if (degree > cpuDegree) {
                        cpuDegree = degree;
                    }
                }
            }
        }
        else {
            isFindCore = false;
        }
    }
    hardwareInfo.CpuDegree = RSDtghCommUtil::num2str<float>(cpuDegree, 2);
    ifstr.close();

    return 0;
}

int RSDeviceStatusUtil::getGpuUseInfo(st_HardwareInfo &hardwareInfo) {
#if 1
    // 基于nvidia-smi工具解析
    const std::string filePath = "/tmp/.nvidia_smi_xxxxxxxx.txt";
    std::string command = "nvidia-smi > " + filePath;

    int ret = system(command.c_str());

    if (ret == -1) {
        return -1;
    }

    std::ifstream ifstr(filePath, std::ios_base::in | std::ios_base::binary);

    if (!ifstr.is_open()) {
        return -2;
    }

    std::string lineContent;

    bool isReady = false;

    int offset = 0;
    while (std::getline(ifstr, lineContent)) {
        // std::cout << "GPU lineContent = " << lineContent << std::endl;
        if (lineContent.size() > 0) {
            std::vector<std::string> elements;
            splitString(lineContent, elements, ' ', false);

            if (isReady == false) {
                for (int i = 0; i < elements.size(); ++i) {
                    if (elements[i] == "GeForce") {
                        isReady = true;
                        break;
                    }
                }

                if (isReady == true) {
                    continue;
                }
            }

            if (isReady == true) {
                // std::cout << "GPU lineContent B = " << lineContent << std::endl;
                hardwareInfo.GpuDegree = RSDtghCommUtil::num2str<float>(RSDtghCommUtil::str2num<float>(elements[2]), 2);
                hardwareInfo.GpuSpaceRate = RSDtghCommUtil::num2str<float>(RSDtghCommUtil::str2num<float>(elements[8]) / RSDtghCommUtil::str2num<float>(elements[10]) * 100, 2);
                hardwareInfo.GpuRate = RSDtghCommUtil::num2str<float>(RSDtghCommUtil::str2num<float>(elements[12]), 2);
                // std::cout << "GpuDegree = " << hardwareInfo.GpuDegree << ", GpuSpaceRate = " << hardwareInfo.GpuSpaceRate << ", GpuRate = " << hardwareInfo.GpuRate << std::endl;
                break;
            }
        }
    }
    ifstr.close();
#else
    nvmlReturn_t result;
    unsigned int device_count;

    result = nvmlInit();
    if (result != NVML_SUCCESS)
        return -1;

    result = nvmlDeviceGetCount(&device_count);
    if (result != NVML_SUCCESS)
        return -2;

    float GpuDegree = 0;
    float GpuSpaceRate = 0;
    float GpuRate = 0;

    for (int i = 0; i < device_count; ++i)
    {
        nvmlDevice_t device;
        result = nvmlDeviceGetHandleByIndex(i, &device);
        if (result != NVML_SUCCESS)
            return -3;

        char device_name[NVML_DEVICE_NAME_BUFFER_SIZE];
        result = nvmlDeviceGetName(device, device_name, NVML_DEVICE_NAME_BUFFER_SIZE);
        if (result != NVML_SUCCESS)
            return -4;

        // std::printf("Device %d: %s\n", i, device_name);

        nvmlUtilization_st device_utilization;
        result = nvmlDeviceGetUtilizationRates(device, &device_utilization);

        if (result != NVML_SUCCESS)
            return -5;

        // std::printf("GPU Util: %u, Mem Util: %u\n", device_utilization.gpu, device_utilization.memory);

        unsigned int temperature;
        result = nvmlDeviceGetTemperature(device, NVML_TEMPERATURE_GPU, &temperature);

        if (result != NVML_SUCCESS)
            return -6;

        GpuDegree = std::max((float)temperature, GpuDegree);
        GpuSpaceRate += device_utilization.memory;
        GpuRate += device_utilization.gpu;
    }

    hardwareInfo.GpuDegree = RSDtghCommUtil::num2str<float>(GpuDegree, 2);
    hardwareInfo.GpuSpaceRate = RSDtghCommUtil::num2str<float>(GpuSpaceRate, 2);
    hardwareInfo.GpuRate = RSDtghCommUtil::num2str<float>(GpuRate, 2);

    // std::cout << "GpuDegree = " << hardwareInfo.GpuDegree << ", GpuSpaceRate = " << hardwareInfo.GpuSpaceRate << ", GpuRate = " << hardwareInfo.GpuRate << std::endl;

    nvmlShutdown();
#endif
    return 0;
}

int RSDeviceStatusUtil::splitString(std::string &content, std::vector<std::string> &elements, const char spliter, const bool keepEmpty) {
    elements.clear();
    std::string subStr;
    for (size_t i = 0; i < content.size(); ++i) {
        const char ch = content[i];
        if (ch == spliter) {
            if (keepEmpty == true && subStr.size() == 0) {
                elements.push_back(subStr);
            }
            else if (subStr.size() > 0) {
                elements.push_back(subStr);
            }
            subStr.clear();
        }
        else {
            subStr.push_back(ch);
        }
    }

    if (subStr.size() > 0) {
        elements.push_back(subStr);
    }
    return 0;
}

// ============================================================= //
// ====================== RSCustomSerialize ==================== //
// ============================================================= //
int RSCustomSerialize::serialize(const st_UploadInfo &info, std::string &encodeData) {
    encodeData.clear();
    Json::Value jsonValue(Json::ValueType::objectValue);

    jsonValue["MsgType"] = info.MsgType;
    jsonValue["DevNo"] = info.DevNo;
    jsonValue["Ack"] = info.Ack;

    Json::FastWriter jsonWriter;
    encodeData = jsonWriter.write(jsonValue);

    return 0;
}

int RSCustomSerialize::serialize(const st_KeepAliveInfo &info, std::string &encodeData) {
    encodeData.clear();

    Json::Value jsonValue(Json::ValueType::objectValue);
    jsonValue["MsgType"] = info.MsgType;
    jsonValue["DevNo"] = info.DevNo;
    jsonValue["MecNo"] = info.MecNo;
    jsonValue["Timestamp"] = info.Timestamp;

    Json::FastWriter jsonWriter;
    encodeData = jsonWriter.write(jsonValue);

    return 0;
}

// 硬件状态信息
int RSCustomSerialize::serialize(const st_HardwareInfo &info, Json::Value &jsonValue) {
    jsonValue = Json::Value(Json::ValueType::objectValue);

    jsonValue["SoftwareVersion"] = info.SoftwareVersion;
    jsonValue["CpuDegree"] = info.CpuDegree;
    jsonValue["CpuRate"] = info.CpuRate;
    jsonValue["RamRate"] = info.RamRate;
    jsonValue["SataFreeSpace"] = info.SataFreeSpace;
    jsonValue["GpuDegree"] = info.GpuDegree;
    jsonValue["GpuRate"] = info.GpuRate;
    jsonValue["GpuSpaceRate"] = info.GpuSpaceRate;
    jsonValue["ErrorCode"] = info.ErrorCode;

    return 0;
}

int RSCustomSerialize::serialize(const st_DeviceStatusInfo &info, std::string &encodeData) {
    encodeData.clear();

    Json::Value jsonValue(Json::ValueType::objectValue);

    jsonValue["MsgType"] = info.MsgType;
    jsonValue["DevNo"] = info.DevNo;
    jsonValue["MecNo"] = info.MecNo;
    jsonValue["Timestamp"] = info.Timestamp;

    Json::Value jsonHardwareList(Json::ValueType::arrayValue);

    for (size_t iter = 0; iter < info.hardwareInfos.size(); ++iter) {
        Json::Value jsonHardware;
        int ret = serialize(info.hardwareInfos[iter], jsonHardware);

        if (ret != 0) {
            return -1;
        }

        jsonHardwareList.append(jsonHardware);
    }
    jsonValue["Hardware_List"] = jsonHardwareList;

    Json::FastWriter jsonWriter;
    encodeData = jsonWriter.write(jsonValue);

    return 0;
}

// Object List
int RSCustomSerialize::serialize(const ObjectList &info, const std::string &deviceNo, Json::Value &jsonValue) {
    jsonValue = Json::Value(Json::ValueType::objectValue);
    size_t pos = info.ID.find_first_of('#');
    if (pos != std::string::npos) {
        jsonValue["ID"] = std::string("O") + info.ID.substr(0, pos) + deviceNo + info.ID.substr(pos + 1);
    }
    else {
        jsonValue["ID"] = std::string("O") + info.ID;
    }
    jsonValue["PtcType"] = info.PtcType;
    jsonValue["VehL"] = info.VehL;
    jsonValue["VehW"] = info.VehW;
    jsonValue["VehH"] = info.VehH;
    jsonValue["XPos"] = info.XPos;
    jsonValue["YPos"] = info.YPos;
    jsonValue["PtcLon"] = info.PtcLon;
    jsonValue["PtcLat"] = info.PtcLat;
    jsonValue["PtcEle"] = info.PtcEle;
    jsonValue["PtcSpeed"] = info.PtcSpeed;
    jsonValue["PtcHeading"] = info.PtcHeading;
    jsonValue["VehType"] = info.VehType;

    return 0;
}

int RSCustomSerialize::serialize(const ObjectSend &info, const std::string &deviceNo, std::string &encodeData) {
    encodeData.clear();
    Json::Value jsonValue(Json::ValueType::objectValue);

    jsonValue["MsgType"] = info.MsgType;
    jsonValue["DevNo"] = info.DevNo;
    jsonValue["MecNo"] = info.MecNo;
    jsonValue["Timestamp"] = info.Timestamp;

    Json::Value jsonObjectList(Json::ValueType::arrayValue);
    for (size_t i = 0; i < info.Obj_List.size(); ++i) {
        Json::Value jsonValue_Obj;
        int ret = serialize(info.Obj_List[i], deviceNo, jsonValue_Obj);

        if (ret != 0) {
            return -1;
        }

        jsonObjectList.append(jsonValue_Obj);
    }

    jsonValue["Obj_List"] = jsonObjectList;

    // Json转为字符串
    Json::FastWriter jsonWriter;
    encodeData = jsonWriter.write(jsonValue);

    return 0;
}

// Event List 信息
int RSCustomSerialize::serialize(const EventList &info, const std::string &deviceNo, Json::Value &jsonValue) {
    jsonValue = Json::Value(Json::ValueType::objectValue);
//    jsonValue["ObjId"] = info.ObjId;
    size_t pos = info.ObjId.find_first_of('#');
    if (pos != std::string::npos) {
        jsonValue["ObjID"] = std::string("O") + info.ObjId.substr(0, pos) + deviceNo + info.ObjId.substr(pos + 1);
    }
    else {
        jsonValue["ID"] = std::string("O") + info.ObjId;
    }

    pos = info.ID.find_first_of('#');
    if (pos != std::string::npos) {
        jsonValue["ID"] = std::string("E") + info.ID.substr(0, pos) + deviceNo + info.ID.substr(pos + 1);
    }
    else {
        jsonValue["ID"] = std::string("E") + info.ID;
    }
    jsonValue["EvtStatus"] = info.EvtStatus;
    jsonValue["EvtType"] = info.EvtType;
    jsonValue["Lon"] = info.Lon;
    jsonValue["Lat"] = info.Lat;
    jsonValue["Ele"] = info.Ele;
    jsonValue["XPos"] = info.XPos;
    jsonValue["YPos"] = info.YPos;
    jsonValue["VehL"] = info.VehL;
    jsonValue["VehW"] = info.VehW;
    jsonValue["VehH"] = info.VehH;

    return 0;
}

int RSCustomSerialize::serialize(const EventSend &info, const std::string &deviceNo, std::string &encodeData) {
    encodeData.clear();
    Json::Value jsonValue(Json::ValueType::objectValue);

    jsonValue["MsgType"] = info.MsgType;
    jsonValue["DevNo"] = info.DevNo;
    jsonValue["MecNo"] = info.MecNo;
    jsonValue["Timestamp"] = info.Timestamp;

    Json::Value jsonObjectList(Json::ValueType::arrayValue);
    for (size_t i = 0; i < info.Evt_List.size(); ++i) {
        Json::Value jsonValue_Obj;
        int ret = serialize(info.Evt_List[i], deviceNo, jsonValue_Obj);

        if (ret != 0) {
            return -1;
        }

        jsonObjectList.append(jsonValue_Obj);
    }

    jsonValue["Evt_List"] = jsonObjectList;

    // Json转为字符串
    Json::FastWriter jsonWriter;
    encodeData = jsonWriter.write(jsonValue);

    return 0;
}


RS_DTGH_OBJECTTYPE RSCustomSerialize::fromRoboToDTGH(const ObjectType &type)
{
    if (type == ObjectType::UNKNOW) {
        return RS_DTGH_OBJECTTYPE::RS_DTGH_UNKNOWN;
    }
    else if (type == ObjectType::CAR || type == ObjectType::TRUCK_BUS || type == ObjectType::ULTRA_VEHICLE) {
        return RS_DTGH_OBJECTTYPE::RS_DTGH_VEHICLE;
    }
    else if (type == ObjectType::BIC) {
        return RS_DTGH_OBJECTTYPE::RS_DTGH_BIKE;
    }
    else if (type == ObjectType::PED) {
        return RS_DTGH_OBJECTTYPE::RS_DTGH_PED;
    }
    else {
        return RS_DTGH_OBJECTTYPE::RS_DTGH_UNKNOWN;
    }
}

}
}
}
