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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_DTGH_CUSTOM_TRANSFORMER_1_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_DTGH_CUSTOM_TRANSFORMER_1_H_

#include "rs_perception/custom/robosense_custom/dtghCustom/dtgh_custom_transformer_utils_1.h"
#include "rs_perception/lidar/map_filter/external/common/rs_hd_map.h"
#include "rs_perception/lidar/map_filter/external/base_map_filter.h"


namespace robosense{
namespace perception{
namespace Dtgh1 {

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
{lidar::EventType::AbnormalSwitchTwice, 20},
{lidar::EventType::Statistical, 998},
{lidar::EventType::LimiteVelocity, 999},
};

class EventConvert {
public:
    using Ptr = std::shared_ptr<EventConvert>;
    EventConvert() = default;

    void init(){
        obj_evt_whole_map.clear();
        tmp_obj_evt_whole_map.clear();
        evt_his_status_map.clear();
    }

    void convert(const RsPerceptionMsg::Ptr& msg_ptr, EventMsg::Ptr& result_ptr) {
        // 1. 先判断有没有开事件检测
        Any::Ptr any_ptr_event_detect = msg_ptr->rs_lidar_result_ptr->any_map.at("event_detect");
        auto event_detect = any_ptr_event_detect->AnyCast<bool>();
        if (!(*event_detect)) {
//          RINFO << "50 event detect is turned off!";
          return;
        }

        // 2. reset事件发送指针
        result_ptr->event_send_ptr.reset(new EventSend);
        auto& evt_s_ptr = result_ptr->event_send_ptr;

        // 3. 从感知结果中获取事件检测结果并进行转换
        Any::Ptr any_ptr_obj_evt_map = msg_ptr->rs_lidar_result_ptr->any_map.at("obj_evt_map");
        auto obj_evt_map = any_ptr_obj_evt_map->AnyCast<std::map<int, std::vector<lidar::EventType>>>();

        tmp_obj_evt_whole_map.clear();
        double timestamp = msg_ptr->rs_lidar_result_ptr->timestamp;
        for (auto obj: msg_ptr->rs_lidar_result_ptr->objects) {
            if (obj_evt_map->find(obj->core_infos_.tracker_id) != obj_evt_map->end()) {
                auto obj_evt_vec = obj_evt_map->at(obj->core_infos_.tracker_id);
                for (auto evt_type: obj_evt_vec) {
                    int type = kEvtType2TypeIDMap.at(evt_type);
                    std::string obj_evt_id = std::to_string(obj->core_infos_.tracker_id) + std::to_string(type);
                    if (obj_evt_whole_map.find(obj_evt_id) != obj_evt_whole_map.end()) {
                        obj_evt_whole_map[obj_evt_id].longitude = obj->supplement_infos_.gps_longtitude;
                        obj_evt_whole_map[obj_evt_id].latitude = obj->supplement_infos_.gps_latitude;
                    }
                    else {
                        obj_evt_whole_map[obj_evt_id].tracker_id = obj->core_infos_.tracker_id;
                        obj_evt_whole_map[obj_evt_id].event_status = 1;
                        obj_evt_whole_map[obj_evt_id].event_type = type;
                        obj_evt_whole_map[obj_evt_id].longitude = obj->supplement_infos_.gps_longtitude;
                        obj_evt_whole_map[obj_evt_id].latitude = obj->supplement_infos_.gps_latitude;
                        obj_evt_whole_map[obj_evt_id].timestamp = timestamp;
                    }
                    tmp_obj_evt_whole_map[obj_evt_id] = obj_evt_whole_map[obj_evt_id];
                    evt_his_status_map[obj_evt_id] = obj_evt_whole_map[obj_evt_id];
                    evt_s_ptr->Evt_list.emplace_back(obj_evt_whole_map[obj_evt_id]);
                }
            }
        }
        if (evt_his_status_map.empty()) {
            return;
        }

        for (auto itr = evt_his_status_map.begin(); itr != evt_his_status_map.end();) {
            if (tmp_obj_evt_whole_map.find(itr->first) != tmp_obj_evt_whole_map.end()) {
                itr++;
                continue;
            }
            else {
                evt_his_status_map[itr->first].event_status = 0;
                evt_his_status_map[itr->first].timestamp = timestamp;
                evt_s_ptr->Evt_list.emplace_back(evt_his_status_map[itr->first]);
                itr = evt_his_status_map.erase(itr);
            }
        }
    }

    std::map<std::string, EventList> obj_evt_whole_map;
    std::map<std::string, EventList> tmp_obj_evt_whole_map;
    std::map<std::string, EventList> evt_his_status_map;
};

class RsDtghSerialize {
public:
    using Ptr = std::shared_ptr<RsDtghSerialize>;
    using ConstPtr = std::shared_ptr<const RsDtghSerialize>;

    void init(){
        event_convert_ptr.reset(new EventConvert);
        event_convert_ptr->init();
        evt_res_ptr.reset(new EventMsg);
        frame_id = 0;
    }

    struct RsDtghObjectHeader {
        unsigned char head[2]; // 帧头 2个字节
        uint16_t frameLen; // 帧长 2个字节
        unsigned char orderWord; // 命令字 1个字节 0x55
        unsigned char reserved[10]; // 预留 10个字节 10个0
        int32_t frameId; // 帧ID 4个字节
        unsigned long long int timestamp; // t0时间戳 6个字节 待确认
    };

    struct RsDtghEventHeader {
        unsigned char head[2]; // 帧头 2个字节
        unsigned char frameLen[2]; // 帧长 2个字节
        unsigned char orderWord; // 命令字 1个字节 0x66
        unsigned char reserved[5]; // 预留 5个字节 5个0
    };

    struct RsDtghTrafficHeader {
        unsigned char head[2]; // 帧头 2个字节
        uint16_t frameLen;; // 帧长 2个字节
        unsigned char orderWord; // 命令字 0x77
        unsigned char reserved[10]; // 预留10个字节
    };

    int serialize (const RsPerceptionMsg::Ptr& msg, native_sdk_3_1::RSSerializeBuffer& en_msg,
                   RsDtgh_1CustomMsgParams& custom_params, const bool& hd_map_disable) {
        frame_id++;
        en_msg.clearInfo();
        if (msg == nullptr) {
            return -1;
        }

        RSEndian<uint16_t> uint16Endian; // 帧长
        RSEndian<uint32_t> uint32Endian; // 帧ID
        RSEndian<long> longEndian; // 时间戳

        char start[] = {(char)(0xFF), (char)(0xEE)};
        char reserved[] = {(char)(0),(char)(0),(char)(0),(char)(0),(char)(0),
                            (char)(0),(char)(0),(char)(0),(char)(0),(char)(0)};

        int offset = 0;

        // 序列化object
        if (custom_params.send_object) {
            int dataMsgOffset = 0;
            if (!en_msg.offsets.empty()) {
                dataMsgOffset = *(en_msg.offsets.rbegin()) + *(en_msg.lengths.rbegin());
            }

            // 帧头 2个字节
            offset = dataMsgOffset;
            memcpy(en_msg.buffers.data() + offset, start, sizeof(start));
            offset += sizeof(start);

            // 帧长 2个字节 先记录位置，序列化物体完成之后再填进去
            int dataLenOffset = offset;
            int objectCnt = msg->rs_lidar_result_ptr->objects.size();
            offset += sizeof(uint16_t);

            // 命令字 1个字节
            char orderWord = 0x55;
            en_msg.buffers[offset] = orderWord;
            offset += 1;

            // 预留 10个字节
            memcpy(en_msg.buffers.data() + offset, reserved, sizeof(reserved));
            offset += sizeof(reserved);

            // 帧ID 4个字节
            uint32_t frameId = frame_id;
            uint32Endian.toTargetEndianArray(frameId, en_msg.buffers.data() + offset,
                                            sizeof(uint32_t),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(int32_t);

            // t0时间戳 6个字节 long有8个字节，因此经过小端模式后，最后两个字节可以挪掉
            long timestamp = msg->rs_lidar_result_ptr->timestamp * 1000;
            longEndian.toTargetEndianArray(timestamp, en_msg.buffers.data() + offset,
                                             sizeof(long),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(long);
            offset -= 2;

            // 序列化目标
            uint16_t objectDataLen = 0;
            for (int i=0; i < objectCnt; i++) {
                int objectOffset = serializeObject(en_msg.buffers.data() + offset,
                                             msg->rs_lidar_result_ptr->objects[i]);
                offset += objectOffset;
                objectDataLen += objectOffset;
            }

            // 填充数据长度
            uint16_t frameLen = objectDataLen + 21;
            uint16Endian.toTargetEndianArray(frameLen, en_msg.buffers.data() + dataLenOffset,
                                             sizeof(uint16_t),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            // 填充完成，保存起止点以及长度等信息
            en_msg.offsets.push_back(dataMsgOffset);
            en_msg.lengths.push_back(offset - dataMsgOffset);
            en_msg.msgCntMap[native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_DTGH_OBJECT] = 1;
            en_msg.types.push_back(native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_DTGH_OBJECT);
        }

        if (hd_map_disable) {
            RDEBUG << "hd_map turned off, only serialize perception message!";
            return 0;
        }

        // 序列化事件
        if (custom_params.send_event) {
            serializeEvent(msg, en_msg);
        }

        // 序列化交通流
        if (custom_params.send_traffic) {
            serializeTraffic(msg, en_msg);
        }

        return 0;
    }

    int serializeObject(char* data, const robosense::perception::Object::Ptr& object) {
        int offset = 0;
        RSEndian<uint16_t> uint16Endian; // 目标ID, 航向角，长宽高
        RSEndian<char> charEndian; // 类型
        RSEndian<int16_t> int16Endian; // 速度
        RSEndian<int32_t> int32Endian; // 经纬度

        //目标ID 2个字节
        uint16_t id;
        if (object->core_infos_.tracker_id == -1) {
            id = 0xFFFF;
        }
        else {
            id = object->core_infos_.tracker_id;
        }
        uint16Endian.toTargetEndianArray(id, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        // 类型 1个字节
        char type;
        ObjectType obj_type = object->core_infos_.type;
        if (obj_type == ObjectType::PED) {
            type = 3;
        }
        else if (obj_type == ObjectType::CAR || obj_type == ObjectType::TRUCK_BUS || obj_type == ObjectType::ULTRA_VEHICLE) {
            type = 1;
        }
        else if (obj_type == ObjectType::BIC) {
            type = 2;
        }
        else {
            type = 0;
        }
        charEndian.toTargetEndianArray(type, data + offset, sizeof(char),
                                       RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(char);

        // 经度 4个字节
        int32_t longitude = object->supplement_infos_.gps_longtitude * 1e+7;
        int32Endian.toTargetEndianArray(longitude, data + offset, sizeof(int32_t),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(int32_t);

        // 纬度 4个字节
        int32_t latitude = object->supplement_infos_.gps_latitude * 1e+7;
        int32Endian.toTargetEndianArray(latitude, data + offset, sizeof(int32_t),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(int32_t);

        // 速度 2个字节
        float vel = object->core_infos_.velocity.norm();
        int16_t velocity = vel * 1000;
        int16Endian.toTargetEndianArray(velocity, data + offset, sizeof(int16_t),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(int16_t);

        // 航向角 2个字节 (如果速度为0，那么就采用框的朝向来代替。)
        float velocityYaw;
        if (vel < 0.5) {
            velocityYaw = atan2(object->core_infos_.direction.y, object->core_infos_.direction.x) * 180.0f / M_PI;
        }
        else {
            velocityYaw = atan2(object->core_infos_.velocity.y, object->core_infos_.velocity.x) * 180.0f / M_PI;
        }
        if (velocityYaw < 0.0f) {
            velocityYaw += 360.0f;
        }
        if (std::abs(velocityYaw - 360.0f) < 0.05f) {
            velocityYaw = 0.0f;
        }
        velocityYaw = eastYaw2NorthYaw(velocityYaw);
        uint16_t vel_yaw = velocityYaw * 100;
        uint16Endian.toTargetEndianArray(vel_yaw, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        // 长度 2个字节
        uint16_t obj_L = object->core_infos_.size.x * 1000;
        uint16Endian.toTargetEndianArray(obj_L, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        // 宽度 2个字节
        uint16_t obj_W = object->core_infos_.size.y * 1000;
        uint16Endian.toTargetEndianArray(obj_W, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        // 高度 2个字节
        uint16_t obj_H = object->core_infos_.size.z * 1000;
        uint16Endian.toTargetEndianArray(obj_H, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        return offset;
    }
    void serializeEvent(const RsPerceptionMsg::Ptr& msg_ptr, native_sdk_3_1::RSSerializeBuffer& en_msg) {
        char start[] = {(char)(0xFF), (char)(0xEE)};
        char reserved[] = {(char)(0),(char)(0),(char)(0),(char)(0),(char)(0)};
        RSEndian<int32_t> int32Endian; // 经纬度
        RSEndian<long> longEndian; // 时间戳

        event_convert_ptr->convert(msg_ptr, evt_res_ptr);
        auto& event_list = evt_res_ptr->event_send_ptr->Evt_list; // vector<EventList>, 最终结果。包含开始的事件和结束的事件。每个事件占一个元素位置。
        // 遍历每个元素位置就是遍历每个事件：包含物体ID以及需要序列化的信息。
        int offset = 0;
        for (auto& itr: event_list) {
            int dataMsgOffset = 0;
            if (!en_msg.offsets.empty()) {
                dataMsgOffset = *(en_msg.offsets.rbegin()) + *(en_msg.lengths.rbegin());
            }
            // 帧头 2个字节
            offset = dataMsgOffset;
            memcpy(en_msg.buffers.data() + offset, start, sizeof(start));
            offset += sizeof(start);

            // 帧长 2个字节
            char frameLen[2] = {(char)(0x16), (char)(0x00)};
            memcpy(en_msg.buffers.data() + offset, frameLen, sizeof(frameLen));
            offset += sizeof(frameLen);

            // 命令字 1个字节
            char orderWord = 0x66;
            en_msg.buffers[offset] = orderWord;
            offset += 1;

            // 预留 5个字节
            memcpy(en_msg.buffers.data() + offset, reserved, sizeof(reserved));
            offset += sizeof(reserved);

            // 事件类型 1个字节
            std::string event_type = kTypeID2EvtTypeStringMap.at(itr.event_type);
            if (kEventTypeString2CharIdMap.find(event_type) == kEventTypeString2CharIdMap.end()) {
                continue;
            }
            en_msg.buffers[offset] = kEventTypeString2CharIdMap.at(event_type);
            offset += 1;

            // 事件状态 1个字节
            en_msg.buffers[offset] = kEventStatusInt2CharIdMap.at(itr.event_status);
            offset += 1;

            // 经度 4个字节
            int32_t longitude = itr.longitude == 0 ? 0 : itr.longitude * 1e+7;
            int32Endian.toTargetEndianArray(longitude, en_msg.buffers.data() + offset,
                                            sizeof(int32_t),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(int32_t);

            // 纬度 4个字节
            int32_t latitude = itr.latitude == 0 ? 0 : itr.latitude * 1e+7;
            int32Endian.toTargetEndianArray(latitude, en_msg.buffers.data() + offset,
                                            sizeof(int32_t),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(int32_t);

            // 时间戳 6个字节 (但是long是8个字节，所以这里offset加完了以后就是28个偏移量了，因此最后统计的时候要减掉2个)
            long timestamp = itr.timestamp * 1000;
            longEndian.toTargetEndianArray(timestamp,en_msg.buffers.data() + offset,
                                           sizeof(long),
                                           RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(long);
            offset -= 2;

            // 填充完成，保存起止点以及长度等信息
            en_msg.offsets.push_back(dataMsgOffset);
            en_msg.lengths.push_back(offset - dataMsgOffset);
            en_msg.msgCntMap[native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_DTGH_EVENT] = 1;
            en_msg.types.push_back(native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_DTGH_EVENT);
        }
    }
    void serializeTraffic(const RsPerceptionMsg::Ptr& msg_ptr, native_sdk_3_1::RSSerializeBuffer& en_msg) {
        // 1. 先判断有没有开事件检测
        Any::Ptr any_ptr_event_detect = msg_ptr->rs_lidar_result_ptr->any_map.at("event_detect");
        auto event_detect = any_ptr_event_detect->AnyCast<bool>();
        if (!(*event_detect)) {
//            RINFO << "421 event detect is turned off!";
            return;
        }
        // 2. 获取count_map结果并进行转换
        Any::Ptr any_ptr_count_map = msg_ptr->rs_lidar_result_ptr->any_map.at("count_map");
        auto count_map = any_ptr_count_map->AnyCast<std::map<int, std::map<int, lidar::StaticMsg>>>();

        if (count_map->empty()) {
//            RINFO << "traffic is none!";
            return;
        }
        std::map<int, int> res_map; // 存结果用。<laneID, vel_num>
        res_map[-1] = 0;
        double start_time = 0.0, end_time = 0.0;
        for (auto& itr: *count_map) {
            for (auto& itrs: itr.second) {
                if (itrs.first == itr.first) {
                    // 该项为统计ROI，记总量的，laneID = -1，只有vel_num和起止时间
                    res_map[-1] += itrs.second.vel_num;
                    start_time = itrs.second.start_time;
                    end_time = itrs.second.end_time;
                }
                else{
                    // 该项为普通车道ROI，有自己车道的vel_num和laneID，但是没有起止时间
                    res_map[itrs.second.laneIdx] = itrs.second.vel_num;
                }
            }
        }

        // 3. 对最终结果按照协议进行写入
        char start[] = {(char)(0xFF), (char)(0xEE)};
        char reserved[] = {(char)(0),(char)(0),(char)(0),(char)(0),(char)(0),
                           (char)(0),(char)(0),(char)(0),(char)(0),(char)(0)};
        RSEndian<uint32_t> uint32Endian; // 帧ID
        RSEndian<long> longEndian; // 时间戳
        RSEndian<uint16_t> uint16Endian; //帧长

        int offset = 0;

        int dataMsgOffset = 0;
        if (!en_msg.offsets.empty()) {
            dataMsgOffset = *(en_msg.offsets.rbegin()) + *(en_msg.lengths.rbegin());
        }

        // 帧头
        offset = dataMsgOffset;
        memcpy(en_msg.buffers.data() + offset, start, sizeof(start));
        offset += sizeof(start);

        // 帧长: 序列化之后再填写
        int dataLenOffset = offset;
        offset += sizeof(uint16_t);

        // 命令字 1个字节
        char orderWord = 0x77;
        en_msg.buffers[offset] = orderWord;
        offset += 1;

        // 预留 10个字节
        memcpy(en_msg.buffers.data() + offset, reserved, sizeof(reserved));
        offset += sizeof(reserved);

        // 帧ID 4个字节
        uint32_t frameId = frame_id;
        uint32Endian.toTargetEndianArray(frameId, en_msg.buffers.data() + offset,
                                         sizeof(uint32_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint32_t);

        // 时间戳 6个字节 采用8-2的方法
        long timestamp = msg_ptr->rs_lidar_result_ptr->timestamp * 1000;
        longEndian.toTargetEndianArray(timestamp, en_msg.buffers.data() + offset,
                                       sizeof(long),
                                       RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(long);
        offset -= 2;

        // 统计开始时间戳（存成毫秒）
        long startTime = start_time * 1000;
        longEndian.toTargetEndianArray(startTime, en_msg.buffers.data() + offset,
                                       sizeof(long),
                                       RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(long);
        offset -= 2;

        // 统计结束时间戳（存成毫秒）
        long endTime = end_time * 1000;
        longEndian.toTargetEndianArray(endTime, en_msg.buffers.data() + offset,
                                       sizeof(long),
                                       RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(long);
        offset -= 2;

        // 总车流量
        uint16_t total_vel_num = res_map[-1];
        uint16Endian.toTargetEndianArray(total_vel_num, en_msg.buffers.data() + offset,
                                         sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        // 各个车道的流量
        uint16_t trafficDataLen = 0;
        for (auto& itr: res_map) {
            if (itr.first == -1) {
                continue;
            }
            char laneId = kLaneIDInt2CharIdMap.at(itr.first);
            char vel_num = (char)(itr.second);
            en_msg.buffers[offset] = laneId;
            offset += 1;
            en_msg.buffers[offset] = vel_num;
            offset += 1;
            trafficDataLen += 2;
        }

        uint16_t frameLen = 35 + trafficDataLen;
        uint16Endian.toTargetEndianArray(frameLen, en_msg.buffers.data() + dataLenOffset,
                                         sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        en_msg.offsets.push_back(dataMsgOffset);
        en_msg.lengths.push_back(offset - dataMsgOffset);
        en_msg.msgCntMap[native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_DTGH_TRAFFIC] = 1;
        en_msg.types.push_back(native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_DTGH_TRAFFIC);
    }

    int frame_id;
protected:
    // 将以正东为参考，逆时针旋转为正 => 以正北为参考，顺时针为正
    float eastYaw2NorthYaw(float eastYaw) {
        float northYaw = (90.0f - eastYaw > 1e-3) ? (90.0f - eastYaw)
                : (90.0f - eastYaw + 360.f);

        return northYaw;
    }

    // 将以正北为参考，顺时针为正　=> 正东为参考，逆时针为正
    float northYaw2EastYaw(float northYaw) {
        float eastYaw = (90.0f - northYaw > 1e-3) ? (90.0f - northYaw)
                : (90.0f - northYaw + 360.f);

        return eastYaw;
    }

private:
    EventConvert::Ptr event_convert_ptr;
    EventMsg::Ptr evt_res_ptr;
};

}
}
}

#endif //RS_PERCEPTION_CUSTOM_ROBOSENSE_DTGH_DTGH_CUSTOM_TRANSFORMER_1_H_