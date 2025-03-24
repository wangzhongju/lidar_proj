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
#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_V2R_ROBOSENSE_V2R_CUSTOM_TRANSFORMER_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_V2R_ROBOSENSE_V2R_CUSTOM_TRANSFORMER_H_
#include "rs_perception/custom/robosense_v2r/robosense_v2r_custom_transformer_utils.h"

namespace robosense {
namespace perception {

class RsBaseRobosenseV2RSerialize {
public:
    using Ptr = std::shared_ptr<RsBaseRobosenseV2RSerialize>;
    using ConstPtr = std::shared_ptr<const RsBaseRobosenseV2RSerialize>;

public:
    // Robosense的V2R协议Header
    struct RSV2RHeader {
        unsigned char head[2];
        unsigned char deviceType;
        unsigned char frameType;
        uint16_t frameLen;
        unsigned long long int deviceId;
        unsigned long long int timestamp;
        uint16_t checkSum;
        unsigned char tail[2];
    };

    // Robosense的V2R协议状态信息
    struct RSV2RStatus {
        unsigned long long int deviceId;
        short int deviceStatus;
    };

public:
    int init(const RsCustomRobosenseV2RMsgParams& customParams) {
        int ret;
        _customParams = customParams;

        // CRC16-25X
        const uint16_t poly = 0x1021;
        const uint16_t init = 0xFFFF;
        const bool refIn = true;
        const bool refOut = true;
        const uint16_t xorOut = 0xFFFF;

        _simpleCRC16Ptr.reset(new RSSimpleCRC16(poly, init, refIn, refOut, xorOut));

        ret = _simpleCRC16Ptr->init();

        if (ret != 0) {
          return -1;
        }
        return 0;
    }

    int serialize(const RsPerceptionMsg::Ptr& msg, native_sdk_3_1::RSSerializeBuffer& en_msg) {
        en_msg.clearInfo();
        if (msg == nullptr) {
            return -1;
        }

        RSEndian<unsigned long long int> uint64Endian;
        RSEndian<uint16_t> uint16Endian;
        // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
        // "RUN HERE 2" << std::endl;

        char start[] = {(char)(0x7E), (char)(0x7E)};
        char end[] = {(char)(0x7E), (char)(0x7D)};

        // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
        // "RUN HERE 2" << std::endl;

        // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
        // "RUN HERE 2" << std::endl;
        int offset = 0;
        {
            int dataMsgOffset = 0;

            if (en_msg.offsets.size() > 0) {
                dataMsgOffset = en_msg.offsets[en_msg.offsets.size() - 1] + en_msg.lengths[en_msg.lengths.size() - 1];
            }

            // 帧头
            offset = dataMsgOffset;
            memcpy(en_msg.buffers.data() + offset, start, sizeof(start));
            offset += sizeof(start);

            // 设备类型: 固定为激光雷达
            int frameDeviceTypeOffset = offset;
            char deviceType = 0x01;
            en_msg.buffers[offset] = deviceType;
            offset += 1;

            // 数据帧类型
            char messageType = 0x00;
            en_msg.buffers[offset] = messageType;
            offset += 1;

            // 数据帧长度: 序列化之后重新填充
            int dataLenOffset = offset;
            int objectCnt = msg->rs_lidar_result_ptr->objects.size();
            offset += sizeof(uint16_t);

            // 设备ID
            unsigned long long int deviceId = _customParams.device_id;
            uint64Endian.toTargetEndianArray(deviceId, en_msg.buffers.data() + offset,
                                             sizeof(unsigned long long int),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(unsigned long long int);

            // 时间戳　
            unsigned long long int timestamp = msg->rs_lidar_result_ptr->timestamp * 1000;  // s => ms
            uint64Endian.toTargetEndianArray(timestamp, en_msg.buffers.data() + offset,
                                             sizeof(unsigned long long int),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(unsigned long long int);

            // 序列化障碍物(s)
            uint16_t objectDataLen = 0;
            for (int i = 0; i < objectCnt; ++i) {
                // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
                // "RUN HERE 2" << std::endl;
                int objectOffset = serialize(en_msg.buffers.data() + offset,
                                             msg->rs_lidar_result_ptr->objects[i]);
                offset += objectOffset;
                objectDataLen += objectOffset;
                // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
                // "RUN HERE 2" << std::endl;
            }

            // 填充数据的长度信息
            const int objectSize = 56;
            uint16_t dataLen = (objectCnt * objectSize);
            uint16Endian.toTargetEndianArray(objectDataLen, en_msg.buffers.data() + dataLenOffset,
                                             sizeof(uint16_t),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
            // "RUN HERE 2" << std::endl; 校验位: 设备类型 <-> 数据区
            uint16_t checkSum = _simpleCRC16Ptr->calcuCheckSum(
            (unsigned char*)(en_msg.buffers.data() + frameDeviceTypeOffset),
            offset - frameDeviceTypeOffset, true);

            uint16Endian.toTargetEndianArray(
            checkSum, en_msg.buffers.data() + offset, sizeof(uint16_t),
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(uint16_t);

            // 帧尾
            memcpy(en_msg.buffers.data() + offset, end, sizeof(end));
            offset += sizeof(end);

            en_msg.offsets.push_back(dataMsgOffset); // 存起点
            en_msg.lengths.push_back(offset - dataMsgOffset); // 存长度
            en_msg.msgCntMap[native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_V2R_DATA_MESSAGE] = 1;
            en_msg.types.push_back(native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_V2R_DATA_MESSAGE);
        }

        {
            // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
            // "RUN HERE 2" << std::endl;
            int dataMsgOffset = 0;
            if (en_msg.offsets.size() > 0) {
                dataMsgOffset = en_msg.offsets[en_msg.offsets.size() - 1] + en_msg.lengths[en_msg.lengths.size() - 1];
            }

            // 帧头
            offset = dataMsgOffset;
            memcpy(en_msg.buffers.data() + offset, start, sizeof(start));
            offset += sizeof(start);

            // 设备类型: 固定为激光雷达
            int frameDeviceTypeOffset = offset;
            char deviceType = 0x01;
            en_msg.buffers[offset] = deviceType;
            offset += 1;

            // 状态帧类型
            char messageType = 0x01;
            en_msg.buffers[offset] = messageType;
            offset += 1;

            // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
            // "RUN HERE 2" << std::endl;

            // 数据帧长度: 当前对单个雷达有效
            uint16_t dataLen = 10;
            uint16Endian.toTargetEndianArray(dataLen, en_msg.buffers.data() + offset, sizeof(uint16_t),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(uint16_t);

            // 设备ID
            unsigned long long int deviceId = _customParams.device_id;
            uint64Endian.toTargetEndianArray(deviceId, en_msg.buffers.data() + offset,
                                             sizeof(unsigned long long int),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(unsigned long long int);

            // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
            // "RUN HERE 2" << std::endl;

            // 时间戳　
            unsigned long long int timestamp = msg->rs_lidar_result_ptr->timestamp * 1000;  // s => ms
            uint64Endian.toTargetEndianArray(timestamp, en_msg.buffers.data() + offset,
                                             sizeof(unsigned long long int),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(unsigned long long int);

            // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
            // "RUN HERE 2" << std::endl;

            // 设备ID
            uint64Endian.toTargetEndianArray(deviceId, en_msg.buffers.data() + offset,
                                             sizeof(unsigned long long int),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(unsigned long long int);

            // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
            // "RUN HERE 2" << std::endl;

            // 设备状态
            uint16_t status = 0x0000;
            uint16Endian.toTargetEndianArray(status, en_msg.buffers.data() + offset, sizeof(uint16_t),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(uint16_t);
            // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
            // "RUN HERE 2" << std::endl;

            // std::cout << __FILE__ << "=> " << __FUNCTION__ << ": " << __LINE__ <<
            // "RUN HERE 2" << std::endl; 校验位: 设备类型 <-> 数据区
            uint16_t checkSum = _simpleCRC16Ptr->calcuCheckSum(
            (unsigned char*)(en_msg.buffers.data() + frameDeviceTypeOffset),
            offset - frameDeviceTypeOffset, true);

            uint16Endian.toTargetEndianArray(checkSum, en_msg.buffers.data() + offset, sizeof(uint16_t),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(uint16_t);

            // 帧尾
            memcpy(en_msg.buffers.data() + offset, end, sizeof(end));
            offset += sizeof(end);

            en_msg.offsets.push_back(dataMsgOffset);
            en_msg.lengths.push_back(offset - dataMsgOffset);
            en_msg.msgCntMap[native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_V2R_STATUS_MESSAGE] = 1;
            en_msg.types.push_back(native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_V2R_STATUS_MESSAGE);
        }

        return 0;
    }

    virtual int serialize(char* data, const robosense::perception::Object::Ptr& object) {
        (void)(data);
        (void)(object);
        return 0;
    }

    int deserialize(const char *data, const int length, const native_sdk_3_1::ROBO_MSG_TYPE msgType,
                    native_sdk_3_1::st_Robov2rRecvMessage& recvMsg){

        if (msgType ==native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_V2R_DATA_MESSAGE) {
            if (length < 0) {
                return -1;
            }

            RSEndian<unsigned long long int> uint64Endian;
            RSEndian<uint16_t> uint16Endian;

            int offset = 0;
            
            // 帧头
            char *begin = new char[2];             
            memcpy(begin, data + offset, 2);
             
            offset += 2;
            //std::cout<<begin<<std::endl; 

            delete[] begin;
            
            // 设备类型: 固定为激光雷达
            char dev_type ; 
            memcpy(&dev_type, data + offset, 1);
            offset += 1;
            //std::cout<<int(dev_type)<<std::endl; 
        

            // 数据帧类型
            char data_type; 
            memcpy(&data_type, data + offset, 1);
            offset += 1;
        
            // 数据的长度信息
            uint16_t dlen;
            uint16Endian.toHostEndianValue(dlen, data + offset ,sizeof(uint16_t),
                                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            int objectCnt;
            if(_customParams.sVersion == "ROBOSENSE_V2R_V1.6"){
                objectCnt = dlen / 76;
            }
            else if (_customParams.sVersion == "ROBOSENSE_V2R_V1.5"){
                objectCnt = dlen / 60;
            }
            else {
                objectCnt = dlen / 56;
            }
            
            offset += sizeof(uint16_t);
            //std::cout<<"dlen::"<<dlen<<"objectcnt::"<<objectCnt<<std::endl;     
            // 设备ID
            unsigned long long int deviceId ;
            uint64Endian.toHostEndianValue(deviceId, data+ offset,
                                             sizeof(unsigned long long int),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(unsigned long long int);

            // 时间戳　
            unsigned long long int timestamp;
            uint64Endian.toHostEndianValue(timestamp,data + offset,sizeof(unsigned long long int),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            recvMsg.msg->rs_lidar_result_ptr->timestamp = static_cast<double>(timestamp / 1000.0);
        
            offset += sizeof(unsigned long long int);

            // 反序列化障碍物(s)
            for (int i = 0; i < objectCnt; ++i) {

                robosense::perception::Object::Ptr v2r_object;
                v2r_object.reset(new robosense::perception::Object());
                int objectOffset = deserialize(data+offset,dlen,v2r_object);
                
                offset += objectOffset;

                recvMsg.msg->rs_lidar_result_ptr->objects.push_back(v2r_object);

            }

            // 校验位

            uint16_t checkSum ;
            uint16Endian.toHostEndianValue(checkSum, data + offset, sizeof(uint16_t),
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            offset += sizeof(uint16_t);

            // 帧尾
            char *end = new char[2];             
            memcpy(end, data + offset, 2);
             
            offset += 2;
            //std::cout<<end<<std::endl; 

            delete[] end;
        }

        if (msgType == native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_V2R_STATUS_MESSAGE) {
            if (length < 0) {
                return -1;
            }
            RSEndian<unsigned long long int> uint64Endian;
            RSEndian<uint16_t> uint16Endian;

            int offset = 0;
            // 帧头
            char *begin = new char[2];             
            memcpy(begin, data + offset, 2);
             
            offset += 2;
            //std::cout<<begin<<std::endl; 

            delete[] begin;
            
            // 设备类型: 固定为激光雷达

            char dev_type ; 
            memcpy(&dev_type, data + offset, 1);
            offset += 1;
            //std::cout<<int(dev_type)<<std::endl; 
        

            // 数据帧类型
            char data_type; 
            memcpy(&data_type, data + offset, 1);
            offset += 1;

            // 数据的长度信息
            
            uint16_t dlen;
            uint16Endian.toHostEndianValue(dlen, data + offset ,sizeof(uint16_t),
                                    RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            
            offset += sizeof(uint16_t);

            // 设备ID
            unsigned long long int deviceId ;
            uint64Endian.toHostEndianValue(deviceId, data+ offset,
                                             sizeof(unsigned long long int),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(unsigned long long int);

            // 时间戳　
            unsigned long long int timestamp;
            uint64Endian.toHostEndianValue(timestamp,data + offset,sizeof(unsigned long long int),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            recvMsg.msg->rs_lidar_result_ptr->timestamp = static_cast<double>(timestamp/1000.0);
        
            offset += sizeof(unsigned long long int);

            // 设备ID
            uint64Endian.toHostEndianValue(deviceId, data + offset,
                                             sizeof(unsigned long long int),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(unsigned long long int);

            // 设备状态
            uint16_t status ;
            uint16Endian.toHostEndianValue(status, data + offset, sizeof(uint16_t),
                                             RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(uint16_t);

            // 校验位

            uint16_t checkSum ;
            uint16Endian.toHostEndianValue(checkSum, data + offset, sizeof(uint16_t),
            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

            offset += sizeof(uint16_t);

            // 帧尾
            char *end = new char[2];             
            memcpy(end, data + offset, 2);
             
            offset += 2;
            //std::cout<<end<<std::endl; 

            delete[] end;
        }
        return 0;
    }

    virtual int deserialize( const char * data, int length,robosense::perception::Object::Ptr& object)  {
        (void)(data);
        (void)(object);
        (void)(length);
        return 0;
    }
protected:
    template <typename T>
    std::string num2str(const T num, int precision) {
        std::stringstream ss;
        ss.setf(std::ios::fixed, std::ios::floatfield);
        ss.precision(precision + 1);
        std::string st;
        ss << num;
        ss >> st;

        return st.substr(0, st.size() - 1);
    }

    // 将以正东为参考，逆时针旋转为正 => 以正北为参考，顺时针为正
    float eastYaw2NorthYaw(float eastYaw) {
        float northYaw = (90.0f - eastYaw > 1e-3) ? (90.0f - eastYaw) : (90.0f - eastYaw + 360.f);

        return northYaw;
    }

    // 将以正北为参考，顺时针为正　=> 正东为参考，逆时针为正
    float northYaw2EastYaw(float northYaw) {
        float eastYaw = (90.0f - northYaw > 1e-3) ? (90.0f - northYaw) : (90.0f - northYaw + 360.f);

        return eastYaw;
    }

protected:
    RSSimpleCRC16::Ptr _simpleCRC16Ptr;
    RsCustomRobosenseV2RMsgParams _customParams;
};

class RsRobosenseV2RVersion1_4Serialize : public RsBaseRobosenseV2RSerialize {
public:
    using Ptr = std::shared_ptr<RsRobosenseV2RVersion1_4Serialize>;
    using ConstPtr = std::shared_ptr<const RsRobosenseV2RVersion1_4Serialize>;

public:
    int serialize(char* data, const robosense::perception::Object::Ptr& object) override {
        int offset = 0;
        RSEndian<double> doubleEndian;
        RSEndian<float> floatEndian;
        RSEndian<uint16_t> uint16Endian;
        RSEndian<unsigned int> uint32Endian;

        // 障碍物区域
        uint16_t roiType = 0xFFFF;
        roiType = static_cast<uint16_t>(object->supplement_infos_.roi_id);
        uint16Endian.toTargetEndianArray(roiType, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        // 障碍物类型　
        uint16_t objectType = static_cast<int>(object->core_infos_.type);
        uint16Endian.toTargetEndianArray(objectType, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        // 障碍物ID
        unsigned int tracker_id;
        if (object->core_infos_.tracker_id == -1) {
            tracker_id = 0xFFFFFFFF;
        }
        else {
            tracker_id = object->core_infos_.tracker_id;
        }
        uint32Endian.toTargetEndianArray(
        tracker_id, data + offset, sizeof(unsigned int),
        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(unsigned int);

        // 障碍物长度　
        floatEndian.toTargetEndianArray(object->core_infos_.size.x, data + offset,
                                        sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物宽度　
        floatEndian.toTargetEndianArray(object->core_infos_.size.y, data + offset,
                                        sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物高度　
        floatEndian.toTargetEndianArray(object->core_infos_.size.z, data + offset,
                                        sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物朝向:
        float directionYaw = std::atan2(object->core_infos_.direction.y, object->core_infos_.direction.x) *
                             180.0f / M_PI;
        if (directionYaw < 0.0) {
            directionYaw += 360.0f;
        }

        if (std::abs(directionYaw - 360.f) < 0.05f) {
            directionYaw = 0.0f;
        }
        directionYaw = eastYaw2NorthYaw(directionYaw);

        floatEndian.toTargetEndianArray(directionYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物速度　
        RsVector3f velocity = object->core_infos_.velocity;
        velocity.z = 0.0f;
        float velocityNorm = velocity.norm();
        floatEndian.toTargetEndianArray(velocityNorm, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物航向(速度方向):速度大于阈值的时候，使用速度方向计算航向；反之，使用朝向表示航向。
        float velocityYaw = 0.0f;
        if (velocityNorm > _customParams.headingConfig.velocityHeadingTh) {
            velocityYaw = std::atan2(velocity.y, velocity.x) * 180.0f / M_PI;
        }
        else {
            velocityYaw = std::atan2(object->core_infos_.direction.y, object->core_infos_.direction.x) * 180.0f / M_PI;
        }

        if (velocityYaw < 0.0f) {
            velocityYaw += 360.0f;
        }

        if (std::abs(velocityYaw - 360.0f) < 0.05f) {
            velocityYaw = 0.0f;
        }

        velocityYaw = eastYaw2NorthYaw(velocityYaw);
        floatEndian.toTargetEndianArray(velocityYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物加速度 / 加速度方向　
        if (_customParams.headingConfig.accelerationValid) {
            RsVector3f acceleration = object->core_infos_.acceleration;
            acceleration.z = 0.0f;
            float accelerationNorm = acceleration.norm();
            floatEndian.toTargetEndianArray(accelerationNorm, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(float);

            float accelerationYaw = 0.0f;
            if (accelerationNorm > _customParams.headingConfig.accelerationHeadingTh) {
                accelerationYaw = std::atan2(acceleration.y, acceleration.x) * 180.0f / M_PI;

                if (accelerationYaw < 0.0f) {
                  accelerationYaw += 360.0f;
                }
            }

            if (std::abs(accelerationYaw - 360.0f) < 0.05f) {
                accelerationYaw = 0.0f;
            }
            accelerationYaw = eastYaw2NorthYaw(accelerationYaw);
            floatEndian.toTargetEndianArray(accelerationYaw, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(float);
        }
        else {
            float accelerationNorm = -1;
            float accelerationYaw = -1;

            floatEndian.toTargetEndianArray(accelerationNorm, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(float);

            floatEndian.toTargetEndianArray(accelerationYaw, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(float);
        }

        // 障碍物经度
        doubleEndian.toTargetEndianArray(object->supplement_infos_.gps_longtitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        // 障碍物纬度　
        doubleEndian.toTargetEndianArray(object->supplement_infos_.gps_latitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        return offset;
    }
    int deserialize( const char * data, int length,robosense::perception::Object::Ptr& object) override{

        if (length < 0) {
            return -1;
        }
        int offset=0;
        
        RSEndian<double> doubleEndian;
        RSEndian<float> floatEndian;
        RSEndian<uint16_t> uint16Endian;
        RSEndian<unsigned int> uint32Endian;

        // 障碍物区域
        uint16_t robo_supplement_roi_id;
        uint16Endian.toHostEndianValue(robo_supplement_roi_id, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);
        object->supplement_infos_.roi_id = robo_supplement_roi_id;

        // 障碍物类型　

        uint16_t objectType;
        uint16Endian.toHostEndianValue(objectType, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                                         
        object->core_infos_.type = static_cast<ObjectType>(objectType);
        offset += sizeof(uint16_t);
        //std::cout<<objectType<<std::endl;

        // 障碍物ID
        unsigned int tracker_id;
        uint32Endian.toHostEndianValue(tracker_id, data + offset, sizeof(unsigned int),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
       
        object->core_infos_.tracker_id = static_cast<int>(tracker_id);
        offset += sizeof(unsigned int);
        //std::cout<<tracker_id<<std::endl;



        // 障碍物长度　
        floatEndian.toHostEndianValue(object->core_infos_.size.x, data + offset,
                                    sizeof(float),RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物宽度　
        floatEndian.toHostEndianValue(object->core_infos_.size.y, data + offset,
                                    sizeof(float),RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物高度　
        floatEndian.toHostEndianValue(object->core_infos_.size.z, data + offset,
                                    sizeof(float),RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物朝向:
        float directionYaw;
        floatEndian.toHostEndianValue(directionYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        object->core_infos_.direction.y = directionYaw;
        offset += sizeof(float);

        // 障碍物速度　

        float velocityNorm ;
        floatEndian.toHostEndianValue(velocityNorm, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        
        object->core_infos_.velocity.x = velocityNorm;
        offset += sizeof(float);
        // 障碍物航向(速度方向):速度大于阈值的时候，使用速度方向计算航向；反之，使用朝向表示航向。

        float velocityYaw ;
        floatEndian.toHostEndianValue(velocityYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        
        object->core_infos_.velocity.y = velocityYaw;
        offset += sizeof(float);
        // 障碍物加速度 / 加速度方向　

        float accelerationNorm,accelerationYaw;
        floatEndian.toHostEndianValue(accelerationNorm, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        object->core_infos_.acceleration.x = accelerationNorm;
        offset += sizeof(float);

        floatEndian.toHostEndianValue(accelerationYaw, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        object->core_infos_.acceleration.y = accelerationYaw;
        offset += sizeof(float);


        // 障碍物经度
        doubleEndian.toHostEndianValue(object->supplement_infos_.gps_longtitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        // 障碍物纬度　
        doubleEndian.toHostEndianValue(object->supplement_infos_.gps_latitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        return offset;
    }
};

class RsRobosenseV2RVersion1_5Serialize : public RsBaseRobosenseV2RSerialize {
public:
    using Ptr = std::shared_ptr<RsRobosenseV2RVersion1_5Serialize>;
    using ConstPtr = std::shared_ptr<const RsRobosenseV2RVersion1_5Serialize>;

public:
    int serialize(char* data, const robosense::perception::Object::Ptr& object) override {
        int offset = 0;
        RSEndian<double> doubleEndian;
        RSEndian<float> floatEndian;
        RSEndian<uint16_t> uint16Endian;
        RSEndian<unsigned int> uint32Endian;

        // 障碍物区域
        uint16_t roiType = 0xFFFF;
        roiType = static_cast<uint16_t>(object->supplement_infos_.roi_id);
        uint16Endian.toTargetEndianArray(roiType, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        // 障碍物类型　
        uint16_t objectType = static_cast<int>(object->core_infos_.type);
        uint16Endian.toTargetEndianArray(objectType, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        // 障碍物ID
        unsigned int tracker_id;
        if (object->core_infos_.tracker_id == -1) {
            tracker_id = 0xFFFFFFFF;
        }
        else {
            tracker_id = object->core_infos_.tracker_id;
        }
        uint32Endian.toTargetEndianArray(tracker_id, data + offset, sizeof(unsigned int),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(unsigned int);

        // 障碍物长度　
        floatEndian.toTargetEndianArray(object->core_infos_.size.x, data + offset,
                                        sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物宽度　
        floatEndian.toTargetEndianArray(object->core_infos_.size.y, data + offset,
                                        sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物高度　
        floatEndian.toTargetEndianArray(object->core_infos_.size.z, data + offset,
                                        sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物朝向:
        float directionYaw = std::atan2(object->core_infos_.direction.y, object->core_infos_.direction.x) * 180.0f / M_PI;
        if (directionYaw < 0.0) {
            directionYaw += 360.0f;
        }

        if (std::abs(directionYaw - 360.f) < 0.05f) {
            directionYaw = 0.0f;
        }
        directionYaw = eastYaw2NorthYaw(directionYaw);

        floatEndian.toTargetEndianArray(directionYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物速度　
        RsVector3f velocity = object->core_infos_.velocity;
        velocity.z = 0.0f;
        float velocityNorm = velocity.norm();
        floatEndian.toTargetEndianArray(velocityNorm, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物航向(速度方向):速度大于阈值的时候，使用速度方向计算航向；反之，使用朝向表示航向。
        float velocityYaw = 0.0f;
        if (velocityNorm > _customParams.headingConfig.velocityHeadingTh) {
            velocityYaw = std::atan2(velocity.y, velocity.x) * 180.0f / M_PI;
        }
        else {
            velocityYaw = std::atan2(object->core_infos_.direction.y, object->core_infos_.direction.x) * 180.0f / M_PI;
        }

        if (velocityYaw < 0.0f) {
            velocityYaw += 360.0f;
        }

        if (std::abs(velocityYaw - 360.0f) < 0.05f) {
            velocityYaw = 0.0f;
        }
        velocityYaw = eastYaw2NorthYaw(velocityYaw);
        floatEndian.toTargetEndianArray(velocityYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物加速度 / 加速度方向　
        if (_customParams.headingConfig.accelerationValid) {
            RsVector3f acceleration = object->core_infos_.acceleration;
            acceleration.z = 0.0f;
            float accelerationNorm = acceleration.norm();
            floatEndian.toTargetEndianArray(accelerationNorm, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(float);

            float accelerationYaw = 0.0f;
            if (accelerationNorm > _customParams.headingConfig.accelerationHeadingTh) {
                accelerationYaw = std::atan2(acceleration.y, acceleration.x) * 180.0f / M_PI;

                if (accelerationYaw < 0.0f) {
                    accelerationYaw += 360.0f;
                }
            }

            if (std::abs(accelerationYaw - 360.0f) < 0.05f) {
                accelerationYaw = 0.0f;
            }
            accelerationYaw = eastYaw2NorthYaw(accelerationYaw);
            floatEndian.toTargetEndianArray(accelerationYaw, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(float);
        }
        else {
            float accelerationNorm = -1;
            float accelerationYaw = -1;

            floatEndian.toTargetEndianArray(accelerationNorm, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(float);

            floatEndian.toTargetEndianArray(accelerationYaw, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(float);
        }

        // 障碍物经度
        doubleEndian.toTargetEndianArray(object->supplement_infos_.gps_longtitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        // 障碍物纬度　
        doubleEndian.toTargetEndianArray(object->supplement_infos_.gps_latitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        // 相对距离
        float relativeDist = std::sqrt(std::pow(object->core_infos_.center.x, 2) +
                                       std::pow(object->core_infos_.center.y, 2));
        floatEndian.toTargetEndianArray(relativeDist, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        return offset;
    }
    int deserialize( const char * data, int length,robosense::perception::Object::Ptr& object) override{

        if (length < 0) {
            return -1;
        }
        int offset=0;
        
        RSEndian<double> doubleEndian;
        RSEndian<float> floatEndian;
        RSEndian<uint16_t> uint16Endian;
        RSEndian<unsigned int> uint32Endian;

        // 障碍物区域
        uint16_t robo_supplement_roi_id;
        uint16Endian.toHostEndianValue(robo_supplement_roi_id, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);
        object->supplement_infos_.roi_id = robo_supplement_roi_id;

        // 障碍物类型　

        uint16_t objectType;
        uint16Endian.toHostEndianValue(objectType, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                                         
        object->core_infos_.type = static_cast<ObjectType>(objectType);
        offset += sizeof(uint16_t);
        //std::cout<<objectType<<std::endl;

        // 障碍物ID
        unsigned int tracker_id;
        uint32Endian.toHostEndianValue(tracker_id, data + offset, sizeof(unsigned int),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
       
        object->core_infos_.tracker_id = static_cast<int>(tracker_id);
        offset += sizeof(unsigned int);
        //std::cout<<tracker_id<<std::endl;

        // 障碍物长度　
        floatEndian.toHostEndianValue(object->core_infos_.size.x, data + offset,
                                    sizeof(float),RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物宽度　
        floatEndian.toHostEndianValue(object->core_infos_.size.y, data + offset,
                                    sizeof(float),RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物高度　
        floatEndian.toHostEndianValue(object->core_infos_.size.z, data + offset,
                                    sizeof(float),RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物朝向:
        float directionYaw;
        floatEndian.toHostEndianValue(directionYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        object->core_infos_.direction.y = directionYaw;
        offset += sizeof(float);

        // 障碍物速度　

        float velocityNorm ;
        floatEndian.toHostEndianValue(velocityNorm, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        
        object->core_infos_.velocity.x = velocityNorm;
        offset += sizeof(float);
        // 障碍物航向(速度方向):速度大于阈值的时候，使用速度方向计算航向；反之，使用朝向表示航向。

        float velocityYaw ;
        floatEndian.toHostEndianValue(velocityYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        
        object->core_infos_.velocity.y = velocityYaw;
        offset += sizeof(float);
        // 障碍物加速度 / 加速度方向　

        float accelerationNorm,accelerationYaw;
        floatEndian.toHostEndianValue(accelerationNorm, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        object->core_infos_.acceleration.x = accelerationNorm;
        offset += sizeof(float);

        floatEndian.toHostEndianValue(accelerationYaw, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        object->core_infos_.acceleration.y = accelerationYaw;
        offset += sizeof(float);

        // 障碍物经度
        doubleEndian.toHostEndianValue(object->supplement_infos_.gps_longtitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        // 障碍物纬度　
        doubleEndian.toHostEndianValue(object->supplement_infos_.gps_latitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);


        // 相对距离
        floatEndian.toHostEndianValue(object->core_infos_.center.y, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        return offset;
    }
};

class RsRobosenseV2RVersion1_6Serialize : public RsBaseRobosenseV2RSerialize {
public:
    using Ptr = std::shared_ptr<RsRobosenseV2RVersion1_6Serialize>;
    using ConstPtr = std::shared_ptr<const RsRobosenseV2RVersion1_6Serialize>;

public:
    int serialize(char* data, const robosense::perception::Object::Ptr& object) override {
        int offset = 0;
        RSEndian<double> doubleEndian;
        RSEndian<float> floatEndian;
        RSEndian<uint16_t> uint16Endian;
        RSEndian<unsigned int> uint32Endian;

        // 障碍物区域
        uint16_t roiType = 0xFFFF;
        roiType = static_cast<uint16_t>(object->supplement_infos_.roi_id);
        uint16Endian.toTargetEndianArray(roiType, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        // 障碍物类型　
        uint16_t objectType = static_cast<int>(object->core_infos_.type);
        uint16Endian.toTargetEndianArray(objectType, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);

        // 障碍物ID
        unsigned int tracker_id;
        if (object->core_infos_.tracker_id == -1) {
            tracker_id = 0xFFFFFFFF;
        }
        else {
            tracker_id = object->core_infos_.tracker_id;
        }
        uint32Endian.toTargetEndianArray(tracker_id, data + offset, sizeof(unsigned int),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(unsigned int);

        float centerX, centerY, centerZ;
        if (_customParams.center == ROBOSENSE_V2R_CENTER_TYPE::ROBOSENSE_V2R_CENTER) {
            centerX = object->core_infos_.center.x;
            centerY = object->core_infos_.center.y;
            centerZ = object->core_infos_.center.z;
        }
        else if (_customParams.center == ROBOSENSE_V2R_CENTER_TYPE::ROBOSENSE_V2R_ANCHOR) {
            centerX = object->core_infos_.anchor.x;
            centerY = object->core_infos_.anchor.y;
            centerZ = object->core_infos_.anchor.z;
        }
        else {
            centerX = object->core_infos_.center.x;
            centerY = object->core_infos_.center.y;
            centerZ = object->core_infos_.center.z;
        }

        // 障碍物中心点: x
        floatEndian.toTargetEndianArray(centerX, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物中心点: y
        floatEndian.toTargetEndianArray(centerY, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物中心点: z
        floatEndian.toTargetEndianArray(centerZ, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物长度　
        floatEndian.toTargetEndianArray(object->core_infos_.size.x, data + offset,
                                    sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物宽度　
        floatEndian.toTargetEndianArray(object->core_infos_.size.y, data + offset,
                                    sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物高度　
        floatEndian.toTargetEndianArray(object->core_infos_.size.z, data + offset,
                                    sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物朝向:
        float directionYaw = std::atan2(object->core_infos_.direction.y, object->core_infos_.direction.x) * 180.0f / M_PI;
        if (directionYaw < 0.0) {
            directionYaw += 360.0f;
        }

        if (std::abs(directionYaw - 360.f) < 0.05f) {
            directionYaw = 0.0f;
        }

        directionYaw = eastYaw2NorthYaw(directionYaw);

        floatEndian.toTargetEndianArray(directionYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物速度　
        RsVector3f velocity = object->core_infos_.velocity;
        velocity.z = 0.0f;
        float velocityNorm = velocity.norm();
        floatEndian.toTargetEndianArray(velocityNorm, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);
        // std::cout << "velocityNorm = " << velocityNorm << std::endl;

        // 障碍物航向(速度方向):速度大于阈值的时候，使用速度方向计算航向；反之，使用朝向表示航向。
        float velocityYaw = 0.0f;
        if (velocityNorm > _customParams.headingConfig.velocityHeadingTh) {
            velocityYaw = std::atan2(velocity.y, velocity.x) * 180.0f / M_PI;
        }
        else {
            velocityYaw = std::atan2(object->core_infos_.direction.y, object->core_infos_.direction.x) * 180.0f / M_PI;
        }

        if (velocityYaw < 0.0f) {
            velocityYaw += 360.0f;
        }

        if (std::abs(velocityYaw - 360.0f) < 0.05f) {
            velocityYaw = 0.0f;
        }

        velocityYaw = eastYaw2NorthYaw(velocityYaw);
        floatEndian.toTargetEndianArray(velocityYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物加速度 / 加速度方向　
        if (_customParams.headingConfig.accelerationValid) {
        RsVector3f acceleration = object->core_infos_.acceleration;
        acceleration.z = 0.0f;
        float accelerationNorm = acceleration.norm();
        floatEndian.toTargetEndianArray(accelerationNorm, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        float accelerationYaw = 0.0f;
        if (accelerationNorm > _customParams.headingConfig.accelerationHeadingTh) {
            accelerationYaw = std::atan2(acceleration.y, acceleration.x) * 180.0f / M_PI;

            if (accelerationYaw < 0.0f) {
                accelerationYaw += 360.0f;
            }
        }

        if (std::abs(accelerationYaw - 360.0f) < 0.05f) {
            accelerationYaw = 0.0f;
        }

        accelerationYaw = eastYaw2NorthYaw(accelerationYaw);
        floatEndian.toTargetEndianArray(accelerationYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);
        }
        else {
            float accelerationNorm = -1;
            float accelerationYaw = -1;

            floatEndian.toTargetEndianArray(accelerationNorm, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(float);

            floatEndian.toTargetEndianArray(accelerationYaw, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
            offset += sizeof(float);
        }

        // 障碍物经度
        doubleEndian.toTargetEndianArray(object->supplement_infos_.gps_longtitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        // 障碍物纬度　
        doubleEndian.toTargetEndianArray(object->supplement_infos_.gps_latitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        // 障碍物高程　
        doubleEndian.toTargetEndianArray(object->supplement_infos_.gps_altitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        return offset;
    }
    int deserialize( const char * data, int length,robosense::perception::Object::Ptr& object) override{
        if (length < 0) {
            return -1;
        }
        int offset = 0;
        
        RSEndian<double> doubleEndian;
        RSEndian<float> floatEndian;
        RSEndian<uint16_t> uint16Endian;
        RSEndian<unsigned int> uint32Endian;

        // 障碍物区域
        uint16_t robo_supplement_roi_id;
        uint16Endian.toHostEndianValue(robo_supplement_roi_id, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(uint16_t);
        object->supplement_infos_.roi_id = robo_supplement_roi_id;

        // 障碍物类型　
        uint16_t objectType;
        uint16Endian.toHostEndianValue(objectType, data + offset, sizeof(uint16_t),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
                                         
        object->core_infos_.type = static_cast<ObjectType>(objectType);
        offset += sizeof(uint16_t);
        //std::cout<<objectType<<std::endl;

        // 障碍物ID
        unsigned int tracker_id;
        uint32Endian.toHostEndianValue(tracker_id, data + offset, sizeof(unsigned int),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
       
        object->core_infos_.tracker_id = tracker_id;
        offset += sizeof(unsigned int);
        //std::cout<<tracker_id<<std::endl;

        // 障碍物中心点: x
        floatEndian.toHostEndianValue(object->core_infos_.center.x, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);
        
        // 障碍物中心点: y
        floatEndian.toHostEndianValue(object->core_infos_.center.y, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        offset += sizeof(float);

        // 障碍物中心点: z
        floatEndian.toHostEndianValue(object->core_infos_.center.z, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);

        offset += sizeof(float);

        // 障碍物长度　
        floatEndian.toHostEndianValue(object->core_infos_.size.x, data + offset,
                                    sizeof(float),RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物宽度　
        floatEndian.toHostEndianValue(object->core_infos_.size.y, data + offset,
                                    sizeof(float),RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物高度　
        floatEndian.toHostEndianValue(object->core_infos_.size.z, data + offset,
                                    sizeof(float),RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(float);

        // 障碍物朝向:
        float directionYaw;
        floatEndian.toHostEndianValue(directionYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        object->core_infos_.direction.y = directionYaw;
        offset += sizeof(float);

        // 障碍物速度　
        float velocityNorm ;
        floatEndian.toHostEndianValue(velocityNorm, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        
        object->core_infos_.velocity.x = velocityNorm;
        offset += sizeof(float);
        // 障碍物航向(速度方向):速度大于阈值的时候，使用速度方向计算航向；反之，使用朝向表示航向。

        float velocityYaw ;
        floatEndian.toHostEndianValue(velocityYaw, data + offset, sizeof(float),
                                        RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        
        object->core_infos_.velocity.y = velocityYaw;
        offset += sizeof(float);
        // 障碍物加速度 / 加速度方向　

        float accelerationNorm,accelerationYaw;
        floatEndian.toHostEndianValue(accelerationNorm, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        object->core_infos_.acceleration.x = accelerationNorm;
        offset += sizeof(float);

        floatEndian.toHostEndianValue(accelerationYaw, data + offset, sizeof(float),
                                            RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        object->core_infos_.acceleration.y = accelerationYaw;
        offset += sizeof(float);

        // 障碍物经度
        doubleEndian.toHostEndianValue(object->supplement_infos_.gps_longtitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        // 障碍物纬度　
        doubleEndian.toHostEndianValue(object->supplement_infos_.gps_latitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);

        // 障碍物高程　
        doubleEndian.toHostEndianValue(object->supplement_infos_.gps_altitude,
                                         data + offset, sizeof(double),
                                         RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN);
        offset += sizeof(double);
        
        return offset;
    }
};

class RsRobosenseV2RSerializeFactory {
public:
    using Ptr = std::shared_ptr<RsRobosenseV2RSerializeFactory>;
    using ConstPtr = std::shared_ptr<const RsRobosenseV2RSerializeFactory>;

    static RsBaseRobosenseV2RSerialize::Ptr getInstance(const ROBOSENSE_V2R_VERSION version) {
        RsBaseRobosenseV2RSerialize::Ptr baseSerializePtr;
        switch (version) {
            case ROBOSENSE_V2R_VERSION::ROBOSENSE_V2R_V1_4: {
                baseSerializePtr.reset(new RsRobosenseV2RVersion1_4Serialize());
                break;
            }
            case ROBOSENSE_V2R_VERSION::ROBOSENSE_V2R_V1_5: {
                baseSerializePtr.reset(new RsRobosenseV2RVersion1_5Serialize());
                break;
            }
            case ROBOSENSE_V2R_VERSION::ROBOSENSE_V2R_V1_6: {
                baseSerializePtr.reset(new RsRobosenseV2RVersion1_6Serialize());
                break;
            }
            default: {
                baseSerializePtr.reset(new RsRobosenseV2RVersion1_6Serialize());
                break;
            }
        }
        return baseSerializePtr;
    }
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_V2R_ROBOSENSE_V2R_CUSTOM_TRANSFORMER_H_