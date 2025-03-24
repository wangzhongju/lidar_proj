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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_XINGYUN_CUSTOM_MSG_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_XINGYUN_CUSTOM_MSG_H_

#include "rs_perception/custom/robosense_custom/xingyunCustom/xingyun_custom_transformer.h"
#include <netinet/in.h>

#ifdef RS_PROTO_FOUND

namespace robosense
{
    namespace perception
    {

        class XingyunMsg
        {
        public:
            using Ptr = std::shared_ptr<XingyunMsg>;

            XingyunMsg()
            {
                xingyun_res_ptr.reset(new Xingyun::EventMsg);
            }
            Xingyun::EventMsg::Ptr xingyun_res_ptr;
        };

        class RsXingyunCustomMsg : public RsBaseCustomMsg
        {
        public:
            using Ptr = std::shared_ptr<RsXingyunCustomMsg>;

            RsXingyunCustomMsg()
            {
                msg_.reset(new RsPerceptionMsg);
                xingyun_msg_ptr.reset(new XingyunMsg);
            
            }

            // load configures from yaml and init custom message serialize function.
            // input: yaml node
            void init(const RsYamlNode &config_node_) override
            {
                RsYamlNode general_node, custom_node, control_node;
                rsYamlSubNode(config_node_, "general", general_node);
                rsYamlSubNode(config_node_, "native", custom_node);
                rsYamlSubNode(config_node_, "socket", control_node);

                // general
                rsYamlRead(general_node, "device_id", custom_params_.device_id);

                // custom
                rsYamlRead(custom_node, "enable_upload", custom_params_.enable_upload);
                rsYamlRead(custom_node, "enable_lidar_object", custom_params_.enable_lidar_object);
                rsYamlRead(custom_node, "enable_wgs84_object", custom_params_.enable_wgs84_object);
                rsYamlRead(custom_node, "upload_time_ms", custom_params_.upload_time_ms);
                rsYamlRead(custom_node, "lidar_work_time_ms", custom_params_.lidar_work_time_ms);
                rsYamlRead(custom_node, "keep_alive_time_ms", custom_params_.keep_alive_time_ms);
                rsYamlRead(custom_node, "device_no", custom_params_.device_no);
                rsYamlRead(custom_node, "mec_no", custom_params_.mec_no);

                send_frequent = custom_params_.upload_time_ms / custom_params_.lidar_work_time_ms;

                event_convert_ptr.reset(new Xingyun::EventConvert);
                event_convert_ptr->init();

                std::thread keep_alive_thread(&Xingyun::RSCustomKeepAlive::startKeepAliveTimer,
                                              &keep_alive_, custom_params_);
                keep_alive_thread.detach();
            }

            // entrance of perception message serialize.
            // input: robosense perception message.
            // output: void. the result of serialization will be recorded in an internal variable.
            void serialization(const RsPerceptionMsg::Ptr &msg) override
            {
                if (msg == nullptr)
                {
                    RINFO << "No perception result!";
                    return;
                }
                msg_ = msg;
                auto &any_map = msg_->rs_lidar_result_ptr->any_map;
                if (any_map.find("hd_map_disable") != any_map.end())
                {
                    Any::Ptr any_ptr_hd_map_disable = any_map["hd_map_disable"];
                    auto hd_map_disable = any_ptr_hd_map_disable->AnyCast<bool>();
                    hd_map_disable_ = *hd_map_disable;
                }
                else
                {
                    hd_map_disable_ = true;
                }

                event_convert_ptr->convert(msg_, xingyun_msg_ptr->xingyun_res_ptr, hd_map_disable_);
                serializeToProto(serializeBuffer_, custom_params_, hd_map_disable_);
            }

            void deSerialization(const RsPerceptionMsg::Ptr &msg) override
            {
                // 自行定义转换函数
                (void)(msg);
            }

            void deSerialization(const std::string &msg, std::vector<RsPerceptionMsg::Ptr> &res) override
            {
                (void)(msg);
                (void)(res);
            }

            void serializeToProto(native_sdk_3_1::RSSerializeBuffer_ &en_msg, RsXingyunCustomMsgParams &custom_params, const bool &hd_map_disable)
            {
                en_msg.clearInfo();

                if (!custom_params.enable_upload)
                {
                    RINFO << "upload disable! No perception result upload!";
                    return;
                }

                int offset = 0;

                // 序列化
                if (custom_params.send_object && custom_params.send_event && custom_params.send_traffic)
                {
                    int dataMsgOffset = 0;
                    if (!en_msg.offsets.empty())
                    {
                        dataMsgOffset = *(en_msg.offsets.rbegin()) + *(en_msg.lengths.rbegin());
                    }

                    offset = dataMsgOffset;

// 星云add
#if 1
                    unsigned char send2fusion[81920] = {0};
                    nebulalink::perceptron3::PerceptronSet temp_perceptronset = nebulalink::perceptron3::PerceptronSet();

                    temp_perceptronset.set_devide_id(std::to_string(custom_params.device_id));
                    temp_perceptronset.set_time_stamp(std::floor(msg_->rs_lidar_result_ptr->timestamp * 1000));
                    temp_perceptronset.set_number_frame(frame_id);

                    nebulalink::perceptron3::PointGPS *temp_perception_gps_ptr = temp_perceptronset.mutable_perception_gps();
                    temp_perception_gps_ptr->set_object_longitude(msg_->rs_lidar_result_ptr->gps_origin.x);
                    temp_perception_gps_ptr->set_object_latitude(msg_->rs_lidar_result_ptr->gps_origin.y);
                    temp_perception_gps_ptr->set_object_elevation(msg_->rs_lidar_result_ptr->gps_origin.z);

                    Xingyun::ObjectSend info = *(xingyun_msg_ptr->xingyun_res_ptr->obj_send_ptr);

                    if (info.Obj_List.size() == 0) {
                        return ;
                    }

                    for (int i = 0; i < info.Obj_List.size(); i++)
                    {
                        nebulalink::perceptron3::Perceptron *proto_perceptron_ptr = temp_perceptronset.add_perceptron();
                        proto_perceptron_ptr->set_object_class_type(info.Obj_List[i].PtcType);
                        proto_perceptron_ptr->set_object_id(info.Obj_List[i].PtcID);
                        proto_perceptron_ptr->set_object_speed(info.Obj_List[i].PtcSpeed);
                        proto_perceptron_ptr->set_object_heading(info.Obj_List[i].PtcHeading);

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
                        //num_obj++;
                    }
                    /*
                    Xingyun::EventSend info_event = *(xingyun_msg_ptr->xingyun_res_ptr->event_send_ptr);

                    for (int i = 0; i < info_event.Evt_List.size(); i++)
                    {
                        nebulalink::perceptron3::LaneJamSenseParams *proto_lane_jam_sense_params_ptr = temp_perceptronset.add_lane_jam_sense_params();
                        proto_lane_jam_sense_params_ptr->set_lane_id(std::to_string(info_event.Evt_List[i].roiIdx));
                        proto_lane_jam_sense_params_ptr->set_lane_veh_num(info_event.Evt_List[i].vel_num);
                    }
                    */
                    // temp_perceptronset.SerializeToArray(data, temp_perceptronset.ByteSize());
                    temp_perceptronset.SerializeToArray(send2fusion + 8, temp_perceptronset.ByteSize());
                    send2fusion[0] = 0xDA;
                    send2fusion[1] = 0xDB;
                    send2fusion[2] = 0xDC;
                    send2fusion[3] = 0xDD;
                    send2fusion[4] = 0x01;
                    send2fusion[5] = 0x04;
                    *((unsigned short *)(send2fusion + 6)) = htons(temp_perceptronset.ByteSize());

#endif
                    /*
                    std::cout<<"temp_perceptronset.ByteSize():"<<temp_perceptronset.ByteSize()<<std::endl;
                    std::cout<<"send2fusion[6]:";
                    printf("%x",send2fusion[6]);
                    std::cout<<std::endl;
                    
                    std::cout<<"send2fusion[7]:";
                    printf("%x",send2fusion[7]);
                    std::cout<<std::endl;
                    */

                    offset = 8 + temp_perceptronset.ByteSize();

                    en_msg.buffers.resize(offset);
                    
                    memcpy(en_msg.buffers.data(), send2fusion, offset);

                    /*                    
                    for(int i = 0;i<offset;i++){
                        printf("%x",send2fusion[i]);
                        std::cout<<" ";
                        //std::cout<<(int) send2fusion[i]<<" ";

                    }
                    
                    std::cout<<std::endl;

                    nebulalink::perceptron::PerceptronSet result__ = nebulalink::perceptron::PerceptronSet();

                    result__.ParseFromArray(send2fusion + 8,temp_perceptronset.ByteSize());

                    std::cout<<"target_num serialization:"<<info.Obj_List.size()<<std::endl;

                    std::cout<<"target_num deserialization:"<<result__.perceptron().size()<<std::endl;
                    */
                    

                    // 填充完成，保存起止点以及长度等信息
                    en_msg.offsets.push_back(dataMsgOffset);
                    en_msg.lengths.push_back(offset - dataMsgOffset);
                    en_msg.msgCntMap[native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_DTGH_OBJECT] = 1;
                    en_msg.types.push_back(native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_DTGH_OBJECT);
                }

                frame_id++;
            }

            // entrance of getting the result of serialization.
            // input: a vector that used to save the buffers.
            // output: void. the result of serialization is added into the vector with a message header.

            int getSerializeMessage(std::vector<RsCharBufferPtr> &msgs) override
            {
                msgs.clear();
                if (hd_map_disable_)
                {
                    RDEBUG << "hd_map is turned off, only send perception message!";
                }

                size_t typesCnt = serializeBuffer_.types.size();

                for (size_t i = 0; i < typesCnt; ++i)
                {
                    const int msgOffset = serializeBuffer_.offsets[i];
                    const int msgLength = serializeBuffer_.lengths[i];
                    RsCharBufferPtr charBufferPtr(new RsCharBuffer);
                    charBufferPtr->resize(msgLength, '\0');
                    memcpy(charBufferPtr->data(), serializeBuffer_.buffers.data() + msgOffset, msgLength);
                    msgs.push_back(charBufferPtr);
                }

                std::string data;
                RsCharBufferPtr charBufferPtr(new RsCharBuffer());
                std::string new_keep_alive_timestamp;
                std::vector<std::string> keep_alive_tmp = keep_alive_.getTimestampAndRes();
                // 插入心跳信息
                new_keep_alive_timestamp = keep_alive_tmp[0];
                if (keep_alive_timestamp != new_keep_alive_timestamp)
                {
                    keep_alive_timestamp = new_keep_alive_timestamp;
                    data = keep_alive_tmp[1];
                    charBufferPtr->resize(data.size(), '\0');
                    memcpy(charBufferPtr->data(), data.data(), data.size());
                    msgs.push_back(charBufferPtr);
                }

                return 0;
            }


        private:
            RsXingyunCustomMsgParams custom_params_;
            Xingyun::EventConvert::Ptr event_convert_ptr;
            XingyunMsg::Ptr xingyun_msg_ptr;
            bool hd_map_disable_ = false;
            native_sdk_3_1::RSSerializeBuffer_ serializeBuffer_;
            Xingyun::RSCustomKeepAlive keep_alive_;
            std::string keep_alive_timestamp;
            int send_frequent = 0;
            int frame_id = 0;
        };

    }
}
CEREAL_REGISTER_TYPE(robosense::perception::RsXingyunCustomMsg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(robosense::perception::RsBaseCustomMsg,
                                     robosense::perception::RsXingyunCustomMsg)

#endif // RS_PROTO_FOUND
#endif // RS_PERCEPTION_CUSTOM_ROBOSENSE_CUSTOM_XINGYUN_CUSTOM_MSG_H_
