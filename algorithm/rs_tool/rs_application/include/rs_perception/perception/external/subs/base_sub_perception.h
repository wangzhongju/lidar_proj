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

#ifndef RS_PERCEPTION_PERCEPTION_SUB_BASE_SUB_PERCEPTION_H_
#define RS_PERCEPTION_PERCEPTION_SUB_BASE_SUB_PERCEPTION_H_

#include "rs_common/external/common.h"
#include "rs_perception/common/external/common.h"
#include "rs_perception/perception/external/common/simple_perception.h"
#include "rs_perception/lidar/pre_fusion/external/pre_fusion.h"
#include "rs_perception/lidar/road_detection/external/road_detection.h"
#include "rs_perception/lidar/basic_detection/external/basic_detection.h"
#include "rs_perception/lidar/tracking/external/tracking.h"
#include "rs_perception/lidar/refine_filter/external/refine_filter.h"
#include "rs_perception/lidar/post_fusion/external/post_fusion.h"
#include "rs_perception/lidar/scence_struct_detection/external/scence_struct_detection.h"
#include "rs_perception/lidar/postprocessing/external/postprocessing.h"
#include "rs_perception/lidar/map_filter/external/map_filter.h"
#include "rs_perception/lidar/util/external/perception_init.h"
#include "rs_perception/lidar/mirror_detection/external/mirror_detection.h"

namespace robosense {
namespace perception {

class BaseSubPerception {
protected:

    virtual ~BaseSubPerception() {
        stop();
    }

    inline void stop() {
        for (auto itr = thread_worker_map_.begin(); itr != thread_worker_map_.end(); ++itr) {
            itr->second.stop();
        }
    }

    // load configures from yaml and init the perception function.
    // input: yaml node
    inline void initConfig(const RsYamlNode& config_node) {
        // 1.获取车体坐标系信息
        // 2.config_manager信息
        RsYamlNode perception_node = lidar::PerceptionInit().perceptionConfigInit(config_node);
        // 获取general_node
        RsYamlNode general_node;
        rsYamlSubNode(perception_node, "general", general_node);
        std::string out_axis = "GLOBAL_AXIS";
        rsYamlRead(general_node, "out_axis", out_axis);
        out_axis_ = kAxisStatusName2TypeMap.at(out_axis);

        // 获取calibration_node
        RsYamlNode calibration_node;
        rsYamlSubNode(perception_node, "calibration", calibration_node);

        // 获取lidar相关node
        RsYamlNode lidar_node, common_node;
        rsYamlSubNode(perception_node, "lidar", lidar_node);
        rsYamlSubNode(lidar_node, "common", common_node);
        rsYamlRead(common_node, "data_fusion", data_fusion_);
        RsYamlNode sub_node = lidar_node["sub"];
        RsYamlNode authorization_node;
        bool file_key_enable{false};
        if (rsYamlSubNode(general_node, "authorization", authorization_node)) {
            file_key_enable = true;
        }

        if (data_fusion_) {
            // init pre fusion
            RsYamlNode pre_fusion_node;
            rsYamlSubNode(lidar_node, "pre_fusion", pre_fusion_node);
            lidar_pre_fusion_ptr_.reset(new lidar::PreFusion);
            lidar_pre_fusion_ptr_->init(pre_fusion_node);
            // init simple perception
            SimplePerception::Ptr tmp_ptr(new SimplePerception);
            RsYamlNode tmp_node;
            if (sub_node.size() >= 1) {
                tmp_node = sub_node[0];
            }
            if (file_key_enable) {
                tmp_node["authorization"] = authorization_node;
            }
            RsYamlNode hardkey_node;
            if (rsYamlSubNode(general_node, "hardkey", hardkey_node)) {
                tmp_node["hardkey"] = hardkey_node;
            }
            tmp_ptr->init(tmp_node);

            simple_perception_map_[RsConfigManager().getRsMainFrameID()] = tmp_ptr;
        } else {
            // init simple perception
            RsYamlNode cali_lidar_node;
            rsYamlSubNode(calibration_node, "lidar", cali_lidar_node);
            for (size_t i = 0; i < cali_lidar_node.size(); ++i) {
                RsYamlNode tmp_cali_node = cali_lidar_node[i];
                std::string frame_id;
                rsYamlRead(tmp_cali_node, "frame_id", frame_id);
                SimplePerception::Ptr tmp_ptr(new SimplePerception);
                RsYamlNode tmp_node;
                if (sub_node.size() >= i) {
                    tmp_node = sub_node[i];
                }
                if (file_key_enable) {
                    tmp_node["authorization"] = authorization_node;
                }
                RsYamlNode hardkey_node;
                if (rsYamlSubNode(general_node, "hardkey", hardkey_node)) {
                tmp_node["hardkey"] = hardkey_node;
            }
                tmp_ptr->init(tmp_node);
                simple_perception_map_[frame_id] = tmp_ptr;
            }
            // init post fusion
            RsYamlNode post_fusion_node;
            rsYamlSubNode(lidar_node, "post_fusion", post_fusion_node);
            lidar_post_fusion_ptr_.reset(new lidar::PostFusion);
            lidar_post_fusion_ptr_->init(post_fusion_node);
        }

        // multi thread
        thread_worker_map_.clear();
        for (auto itr = simple_perception_map_.begin(); itr != simple_perception_map_.end(); ++itr) {
            const auto& frame_id = itr->first;
            thread_worker_map_[frame_id].bind([frame_id, this](){
                this->simple_perception_map_.at(frame_id)->perception(this->msg_ptr_->sub_lidar_msgs_map.at(frame_id));
                return true;
            });
            thread_worker_map_[frame_id].start();
        }

        // lidar_refine_filter
        {
            RsYamlNode refine_filter_node;
            bool enable_refine = false;
            rsYamlSubNode(lidar_node, "refine_filter", refine_filter_node);
            rsYamlRead(refine_filter_node, "enable", enable_refine);
            if (enable_refine) {
                lidar_refine_filter_ptr_.reset(new lidar::RefineFilter);
                lidar_refine_filter_ptr_->init(refine_filter_node);
            }
        }

        thread_worker_map_module_.clear();

        

        // init scene_struct_detection
        {
            RsYamlNode scene_struct_detection_node;
            bool enable_scene_struct = false;
            rsYamlSubNode(lidar_node, "scene_struct_detection", scene_struct_detection_node);
            rsYamlRead(scene_struct_detection_node, "enable", enable_scene_struct);
            if (enable_scene_struct) {
                scene_struct_detection_ptr_.reset(new lidar::ScenceStructDetection);
                scene_struct_detection_ptr_->init(scene_struct_detection_node);
                thread_worker_map_module_["scene_struct_detection"].bind([this](){
                    this->scene_struct_detection_ptr_->perception(this->msg_ptr_->rs_lidar_result_ptr);
                    Any::Ptr any_ptr;
                    scene_struct_detection_ptr_->getResult(any_ptr);
                    Any::Ptr any_ptr_road_board
                    (new Any(any_ptr->AnyCast<lidar::ScenceStructDetectionMsg>()->road_boards_boxes_));
                    this->msg_ptr_->any_map["road_board"] = any_ptr_road_board;
                    Any::Ptr any_ptr_tunnel(new Any(any_ptr->AnyCast<lidar::ScenceStructDetectionMsg>()->tunnel_box_));
                    this->msg_ptr_->any_map["tunnel"] = any_ptr_tunnel;
                    return true;
                });
                thread_worker_map_module_["scene_struct_detection"].start();
            }
        }

        // lidar_road_detection
        {
            RsYamlNode road_detection_node;
            bool enable_road = false;
            rsYamlSubNode(lidar_node, "road_detection", road_detection_node);
            rsYamlRead(road_detection_node, "enable", enable_road);
            if (enable_road) {
                lidar_road_detection_ptr_.reset(new lidar::RoadDetection);
                lidar_road_detection_ptr_->init(road_detection_node);
                thread_worker_map_module_["road_detection"].bind([this](){
                    this->lidar_road_detection_ptr_->perception(this->msg_ptr_->rs_lidar_result_ptr);
                    Any::Ptr any_ptr;
                    lidar_road_detection_ptr_->getResult(any_ptr);
                    const auto& result_ptr = msg_ptr_->rs_lidar_result_ptr;
                    const auto ptr = any_ptr->AnyCast<lidar::RoadDetectionMsg>();
                    result_ptr->lanes = ptr->lanes;
                    result_ptr->roadedges = ptr->roadedges;
                    return true;
                });
                thread_worker_map_module_["road_detection"].start();
            }
        }

        // lidar_basic_detection
        {
            RsYamlNode basic_detection_node;
            bool enable_basic = false;
            rsYamlSubNode(lidar_node, "basic_detection", basic_detection_node);
            rsYamlRead(basic_detection_node, "enable", enable_basic);
            if (enable_basic) {
                lidar_basic_detection_ptr_.reset(new lidar::BasicDetection);
                lidar_basic_detection_ptr_->init(basic_detection_node);
                thread_worker_map_module_["basic_detection"].bind([this](){
                    this->lidar_basic_detection_ptr_->perception(this->msg_ptr_->rs_lidar_result_ptr);
                    Any::Ptr any_ptr;
                    lidar_basic_detection_ptr_->getResult(any_ptr);
                    const auto& result_ptr = msg_ptr_->rs_lidar_result_ptr;
                    const auto& ptr = any_ptr->AnyCast<lidar::BasicDetectionMsg>();
                    result_ptr->freespace_ptr = ptr->freespace_ptr;
                    result_ptr->attention_objects = ptr->attention_objects;
                    for (size_t i = 0; i < ptr->ai_attention_idx.size(); ++i) {
                        result_ptr->objects[ptr->ai_attention_idx[i]]->core_infos_.attention_type = AttentionType::ATTENTION;
                    }
                    return true;
                });
                thread_worker_map_module_["basic_detection"].start();
            }
        }

        // lidar_tracking
        {
            RsYamlNode tracking_node;
            bool enable_track = false;
            rsYamlSubNode(lidar_node, "tracking", tracking_node);
            rsYamlRead(tracking_node, "enable", enable_track);
            if (enable_track) {
                lidar_tracking_ptr_.reset(new lidar::Tracking);
                lidar_tracking_ptr_->init(tracking_node);
            }
        }

        // init mirror_detection
        {
            RsYamlNode mirror_node;
            bool enable_mirror_detection = false;
            rsYamlSubNode(lidar_node, "mirror_detection", mirror_node);
            rsYamlRead(mirror_node, "enable", enable_mirror_detection);
            if (enable_mirror_detection) {
                mirror_detection_ptr_.reset(new lidar::MirrorDetection);
                mirror_detection_ptr_->init(mirror_node);
            }
        }
        // lidar_postprocessing
        {
            RsYamlNode postprocessing_node;
            rsYamlSubNode(lidar_node, "postprocessing", postprocessing_node);
            lidar_postprocessing_ptr_.reset(new lidar::Postprocessing);
            lidar_postprocessing_ptr_->init(postprocessing_node);
        }

        // lidar map filter
        {
            RsYamlNode hd_map_node;
            bool enable_filter = false;
            rsYamlSubNode(perception_node, "hdmap", hd_map_node);
            rsYamlRead(hd_map_node, "enable", enable_filter);
            if (enable_filter) {
                lidar_map_filter_ptr_.reset(new lidar::MapFilter);
                lidar_map_filter_ptr_->init(hd_map_node);
            }
        }
    }

    // init the perception message for each frame
    // input: a robosense perception message struct without any data.
    // output: void. a robosense perception message struct with pose data.
    inline void initMsg(const RsPerceptionMsg::Ptr& msg_ptr) {
        msg_ptr_ = msg_ptr;
        const auto& pose_map = RsConfigManager().getPose();

        for (auto itr = pose_map.begin(); itr != pose_map.end(); ++itr) {
            const auto& frame_id = itr->first;
            if (msg_ptr_->sub_lidar_msgs_map.find(frame_id) == msg_ptr_->sub_lidar_msgs_map.end()) {
                RERROR << "initMsg failed! can not find msg with frame_id " << frame_id;
                RS_THROW("error msg!");
            }
            const auto& cur_msg_ptr = msg_ptr_->sub_lidar_msgs_map.at(frame_id);
            const auto& pose = itr->second;
            *cur_msg_ptr->status_pose_map.at(AxisStatus::VEHICLE_AXIS) = pose;

            *cur_msg_ptr->status_pose_map.at(AxisStatus::ALIGN_AXIS) = pose;
            cur_msg_ptr->status_pose_map.at(AxisStatus::ALIGN_AXIS)->x = 0;
            cur_msg_ptr->status_pose_map.at(AxisStatus::ALIGN_AXIS)->y = 0;

            *cur_msg_ptr->status_pose_map.at(AxisStatus::ROTATE_AXIS) = pose;
            cur_msg_ptr->status_pose_map.at(AxisStatus::ROTATE_AXIS)->x = 0;
            cur_msg_ptr->status_pose_map.at(AxisStatus::ROTATE_AXIS)->y = 0;

            *cur_msg_ptr->status_pose_map.at(AxisStatus::GLOBAL_AXIS) = pose;
        }
    }

    bool data_fusion_ = false;
    AxisStatus out_axis_ = AxisStatus::GLOBAL_AXIS;
    std::map<std::string, SimplePerception::Ptr> simple_perception_map_;
    std::map<std::string, ThreadWorker> thread_worker_map_;
    std::map<std::string, ThreadWorker> thread_worker_map_module_;
    RsPerceptionMsg::Ptr msg_ptr_;

    lidar::PreFusion::Ptr lidar_pre_fusion_ptr_;
    lidar::PostFusion::Ptr lidar_post_fusion_ptr_;
    lidar::RefineFilter::Ptr lidar_refine_filter_ptr_;
    lidar::RoadDetection::Ptr lidar_road_detection_ptr_;
    lidar::BasicDetection::Ptr lidar_basic_detection_ptr_;
    lidar::Tracking::Ptr lidar_tracking_ptr_;
    lidar::Postprocessing::Ptr lidar_postprocessing_ptr_;
    lidar::MapFilter::Ptr lidar_map_filter_ptr_;
    lidar::ScenceStructDetection::Ptr scene_struct_detection_ptr_;
    lidar::MirrorDetection::Ptr mirror_detection_ptr_;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_PERCEPTION_SUB_BASE_SUB_PERCEPTION_H_
