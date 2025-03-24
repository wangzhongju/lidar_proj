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

#ifndef RS_PERCEPTION_PERCEPTION_EXTERNAL_COMMON_SIMPLE_PERCEPTION_H_
#define RS_PERCEPTION_PERCEPTION_EXTERNAL_COMMON_SIMPLE_PERCEPTION_H_

#include <algorithm>
#include "rs_perception/lidar/preprocessing/external/preprocessing.h"
#include "rs_perception/lidar/ai_detection/external/ai_detection.h"
#include "rs_perception/lidar/ground_filter/external/ground_filter.h"
#include "rs_perception/lidar/segmentor/external/segmentor.h"


namespace robosense {
namespace perception {

class SimplePerception {
public:
    using Ptr = std::shared_ptr<SimplePerception>;

    // init simple perception method
    // input: yaml node
    inline void init(const RsYamlNode& config_node) {
        // init preprocessing
        {
            RsYamlNode preprocessing_node;
            rsYamlSubNode(config_node, "preprocessing", preprocessing_node);
            preprocessing_node["authorization"] = config_node["authorization"];
            preprocessing_node["hardkey"] = config_node["hardkey"];
            lidar_preprocessing_ptr_.reset(new lidar::Preprocessing);
            lidar_preprocessing_ptr_->init(preprocessing_node);
        }
        // init ai detection
        {
            RsYamlNode ai_detection_node;
            bool enable_ai = false;
            rsYamlSubNode(config_node, "ai_detection", ai_detection_node);
            rsYamlRead(ai_detection_node, "enable", enable_ai);
            if (enable_ai) {
                lidar_ai_detection_ptr_.reset(new lidar::AiDetection);
                lidar_ai_detection_ptr_->init(ai_detection_node);
            }
        }
        // init ground filter
        {
            RsYamlNode ground_filter_node;
            bool enable_ground = false;
            rsYamlSubNode(config_node, "ground_filter", ground_filter_node);
            rsYamlRead(ground_filter_node, "enable", enable_ground);
            if (enable_ground) {
                lidar_ground_filter_ptr_.reset(new lidar::GroundFilter);
                lidar_ground_filter_ptr_->init(ground_filter_node);
            }
        }
        // lidar_segmentor
        {
            RsYamlNode segmentor_node;
            bool enable_seg = false;
            rsYamlSubNode(config_node, "segmentor", segmentor_node);
            rsYamlRead(segmentor_node, "enable", enable_seg);
            if (enable_seg) {
                lidar_segmentor_ptr_.reset(new lidar::Segmentor);
                lidar_segmentor_ptr_->init(segmentor_node);
            }
        }
    }

    // entrance of simple perception
    // input: robosense perception message struct
    // output: void. The perception result of each step will be recorded in the "msg_ptr"
    inline void perception(const RsLidarFrameMsg::Ptr& msg_ptr) {
        Any::Ptr any_ptr;
        lidar_preprocessing_ptr_->perception(msg_ptr);
        lidar_preprocessing_ptr_->getResult(any_ptr);
        {  // copy preprocessing_msg to msg
            msg_ptr->valid_indices = any_ptr->AnyCast<lidar::PreprocessingMsg>()->valid_indices;
        }
        if (lidar_ai_detection_ptr_ != nullptr) {
            msg_ptr->updateRotateAxis(lidar_ai_detection_ptr_->getRotateAngle());
            lidar_ai_detection_ptr_->perception(msg_ptr);
            lidar_ai_detection_ptr_->getResult(any_ptr);
            {  // copy ai_detection_msg to msg
                msg_ptr->background_indices = any_ptr->AnyCast<lidar::AiDetectionMsg>()->background_indices;
                msg_ptr->ground_indices = any_ptr->AnyCast<lidar::AiDetectionMsg>()->ground_indices;
                msg_ptr->non_ground_indices = any_ptr->AnyCast<lidar::AiDetectionMsg>()->non_ground_indices;
                msg_ptr->objects = any_ptr->AnyCast<lidar::AiDetectionMsg>()->objects;
            }
        }
        if (lidar_ground_filter_ptr_ != nullptr) {
            lidar_ground_filter_ptr_->perception(msg_ptr);
            lidar_ground_filter_ptr_->getResult(any_ptr);
            if (lidar_ai_detection_ptr_ == nullptr) {  
                // todo copy result from ground_filter to msg_ptr
                msg_ptr->background_indices = any_ptr->AnyCast<lidar::GroundFilterMsg>()->background_indices;
                msg_ptr->ground_indices = any_ptr->AnyCast<lidar::GroundFilterMsg>()->ground_indices;
                msg_ptr->non_ground_indices = any_ptr->AnyCast<lidar::GroundFilterMsg>()->non_ground_indices;
            } else {
                std::vector<int> ai_ground_indices = msg_ptr->ground_indices;
                std::vector<int> ai_non_ground_indices = msg_ptr->non_ground_indices;
                std::vector<int> ai_background_indices = msg_ptr->background_indices;
                auto& g_ground_indices = any_ptr->AnyCast<lidar::GroundFilterMsg>()->ground_indices;
                auto& g_background_indices = any_ptr->AnyCast<lidar::GroundFilterMsg>()->background_indices;
                auto& g_non_ground_indices = any_ptr->AnyCast<lidar::GroundFilterMsg>()->non_ground_indices;
                std::sort(ai_background_indices.begin(), ai_background_indices.end());
                std::sort(ai_non_ground_indices.begin(), ai_non_ground_indices.end());
                std::sort(ai_ground_indices.begin(), ai_ground_indices.end());
                std::sort(g_background_indices.begin(), g_background_indices.end());
                std::sort(g_ground_indices.begin(), g_ground_indices.end());
                // 几何地面与ai背景交集
                std::vector<int> insec_indices;
                std::set_intersection(g_ground_indices.begin(), g_ground_indices.end(),
                                      ai_ground_indices.begin(), ai_ground_indices.end(),
                                      std::inserter(insec_indices, insec_indices.begin()));
                std::vector<int> leave_back_indices;
                std::set_difference(ai_background_indices.begin(), ai_background_indices.end(),
                                    insec_indices.begin(), insec_indices.end(),
                                    std::inserter( leave_back_indices, leave_back_indices.begin()));
                std::vector<int> leave_non_indices;
                std::set_difference(ai_non_ground_indices.begin(), ai_non_ground_indices.end(),
                                    insec_indices.begin(), insec_indices.end(),
                                    std::inserter(leave_non_indices, leave_non_indices.begin()));
                ai_ground_indices.insert(ai_ground_indices.end(), insec_indices.begin(), insec_indices.end());
                ai_background_indices = leave_back_indices;
                ai_non_ground_indices = leave_non_indices;
                msg_ptr->ground_indices = ai_ground_indices;
                msg_ptr->non_ground_indices = ai_non_ground_indices;
                msg_ptr->background_indices = ai_background_indices;
            }
        }

        if (lidar_segmentor_ptr_ != nullptr) {
            lidar_segmentor_ptr_->perception(msg_ptr);
            lidar_segmentor_ptr_->getResult(any_ptr);
            {
                const auto seg_objects = any_ptr->AnyCast<lidar::SegmentorMsg>()->objects;
                auto& msg_objects = msg_ptr->objects;
                msg_objects.insert(msg_objects.end(), seg_objects.begin(), seg_objects.end());
            }
        }
    }

private:
    inline std::string name() {
        return "SimplePerception";
    }

    lidar::Preprocessing::Ptr lidar_preprocessing_ptr_;
    lidar::AiDetection::Ptr lidar_ai_detection_ptr_;
    lidar::GroundFilter::Ptr lidar_ground_filter_ptr_;
    lidar::Segmentor::Ptr lidar_segmentor_ptr_;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_PERCEPTION_EXTERNAL_COMMON_SIMPLE_PERCEPTION_H_
