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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_NATIVE_CUSTOM_TRANSFORMER_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_NATIVE_CUSTOM_TRANSFORMER_H_

#include "rs_perception/communication/external/common/basic_type.h"
#include "rs_perception/custom/common/base_custom_params.h"
#include "rs_perception/custom/common/base_custom_msg.h"
#include "lz4/include/lz4.h"

namespace robosense {
namespace perception {
namespace Native {

//inline void transform(const RsPerceptionMsg::Ptr& msg, RsNativeCustomMsg &custom_msg) {
//    custom_msg.msg_ = msg;
//}

class NativeConvert {
public:
    using Ptr = std::shared_ptr<NativeConvert>;

    NativeConvert() {
        msg_.reset(new RsPerceptionMsg);
    }

    void init(RsCommonCustomMsgParams& custom_params) {
        custom_params_ = custom_params;
    }
    void serialization(const RsPerceptionMsg::Ptr &msg) {
        msg_ = msg;
    };
    void deserialization(const RsPerceptionMsg::Ptr &msg) {
        auto& res_ptr = msg->rs_lidar_result_ptr;
        res_ptr = msg_->rs_lidar_result_ptr;
    }

    template<class Archive>
    void serialize(Archive &archive) {
        archive(msg_->rs_lidar_result_ptr->frame_id);
        archive(msg_->rs_lidar_result_ptr->timestamp);
        archive(msg_->rs_lidar_result_ptr->global_pose_ptr);
        archive(msg_->rs_lidar_result_ptr->gps_origin);
        archive(msg_->rs_lidar_result_ptr->status_pose_map);
        archive(msg_->rs_lidar_result_ptr->status);
        archive(msg_->rs_lidar_result_ptr->valid_indices);
        archive(msg_->rs_lidar_result_ptr->objects);

        archive(custom_params_.send_point_cloud);
        archive(custom_params_.send_attention_objects);
        archive(custom_params_.send_freespace);
        archive(custom_params_.send_lane);
        archive(custom_params_.send_roadedge);
        archive(custom_params_.send_sematic);

        if (custom_params_.send_point_cloud) {
            archive(msg_->rs_lidar_result_ptr->scan_ptr);
        }
        if (custom_params_.send_attention_objects) {
            archive(msg_->rs_lidar_result_ptr->attention_objects);
        }
        if (custom_params_.send_freespace) {
            archive(msg_->rs_lidar_result_ptr->freespace_ptr);
        }
        if (custom_params_.send_lane) {
            archive(msg_->rs_lidar_result_ptr->lanes);
        }
        if (custom_params_.send_roadedge) {
            archive(msg_->rs_lidar_result_ptr->roadedges);
        }
        if (custom_params_.send_sematic) {
            archive(msg_->rs_lidar_result_ptr->non_ground_indices);
            archive(msg_->rs_lidar_result_ptr->ground_indices);
            archive(msg_->rs_lidar_result_ptr->background_indices);
        }

        // extra_infos
        archive(custom_params_.device_id);
    }


    RsCommonCustomMsgParams custom_params_;
    RsPerceptionMsg::Ptr msg_;
};

}  // namespace Native
}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_NATIVE_NATIVE_CUSTOM_TRANSFORMER_H_
