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

#ifndef RS_PERCEPTION_PERCEPTION_EXTERNAL_SUBS_V2R_PERCEPTION_H_
#define RS_PERCEPTION_PERCEPTION_EXTERNAL_SUBS_V2R_PERCEPTION_H_

#include "rs_perception/perception/external/common/base_perception.h"
#include "rs_perception/perception/external/subs/base_sub_perception.h"

namespace robosense {
namespace perception {

class V2rPerception: public BasePerception, BaseSubPerception  {
public:
    using Ptr = std::shared_ptr<V2rPerception>;

    ~V2rPerception() {
        stop();
        BaseSubPerception::stop();
    }

    // load configures from yaml and init the perception function.
    // input: yaml node
    void init(const RsYamlNode& config_node) override;

    // entrance of perception function of Pseries strategy
    // input: a robosense perception message struct with pose map data.
    //        the pose map data include the calibration relationship between each axis defined by robosense.
    // output: void. all the detail of robosense perception message struct will be re-written in this function.
    void perception(const RsPerceptionMsg::Ptr& msg_ptr) override;

    // start the thread and wait for data.
    void start() override;

    // stop the thread.
    void stop() override {
        pipeline_thread_worker_ptr_->stop();
    }

    // entrance of synchronized data of Pseries strategy.
    // input: a robosense perception message struct.
    // output: void.
    void addData(const RsPerceptionMsg::Ptr& msg_ptr) override;

    // entrance of callback function register
    // input: a callback function or a lambda function
    // output: void
    void regPerceptionCallback(const std::function<void(const RsPerceptionMsg::Ptr& msg_ptr)>& cb) override;

private:
    inline std::string name() override {
        return "V2rPerception";
    }

    // the detail of V2r strategy perception function.
    inline void process() {
        Any::Ptr any_ptr(new Any);
        //  step 1. simple_perception
        if (data_fusion_) {
            //  pre_fusion
            lidar_pre_fusion_ptr_->perception(msg_ptr_);
            //  simple_perception
            {
                auto main_frame_id = RsConfigManager().getRsMainFrameID();
                auto simple_perception_ptr_ = simple_perception_map_.at(main_frame_id);
                simple_perception_ptr_->perception(msg_ptr_->rs_lidar_result_ptr);
            }
        } else {
            //  simple_perception
            for (auto itr = thread_worker_map_.begin(); itr != thread_worker_map_.end(); ++itr) {
                itr->second.wakeUp();
            }
            for (auto itr = thread_worker_map_.begin(); itr != thread_worker_map_.end(); ++itr) {
                itr->second.join();
            }
            //  post_fusion
            {
                lidar_post_fusion_ptr_->perception(msg_ptr_);
                lidar_post_fusion_ptr_->getResult(any_ptr);
                msg_ptr_->rs_lidar_result_ptr = any_ptr->AnyCast<lidar::PostFusionMsg>()->fusion_msg_ptr;
            }
        }
        // step 2. refine_filter
        if (lidar_refine_filter_ptr_ != nullptr) {
            lidar_refine_filter_ptr_->perception(msg_ptr_->rs_lidar_result_ptr);
            lidar_refine_filter_ptr_->getResult(any_ptr);
            const auto& result_ptr = msg_ptr_->rs_lidar_result_ptr;
            const auto& ptr = any_ptr->AnyCast<lidar::RefineFilterMsg>();
            result_ptr->ground_indices = ptr->ground_indices;
            result_ptr->background_indices = ptr->background_indices;
            result_ptr->non_ground_indices = ptr->non_ground_indices;
            result_ptr->objects = ptr->objects;
        }

        //  step 3. road_detection
//        if (lidar_road_detection_ptr_ != nullptr) {
//            lidar_road_detection_ptr_->perception(msg_ptr_->rs_lidar_result_ptr);
//            lidar_road_detection_ptr_->getResult(any_ptr);
//            const auto& result_ptr = msg_ptr_->rs_lidar_result_ptr;
//            const auto ptr = any_ptr->AnyCast<lidar::RoadDetectionMsg>();
//            result_ptr->lanes = ptr->lanes;
//            result_ptr->roadedges = ptr->roadedges;
//        }

        //  step 4. basic_detection
//        if (lidar_basic_detection_ptr_ != nullptr) {
//            lidar_basic_detection_ptr_->perception(msg_ptr_->rs_lidar_result_ptr);
//            lidar_basic_detection_ptr_->getResult(any_ptr);
//            const auto& result_ptr = msg_ptr_->rs_lidar_result_ptr;
//            const auto& ptr = any_ptr->AnyCast<lidar::BasicDetectionMsg>();
//            result_ptr->freespace_ptr = ptr->freespace_ptr;
//            result_ptr->attention_objects = ptr->attention_objects;
//        }

        // step 5. tracking
        if (lidar_tracking_ptr_ != nullptr) {
            lidar_tracking_ptr_->perception(msg_ptr_->rs_lidar_result_ptr);
            lidar_tracking_ptr_->getResult(any_ptr);
            msg_ptr_->rs_lidar_result_ptr->any_map[lidar::SEQUENCES] = any_ptr;  // 填充序列信息
            std::map<unsigned int, Object::Ptr> unique_object_map;
            for (const auto& obj : msg_ptr_->rs_lidar_result_ptr->objects) {
                unique_object_map[obj->supplement_infos_.unique_id] = obj;
            }
            for (const auto& track_obj : any_ptr->AnyCast<lidar::TrackingMsg>()->objects) {
                if (unique_object_map.find(track_obj->unique_id) != unique_object_map.end()) {
                    const auto& obj = unique_object_map.at(track_obj->unique_id);
                    obj->core_infos_.tracker_id = track_obj->tracker_id;
                    obj->core_infos_.age = track_obj->age;
                    obj->core_infos_.velocity = track_obj->velocity;
                    obj->core_infos_.relative_velocity = track_obj->relative_velocity;
                    obj->core_infos_.velocity_cov = track_obj->velocity_cov;
                    // obj->core_infos_.relative_velocity_cov = track_obj->relative_velocity_cov;
                    obj->core_infos_.acceleration_cov = track_obj->acceleration_cov;
                    obj->core_infos_.acceleration = track_obj->acceleration;
                    obj->core_infos_.angle_velocity = track_obj->angle_velocity;
                    obj->core_infos_.angle_velocity_cov = track_obj->angle_velocity_cov;
                    obj->core_infos_.angle_acceleration_cov = track_obj->angle_acceleration_cov;
                    obj->core_infos_.angle_acceleration = track_obj->angle_acceleration;
                    obj->core_infos_.motion_state = track_obj->motion_state;
                    obj->supplement_infos_.tracking_state = track_obj->tracking_state;
                    obj->supplement_infos_.trajectory = track_obj->trajectory;
                    obj->supplement_infos_.history_velocity = track_obj->history_velocity;
                    obj->supplement_infos_.history_type = track_obj->history_type;
                } else {
                    Object::Ptr tmp_obj(new Object);
                    tmp_obj->supplement_infos_.unique_id = track_obj->unique_id;
                    tmp_obj->core_infos_.tracker_id = track_obj->tracker_id;
                    tmp_obj->core_infos_.age = track_obj->age;
                    tmp_obj->core_infos_.center = track_obj->center;
                    tmp_obj->core_infos_.size = track_obj->size;
                    tmp_obj->core_infos_.direction = track_obj->direction;
                    tmp_obj->core_infos_.type = track_obj->type;
                    tmp_obj->supplement_infos_.mode = track_obj->mode;
                    tmp_obj->core_infos_.velocity = track_obj->velocity;
                    tmp_obj->core_infos_.relative_velocity = track_obj->relative_velocity;
                    tmp_obj->core_infos_.velocity_cov = track_obj->velocity_cov;
                    // tmp_obj->core_infos_.relative_velocity_cov = track_obj->relative_velocity_cov;
                    tmp_obj->core_infos_.acceleration_cov = track_obj->acceleration_cov;
                    tmp_obj->core_infos_.acceleration = track_obj->acceleration;
                    tmp_obj->core_infos_.angle_velocity = track_obj->angle_velocity;
                    tmp_obj->core_infos_.angle_velocity_cov = track_obj->angle_velocity_cov;
                    tmp_obj->core_infos_.angle_acceleration_cov = track_obj->angle_acceleration_cov;
                    tmp_obj->core_infos_.angle_acceleration = track_obj->angle_acceleration;
                    tmp_obj->core_infos_.motion_state = track_obj->motion_state;
                    tmp_obj->supplement_infos_.tracking_state = track_obj->tracking_state;
                    tmp_obj->supplement_infos_.trajectory = track_obj->trajectory;
                    tmp_obj->supplement_infos_.history_velocity = track_obj->history_velocity;
                    tmp_obj->supplement_infos_.history_type = track_obj->history_type;
                    msg_ptr_->rs_lidar_result_ptr->objects.emplace_back(tmp_obj);
                }
            }
        }

        //  step 6. postprocessing
        lidar_postprocessing_ptr_->perception(msg_ptr_->rs_lidar_result_ptr);
        lidar_postprocessing_ptr_->getResult(any_ptr);
        const auto& result_ptr = msg_ptr_->rs_lidar_result_ptr;
        const auto& ptr = any_ptr->AnyCast<lidar::PostprocessingMsg>();
        result_ptr->attention_objects = ptr->attention_objects;

        // step 7 map filter
        if (lidar_map_filter_ptr_ != nullptr) {
            lidar_map_filter_ptr_->perception(msg_ptr_->rs_lidar_result_ptr);
            lidar_map_filter_ptr_->getResult(any_ptr);
            const auto& result_ptr = msg_ptr_->rs_lidar_result_ptr;
            const auto& ptr = any_ptr->AnyCast<lidar::MapFilterMsg>();

            bool hd_map_disable = false;
            Any::Ptr any_ptr_hd_map_disable(new Any(hd_map_disable));
            result_ptr->any_map["hd_map_disable"] = any_ptr_hd_map_disable;

            if (ptr->filter_obj) {
                result_ptr->objects = ptr->objects;
            }

            Any::Ptr any_ptr_event_detect(new Any(ptr->event_detect));
            result_ptr->any_map["event_detect"] = any_ptr_event_detect;
            if (ptr->event_detect) {
                Any::Ptr any_ptr_count_map(new Any(ptr->count_map));
                Any::Ptr any_ptr_jam_map(new Any(ptr->jam_map));
                Any::Ptr any_ptr_obj_evt_map(new Any(ptr->obj_evt_map));
                result_ptr->any_map["count_map"] = any_ptr_count_map;
                result_ptr->any_map["jam_map"] = any_ptr_jam_map;
                result_ptr->any_map["obj_evt_map"] = any_ptr_obj_evt_map;
            }
        }
        else {
            bool hd_map_disable = true;
            Any::Ptr any_ptr_hd_map_disable(new Any(hd_map_disable));
            result_ptr->any_map["hd_map_disable"] = any_ptr_hd_map_disable;
        }

        msg_ptr_->rs_lidar_result_ptr->transAxis(out_axis_, name());
    }

    // for the V2R strategy, the global pose and GPS information need to be initialized
    // input: yaml node.
    inline void initV2rConfig(const RsYamlNode& config_node) {
        RsYamlNode v2r_node;
        rsYamlSubNode(config_node, "V2rPerception", v2r_node);
        // init global pose
        RsYamlNode global_pose_node;
        rsYamlSubNode(v2r_node, "global_pose", global_pose_node);
        v2r_global_pose_.reset(new RsPose);
        v2r_global_pose_->load(global_pose_node);

        // init gps origin
        RsYamlNode gps_node;
        rsYamlSubNode(v2r_node, "gps", gps_node);
        rsYamlRead(gps_node, "gps_longtitude", gps_origin_.x);
        rsYamlRead(gps_node, "gps_latitude", gps_origin_.y);
        rsYamlRead(gps_node, "gps_altitude", gps_origin_.z);
    }
    RsVector3d gps_origin_;
    RsPose::Ptr v2r_global_pose_;

    int32_t frame_idx_ = 0;
    PipelineThreadWorker<RsPerceptionMsg::Ptr>::Ptr pipeline_thread_worker_ptr_;
    std::mutex mx_perception_cb_;
    std::vector<std::function<void(const RsPerceptionMsg::Ptr &msg_ptr)> > perception_cb_list_;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_PERCEPTION_EXTERNAL_SUBS_V2R_PERCEPTION_H_
