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

#ifndef RS_PERCEPTION_PERCEPTION_EXTERNAL_SUBS_SMARTSENSOR_PERCEPTION_H_
#define RS_PERCEPTION_PERCEPTION_EXTERNAL_SUBS_SMARTSENSOR_PERCEPTION_H_

#include "rs_perception/perception/external/common/base_perception.h"
#include "rs_perception/perception/external/subs/base_sub_perception.h"
//#include <opencv2/opencv.hpp>
//#include <opencv2/core.hpp>
namespace robosense {
namespace perception {

class SmartSensorPerception: public BasePerception, BaseSubPerception {
public:
    using Ptr = std::shared_ptr<SmartSensorPerception>;

    ~SmartSensorPerception() {
        stop();
        BaseSubPerception::stop();
    }

    // load configures from yaml and init the perception function.
    // input: yaml node
    void init(const RsYamlNode& config_node) override;

    // entrance of perception function of SmartSensor strategy
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

    // entrance of synchronized data of SmartSensor strategy.
    // input: a robosense perception message struct.
    // output: void.
    void addData(const RsPerceptionMsg::Ptr& msg_ptr) override;

    // entrance of callback function register
    // input: a callback function or a lambda function
    // output: void
    void regPerceptionCallback(const std::function<void(const RsPerceptionMsg::Ptr& msg_ptr)>& cb) override;

private:
    inline std::string name() override {
        return "SmartSensorPerception";
    }

    // the detail of SmartSensor strategy perception function.
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
//            for (size_t i = 0; i < ptr->ai_attention_idx.size(); ++i) {
//                result_ptr->objects[ptr->ai_attention_idx[i]]->core_infos_.attention_type = AttentionType::ATTENTION;
//            }
//        }
        for (auto itr = thread_worker_map_module_.begin(); itr != thread_worker_map_module_.end(); ++itr) {
            itr->second.wakeUp();
        }
        for (auto itr = thread_worker_map_module_.begin(); itr != thread_worker_map_module_.end(); ++itr) {
            itr->second.join();
        }
        // step 5. tracking
        if (lidar_tracking_ptr_ != nullptr) {
            lidar_tracking_ptr_->perception(msg_ptr_->rs_lidar_result_ptr);
            lidar_tracking_ptr_->getResult(any_ptr);
            msg_ptr_->rs_lidar_result_ptr->any_map[lidar::SEQUENCES] = any_ptr;  // 填充序列信息
            std::map<unsigned int, Object::Ptr> unique_object_map;
            for (const auto& obj : msg_ptr_->rs_lidar_result_ptr->objects) {
                unique_object_map[obj->supplement_infos_.unique_id] = obj;
            }
            for (const auto& obj : msg_ptr_->rs_lidar_result_ptr->attention_objects) {
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
                } else if (track_obj->mode == ModeType::PREDICT) {
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

        if (mirror_detection_ptr_ != nullptr) {
            mirror_detection_ptr_->perception(msg_ptr_->rs_lidar_result_ptr);
            mirror_detection_ptr_->getResult(any_ptr);
            std::map<unsigned int, Object::Ptr> unique_object_map;
            for (const auto &obj: msg_ptr_->rs_lidar_result_ptr->objects) {
                unique_object_map[obj->supplement_infos_.unique_id] = obj;
            }
            auto mirror_objects = any_ptr->AnyCast<lidar::MirrorDetectionMsg>()->mirror_vec;
            auto fv_intensity = any_ptr->AnyCast<lidar::MirrorDetectionMsg>()->fv_intensity;
            auto fv_size = any_ptr->AnyCast<lidar::MirrorDetectionMsg>()->fv_size;
//            cv::Mat polygon_mat = cv::Mat::zeros(static_cast<int>(fv_size.x), static_cast<int>(fv_size.y), CV_8UC3);
//            int intensity = 0;
//            for (int i = 0; i < static_cast<int>(fv_size.x); ++i) {
//                for (int j = 0; j < static_cast<int>(fv_size.y); ++j) {
//                    int mat_idx = i * static_cast<int>(fv_size.y) + j;
//                    if (fv_intensity[mat_idx] > 0) {
//                        intensity = 64 + fv_intensity[mat_idx];
//                        intensity = intensity > 255? 255: intensity;
//                        polygon_mat.at<cv::Vec3b>(i, j)[0] = intensity;
//                        polygon_mat.at<cv::Vec3b>(i, j)[1] = intensity;
//                        polygon_mat.at<cv::Vec3b>(i, j)[2] = intensity;
//                    }
//                }
//            }

            for (auto mirror: mirror_objects) {
//                cv::Scalar color_(0,255,0);
                if (unique_object_map.find(mirror->unique_id) != unique_object_map.end()) {
                    if (mirror->is_mirror) {
                        const auto &obj = unique_object_map.at(mirror->unique_id);
                        Any::Ptr any_(new Any(mirror->is_mirror));
                        obj->any_map["mirror"] = any_;
//                        color_[0] = 0;
//                        color_[1] = 0;
//                        color_[2] = 255;
                    }
                }
//                auto polygon = mirror->fv_polygon;
//                for (int j = 0; j < polygon.size() - 1; ++j) {
//                    cv::line(polygon_mat, cvPoint(polygon[j].x, polygon[j].y),
//                             cvPoint(polygon[j+1].x, polygon[j+1].y), color_ , 2);
//                }
//                cv::line(polygon_mat, cvPoint(polygon.back().x, polygon.back().y),
//                         cvPoint(polygon[0].x, polygon[0].y), color_ , 2);
//                cv::flip(polygon_mat, polygon_mat, -1);
//                cv::imshow("polygon_mat", polygon_mat);
//                cv::waitKey(1);
            }

            VecObjectPtr real_objects;
            const auto &obj_vec = msg_ptr_->rs_lidar_result_ptr->objects;
            for (const auto& obj: obj_vec) {
                if(obj->any_map.find("mirror") != obj->any_map.end()) {  // 有可能是mirror物体
                    if (obj->any_map.at("mirror")->AnyCast<bool>()) {  // 一定是mirror物体
                        continue;
                    }
                    else { // 不是mirror物体
                        real_objects.emplace_back(obj);
                    }
                }
                else {  // 一定不是mirror物体
                    real_objects.emplace_back(obj);
                }
            }
            msg_ptr_->rs_lidar_result_ptr->objects = real_objects;
        }
        msg_ptr_->rs_lidar_result_ptr->transAxis(out_axis_, name());
    }

    int32_t frame_idx_ = 0;
    PipelineThreadWorker<RsPerceptionMsg::Ptr>::Ptr pipeline_thread_worker_ptr_;
    std::mutex mx_perception_cb_;
    std::vector<std::function<void(const RsPerceptionMsg::Ptr &msg_ptr)> > perception_cb_list_;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_PERCEPTION_EXTERNAL_SUBS_SMARTSENSOR_PERCEPTION_H_
