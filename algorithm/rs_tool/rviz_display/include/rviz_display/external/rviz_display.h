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

#ifndef RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_RVIZ_DISPLAY_H_
#define RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_RVIZ_DISPLAY_H_

#include "rviz_display/external/common/utils.h"
#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND

#include "rviz_display/external/ros/cloud_pubs.h"
#include "rviz_display/external/ros/marker_pubs.h"
#include "rviz_display/external/ros/pre_marker_pubs.h"

namespace robosense {
namespace perception {

class RvizDisplay {
public:
    using Ptr = std::shared_ptr<RvizDisplay>;

    RvizDisplay() {
        nh_ptr_.reset(new ros::NodeHandle);
        V2r_global_pose.reset(new RsPose);
    }

    // load configures from yaml and init rviz display function.
    // input: yaml node
    inline void init(const RsYamlNode &config_node) {
        BasePubOptions base_options;
        rsYamlRead(config_node, _prefix, base_options.pre_fix);
        rsYamlRead(config_node, _frame_id, base_options.frame_id);
        rsYamlRead(config_node, _map_frame_id, base_options.map_frame_id);

        std::string strategy;
        rsYamlRead(config_node, _strategy, strategy);
        if (kDisplayMode2TypeMap.find(strategy) != kDisplayMode2TypeMap.end()) {
            base_options.mode = kDisplayMode2TypeMap.at(strategy);
        }

        std::string display_axis;
        rsYamlRead(config_node, _display_axis, display_axis);
        display_axis_ = kAxisStatusName2TypeMap.at(display_axis);

        {  // pub map
            RsYamlNode map_node;
            rsYamlSubNode(config_node, _map, map_node);
            std::vector<std::string> file_vec;
            file_vec.resize(map_node.size());
            for (size_t i = 0; i < map_node.size(); ++i) {
                file_vec[i] = map_node[i].as<std::string>();
            }

            pub_map_ = nh_ptr_->advertise<pcl::PointCloud<pcl::PointXYZI> >
            (base_options.pre_fix + "percept_map_rviz", 1, true);
            pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            for (size_t i = 0; i < file_vec.size(); ++i) {
                if (!fileExist(file_vec[i])) {
                    continue;
                }
                pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                if (pcl::io::loadPCDFile(file_vec[i], *tmp_cloud_ptr) == 0) {
                    *map_cloud_ptr += *tmp_cloud_ptr;
                }
            }
            if (!map_cloud_ptr->empty()) {
                map_cloud_ptr->header.frame_id = base_options.map_frame_id;
                pub_map_.publish(map_cloud_ptr);
            }
        }

        {  // pub_road
            RsYamlNode road_node;
            rsYamlSubNode(config_node, _road, road_node);
            std::vector<std::string> file_vec;
            file_vec.resize(road_node.size());
            for (size_t i = 0; i < road_node.size(); ++i) {
                file_vec[i] = road_node[i].as<std::string>();
            }

            pub_road_ = nh_ptr_->advertise<pcl::PointCloud<pcl::PointXYZI> >
            (base_options.pre_fix + "percept_road_rviz", 1, true);
            pcl::PointCloud<pcl::PointXYZI>::Ptr road_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            for (size_t i = 0; i < file_vec.size(); ++i) {
                if (!fileExist(file_vec[i])) {
                    continue;
                }
                pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                if (pcl::io::loadPCDFile(file_vec[i], *tmp_cloud_ptr) == 0) {
                    *road_cloud_ptr += *tmp_cloud_ptr;
                }
            }
            if (!road_cloud_ptr->empty()) {
                road_cloud_ptr->header.frame_id = base_options.map_frame_id;
                pub_road_.publish(road_cloud_ptr);
            }
        }

        {
            CloudPubOptions options;
            options.base_options = base_options;
            RsYamlNode pub_keys_node;
            rsYamlSubNode(config_node, "pub_cloud_keys", pub_keys_node);
            options.pub_keys.resize(pub_keys_node.size());
            for (size_t i = 0; i < pub_keys_node.size(); ++i) {
                options.pub_keys[i] = pub_keys_node[i].as<std::string>();
            }
            cloud_pub_ptr_.reset(new CloudPubs);
            cloud_pub_ptr_->init(options);
        }
        {
            MarkerPubOptions options;
            options.base_options = base_options;
            RsYamlNode pub_keys_node;
            rsYamlSubNode(config_node, "pub_marker_keys", pub_keys_node);
            options.pub_keys.resize(pub_keys_node.size());
            for (size_t i = 0; i < pub_keys_node.size(); ++i) {
                options.pub_keys[i] = pub_keys_node[i].as<std::string>();
            }
            marker_pub_ptr_.reset(new MarkerPubs);
            marker_pub_ptr_->init(options);
        }
        {
            PreMarkerPubOptions options;
            options.base_options = base_options;
            pre_marker_pub_ptr_.reset(new PreMarkerPubs);
            pre_marker_pub_ptr_->init(options);

            if (is_v2r) {
                pre_marker_pub_ptr_->addV2rPose(V2r_global_pose);
            }

            pre_marker_pub_ptr_->display();
        }
    }

    // entrance of display the result
    // input: robosense perception message struct
    // output: void. the result will be published into ros space.
    inline void display(const RsPerceptionMsg::Ptr &msg_ptr) {
        axis_status = msg_ptr->rs_lidar_result_ptr->status;
        msg_ptr->rs_lidar_result_ptr->transAxis(display_axis_, "Rviz_display");
        cloud_pub_ptr_->display(msg_ptr);
        marker_pub_ptr_->display(msg_ptr);
        msg_ptr->rs_lidar_result_ptr->transAxis(axis_status, "Rviz_display");
    }

    // entrance of start the thread;
    inline void start() {
        pipeline_thread_worker_ptr_.reset(new PipelineThreadWorker<RsPerceptionMsg::Ptr>);
        pipeline_thread_worker_ptr_->bind([this](const RsPerceptionMsg::Ptr& msg_ptr){
            this->display(msg_ptr);
            for (auto &cb : this->display_cb_list_) {
                cb(msg_ptr);
            }
        });
        pipeline_thread_worker_ptr_->start();
    }

    // entrance of stop the thread.
    inline void stop() {
        pipeline_thread_worker_ptr_->stop();
    }

    // entrance of add data to display function.
    // input: a robosense perception message struct.
    // output: void.
    inline void addData(const RsPerceptionMsg::Ptr& msg_ptr) {
        pipeline_thread_worker_ptr_->add(msg_ptr);
    }

    // entrance of callback function register
    // input: a callback function or a lambda function
    // output: void
    inline void regDisplayCallback(const std::function<void(const RsPerceptionMsg::Ptr& msg_ptr)>& cb) {
        std::lock_guard<std::mutex> lg(mx_display_cb_);
        display_cb_list_.emplace_back(cb);
    }

    // entrance of add global pose when using v2r perception strategy.
    inline void addV2rPose(const RsPose::Ptr &pose) {
        is_v2r = true;
        V2r_global_pose = pose;
    }


private:
    std::unique_ptr<ros::NodeHandle> nh_ptr_;
    CloudPubs::Ptr cloud_pub_ptr_;
    MarkerPubs::Ptr marker_pub_ptr_;
    PreMarkerPubs::Ptr pre_marker_pub_ptr_;
    ros::Publisher pub_map_, pub_road_;
    AxisStatus axis_status, display_axis_;
    PipelineThreadWorker<RsPerceptionMsg::Ptr>::Ptr pipeline_thread_worker_ptr_;
    std::mutex mx_display_cb_;
    std::vector<std::function<void(const RsPerceptionMsg::Ptr &msg_ptr)> > display_cb_list_;
    bool is_v2r = false;
    RsPose::Ptr V2r_global_pose;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_ROS_FOUND

#endif  // RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_RVIZ_DISPLAY_H_
