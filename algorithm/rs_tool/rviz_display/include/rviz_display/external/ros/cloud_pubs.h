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

#ifndef RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_CLOUD_PUBS_H_
#define RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_CLOUD_PUBS_H_

#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND

#include "rviz_display/external/common/base_cloud_pub.h"
#include "rviz_display/external/common/utils.h"

namespace robosense {
namespace perception {

const char _origin[] = "origin";
const char _ground[] = "ground";
const char _background[] = "background";
const char _non_ground[] = "non_ground";
const char _clusters[] = "clusters";
const char _sematic[] = "sematic";

struct CloudPubOptions {
    BasePubOptions base_options;
    std::vector<std::string> pub_keys;
};

class CloudPubs : public BaseCloudPub {
public:
    using Ptr = std::shared_ptr<CloudPubs>;

    CloudPubs() {
        nh_ptr_.reset(new ros::NodeHandle);
    }

    // load configures from yaml and init cloud publish function.
    // input: yaml node
    inline void init(const CloudPubOptions &options) {
        base_options_ = options.base_options;

        pub_keys_set_.clear();
        for (auto itr = options.pub_keys.begin(); itr != options.pub_keys.end(); ++itr) {
            pub_keys_set_.insert(*itr);
        }

        origin_pub_ = nh_ptr_->advertise<pcl::PointCloud<pcl::PointXYZI> >
        (base_options_.pre_fix + "percept_origin_rviz", 1, true);
        bg_pub_ = nh_ptr_->advertise<pcl::PointCloud<pcl::PointXYZI> >
        (base_options_.pre_fix + "percept_background_rviz", 1, true);
        g_pub_ = nh_ptr_->advertise<pcl::PointCloud<pcl::PointXYZI> >
        (base_options_.pre_fix + "percept_ground_rviz", 1, true);
        ng_pub_ = nh_ptr_->advertise<pcl::PointCloud<pcl::PointXYZI> >
        (base_options_.pre_fix + "percept_non_ground_rviz", 1, true);

        cluster_pub_ = nh_ptr_->advertise<pcl::PointCloud<pcl::PointXYZRGB> >
        (base_options_.pre_fix + "percept_cluster_rviz", 1, true);
        sematic_pub_ = nh_ptr_->advertise<pcl::PointCloud<pcl::PointXYZRGB> >
        (base_options_.pre_fix + "percept_sematic_rviz", 1, true);
        genClusterColors();
    }

    // entrance of display the point-cloud
    // input: robosense perception message struct
    // output: void. the result will be published into ros space.
    inline void display(const RsPerceptionMsg::Ptr &msg_ptr) override {
        msg_ptr_ = msg_ptr;
        if (msg_ptr_->rs_lidar_result_ptr->scan_ptr->points.empty()) {
            return;
        }
        transRsCloudToPclCloud(msg_ptr_->rs_lidar_result_ptr->scan_ptr, cloud_ptr_);
        ros::Time time(msg_ptr->rs_lidar_result_ptr->timestamp);
        auto stamp = time.toNSec() / 1.e3;

        if (pub_keys_set_.find(_origin) != pub_keys_set_.end()) {
            // publish origin cloud
            cloud_ptr_->header.stamp = stamp;
            cloud_ptr_->header.frame_id = base_options_.frame_id;
            origin_pub_.publish(cloud_ptr_);
        }
        if (pub_keys_set_.find(_background) != pub_keys_set_.end()) {
            // publish background cloud
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                genBackground(cloud_ptr);
                cloud_ptr->header.stamp = stamp;
                cloud_ptr->header.frame_id = base_options_.frame_id;
                bg_pub_.publish(cloud_ptr);
            }
        }
        if (pub_keys_set_.find(_ground) != pub_keys_set_.end()) {
            // publish ground cloud
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                genGround(cloud_ptr);
                cloud_ptr->header.stamp = stamp;
                cloud_ptr->header.frame_id = base_options_.frame_id;
                g_pub_.publish(cloud_ptr);
            }
        }
        if (pub_keys_set_.find(_non_ground) != pub_keys_set_.end()) {
            // publish non_ground cloud
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                genNonGround(cloud_ptr);
                cloud_ptr->header.stamp = stamp;
                cloud_ptr->header.frame_id = base_options_.frame_id;
                ng_pub_.publish(cloud_ptr);
            }
        }
        if (pub_keys_set_.find(_clusters) != pub_keys_set_.end()) {
            // publish cluster cloud
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
                genCluster(cloud_ptr);
                cloud_ptr->header.stamp = stamp;
                cloud_ptr->header.frame_id = base_options_.frame_id;
                cluster_pub_.publish(cloud_ptr);
            }
        }
        if (pub_keys_set_.find(_sematic) != pub_keys_set_.end()) {
            // publish sematic cloud
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
                genSematic(cloud_ptr);
                cloud_ptr->header.stamp = stamp;
                cloud_ptr->header.frame_id = base_options_.frame_id;
                sematic_pub_.publish(cloud_ptr);
            }
        }
    }

private:
    inline std::string name() {
        return "CloudPubs";
    }

    std::unique_ptr<ros::NodeHandle> nh_ptr_;
    ros::Publisher origin_pub_, bg_pub_, g_pub_, ng_pub_, cluster_pub_, sematic_pub_;
    std::set<std::string> pub_keys_set_;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_ROS_FOUND

#endif  // RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_CLOUD_PUBS_H_
