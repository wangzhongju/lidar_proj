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

#ifndef RS_RVIZ_DISPLAY_EXTERNAL_COMMON_BASE_CLOUD_PUB_H_
#define RS_RVIZ_DISPLAY_EXTERNAL_COMMON_BASE_CLOUD_PUB_H_

#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND

#include "rviz_display/external/common/base_pub.h"

namespace robosense {
namespace perception {

class BaseCloudPub {
public:
    using Ptr = std::shared_ptr<BaseCloudPub>;

    virtual ~BaseCloudPub() = default;

    virtual void display(const RsPerceptionMsg::Ptr &msg_ptr) = 0;

protected:
    virtual std::string name() = 0;

    const std::map<ObjectType, ColorType> origin_color_map = {
    {ObjectType::UNKNOW,        genColor(Color::Lavender)},
    {ObjectType::CONE,          genColor(Color::Lavender)},
    {ObjectType::PED,           genColor(Color::Yellow)},
    {ObjectType::BIC,           genColor(Color::CyanBlue)},
    {ObjectType::CAR,           genColor(Color::Red)},
    {ObjectType::TRUCK_BUS,     genColor(Color::Blue)},
    {ObjectType::ULTRA_VEHICLE, genColor(Color::Blue)},
    };

    const std::map<ObjectType, ColorType> efficient_color_map = {
    {ObjectType::UNKNOW,        genColor(Color::Lavender)},
    {ObjectType::CONE,          genColor(Color::Lavender)},
    {ObjectType::PED,           genColor(Color::Yellow)},
    {ObjectType::BIC,           genColor(Color::Green)},
    {ObjectType::CAR,           genColor(Color::CyanBlue)},
    {ObjectType::TRUCK_BUS,     genColor(Color::CyanBlue)},
    {ObjectType::ULTRA_VEHICLE, genColor(Color::CyanBlue)},
    };

    inline void genClusterColors() {
        colors_.clear();
        colors_.resize(colors_nums_);
        for (size_t i = 0; i < colors_.size(); ++i) {
            auto &color = colors_[i];
            unsigned int seed;
            seed = static_cast<unsigned int> (i * 3 + 0);
            color.x = rand_r(&seed) % 255 + 60;
            seed = static_cast<unsigned int> (i * 3 + 1);
            color.y = rand_r(&seed) % 255 + 60;
            seed = static_cast<unsigned int> (i * 3 + 2);
            color.z = rand_r(&seed) % 255 + 60;

            color.x = color.x < 255 ? color.x : 255;
            color.y = color.y < 255 ? color.y : 255;
            color.z = color.z < 255 ? color.z : 255;
        }
    }

    inline void genBackground(const pcl::PointCloud<pcl::PointXYZI>::Ptr &bg_cloud_ptr) {
        const auto &indice = msg_ptr_->rs_lidar_result_ptr->background_indices;
        bg_cloud_ptr->points.reserve(indice.size());
        for (size_t i = 0; i < indice.size(); ++i) {
            bg_cloud_ptr->points.emplace_back(cloud_ptr_->points[indice[i]]);
        }
    }

    inline void genGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr &g_cloud_ptr) {
        const auto &indice = msg_ptr_->rs_lidar_result_ptr->ground_indices;
        g_cloud_ptr->points.reserve(indice.size());
        for (size_t i = 0; i < indice.size(); ++i) {
            g_cloud_ptr->points.emplace_back(cloud_ptr_->points[indice[i]]);
        }
    }

    inline void genNonGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr &ng_cloud_ptr) {
        const auto &indice = msg_ptr_->rs_lidar_result_ptr->non_ground_indices;
        ng_cloud_ptr->points.reserve(indice.size());
        for (size_t i = 0; i < indice.size(); ++i) {
            ng_cloud_ptr->points.emplace_back(cloud_ptr_->points[indice[i]]);
        }
    }

    inline void genCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cluster_cloud_ptr) {
        cluster_cloud_ptr->reserve(cloud_ptr_->size());

        const auto &objects = msg_ptr_->rs_lidar_result_ptr->objects;
        for (size_t i = 0; i < objects.size(); ++i) {
            const auto &obj = objects[i];
            const auto &cloud_indices = obj->supplement_infos_.cloud_indices;
            for (size_t j = 0; j < cloud_indices.size(); ++j) {
                const auto &pt = cloud_ptr_->points[cloud_indices[j]];
                pcl::PointXYZRGB pt_rgb;
                pt_rgb.x = pt.x;
                pt_rgb.y = pt.y;
                pt_rgb.z = pt.z;
                pt_rgb.r = colors_[i % colors_nums_].x;
                pt_rgb.g = colors_[i % colors_nums_].y;
                pt_rgb.b = colors_[i % colors_nums_].z;
                cluster_cloud_ptr->points.emplace_back(pt_rgb);
            }
        }
    }

    inline void genSematic(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &sematic_cloud_ptr) {
        sematic_cloud_ptr->resize(cloud_ptr_->size());

        for (size_t i = 0; i < cloud_ptr_->size(); ++i) {
            const auto &pt = cloud_ptr_->points[i];
            auto &se_pt = sematic_cloud_ptr->points[i];
            se_pt.x = pt.x;
            se_pt.y = pt.y;
            se_pt.z = pt.z;
            //默认为白色
            se_pt.r = 200;
            se_pt.g = 200;
            se_pt.b = 200;
        }

        for (size_t i = 0; i < msg_ptr_->rs_lidar_result_ptr->objects.size(); ++i) {
            const auto &obj = msg_ptr_->rs_lidar_result_ptr->objects[i];
            ColorType color;
            if (DisplayMode::ORIGIN == base_options_.mode) {
                color = origin_color_map.at(obj->core_infos_.type);
            } else {
                color = efficient_color_map.at(obj->core_infos_.type);
            }
            for (size_t j = 0; j < obj->supplement_infos_.cloud_indices.size(); ++j) {
                const auto &pt_idx = obj->supplement_infos_.cloud_indices[j];
                auto &se_pt = sematic_cloud_ptr->points[pt_idx];
                se_pt.r = color.r * 255;
                se_pt.g = color.g * 255;
                se_pt.b = color.b * 255;
            }
        }

        for (size_t i = 0; i < msg_ptr_->rs_lidar_result_ptr->ground_indices.size(); ++i) {
            const auto &pt_idx = msg_ptr_->rs_lidar_result_ptr->ground_indices[i];
            auto &se_pt = sematic_cloud_ptr->points[pt_idx];
            se_pt.r = 45;
            se_pt.g = 84;
            se_pt.b = 39;
        }
    }

    BasePubOptions base_options_;
    RsPerceptionMsg::Ptr msg_ptr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr_;
    int colors_nums_ = 100;
    std::vector<RsVector3f> colors_;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_ROS_FOUND

#endif  // RS_RVIZ_DISPLAY_EXTERNAL_COMMON_BASE_CLOUD_PUB_H_
