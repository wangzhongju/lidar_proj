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

#ifndef RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_PRE_MARKER_PUBS_H_
#define RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_PRE_MARKER_PUBS_H_

#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND

#include "rviz_display/external/common/base_marker_pub.h"
#include "rviz_display/external/ros/ros_common.h"
#include "rviz_display/external/common/geo.h"

namespace robosense {
namespace perception {

struct PreMarkerPubOptions {
    BasePubOptions base_options;
};

class PreMarkerPubs {
public:
    using Ptr = std::shared_ptr<PreMarkerPubs>;

    PreMarkerPubs() {
        nh_ptr_.reset(new ros::NodeHandle);
        V2r_global_pose.reset(new RsPose);
    }

    // load configures from yaml and init pre marker display function.
    // pre marker include perception range/AI preprocessing range and roi range(if using v2r strategy)
    // input: yaml node
    inline void init(const PreMarkerPubOptions &options) {
        base_options_ = options.base_options;

        marker_array_ptr_.reset(new ROS_MARKER_ARRAY);
        perception_pub_ = nh_ptr_->advertise<ROS_MARKER_ARRAY>
        (base_options_.pre_fix + "perception_pre_known_rviz", 1, true);
    }

    // entrance of display the pre marker
    // input: robosense perception message struct
    // output: void. the result will be published into ros space.
    inline void display() {
        const auto &attris_map = RsConfigManager().getCollect();
        marker_array_ptr_->markers.resize(attris_map.size());

        if (is_v2r) {
            pose_map = RsConfigManager().getPose();
        }

        int idx = 0;
        for (auto itr = attris_map.begin(); itr != attris_map.end(); ++itr, idx++) {
            auto &ros_marker = marker_array_ptr_->markers[idx];
            Marker marker;
            if (itr->first == "roi") {
                const auto roi_map = itr->second.pts_map;
                const auto type_map = itr->second.type_map;
                drawRoiMesh(roi_map, type_map, marker, itr->first, base_options_.map_frame_id, 0.1);
            } else {
                std::vector<std::vector<RsVector3f>> triangle_list;
                if (is_v2r) {
                    std::string base = "/PolarisAiDetection";
                    size_t length = itr->first.size() - base.size();
                    std::string frame_id = itr->first.substr(0, length);
                    if (pose_map.find(frame_id) != pose_map.end()) {
                        VecRsVec3f polygons;
                        const auto& pose = pose_map.at(frame_id);
                        calculateNewCoordinate(pose, itr->second.pts, polygons, false);
                        triangle_list = triangulate(polygons);
                    }
                    else {
                        const auto polygons = itr->second.pts;
                        triangle_list = triangulate(polygons);
                    }
                }
                else {
                    const auto polygons = itr->second.pts;
                    triangle_list = triangulate(polygons);
                }

                ColorType color;
                if (itr->first == "vehicle_filter") {
                    color.r = 0;
                    color.g = 0.5;
                    color.b = 0.3;
                } else {
                    color.r = randUni();
                    color.g = randUni();
                    color.b = randUni();
                }
                drawMesh(triangle_list, marker, itr->first, base_options_.frame_id, color, 0.1);
            }
            transMarker(marker, ros_marker);
        }
        perception_pub_.publish(*marker_array_ptr_);
    }

    // entrance of add global pose when using v2r perception strategy.
    inline void addV2rPose(const RsPose::Ptr &pose) {
        is_v2r = true;
        V2r_global_pose = pose;
    }

private:
    inline void drawMesh(const std::vector<std::vector<RsVector3f> > &triangle_list, Marker &marker,
                         const std::string &ns, const std::string &frame_id,
                         const ColorType &color, float alpha = 0) {
        int tri_size = static_cast<int>(triangle_list.size());
        if (tri_size <= 1) {
            return;
        }

        marker.ns = ns;
        marker.id = 0;
        marker.frame_id = frame_id;
        marker.color_type = color;
        marker.scale_type.x = 1.;
        marker.scale_type.y = 1.;
        marker.scale_type.z = 1.;
        marker.type = MarkerType::TRIANGLE_LIST;
        marker.action = ActionType::ADD;
        marker.color_type.a = alpha;

        marker.points.clear();
        marker.points.reserve(tri_size * 3);
        for (int i = 0; i < tri_size; ++i) {
            PositionType p0, p1, p2;
            p0.x = triangle_list[i][0].x;
            p0.y = triangle_list[i][0].y;
            p0.z = triangle_list[i][0].z;

            p1.x = triangle_list[i][1].x;
            p1.y = triangle_list[i][1].y;
            p1.z = triangle_list[i][1].z;

            p2.x = triangle_list[i][2].x;
            p2.y = triangle_list[i][2].y;
            p2.z = triangle_list[i][2].z;

            marker.points.emplace_back(p2);
            marker.points.emplace_back(p1);
            marker.points.emplace_back(p0);
        }
    }

    inline void drawRoiMesh(const std::map<int, VecRsVec3f> &roi_map,
                            const std::map<int, int> &type_map, Marker &marker,
                            const std::string &ns, const std::string &frame_id, float alpha = 0) {
        if (roi_map.empty()) {
            return;
        }

        marker.ns = ns;
        marker.id = 0;
        marker.frame_id = frame_id;
        marker.scale_type.x = 1.;
        marker.scale_type.y = 1.;
        marker.scale_type.z = 1.;
        marker.type = MarkerType::TRIANGLE_LIST;
        marker.action = ActionType::ADD;
        marker.color_type.a = alpha;

        marker.points.clear();
        marker.points.reserve(1000);
        marker.colors.clear();
        marker.colors.reserve(1000);

        for (const auto& item: roi_map) {
            const auto& roi_id = item.first;
            auto polygon = item.second;
            int pts_num = static_cast<int>(polygon.size());
            if (pts_num < 3) {
                continue;
            }

            std::vector<std::vector<RsVector3f>> triangles;
            std::vector<std::vector<RsVector3f>> triangles1;
            std::vector<std::vector<RsVector3f>> triangles2;

            if (is_v2r) {
                RsPose pose;
                VecRsVec3f new_polygon;
                calculateNewCoordinate(pose, polygon, new_polygon, true);
                triangles1 = triangulate(new_polygon);
                std::reverse(new_polygon.begin(), new_polygon.end());
                triangles2 = triangulate(new_polygon);
            }
            else {
                triangles1 = triangulate(polygon);
                std::reverse(polygon.begin(), polygon.end());
                triangles2 = triangulate(polygon);
            }

            triangles2.size() > triangles1.size() ? triangles = triangles2 : triangles = triangles1;

            for (auto & triangle : triangles) {
                PositionType p0, p1, p2;
                p0.x = triangle[0].x;
                p0.y = triangle[0].y;
                p0.z = 0.0;  // triangle_list[i][0].z;

                p1.x = triangle[1].x;
                p1.y = triangle[1].y;
                p1.z = 0.0;  // triangle_list[i][1].z;

                p2.x = triangle[2].x;
                p2.y = triangle[2].y;
                p2.z = 0.0;  // triangle_list[i][2].z;

                float temp = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
                if (temp > 0) {
                    marker.points.emplace_back(p0);
                    marker.points.emplace_back(p1);
                    marker.points.emplace_back(p2);
                }
                else {
                    marker.points.emplace_back(p2);
                    marker.points.emplace_back(p1);
                    marker.points.emplace_back(p0);
                }

                marker.colors.emplace_back(getRoiColor(type_map.at(roi_id)));
                marker.colors.emplace_back(getRoiColor(type_map.at(roi_id)));
                marker.colors.emplace_back(getRoiColor(type_map.at(roi_id)));
            }

        }
    }

    inline void calculateNewCoordinate(const RsPose &pose , const VecRsVec3f& old_polygons, VecRsVec3f& new_polygons, bool is_roi) {
        // global pose 转换
        Eigen::Matrix4f new_mat;
        if (!is_roi) {
            new_mat = poseToEigenMat(*V2r_global_pose);
        } else {
            new_mat = poseToEigenMat(pose);
        }
        Eigen::Matrix4f global_mat = poseToEigenMat(pose);
        global_mat = new_mat * global_mat;

        // base_link -> real global
        Eigen::Matrix4f vehicle_mat = poseToEigenMat(pose);
        Eigen::Matrix4f transform_mat = global_mat * vehicle_mat.inverse();

        for (const auto& tt: old_polygons) {
            RsVector3f tmp;
            Eigen::Vector4f tt_e;
            tt_e << tt.x , tt.y , tt.z , 1;
            tt_e = transform_mat * tt_e;
            tmp.x = tt_e.x();
            tmp.y = tt_e.y();
            tmp.z = tt_e.z();
            new_polygons.emplace_back(tmp);
        }
    }

    inline Eigen::Matrix4f poseToEigenMat(const RsPose& pose) {
        Eigen::AngleAxisf init_rotation_x(pose.roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(pose.pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(pose.yaw, Eigen::Vector3f::UnitZ());

        Eigen::Translation3f init_translation(pose.x, pose.y, pose.z);

        return (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    }

    inline ColorType getRoiColor(int idx) {
        ColorType color_temp;
        if (idx == 0) {
            color_temp.a = 0.8;
            color_temp.r = 218 / 255.0;
            color_temp.g = 112 / 255.0;
            color_temp.b = 214 / 255.0;
        } else if (idx == 1) {
            color_temp.a = 0.8;
            color_temp.r = 200 / 255.0;
            color_temp.g = 100 / 255.0;
            color_temp.b = 50 / 255.0;
        } else {
            color_temp.a = 0.8;
            color_temp.r = 100 / 255.0;
            color_temp.g = 200 / 255.0;
            color_temp.b = 50 / 255.0;
        }
        return color_temp;
    }

    inline std::string name() {
        return "PreMarkerPubs";
    }

    BasePubOptions base_options_;
    std::unique_ptr<ros::NodeHandle> nh_ptr_;

    ROS_MARKER_ARRAY::Ptr marker_array_ptr_;
    ros::Publisher perception_pub_;

    bool is_v2r = false;
    RsPose::Ptr V2r_global_pose;
    std::map<std::string, RsPose> pose_map;
};


}  // namespace perception
}  // namespace robosense

#endif  // RS_ROS_FOUND

#endif  // RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_PRE_MARKER_PUBS_H_
