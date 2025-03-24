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

#ifndef RS_RVIZ_DISPLAY_EXTERNAL_COMMON_BASE_MARKER_PUB_H_
#define RS_RVIZ_DISPLAY_EXTERNAL_COMMON_BASE_MARKER_PUB_H_

#include "rviz_display/external/common/base_pub.h"
#include "rs_perception/common/external/basic_type/rs_bbox.h"

namespace robosense {
namespace perception {

const char _acc_dir[] = "acc_dir";
const char _atten[] = "atten";
const char _barrier[] = "barrier";
const char _box_infos[] = "box_infos";
const char _box_lines[] = "box_lines";
const char _cube[] = "cube";
const char _cylinder[] = "cylinder";
const char _cylinder_lines[] = "cylinder_lines";
const char _freespace[] = "freespace";
const char _gps[] = "gps";
const char _label_infos[] = "label_infos";
const char _lane[] = "lane";
const char _polygon[] = "polygon";
const char _atten_polygon[] = "atten_polygon";
const char _roadedge[] = "roadedge";
const char _track_infos[] = "track_infos";
const char _trajectory[] = "trajectory";
const char _vel_dir[] = "vel_dir";
const char _atten_label[] = "atten_label";
const char _road_boad[] = "road_board";
const char _tunnel[] = "tunnel";
const char _scene_struct_cube[] = "scene_struct_cube";
const char _scene_struct_box_lines[] = "scene_struct_box_lines";
const char _scene_struct_info[] = "scene_struct_info";
const char _mirror_cube[] = "mirror_cube";
const char _mirror_box_lines[] = "mirror_box_lines";

class BaseMarkerPub {
public:
    using Ptr = std::shared_ptr<BaseMarkerPub>;

    virtual ~BaseMarkerPub() = default;

    virtual void display(const RsPerceptionMsg::Ptr &msg_ptr) = 0;

protected:
    virtual std::string name() = 0;

    const std::set<ObjectType> cylinder_set = {
    ObjectType::CONE,
    ObjectType::PED,
    };

    const std::map<ObjectType, ColorType> origin_color_map = {
    {ObjectType::UNKNOW,        genColor(Color::Lavender)},
    {ObjectType::CONE,          genColor(Color::Lavender)},
    {ObjectType::PED,           genColor(Color::Yellow)},
    {ObjectType::BIC,           genColor(Color::Green)},
    {ObjectType::CAR,           genColor(Color::Red)},
    {ObjectType::TRUCK_BUS,     genColor(Color::Blue)},
    {ObjectType::ULTRA_VEHICLE, genColor(Color::Blue)},
    };

    const std::map<ObjectType, ColorType> efficient_color_map = {
    {ObjectType::UNKNOW,        genColor(Color::Lavender)},
    {ObjectType::CONE,          genColor(Color::Lavender)},
    {ObjectType::PED,           genColor(Color::Yellow)},
    {ObjectType::BIC,           genColor(Color::Green)},
    {ObjectType::CAR,           genColor(Color::Red)},
    {ObjectType::TRUCK_BUS,     genColor(Color::Red)},
    {ObjectType::ULTRA_VEHICLE, genColor(Color::Red)},
    };

    const ColorType default_color_type = ColorType(0., 0., 0., 0.01);
    const ScaleType default_scale_type = ScaleType(0.8, 0.8, 0.8);

    template<typename T>
    inline std::string num2str(T num, int precision) {
        std::stringstream ss;
        ss.setf(std::ios::fixed, std::ios::floatfield);
        ss.precision(precision);
        std::string st;
        ss << num;
        ss >> st;

        return st;
    }

    inline void genFunc(const std::vector<std::string> &pub_keys) {
        func_set_.clear();
        marker_list_map_.clear();
        for (const auto &str : pub_keys) {
            std::function<void(const RsPerceptionMsg::Ptr &)> func;
            std::vector<Marker> marker_list;
            bool flag = false;
            if (_acc_dir == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genAccDir(msg_ptr); };
                flag = true;
            } else if (_barrier == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genBarrier(msg_ptr); };
                flag = true;
            } else if (_atten == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genAtten(msg_ptr); };
                flag = true;
            } else if (_box_infos == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genBoxInfos(msg_ptr); };
                flag = true;
            } else if (_box_lines == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genBoxLines(msg_ptr); };
                flag = true;
            } else if (_cube == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genCube(msg_ptr); };
                flag = true;
            } else if (_cylinder_lines == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genCylinderLines(msg_ptr); };
                flag = true;
            } else if (_cylinder == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genCylinder(msg_ptr); };
                flag = true;
            } else if (_freespace == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genFreespace(msg_ptr); };
                flag = true;
            } else if (_gps == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genGps(msg_ptr); };
                flag = true;
            } else if (_label_infos == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genLabelInfos(msg_ptr); };
                flag = true;
            } else if (_lane == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genLane(msg_ptr); };
                flag = true;
            } else if (_atten_polygon == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genAttenPolygon(msg_ptr); };
                flag = true;
            } else if (_polygon == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genPolygon(msg_ptr); };
                flag = true;
            } else if (_atten_label == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genAttenLabel(msg_ptr); };
                flag = true;
            } else if (_roadedge == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genRoadedge(msg_ptr); };
                flag = true;
            } else if (_track_infos == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genTrackInfos(msg_ptr); };
                flag = true;
            } else if (_trajectory == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genTrajectory(msg_ptr); };
                flag = true;
            } else if (_vel_dir == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genVelDir(msg_ptr); };
                flag = true;
            } else if (_scene_struct_cube == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genSceneStructCube(msg_ptr); };
                flag = true;
            } else if (_scene_struct_box_lines == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genSceneStructBoxLines(msg_ptr); };
                flag = true;
            } else if (_scene_struct_info == str) {
                func = [this](const RsPerceptionMsg::Ptr &msg_ptr) { genSceneStructInfo(msg_ptr); };
                flag = true;
            } else if (_mirror_box_lines == str) {
                func = [this](const RsPerceptionMsg::Ptr& msg_ptr) { genMirrorBoxLines(msg_ptr); };
                flag = true;
            } else if (_mirror_cube == str) {
                func = [this](const RsPerceptionMsg::Ptr& msg_ptr) { genMirrorCube(msg_ptr); };
                flag = true;
            }

            if (flag) {
                func_set_.push_back(func);
                marker_list_map_[str] = marker_list;
            }
        }
    }

    inline void genAccDir(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_acc_dir) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _acc_dir << " begin!";

        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;
        auto &marker_list = marker_list_map_.at(_acc_dir);
        if (marker_list.size() < obj_vec.size()) {
            marker_list.resize(obj_vec.size());
        }

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type = default_color_type;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = "acc_dir";
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];

            auto &tmp_marker = marker_list[i];

            tmp_marker.color_type.r = 0.9f;
            tmp_marker.color_type.g = 0.9f;
            tmp_marker.color_type.b = 0.f;

            drawAccArrow(tmp_obj, tmp_marker, 0.5);
        }
        RTRACE << name() << ": " << _acc_dir << " end!";
    }

    inline void genBarrier(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_barrier) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _barrier << " begin!";

        auto &marker_list = marker_list_map_.at(_barrier);
        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->attention_objects;

        std::vector<std::vector<RsVector3f> > reference_vec(obj_vec.size());
        for (size_t k = 0; k < obj_vec.size(); ++k) {
            const auto &poly = obj_vec[k]->supplement_infos_.polygon;
            const auto &l_idx = obj_vec[k]->supplement_infos_.left_point_index;
            const auto &r_idx = obj_vec[k]->supplement_infos_.right_point_index;
            auto &refer = reference_vec[k];
            refer.reserve(poly.size());
            int poly_idx = -1;
            for (int i = l_idx; poly_idx != r_idx; ++i) {
                poly_idx = i % poly.size();
                refer.emplace_back(poly[poly_idx]);
            }
            refer.resize(refer.size());
        }

        size_t ref_box_cc = 0;
        for (size_t i = 0; i < reference_vec.size(); ++i) {
            ref_box_cc += reference_vec[i].size() - 1;
        }

        if (marker_list.size() < ref_box_cc) {
            marker_list.resize(ref_box_cc);
        }

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type.r = 0.5;
            tmp_maker.color_type.g = 0.5;
            tmp_maker.color_type.b = 0.5;
            tmp_maker.color_type.a = 0.01;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = "barrier";
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }

        int attention_marker_idx = 0;
        for (size_t i = 0; i < reference_vec.size(); ++i) {
            const auto &obj = obj_vec[i];
            const auto &refer = reference_vec[i];
            if (obj->core_infos_.attention_type != AttentionType::ATTENTION) {
                continue;
            }
            if (obj->supplement_infos_.mode != ModeType::BSD) {
                continue;
            }
            if (refer.empty() || refer.size() <= 1) {
                continue;
            }
            if (obj->any_map.find("type") == obj->any_map.end()) {
                continue;
            } else {
                if (*obj->any_map.at("type")->AnyCast<std::string>() != "barrier") {
                    continue;
                }
            }
            RsBBox box;
            box.center.z = obj->core_infos_.center.z;
            box.size.z = obj->core_infos_.size.z;

            for (size_t j = 1; j < refer.size(); ++j) {
                const auto &pt0 = refer[j - 1];
                const auto &pt1 = refer[j];
                auto y_diff = pt1.y - pt0.y;
                auto x_diff = pt1.x - pt0.x;
                box.angle = std::atan2(y_diff, x_diff);
                box.center.x = (pt0.x + pt1.x) / 2.;
                box.center.y = (pt0.y + pt1.y) / 2.;
                box.size.x = std::sqrt(y_diff * y_diff + x_diff * x_diff);
                box.size.y = 0.01;
                drawCube(box, marker_list[attention_marker_idx], 0.5f);
                attention_marker_idx++;
            }
        }
        RTRACE << name() << ": " << _barrier << " end!";
    }

    inline void genAtten(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_atten) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _atten << " begin!";

        auto &marker_list = marker_list_map_.at(_atten);
        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->attention_objects;

        std::vector<std::vector<RsVector3f> > reference_vec(obj_vec.size());
        for (size_t k = 0; k < obj_vec.size(); ++k) {
            const auto &poly = obj_vec[k]->supplement_infos_.polygon;
            const auto &l_idx = obj_vec[k]->supplement_infos_.left_point_index;
            const auto &r_idx = obj_vec[k]->supplement_infos_.right_point_index;
            auto &refer = reference_vec[k];
            refer.reserve(poly.size());
            int poly_idx = -1;
            for (int i = l_idx; poly_idx != r_idx; ++i) {
                poly_idx = i % poly.size();
                refer.emplace_back(poly[poly_idx]);
            }
            refer.resize(refer.size());
        }

        size_t ref_box_cc = 0;
        for (size_t i = 0; i < reference_vec.size(); ++i) {
            ref_box_cc += reference_vec[i].size() - 1;
        }

//        if (marker_list.size() < ref_box_cc) {
//            marker_list.resize(ref_box_cc);
//        }
        marker_list.clear();
        marker_list.resize(ref_box_cc);

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type.r = 0.6;
            tmp_maker.color_type.g = 0.6;
            tmp_maker.color_type.b = 0.4;
            tmp_maker.color_type.a = 0.01;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = "attention";
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }
        int attention_marker_idx = 0;
        for (size_t i = 0; i < reference_vec.size(); ++i) {
            const auto &obj = obj_vec[i];
            const auto &refer = reference_vec[i];
            if (obj->core_infos_.attention_type != AttentionType::ATTENTION) {
                continue;
            }
            if (obj->supplement_infos_.mode != ModeType::BSD) {
                continue;
            }
            if (refer.empty() || refer.size() <= 1) {
                continue;
            }
            if (obj->any_map.find("type") != obj->any_map.end()) {
                if (*obj->any_map.at("type")->AnyCast<std::string>() == "barrier") {
                    continue;
                }
            }
            RsBBox box;
            box.center.z = obj->core_infos_.center.z;
            box.size.z = obj->core_infos_.size.z;
            for (size_t j = 1; j < refer.size(); ++j) {
                const auto &pt0 = refer[j - 1];
                const auto &pt1 = refer[j];
                auto y_diff = pt1.y - pt0.y;
                auto x_diff = pt1.x - pt0.x;
                box.angle = std::atan2(y_diff, x_diff);
                box.center.x = (pt0.x + pt1.x) / 2.;
                box.center.y = (pt0.y + pt1.y) / 2.;
                box.size.x = std::sqrt(y_diff * y_diff + x_diff * x_diff);
                box.size.y = 0.01;
                drawCube(box, marker_list[attention_marker_idx], 0.5f);
                attention_marker_idx++;
            }
        }
        RTRACE << name() << ": " << _atten << " end!";
    }

    inline void genBoxInfos(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_box_infos) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _box_infos << " begin!";

        auto &marker_list = marker_list_map_.at(_box_infos);

        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;

        if (marker_list.size() < obj_vec.size()) {
            marker_list.resize(obj_vec.size());
        }

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type = default_color_type;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = "box_info";
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];

            if (ObjectType::UNKNOW == tmp_obj->core_infos_.type) {
                continue;
            }
            if(tmp_obj->any_map.find("mirror") != tmp_obj->any_map.end()) {
                if (tmp_obj->any_map.at("mirror")->AnyCast<bool>()) {
                    continue;
                }
            }
            const auto &center = tmp_obj->core_infos_.center;
            const auto &size = tmp_obj->core_infos_.size;
            const auto &anchor = tmp_obj->core_infos_.anchor;
            auto &tmp_marker = marker_list[i];

            tmp_marker.color_type.r = .7f;
            tmp_marker.color_type.g = .5f;
            tmp_marker.color_type.b = .0f;

            std::string text_box =
            " size(" + num2str<float>(size.x, 1) + " " +
            num2str<float>(size.y, 1) + " " + num2str<float>(size.z, 1) + ")\n center("
            + num2str<float>(center.x, 1) + " " + num2str<float>(center.y, 1) + " "
            + num2str<float>(center.z, 1) + ") "+num2str<float>(anchor.norm(), 1) ;  // 0.2 meter as chassis

            RsVector3f pos = center;
            pos.z += size.z * 0.5f + 0.5f;

            drawText(pos, text_box, tmp_marker, 1.);
        }
        RTRACE << name() << ": " << _box_infos << " end!";
    }

    inline void genBoxLines(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_box_lines) == marker_list_map_.end()) {
            return;
        }

        RTRACE << name() << ": " << _box_lines << " begin!";

        auto &marker_list = marker_list_map_.at(_box_lines);

        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;
        marker_list.resize(1);

        auto &box_marker = marker_list[0];
        box_marker.color_type = default_color_type;
        box_marker.scale_type.x = 0.05;
        box_marker.scale_type.y = 0.05;
        box_marker.scale_type.z = 0.05;
        box_marker.ns = "box";
        box_marker.id = 0;
        box_marker.frame_id = base_options_.frame_id;
        box_marker.type = MarkerType::LINE_LIST;
        box_marker.action = ActionType::ADD;

        genOrientation(box_marker);

//        int pts_size = static_cast<int>(box_marker.points.size());
//        int obj_size = static_cast<int>(obj_vec.size());
//
//        if (pts_size < obj_size * 24) {
//            box_marker.points.resize(obj_size * 24);
//            box_marker.colors.resize(obj_size * 24);
//        }
        box_marker.points.clear();
        box_marker.points.resize(obj_vec.size() * 48);
        box_marker.colors.resize(obj_vec.size() * 48);

        for (size_t i = 0; i < box_marker.points.size(); ++i) {
            box_marker.colors[i].a = 0.01;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];
//            if (cylinder_set.find(tmp_obj->core_infos_.type) != cylinder_set.end() &&
//                DisplayMode::EFFICIENT == base_options_.mode) {
//                continue;
//            }

            if (ObjectType::UNKNOW == tmp_obj->core_infos_.type ||
                ObjectType::PED == tmp_obj->core_infos_.type ||
                ObjectType::CONE == tmp_obj->core_infos_.type) {
                continue;
            }
            if(tmp_obj->any_map.find("mirror") != tmp_obj->any_map.end()) {
                if (tmp_obj->any_map.at("mirror")->AnyCast<bool>()) {
                    continue;
                }
            }

            ColorType color;
            if (DisplayMode::ORIGIN == base_options_.mode) {
                color = origin_color_map.at(tmp_obj->core_infos_.type);
            } else {
                color = efficient_color_map.at(tmp_obj->core_infos_.type);
            }
            bool is_attention = (tmp_obj->core_infos_.attention_type == AttentionType::ATTENTION);
            float tmp_alpha;
            if (is_attention) {
                tmp_alpha = 1.;
            } else {
                tmp_alpha = 0.7;
            }

            const auto &center = tmp_obj->core_infos_.center;
            const auto &size = tmp_obj->core_infos_.size;
            const auto &direction = tmp_obj->core_infos_.direction;
            RsBBox box(center, size, direction);

            int idx = i * 24;
            drawBoxLine(box, box_marker, idx, color, tmp_alpha);
        }

        if (box_marker.points.empty()) {
            box_marker.color_type.a = 0.01;
            PositionType pt;
            pt.x = 0;
            pt.y = 0;
            pt.z = 0;
            box_marker.points.emplace_back(pt);
            box_marker.points.emplace_back(pt);
        }

        RTRACE << name() << ": " << _box_lines << " end!";
    }

    inline void genCube(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_cube) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _cube << " begin!";

        auto &marker_list = marker_list_map_.at(_cube);

        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;
        if (marker_list.size() < obj_vec.size()) {
            marker_list.resize(obj_vec.size());
        }

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type = default_color_type;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = "cube";
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];
//            if (cylinder_set.find(tmp_obj->core_infos_.type) != cylinder_set.end() &&
//                DisplayMode::EFFICIENT == base_options_.mode) {
//                continue;
//            }

            if (ObjectType::UNKNOW == tmp_obj->core_infos_.type ||
                ObjectType::PED == tmp_obj->core_infos_.type ||
                ObjectType::CONE == tmp_obj->core_infos_.type) {
                continue;
            }
            if(tmp_obj->any_map.find("mirror") != tmp_obj->any_map.end()) {
                if (tmp_obj->any_map.at("mirror")->AnyCast<bool>()) {
                    continue;
                }
            }

            ColorType color;
            if (DisplayMode::ORIGIN == base_options_.mode) {
                color = origin_color_map.at(tmp_obj->core_infos_.type);
            } else {
                color = efficient_color_map.at(tmp_obj->core_infos_.type);
            }
            const auto &center = tmp_obj->core_infos_.center;
            const auto &size = tmp_obj->core_infos_.size;
            auto &tmp_marker = marker_list[i];

            tmp_marker.color_type = color;
            const auto &direction = tmp_obj->core_infos_.direction;
            RsBBox box(center, size, direction);

            bool is_attention = (tmp_obj->core_infos_.attention_type == AttentionType::ATTENTION);
            float tmp_alpha = 0.;
            if (is_attention) {
                tmp_alpha = .7;
            } else {
                tmp_alpha = 0.5;
            }

            drawCube(box, tmp_marker, tmp_alpha);
        }

        RTRACE << name() << ": " << _cube << " end!";
    }

    inline void genCylinderLines(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_cylinder_lines) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _cylinder_lines << " begin!";

        auto &marker_list = marker_list_map_.at(_cylinder_lines);
        marker_list.resize(1);
        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;

        auto &marker = marker_list[0];
        marker.color_type = default_color_type;
        marker.scale_type.x = 0.05;
        marker.scale_type.y = 0.05;
        marker.scale_type.z = 0.05;
        marker.ns = "cylinder_box";
        marker.id = 0;
        marker.frame_id = base_options_.frame_id;
        marker.type = MarkerType::LINE_LIST;
        marker.action = ActionType::ADD;

        genOrientation(marker);
        marker.points.clear();
        marker.points.resize(obj_vec.size() * 48);
        marker.colors.resize(obj_vec.size() * 48);

        for (size_t i = 0; i < marker.points.size(); ++i) {
            marker.colors[i].a = 0.01;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];
            if (cylinder_set.find(tmp_obj->core_infos_.type) == cylinder_set.end()) {
                continue;
            }

            ColorType color;
            if (DisplayMode::ORIGIN == base_options_.mode) {
                color = origin_color_map.at(tmp_obj->core_infos_.type);
            } else {
                color = efficient_color_map.at(tmp_obj->core_infos_.type);
            }
            bool is_attention = (tmp_obj->core_infos_.attention_type == AttentionType::ATTENTION);
            float tmp_alpha;
            if (is_attention) {
                tmp_alpha = 1.;
            } else {
                tmp_alpha = 0.3;
            }
            const auto &center = tmp_obj->core_infos_.center;
            const auto &size = tmp_obj->core_infos_.size;
            const auto &direction = tmp_obj->core_infos_.direction;
            RsBBox box(center, size, direction);

            int idx = i * 48;
            std::vector<PositionType> cub_points;

            std::vector<RsVector3f> corners = calRotationCorners(box);

            for (int i = 0; i < 16; ++i) {
                PositionType pts;
                pts.x = corners[i].x;
                pts.y = corners[i].y;
                pts.z = corners[i].z;
                cub_points.push_back(pts);
            }

            for (int j = 0; j < 48; ++j) {
                marker.colors[idx + j].a = tmp_alpha;
                marker.colors[idx + j].r = color.r;
                marker.colors[idx + j].g = color.g;
                marker.colors[idx + j].b = color.b;
            }
            marker.points[idx + 0] = cub_points[0];
            marker.points[idx + 1] = cub_points[1];

            marker.points[idx + 2] = cub_points[1];
            marker.points[idx + 3] = cub_points[2];

            marker.points[idx + 4] = cub_points[2];
            marker.points[idx + 5] = cub_points[3];

            marker.points[idx + 6] = cub_points[3];
            marker.points[idx + 7] = cub_points[4];

            marker.points[idx + 8] = cub_points[4];
            marker.points[idx + 9] = cub_points[5];

            marker.points[idx + 10] = cub_points[5];
            marker.points[idx + 11] = cub_points[6];

            marker.points[idx + 12] = cub_points[6];
            marker.points[idx + 13] = cub_points[7];

            marker.points[idx + 14] = cub_points[7];
            marker.points[idx + 15] = cub_points[0];

            marker.points[idx + 16] = cub_points[8];
            marker.points[idx + 17] = cub_points[9];

            marker.points[idx + 18] = cub_points[9];
            marker.points[idx + 19] = cub_points[10];

            marker.points[idx + 20] = cub_points[10];
            marker.points[idx + 21] = cub_points[11];

            marker.points[idx + 22] = cub_points[11];
            marker.points[idx + 23] = cub_points[12];

            marker.points[idx + 24] = cub_points[12];
            marker.points[idx + 25] = cub_points[13];

            marker.points[idx + 26] = cub_points[13];
            marker.points[idx + 27] = cub_points[14];

            marker.points[idx + 28] = cub_points[14];
            marker.points[idx + 29] = cub_points[15];

            marker.points[idx + 30] = cub_points[15];
            marker.points[idx + 31] = cub_points[8];

            marker.points[idx + 32] = cub_points[0];
            marker.points[idx + 33] = cub_points[8];

            marker.points[idx + 34] = cub_points[1];
            marker.points[idx + 35] = cub_points[9];

            marker.points[idx + 36] = cub_points[2];
            marker.points[idx + 37] = cub_points[10];

            marker.points[idx + 38] = cub_points[3];
            marker.points[idx + 39] = cub_points[11];

            marker.points[idx + 40] = cub_points[4];
            marker.points[idx + 41] = cub_points[12];

            marker.points[idx + 42] = cub_points[5];
            marker.points[idx + 43] = cub_points[13];

            marker.points[idx + 44] = cub_points[6];
            marker.points[idx + 45] = cub_points[14];

            marker.points[idx + 46] = cub_points[7];
            marker.points[idx + 47] = cub_points[15];
        }

        if (marker.points.empty()) {
            marker.color_type.a = 0.01;
            PositionType pt;
            pt.x = 0;
            pt.y = 0;
            pt.z = 0;
            marker.points.emplace_back(pt);
            marker.points.emplace_back(pt);
        }


        RTRACE << name() << ": " << _cylinder_lines << " end!";
    }

    inline void genCylinder(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_cylinder) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _cylinder << " begin!";

        auto &marker_list = marker_list_map_.at(_cylinder);

        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;

        if (marker_list.size() < obj_vec.size()) {
            marker_list.resize(obj_vec.size());
        }

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type = default_color_type;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = "cylinder";
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];
            if (cylinder_set.find(tmp_obj->core_infos_.type) == cylinder_set.end()) {
                continue;
            }

            ColorType color;
            if (DisplayMode::ORIGIN == base_options_.mode) {
                color = origin_color_map.at(tmp_obj->core_infos_.type);
            } else {
                color = efficient_color_map.at(tmp_obj->core_infos_.type);
            }
            const auto &center = tmp_obj->core_infos_.center;
            const auto &size = tmp_obj->core_infos_.size;
            auto &tmp_marker = marker_list[i];

            tmp_marker.color_type = color;
            drawCylinder(tmp_obj, tmp_marker, 0.5);
        }

        RTRACE << name() << ": " << _cylinder << " end!";
    }

    inline void genFreespace(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_freespace) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _freespace << " begin!";

        auto &marker_list = marker_list_map_.at(_freespace);
        marker_list.resize(1);

        const auto &freespace_ptr = msg_ptr->rs_lidar_result_ptr->freespace_ptr;
        auto &marker = marker_list[0];
        marker.ns = "freespace";
        marker.id = 0;
        marker.frame_id = base_options_.frame_id;
        marker.color_type = default_color_type;
        marker.scale_type.x = 1.;
        marker.scale_type.y = 1.;
        marker.scale_type.z = 1.;
        float alpha = 0.4;
        genOrientation(marker);
        if (DisplayMode::EFFICIENT == base_options_.mode) {
            marker.color_type.r = 0.;
            marker.color_type.g = 1.;
            marker.color_type.b = 0.5;
        } else {
            marker.color_type.r = 0.;
            marker.color_type.g = 0.9;
            marker.color_type.b = 0.;
        }
        drawFreespace(freespace_ptr, marker, alpha);

        RTRACE << name() << ": " << _freespace << " end!";
    }

    inline void genGps(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_gps) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _gps << " begin!";

        auto &marker_list = marker_list_map_.at(_gps);

        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;
        if (marker_list.size() < obj_vec.size()) {
            marker_list.resize(obj_vec.size());
        }

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type = default_color_type;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = "gps";
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];

            if(tmp_obj->any_map.find("mirror") != tmp_obj->any_map.end()) {
                if (tmp_obj->any_map.at("mirror")->AnyCast<bool>()) {
                    continue;
                }
            }

            const auto &center = tmp_obj->core_infos_.center;
            const auto &size = tmp_obj->core_infos_.size;
            auto &tmp_marker = marker_list[i];

            tmp_marker.color_type.r = 1.;
            tmp_marker.color_type.g = 1.;
            tmp_marker.color_type.b = 1.;
            RsVector3f pos = center;
            pos.z -= .5f;
            std::string text_label = "<" + num2str<double>(tmp_obj->supplement_infos_.gps_longtitude, 8) + "," +
                                     num2str<double>(tmp_obj->supplement_infos_.gps_latitude, 8) +
                                     "," + num2str<double>(tmp_obj->supplement_infos_.gps_altitude, 8) + ">";
            drawText(pos, text_label, tmp_marker, 1.);
        }

        RTRACE << name() << ": " << _gps << " end!";
    }

    inline void genLabelInfos(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_label_infos) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _label_infos << " begin!";

        auto &marker_list = marker_list_map_.at(_label_infos);

        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;
        if (marker_list.size() < obj_vec.size()) {
            marker_list.resize(obj_vec.size());
        }

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type = default_color_type;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = "label_info";
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];

            if(tmp_obj->any_map.find("mirror") != tmp_obj->any_map.end()) {
                if (tmp_obj->any_map.at("mirror")->AnyCast<bool>()) {
                    continue;
                }
            }

            const auto &center = tmp_obj->core_infos_.center;
            const auto &size = tmp_obj->core_infos_.size;
            auto &tmp_marker = marker_list[i];

            tmp_marker.color_type.r = 1.;
            tmp_marker.color_type.g = 1.;
            tmp_marker.color_type.b = 1.;
            RsVector3f pos = center;
            pos.z += size.z * 0.5f + 2.1f;

            std::string text_label = kObjectType2NameMap.at(tmp_obj->core_infos_.type);

            drawText(pos, text_label, tmp_marker, 1.);
        }

        RTRACE << name() << ": " << _label_infos << " end!";
    }

    inline void genLane(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_lane) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _lane << " begin!";

        auto &marker_list = marker_list_map_.at(_lane);
        marker_list.resize(1);

        const auto &lanes = msg_ptr->rs_lidar_result_ptr->lanes;
        auto &lane_marker = marker_list[0];
        lane_marker.ns = "lane";
        lane_marker.id = 0;
        lane_marker.frame_id = base_options_.frame_id;
        lane_marker.scale_type.x = 0.2;
        lane_marker.scale_type.y = 0.2;
        lane_marker.scale_type.z = 0.2;
        float alpha = 1.;
        genOrientation(lane_marker);
        drawLane(lanes, lane_marker, alpha);

        RTRACE << name() << ": " << _lane << " end!";
    }

    inline void genPolygon(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_polygon) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _polygon << " begin!";

        auto &marker_list = marker_list_map_.at(_polygon);
        marker_list.resize(1);

        auto &tmp_marker = marker_list[0];
        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;
        tmp_marker.ns = "polygon";
        tmp_marker.id = 0;
        tmp_marker.frame_id = base_options_.frame_id;
        tmp_marker.type = MarkerType::LINE_LIST;
        tmp_marker.action = ActionType::ADD;
        tmp_marker.scale_type.x = 0.05;
        tmp_marker.scale_type.y = 0.05;
        tmp_marker.scale_type.z = 0.05;
        tmp_marker.color_type.r = 0.7f;
        tmp_marker.color_type.g = 0.5f;
        tmp_marker.color_type.b = 0.f;
        tmp_marker.color_type.a = 1.f;

        genOrientation(tmp_marker);

        int polygon_size = 0;
        for (size_t i = 0; i < obj_vec.size(); ++i) {
            polygon_size += obj_vec[i]->supplement_infos_.polygon.size();
        }
        tmp_marker.points.reserve(polygon_size * 6);
        tmp_marker.colors.reserve(polygon_size * 6);
        tmp_marker.points.clear();

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &obj = obj_vec[i];

            std::vector<PositionType> cub_points;

            const auto &polygon = obj->supplement_infos_.polygon;
            size_t p_size = polygon.size();
            float p_low = obj->core_infos_.center.z - obj->core_infos_.size.z * 0.5f;
            float p_high = obj->core_infos_.center.z + obj->core_infos_.size.z * 0.5f;

            for (size_t i = 0; i < p_size; ++i) {
                PositionType pts;
                pts.x = polygon[i].x;
                pts.y = polygon[i].y;
                pts.z = p_low;
                cub_points.emplace_back(pts);
            }

            for (size_t i = 0; i < p_size; ++i) {
                PositionType pts;
                pts.x = polygon[i].x;
                pts.y = polygon[i].y;
                pts.z = p_high;
                cub_points.emplace_back(pts);
            }

            for (size_t i = 0; i < p_size; ++i) {
                size_t next = i + 1;
                next = next < p_size ? next : 0;

                tmp_marker.points.emplace_back(cub_points[i]);
                tmp_marker.points.emplace_back(cub_points[next]);
            }

            for (size_t i = 0; i < p_size; ++i) {
                size_t next = i + 1;
                next = next < p_size ? next : 0;

                tmp_marker.points.emplace_back(cub_points[i + p_size]);
                tmp_marker.points.emplace_back(cub_points[next + p_size]);
            }

            for (size_t i = 0; i < p_size; ++i) {
                tmp_marker.points.emplace_back(cub_points[i]);
                tmp_marker.points.emplace_back(cub_points[i + p_size]);
            }
        }

        if (tmp_marker.points.empty()) {
            tmp_marker.color_type.a = 0.01;
            PositionType pt;
            pt.x = 0;
            pt.y = 0;
            pt.z = 0;
            tmp_marker.points.emplace_back(pt);
            tmp_marker.points.emplace_back(pt);
        }

        RTRACE << name() << ": " << _polygon << " end!";
    }

    inline void genAttenPolygon(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_atten_polygon) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _polygon << " begin!";

        auto &marker_list = marker_list_map_.at(_atten_polygon);
        marker_list.resize(1);

        auto &tmp_marker = marker_list[0];
        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->attention_objects;
        tmp_marker.ns = "atten_polygon";
        tmp_marker.id = 0;
        tmp_marker.frame_id = base_options_.frame_id;
        tmp_marker.type = MarkerType::LINE_LIST;
        tmp_marker.action = ActionType::ADD;
        tmp_marker.scale_type.x = 0.05;
        tmp_marker.scale_type.y = 0.05;
        tmp_marker.scale_type.z = 0.05;
        tmp_marker.color_type.r = 0.7f;
        tmp_marker.color_type.g = 0.5f;
        tmp_marker.color_type.b = 0.f;
        tmp_marker.color_type.a = 1.f;

        genOrientation(tmp_marker);

        int polygon_size = 0;
        for (size_t i = 0; i < obj_vec.size(); ++i) {
            polygon_size += obj_vec[i]->supplement_infos_.polygon.size();
        }
        tmp_marker.points.reserve(polygon_size * 6);
        tmp_marker.colors.reserve(polygon_size * 6);
        tmp_marker.points.clear();
        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &obj = obj_vec[i];
            if (obj->supplement_infos_.mode == ModeType::FOCUS) {
                continue;
            }
            if (obj->any_map.find("type") != obj->any_map.end() &&
                *obj->any_map.at("type")->AnyCast<std::string>() == "barrier") {
                continue;
            }
            std::vector<PositionType> cub_points;

            const auto &polygon = obj->supplement_infos_.polygon;
            size_t p_size = polygon.size();
            float p_low = obj->core_infos_.center.z - obj->core_infos_.size.z * 0.5f;
            float p_high = obj->core_infos_.center.z + obj->core_infos_.size.z * 0.5f;

            for (size_t i = 0; i < p_size; ++i) {
                PositionType pts;
                pts.x = polygon[i].x;
                pts.y = polygon[i].y;
                pts.z = p_low;
                cub_points.emplace_back(pts);
            }

            for (size_t i = 0; i < p_size; ++i) {
                PositionType pts;
                pts.x = polygon[i].x;
                pts.y = polygon[i].y;
                pts.z = p_high;
                cub_points.emplace_back(pts);
            }

            for (size_t i = 0; i < p_size; ++i) {
                size_t next = i + 1;
                next = next < p_size ? next : 0;

                tmp_marker.points.emplace_back(cub_points[i]);
                tmp_marker.points.emplace_back(cub_points[next]);
            }

            for (size_t i = 0; i < p_size; ++i) {
                size_t next = i + 1;
                next = next < p_size ? next : 0;

                tmp_marker.points.emplace_back(cub_points[i + p_size]);
                tmp_marker.points.emplace_back(cub_points[next + p_size]);
            }

            for (size_t i = 0; i < p_size; ++i) {
                tmp_marker.points.emplace_back(cub_points[i]);
                tmp_marker.points.emplace_back(cub_points[i + p_size]);
            }
        }

        if (tmp_marker.points.empty()) {
            tmp_marker.color_type.a = 0.01;
            PositionType pt;
            pt.x = 0;
            pt.y = 0;
            pt.z = 0;
            tmp_marker.points.emplace_back(pt);
            tmp_marker.points.emplace_back(pt);
        }
        RTRACE << name() << ": " << _atten_polygon << " end!";
    }

    inline void genRoadedge(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_roadedge) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _roadedge << " begin!";

        auto &marker_list = marker_list_map_.at(_roadedge);
        marker_list.resize(1);
        const auto &roadedges = msg_ptr->rs_lidar_result_ptr->roadedges;
        auto &roadedge_marker = marker_list[0];
        roadedge_marker.ns = "roadedge";
        roadedge_marker.id = 0;
        roadedge_marker.frame_id = base_options_.frame_id;
        roadedge_marker.scale_type.x = 0.2;
        roadedge_marker.scale_type.y = 0.2;
        roadedge_marker.scale_type.z = 0.2;
        float alpha = 1.;
        genOrientation(roadedge_marker);
        drawRoadedge(roadedges, roadedge_marker, alpha);

        RTRACE << name() << ": " << _roadedge << " end!";
    }

    inline void genTrackInfos(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_track_infos) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _track_infos << " begin!";

        auto &marker_list = marker_list_map_.at(_track_infos);

        auto &all_obj_vec = msg_ptr->rs_lidar_result_ptr->objects;
        auto &all_atten_obj_vec = msg_ptr->rs_lidar_result_ptr->attention_objects;
        VecObjectPtr obj_vec;
        for (auto &obj : all_obj_vec) {
            obj_vec.push_back(obj);
        }
        for (auto &obj : all_atten_obj_vec) {
            if (obj->core_infos_.tracker_id > -1) {
                obj_vec.push_back(obj);
            }
        }

        if (marker_list.size() < obj_vec.size()) {
            marker_list.resize(obj_vec.size());
        }

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type = default_color_type;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = "track_info";
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];
            if (tmp_obj->core_infos_.type == ObjectType::CONE) {
                continue;
            }
            if(tmp_obj->any_map.find("mirror") != tmp_obj->any_map.end()) {
                if (tmp_obj->any_map.at("mirror")->AnyCast<bool>()) {
                    continue;
                }
            }
            ColorType color;
            if (DisplayMode::ORIGIN == base_options_.mode) {
                color = origin_color_map.at(tmp_obj->core_infos_.type);
            } else {
                color = efficient_color_map.at(tmp_obj->core_infos_.type);
            }
            const auto &center = tmp_obj->core_infos_.center;
            const auto &size = tmp_obj->core_infos_.size;
            float velocity = tmp_obj->core_infos_.velocity.norm();
            auto &tmp_marker = marker_list[i];

            tmp_marker.color_type.r = 1.;
            tmp_marker.color_type.g = 1.;
            tmp_marker.color_type.b = 1.;
//        if (DisplayMode::EFFICIENT == base_options_.mode) {
////            tmp_marker.scale_type.x = 0.8;
////            tmp_marker.scale_type.y = 0.8;
////            tmp_marker.scale_type.z = 0.8;
//            tmp_marker.scale_type = default_scale_type;
//        }
            RsVector3f pos = center;
            pos.z += size.z * 0.5f + 1.5f;
            std::string text_track =
            // "[" + num2str<int>(static_cast<int>(tmp_obj->core_infos_.lane_pos), 0) + "]" +
            "<" + num2str<int>(tmp_obj->core_infos_.tracker_id, 0) + ">" +
            num2str<float>(velocity * 3.6f, 1) + "km/h";
            drawText(pos, text_track, tmp_marker, 1.);
        }

        RTRACE << name() << ": " << _track_infos << " end!";
    }

    inline void genTrajectory(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_trajectory) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _trajectory << " begin!";

        auto &marker_list = marker_list_map_.at(_trajectory);
        marker_list.resize(1);
        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;

        auto &box_marker = marker_list[0];
        box_marker.color_type = default_color_type;
        box_marker.scale_type.x = 0.1;
        box_marker.scale_type.y = 0.1;
        box_marker.scale_type.z = 0.1;
        box_marker.ns = "trajectory";
        box_marker.id = 0;
        box_marker.frame_id = base_options_.frame_id;
        box_marker.type = MarkerType::POINTS;
        box_marker.action = ActionType::ADD;

        genOrientation(box_marker);

        size_t point_nums = 0;
        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];

            point_nums += tmp_obj->supplement_infos_.trajectory.size();
        }
        box_marker.points.clear();
        box_marker.colors.clear();
        box_marker.points.resize(point_nums);
        box_marker.colors.resize(point_nums);

        size_t tidx = 0;
        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];

            if(tmp_obj->any_map.find("mirror") != tmp_obj->any_map.end()) {
                if (tmp_obj->any_map.at("mirror")->AnyCast<bool>()) {
                    continue;
                }
            }

            ColorType color;
            if (DisplayMode::ORIGIN == base_options_.mode) {
                color = origin_color_map.at(tmp_obj->core_infos_.type);
            } else {
                color = efficient_color_map.at(tmp_obj->core_infos_.type);
            }
            bool is_attention = (tmp_obj->core_infos_.attention_type == AttentionType::ATTENTION);
            float tmp_alpha;
            if (is_attention) {
                tmp_alpha = 1.;
            } else {
                tmp_alpha = 0.3;
            }

            for (size_t j = 0; j < tmp_obj->supplement_infos_.trajectory.size(); j++) {
                auto &trajectory = tmp_obj->supplement_infos_.trajectory;
                PositionType pt;
                pt.x = trajectory[j].x;
                pt.y = trajectory[j].y;
                pt.z = trajectory[j].z;

                box_marker.points[tidx] = pt;

                box_marker.colors[tidx].a = tmp_alpha;
                box_marker.colors[tidx].r = color.r;
                box_marker.colors[tidx].g = color.g;
                box_marker.colors[tidx].b = color.b;

                tidx++;
            }
        }

        if (box_marker.points.empty()) {
            PositionType pt(0,0,0);
            box_marker.points.emplace_back(pt);
            box_marker.points.emplace_back(pt);
            box_marker.color_type.a = 0.01;
        }

        RTRACE << name() << ": " << _trajectory << " end!";
    }

    inline void genVelDir(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_vel_dir) == marker_list_map_.end()) {
            return;
        }

        RTRACE << name() << ": " << _vel_dir << " begin!";
        auto &marker_list = marker_list_map_.at(_vel_dir);

        auto &all_obj_vec = msg_ptr->rs_lidar_result_ptr->objects;
        auto &all_atten_obj_vec = msg_ptr->rs_lidar_result_ptr->attention_objects;
        VecObjectPtr obj_vec;
        for (auto &obj : all_obj_vec) {
            if (obj->core_infos_.velocity.norm() > 0.1f) {
                obj_vec.push_back(obj);
            }
        }
        for (auto &obj : all_atten_obj_vec) {
            if (obj->core_infos_.tracker_id > -1 && obj->core_infos_.velocity.norm() > 0.1f) {
                obj_vec.push_back(obj);
            }
        }

        if (marker_list.size() < obj_vec.size()) {
            marker_list.resize(obj_vec.size());
        }

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type = default_color_type;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = "vel_dir";
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];
            auto &tmp_marker = marker_list[i];

            if (tmp_obj->core_infos_.type == ObjectType::CONE) {
                tmp_marker.type = MarkerType::LINE_LIST;
                tmp_marker.action = ActionType::ADD;
                tmp_marker.points.clear();
                tmp_marker.colors.clear();
                PositionType pt;
                pt.x = tmp_obj->core_infos_.center.x;
                pt.y = tmp_obj->core_infos_.center.y;
                pt.z = tmp_obj->core_infos_.center.z;
                tmp_marker.points.emplace_back(pt);
                tmp_marker.points.emplace_back(pt);
                continue;
            }

            tmp_marker.color_type.r = 0.3f;
            tmp_marker.color_type.g = 0.9f;
            tmp_marker.color_type.b = 0.f;

            drawVelArrow(tmp_obj, tmp_marker, 0.5);
        }

        RTRACE << name() << ": " << _vel_dir << " end!";
    }

    inline void genAttenLabel(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_atten_label) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _atten_label << " begin!";

        auto &marker_list = marker_list_map_.at(_atten_label);

        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->attention_objects;
        if (marker_list.size() < obj_vec.size()) {
            marker_list.resize(obj_vec.size());
        }

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type = default_color_type;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = "atten_label";
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];

            if (tmp_obj->supplement_infos_.mode == ModeType::FOCUS) {
                continue;
            }

            bool is_barrier = false;
            if (tmp_obj->any_map.find("type") != tmp_obj->any_map.end() &&
                *tmp_obj->any_map.at("type")->AnyCast<std::string>() == "barrier") {
                is_barrier = true;
            }

            const auto &center = tmp_obj->core_infos_.center;
            const auto &size = tmp_obj->core_infos_.size;
            auto &tmp_marker = marker_list[i];

            tmp_marker.color_type.r = 1.;
            tmp_marker.color_type.g = 1.;
            tmp_marker.color_type.b = 1.;
            RsVector3f pos = center;
            pos.z += size.z * 0.5f + 0.9f;

            std::string text_label = "Unknown";
            if (is_barrier) {
                text_label = "Barrier";
            }

            drawText(pos, text_label, tmp_marker, 1.);
        }
        RTRACE << name() << ": " << _atten_label << " end!";
    }

    inline void genSceneStructInfo(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_scene_struct_info) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _scene_struct_info << " begin!";
        auto temp_any_map = msg_ptr->rs_lidar_result_ptr->any_map;
        if (temp_any_map.find(_road_boad) == temp_any_map.end() &&
        temp_any_map.find(_tunnel) == temp_any_map.end()) {
            RTRACE << name() << ": " << _scene_struct_info << " no road board and tunnel!";
            return;
        }
        auto &marker_list = marker_list_map_.at(_scene_struct_info);

        // get board box
        std::vector<RsBBox> board_bbox_vec;
        board_bbox_vec.clear();
        if (temp_any_map.find(_road_boad) != temp_any_map.end()) {
            Any::Ptr any_ptr_board = temp_any_map.at("road_board");
            const auto &bbox_vec = *any_ptr_board->AnyCast<std::vector<RsBBox>>();
            for (size_t i = 0; i < bbox_vec.size(); ++i) {
                const auto &tmp_bbox = bbox_vec[i];
                if (tmp_bbox.size.x > 0 && tmp_bbox.size.y > 0 && tmp_bbox.size.z > 0) {
                    board_bbox_vec.emplace_back(tmp_bbox);
                }
            }
        }
        // get tunnel box
        std::vector<RsBBox> tunnel_bbox_vec;
        tunnel_bbox_vec.clear();
        if (temp_any_map.find(_tunnel) != temp_any_map.end()) {
            Any::Ptr any_ptr_tunnel = temp_any_map.at("tunnel");
            const auto &tunnel_bbox = *any_ptr_tunnel->AnyCast<RsBBox>();
            if (tunnel_bbox.center.x > 0 && static_cast<int>(tunnel_bbox.size.x) > 0 && 
                tunnel_bbox.size.y > 0 && tunnel_bbox.size.z > 0) {
                tunnel_bbox_vec.emplace_back(tunnel_bbox);
            }
        }
        if (marker_list.size() < board_bbox_vec.size() + tunnel_bbox_vec.size()) {
            marker_list.resize(board_bbox_vec.size() + tunnel_bbox_vec.size());
        }
        // init marker lists
        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_marker = marker_list[i];
            tmp_marker.type = MarkerType::TEXT_VIEW_FACING;
            tmp_marker.color_type = default_color_type;
            tmp_marker.scale_type = default_scale_type;
            tmp_marker.ns = "scene_struct_info";
            tmp_marker.id = i;
            tmp_marker.frame_id = base_options_.frame_id;
            tmp_marker.points.clear();
        }

        const int font_size_max = 1;
        // draw board text
        for (size_t i = 0; i < board_bbox_vec.size(); ++i) {
            auto &tmp_bbox = board_bbox_vec[i];
            const auto &center = tmp_bbox.center;
            auto &size = tmp_bbox.size;
            auto &tmp_marker = marker_list[i];
            if (center.x > 80) {
                tmp_marker.scale_type = ScaleType(size.x, size.y, size.z);
            } else if (center.x > 60) {
                tmp_marker.scale_type = ScaleType(2., 2., 2.);
            } else if (center.x > 15) {
                tmp_marker.scale_type = ScaleType(1., 1., 1.);
            } else {
                tmp_marker.scale_type = ScaleType(0.5, 0.5, 0.5);
            }

            std::string text_box = "(distance:" + num2str<float>(center.x, 1) + "m)";

            RsVector3f pos = center;
            pos.z += size.z * 0.5f + 0.2f;
            drawText(pos, text_box, tmp_marker, 1.);
        }
        // draw tunnel text
        for (size_t i = 0; i < tunnel_bbox_vec.size(); ++i) {
            auto &tmp_bbox = tunnel_bbox_vec[i];
            const auto &center = tmp_bbox.center;
            auto &size = tmp_bbox.size;
            auto &tmp_marker = marker_list[i + board_bbox_vec.size()];

            tmp_marker.scale_type = ScaleType(4., 4., 4.);
            tmp_marker.color_type = ColorType(204 / 255.0, 51 / 255.0, 153 / 255.0);
            std::string text_box = "( tunnel distance:" + num2str<float>(center.x, 1) + "m )";

            RsVector3f pos = center;
            pos.z += size.z * 0.5f + 0.2f;
            drawText(pos, text_box, tmp_marker, 1.);
        }

        RTRACE << name() << ": " << _scene_struct_info << " end!";
    }

    inline void genSceneStructCube(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_scene_struct_cube) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _scene_struct_cube << " begin!";
        auto temp_any_map = msg_ptr->rs_lidar_result_ptr->any_map;
        if (temp_any_map.find(_road_boad) == temp_any_map.end() &&
        temp_any_map.find(_tunnel) == temp_any_map.end()) {
            RTRACE << name() << ": " << _scene_struct_cube << " no road board and tunnel!";
            return;
        }
        auto &marker_list = marker_list_map_.at(_scene_struct_cube);

        // get board box
        std::vector<RsBBox> board_bbox_vec;
        board_bbox_vec.clear();
        if (temp_any_map.find(_road_boad) != temp_any_map.end()) {
            Any::Ptr any_ptr_board = temp_any_map.at("road_board");
            const auto &bbox_vec = *any_ptr_board->AnyCast<std::vector<RsBBox>>();
            for (size_t i = 0; i < bbox_vec.size(); ++i) {
                const auto &tmp_bbox = bbox_vec[i];
                if (tmp_bbox.size.x > 0 && tmp_bbox.size.y > 0 && tmp_bbox.size.z > 0) {
                    board_bbox_vec.emplace_back(tmp_bbox);
                }
            }
        }
        // get tunnel box
        std::vector<RsBBox> tunnel_bbox_vec;
        tunnel_bbox_vec.clear();
        if (temp_any_map.find(_tunnel) != temp_any_map.end()) {
            Any::Ptr any_ptr_tunnel = temp_any_map.at("tunnel");
            const auto &tunnel_bbox = *any_ptr_tunnel->AnyCast<RsBBox>();
            if (tunnel_bbox.center.x > 0 && static_cast<int>(tunnel_bbox.size.x) > 0 && 
                tunnel_bbox.size.y > 0 && tunnel_bbox.size.z > 0) {
                tunnel_bbox_vec.emplace_back(tunnel_bbox);
            }
        }
        if (marker_list.size() < board_bbox_vec.size() + tunnel_bbox_vec.size()) {
            marker_list.resize(board_bbox_vec.size() + tunnel_bbox_vec.size());
        }
        // init marker lists
        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_marker = marker_list[i];
            tmp_marker.color_type = default_color_type;
            tmp_marker.scale_type = default_scale_type;
            tmp_marker.ns = "scene_struct_cube";
            tmp_marker.id = i;
            tmp_marker.frame_id = base_options_.frame_id;
            tmp_marker.action = ActionType::MODIFY;
            tmp_marker.points.clear();
        }

        const int font_size_max = 1;
        // draw board cube
        for (size_t i = 0; i < board_bbox_vec.size(); ++i) {
            auto &tmp_bbox = board_bbox_vec[i];
            const auto &center = tmp_bbox.center;
            auto &size = tmp_bbox.size;
            auto &tmp_marker = marker_list[i];
            if (tmp_bbox.size.x > 0 && tmp_bbox.size.y > 0 && tmp_bbox.size.z > 0) {
                tmp_marker.type = MarkerType::CUBE;
                tmp_marker.color_type.a = 0.2;

                tmp_marker.position.x = tmp_bbox.center.x;
                tmp_marker.position.y = tmp_bbox.center.y;
                tmp_marker.position.z = tmp_bbox.center.z;

                tmp_marker.scale_type.x = tmp_bbox.size.x;
                tmp_marker.scale_type.y = tmp_bbox.size.y;
                tmp_marker.scale_type.z = tmp_bbox.size.z;
                drawCube(tmp_bbox, tmp_marker, 0.2);
            }
        }
        // draw tunnel cube
        for (size_t i = 0; i < tunnel_bbox_vec.size(); ++i) {
            auto &tmp_bbox = tunnel_bbox_vec[i];
            const auto &center = tmp_bbox.center;
            auto &size = tmp_bbox.size;
            auto &tmp_marker = marker_list[i + board_bbox_vec.size()];
            tmp_marker.color_type.r = 255 / 255.0;
            tmp_marker.color_type.g = 204 / 255.0;
            tmp_marker.color_type.b = 204 / 255.0;
            tmp_marker.color_type.a = 0.01;
            if (tmp_bbox.center.x > 0 && static_cast<int>(tmp_bbox.size.x) > 0 &&
                tmp_bbox.size.y > 0 && tmp_bbox.size.z > 0) {
                tmp_marker.type = MarkerType::CUBE;
                tmp_marker.action = ActionType::MODIFY;   // ::ADD;

                tmp_marker.color_type.a = 0.2;

                tmp_marker.position.x = tmp_bbox.center.x;
                tmp_marker.position.y = tmp_bbox.center.y;
                tmp_marker.position.z = tmp_bbox.center.z;

                tmp_marker.scale_type.x = tmp_bbox.size.x;
                tmp_marker.scale_type.y = tmp_bbox.size.y;
                tmp_marker.scale_type.z = tmp_bbox.size.z;
                drawCube(tmp_bbox, tmp_marker, 0.2);
            }
        }

        RTRACE << name() << ": " << _scene_struct_cube << " end!";
    }

    inline void genSceneStructBoxLines(const RsPerceptionMsg::Ptr &msg_ptr) {
        if (marker_list_map_.find(_scene_struct_box_lines) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _scene_struct_box_lines << " begin!";

        auto temp_any_map = msg_ptr->rs_lidar_result_ptr->any_map;
        if (temp_any_map.find(_road_boad) == temp_any_map.end() &&
        temp_any_map.find(_tunnel) == temp_any_map.end()) {
            RTRACE << name() << ": " << _scene_struct_box_lines << " no road board and tunnel!";
            return;
        }

        auto &marker_list = marker_list_map_.at(_scene_struct_box_lines);
        marker_list.resize(1);

        // init marker lists
//        for (size_t i = 0; i < marker_list.size(); ++i) {
//
//        }
        auto &box_marker = marker_list[0];
        box_marker.color_type = default_color_type;
        box_marker.scale_type = default_scale_type;
        box_marker.ns = "scene_struct_box_line";
        box_marker.id = 0;
        box_marker.frame_id = base_options_.frame_id;
        box_marker.type = MarkerType::LINE_LIST;
        box_marker.action = ActionType::ADD;
        box_marker.points.clear();

        // get board box
        std::vector<RsBBox> board_bbox_vec;
        board_bbox_vec.clear();
        if (temp_any_map.find(_road_boad) != temp_any_map.end()) {
            Any::Ptr any_ptr_board = temp_any_map.at("road_board");
            const auto &bbox_vec = *any_ptr_board->AnyCast<std::vector<RsBBox>>();
            for (size_t i = 0; i < bbox_vec.size(); ++i) {
                const auto &tmp_bbox = bbox_vec[i];
                if (tmp_bbox.size.x > 0 && tmp_bbox.size.y > 0 && tmp_bbox.size.z > 0) {
                    board_bbox_vec.emplace_back(tmp_bbox);
                }
            }
        }
        // get tunnel box
        std::vector<RsBBox> tunnel_bbox_vec;
        tunnel_bbox_vec.clear();
        if (temp_any_map.find(_tunnel) != temp_any_map.end()) {
            Any::Ptr any_ptr_tunnel = temp_any_map.at("tunnel");
            const auto &tunnel_bbox = *any_ptr_tunnel->AnyCast<RsBBox>();
            if (tunnel_bbox.center.x > 0 && static_cast<int>(tunnel_bbox.size.x) > 0 && 
                tunnel_bbox.size.y > 0 && tunnel_bbox.size.z > 0) {
                tunnel_bbox_vec.emplace_back(tunnel_bbox);
            }
        }

        auto pts_size = static_cast<int>(box_marker.points.size());
        auto obj_size = static_cast<int>(board_bbox_vec.size() + tunnel_bbox_vec.size());
        if (pts_size < obj_size * 24) {
            box_marker.points.resize(obj_size * 24);
            box_marker.colors.resize(obj_size * 24);
        }

        for (size_t i = 0; i < box_marker.points.size(); ++i) {
            box_marker.colors[i].a = 0.01;
        }
        // draw board line
        for (size_t i = 0; i < board_bbox_vec.size(); ++i) {
            const auto &tmp_box = board_bbox_vec[i];
            float tmp_alpha = 1.0;
            ColorType color(1.0, 1.0, 1.0, tmp_alpha);

            box_marker.scale_type = ScaleType(0.15, 0.15, 0.15);
            if (tmp_box.center.x < 30) {
                box_marker.scale_type = ScaleType(0.05, 0.05, 0.05);
            } else if (tmp_box.center.x < 50) {
                box_marker.scale_type = ScaleType(0.1, 0.1, 0.1);
            }

            int idx = i * 24;
            drawBoxLine(tmp_box, box_marker, idx, color, tmp_alpha);
        }
        // draw tunnel line
        for (size_t i = 0; i < tunnel_bbox_vec.size(); ++i) {
            const auto& tmp_box = tunnel_bbox_vec[i];
            float tmp_alpha = 1.0;
            ColorType color(1.0, 204 / 255.0, 204 / 255.0, tmp_alpha);
            box_marker.scale_type = ScaleType(0.15, 0.15, 0.15);
            if (tmp_box.center.x < 30) {
                box_marker.scale_type = ScaleType(0.05, 0.05, 0.05);
            } else if (tmp_box.center.x < 50) {
                box_marker.scale_type = ScaleType(0.1, 0.1, 0.1);
            }
            int idx = (i + board_bbox_vec.size()) * 24;
            drawBoxLine(tmp_box, box_marker, idx, color, tmp_alpha);
        }

        if (box_marker.points.empty()) {
            PositionType pt(0,0,0);
            box_marker.points.emplace_back(pt);
            box_marker.points.emplace_back(pt);
            box_marker.color_type.a = 0.01;
        }

        RTRACE << name() << ": " << _scene_struct_box_lines << " end!";
    }
    
    inline void genMirrorCube(const RsPerceptionMsg::Ptr& msg_ptr) {
        if (marker_list_map_.find(_mirror_cube) == marker_list_map_.end()) {
            return;
        }
        RTRACE << name() << ": " << _mirror_cube << " begin!";

        auto &marker_list = marker_list_map_.at(_mirror_cube);

        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;
        if (marker_list.size() < obj_vec.size()) {
            marker_list.resize(obj_vec.size());
        }

        for (size_t i = 0; i < marker_list.size(); ++i) {
            auto &tmp_maker = marker_list[i];
            tmp_maker.color_type = default_color_type;
            tmp_maker.scale_type = default_scale_type;
            tmp_maker.ns = _mirror_cube;
            tmp_maker.id = i;
            tmp_maker.frame_id = base_options_.frame_id;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];

            if (tmp_obj->any_map.find("mirror") == tmp_obj->any_map.end()) {
                continue;
            }

            ColorType color;
            if (DisplayMode::ORIGIN == base_options_.mode) {
                color = origin_color_map.at(tmp_obj->core_infos_.type);
            } else {
                color = efficient_color_map.at(tmp_obj->core_infos_.type);
            }
            const auto &center = tmp_obj->core_infos_.center;
            const auto &size = tmp_obj->core_infos_.size;
            auto &tmp_marker = marker_list[i];

            color = ColorType(0., 0., 0., 0.);
            tmp_marker.color_type = color;
            const auto &direction = tmp_obj->core_infos_.direction;
            RsBBox box(center, size, direction);

            bool is_attention = (tmp_obj->core_infos_.attention_type == AttentionType::ATTENTION);
            float tmp_alpha = 0.;
            if (is_attention) {
                tmp_alpha = .7;
            } else {
                tmp_alpha = 0.5;
            }

            drawCube(box, tmp_marker, tmp_alpha);
        }

        RTRACE << name() << ": " << _mirror_cube << " end!";
    }

    inline void genMirrorBoxLines(const RsPerceptionMsg::Ptr& msg_ptr) {
        if (marker_list_map_.find(_mirror_box_lines) == marker_list_map_.end()) {
            return;
        }

        RTRACE << name() << ": " << _mirror_box_lines << " begin!";

        auto &marker_list = marker_list_map_.at(_mirror_box_lines);

        const auto &obj_vec = msg_ptr->rs_lidar_result_ptr->objects;
        marker_list.resize(1);

        auto &box_marker = marker_list[0];
        box_marker.color_type = default_color_type;
        box_marker.scale_type.x = 0.05;
        box_marker.scale_type.y = 0.05;
        box_marker.scale_type.z = 0.05;
        box_marker.ns = _mirror_box_lines;
        box_marker.id = 0;
        box_marker.frame_id = base_options_.frame_id;
        box_marker.type = MarkerType::LINE_LIST;
        box_marker.action = ActionType::ADD;

        genOrientation(box_marker);

        int pts_size = static_cast<int>(box_marker.points.size());
        int obj_size = static_cast<int>(obj_vec.size());

        if (pts_size < obj_size * 24) {
            box_marker.points.resize(obj_size * 24);
            box_marker.colors.resize(obj_size * 24);
        }

        for (size_t i = 0; i < box_marker.points.size(); ++i) {
            box_marker.colors[i].a = 0.01;
        }

        for (size_t i = 0; i < obj_vec.size(); ++i) {
            const auto &tmp_obj = obj_vec[i];


            if (ObjectType::UNKNOW == tmp_obj->core_infos_.type ||
                ObjectType::PED == tmp_obj->core_infos_.type ||
                ObjectType::CONE == tmp_obj->core_infos_.type) {
                continue;
            }
            if(tmp_obj->any_map.find("mirror") == tmp_obj->any_map.end()) {
                continue;
            }

            ColorType color;
            if (DisplayMode::ORIGIN == base_options_.mode) {
                color = origin_color_map.at(tmp_obj->core_infos_.type);
            } else {
                color = efficient_color_map.at(tmp_obj->core_infos_.type);
            }
            bool is_attention = (tmp_obj->core_infos_.attention_type == AttentionType::ATTENTION);
            float tmp_alpha;
            if (is_attention) {
                tmp_alpha = 1.;
            } else {
                tmp_alpha = 0.7;
            }


            const auto &center = tmp_obj->core_infos_.center;
            const auto &size = tmp_obj->core_infos_.size;
            const auto &direction = tmp_obj->core_infos_.direction;
            RsBBox box(center, size, direction);

            int idx = i * 24;

            drawBoxLine(box, box_marker, idx, color, tmp_alpha);
        }

        RTRACE << name() << ": " << _mirror_box_lines << " end!";
    }
    BasePubOptions base_options_;
    std::map<std::string, std::vector<Marker> > marker_list_map_;
    std::vector<std::function<void(const RsPerceptionMsg::Ptr &)>> func_set_;

    virtual void genOrientation(Marker &marker, const float &direction = 0)  = 0;

    //=======================================
    //  draw components
    //=======================================
    inline void drawBoxLine(const RsBBox &tmp_box, Marker &box_marker, const int& idx,
                            const ColorType& box_color, const float& alpha) {
        std::vector<PositionType> cub_points;
        std::vector<RsVector3f> corners;
        tmp_box.corners(corners);

        for (int i = 0; i < 8; ++i) {
            PositionType pts;
            pts.x = corners[i].x;
            pts.y = corners[i].y;
            pts.z = corners[i].z;
            cub_points.push_back(pts);
        }

        for (int j = 0; j < 24; ++j) {
            box_marker.colors[idx + j].r = box_color.r;
            box_marker.colors[idx + j].g = box_color.g;
            box_marker.colors[idx + j].b = box_color.b;
            box_marker.colors[idx + j].a = alpha;
        }
        box_marker.points[idx + 0] = cub_points[0];
        box_marker.points[idx + 1] = cub_points[1];
        box_marker.points[idx + 2] = cub_points[1];
        box_marker.points[idx + 3] = cub_points[2];
        box_marker.points[idx + 4] = cub_points[2];
        box_marker.points[idx + 5] = cub_points[3];
        box_marker.points[idx + 6] = cub_points[3];
        box_marker.points[idx + 7] = cub_points[0];

        box_marker.points[idx + 8] = cub_points[4];
        box_marker.points[idx + 9] = cub_points[5];
        box_marker.points[idx + 10] = cub_points[5];
        box_marker.points[idx + 11] = cub_points[6];
        box_marker.points[idx + 12] = cub_points[6];
        box_marker.points[idx + 13] = cub_points[7];
        box_marker.points[idx + 14] = cub_points[7];
        box_marker.points[idx + 15] = cub_points[4];

        box_marker.points[idx + 16] = cub_points[0];
        box_marker.points[idx + 17] = cub_points[4];
        box_marker.points[idx + 18] = cub_points[1];
        box_marker.points[idx + 19] = cub_points[5];
        box_marker.points[idx + 20] = cub_points[2];
        box_marker.points[idx + 21] = cub_points[6];
        box_marker.points[idx + 22] = cub_points[3];
        box_marker.points[idx + 23] = cub_points[7];
        genOrientation(box_marker);
    }

    inline void drawText(const RsVector3f &pos, const std::string &info, Marker &marker, float alpha = 1.) {
        marker.type = MarkerType::TEXT_VIEW_FACING;
        marker.action = ActionType::ADD;
        marker.position.x = pos.x;
        marker.position.y = pos.y;
        marker.position.z = pos.z;
        marker.color_type.a = alpha;
        marker.text = info;
        genOrientation(marker);
    }

    inline void drawAccArrow(const Object::Ptr &obj, Marker &marker, float alpha = 1.) {
        marker.type = MarkerType::ARROW;
        marker.action = ActionType::ADD;

        const auto &center = obj->core_infos_.center;
        const auto &size = obj->core_infos_.size;
        const auto &direction = obj->core_infos_.direction;
        RsBBox box(center, size, direction);
        float box_size = box.volume();
        if (box_size > 0) {
            marker.color_type.a = alpha;

            const auto &acc = obj->core_infos_.acceleration;
            auto arrow_length = acc.norm();
            float main_direction = std::atan2(acc.y, acc.x);

            marker.scale_type.x = sqrtf(arrow_length + 1.1) - 1.0;
            if (obj->core_infos_.type == ObjectType::UNKNOW) {
                marker.scale_type.y = 0.1;
                marker.scale_type.z = 0.1;
            } else {
                marker.scale_type.y = 0.2;
                marker.scale_type.z = 0.2;
            }
            genOrientation(marker, main_direction);
            marker.position.x = obj->core_infos_.center.x;
            marker.position.y = obj->core_infos_.center.y;
            marker.position.z = obj->core_infos_.center.z;
        } else {
            marker.color_type.a = 0.01;
        }
    }

    inline void drawCube(const RsBBox &box, Marker &marker, float alpha = 1.) {
        marker.type = MarkerType::CUBE;
        marker.action = ActionType::ADD;

        marker.color_type.a = alpha;

        marker.position.x = box.center.x;
        marker.position.y = box.center.y;
        marker.position.z = box.center.z;

        marker.scale_type.x = box.size.x;
        marker.scale_type.y = box.size.y;
        marker.scale_type.z = box.size.z;
        genOrientation(marker, box.angle);
    }

    inline std::vector<RsVector3f> calRotationCorners(const RsBBox &box) {
        std::vector<RsVector3f> corners;
        float max_size = std::max(box.size.x, box.size.y) / 2.;

        corners.resize(16, RsVector3f(0, 0, 0));

        for (int i = 0; i < 8; ++i) {
            float angle = RS_M_PI / 4 * i;
            RsVector3f dir = RsVector3f(std::cos(angle), std::sin(angle), 0);
            corners[i] = box.center + dir * max_size;
            corners[i].z -= box.size.z / 2.;
            corners[i + 8] = box.center + dir * max_size;
            corners[i + 8].z += box.size.z / 2.;
        }
        return corners;
    }

    inline void drawCylinder(const Object::Ptr &obj, Marker &marker, float alpha = 1.) {
        marker.type = MarkerType::CYLINDER;
        marker.action = ActionType::ADD;

        const auto &center = obj->core_infos_.center;
        const auto &size = obj->core_infos_.size;
        const auto &direction = obj->core_infos_.direction;
        RsBBox box(center, size, direction);
        float box_size = box.volume();
        if (box_size > 0) {
            marker.color_type.a = alpha;

            marker.position.x = box.center.x;
            marker.position.y = box.center.y;
            marker.position.z = box.center.z;

            marker.scale_type.x = box.size.x;
            marker.scale_type.y = box.size.y;
            marker.scale_type.z = box.size.z;
            genOrientation(marker, box.angle);
        } else {
            marker.color_type.a = 0.01;
        }
    }

    inline void drawFreespace(const RsFreeSpace::Ptr &freespace_ptr, Marker &marker, float alpha = 1.) {
        size_t fs_size = freespace_ptr->fs_pts.size();
        if (static_cast<int>(fs_size) <= 1) {
            return;
        }

        marker.type = MarkerType::TRIANGLE_LIST;
        marker.action = ActionType::ADD;
        marker.color_type.a = alpha;

        marker.points.clear();
        marker.points.reserve(fs_size * 3);
        PositionType tri_pt0;
        tri_pt0.x = tri_pt0.y = tri_pt0.z = 0.;
        for (size_t i = 1; i < fs_size; ++i) {
            const auto &pre_pt = freespace_ptr->fs_pts[i - 1];
            const auto &cur_pt = freespace_ptr->fs_pts[i];
            // 当前扇区三角形
            PositionType tri_pt1, tri_pt2;
            tri_pt1.x = pre_pt.x;
            tri_pt1.y = pre_pt.y;
            tri_pt1.z = 0.;
            tri_pt2.x = cur_pt.x;
            tri_pt2.y = cur_pt.y;
            tri_pt2.z = 0.;

            marker.points.emplace_back(tri_pt0);
            marker.points.emplace_back(tri_pt1);
            marker.points.emplace_back(tri_pt2);
        }
        PositionType tri_pt1, tri_pt2;
        const auto &pre_pt = freespace_ptr->fs_pts[freespace_ptr->fs_pts.size() - 1];
        const auto &cur_pt = freespace_ptr->fs_pts[0];
        tri_pt1.x = pre_pt.x;
        tri_pt1.y = pre_pt.y;
        tri_pt1.z = 0.;
        tri_pt2.x = cur_pt.x;
        tri_pt2.y = cur_pt.y;
        tri_pt2.z = 0.;
        marker.points.emplace_back(tri_pt0);
        marker.points.emplace_back(tri_pt1);
        marker.points.emplace_back(tri_pt2);
        marker.points.resize(marker.points.size());
    }

    inline void drawLane(const std::vector<Lane::Ptr> &lanes, Marker &marker, float alpha) {
        marker.type = MarkerType::LINE_LIST;
        marker.action = ActionType::ADD;
        marker.scale_type.x = 0.2;
        marker.scale_type.y = 0.2;
        marker.scale_type.z = 0.2;

        int temp_num = 30;
        marker.points.clear();
        marker.colors.clear();
        marker.points.reserve(2 * temp_num * lanes.size());
        marker.colors.reserve(2 * temp_num * lanes.size());
        for (size_t i = 0; i < lanes.size(); ++i) {
            auto &obj = lanes[i];
            float min_x = obj->curve.x_start;
            float max_x = obj->curve.x_end;

            float delte = (max_x - min_x) / temp_num;
            for (int j = 0; j < temp_num; j++) {
                PositionType pt_start;
                PositionType pt_end;
                pt_start.x = min_x + j * delte;
                pt_start.y = obj->curve.b * pt_start.x * pt_start.x + obj->curve.c * pt_start.x + obj->curve.d;
                pt_start.z = 0.0f;

                pt_end.x = min_x + (j + 1) * delte;
                pt_end.y = obj->curve.b * pt_end.x * pt_end.x + obj->curve.c * pt_end.x + obj->curve.d;
                pt_end.z = 0.0f;

                marker.points.emplace_back(pt_start);
                marker.points.emplace_back(pt_end);
                ColorType color;
                color.r = 0.0;
                color.g = 1.0;
                color.b = 0.0;
                color.a = alpha;
                if (obj->lane_id == LanePosition::LEFT_EGO ||
                    obj->lane_id == LanePosition::RIGHT_EGO) {
                    color.r = 1.0f;
                    color.g = 1.0f;
                    color.b = 0.0f;
                }
                marker.colors.emplace_back(color);
                marker.colors.emplace_back(color);
                if (obj->measure_status == MeasureStatus::PREDICTION &&
                    DisplayMode::ORIGIN == base_options_.mode) {
                    j++;  // 预测的曲线画虚线
                }
            }
        }

        if (marker.points.empty()) {
            PositionType pt(0,0,0);
//            ColorType color(0,0,0,0.01);
            marker.points.emplace_back(pt);
            marker.points.emplace_back(pt);
//            marker.colors.emplace_back(color);
//            marker.colors.emplace_back(color);
            marker.color_type.a = 0.01;
        }
    }

    inline void drawRoadedge(const std::vector<Roadedge::Ptr> &roadedges, Marker &marker, float alpha = 1.) {
        marker.type = MarkerType::LINE_LIST;
        marker.action = ActionType::ADD;
        marker.color_type.a = alpha;
        marker.color_type.r = 1.0;
        marker.color_type.g = 0.0;
        marker.color_type.b = 0.0;
        marker.scale_type.x = 0.2;
        marker.scale_type.y = 0.2;
        marker.scale_type.z = 0.2;

        int temp_num = 30;
        marker.points.clear();
        marker.points.reserve(2 * temp_num * roadedges.size());
        for (size_t i = 0; i < roadedges.size(); ++i) {
            auto &obj = roadedges[i];
            float min_x = obj->curve.x_start;
            float max_x = obj->curve.x_end;

            float delte = (max_x - min_x) / temp_num;
            for (int j = 0; j < temp_num; j++) {
                PositionType pt_start;
                PositionType pt_end;
                pt_start.x = min_x + j * delte;
                pt_start.y = obj->curve.b * pt_start.x * pt_start.x + obj->curve.c * pt_start.x + obj->curve.d;
                pt_start.z = 0.0f;

                pt_end.x = min_x + (j + 1) * delte;
                pt_end.y = obj->curve.b * pt_end.x * pt_end.x + obj->curve.c * pt_end.x + obj->curve.d;
                pt_end.z = 0.0f;

                marker.points.emplace_back(pt_start);
                marker.points.emplace_back(pt_end);
                if (obj->measure_status == MeasureStatus::PREDICTION &&
                    DisplayMode::ORIGIN == base_options_.mode) {
                    j++;  // 预测的曲线画虚线
                }
            }
        }

        if (marker.points.empty()) {
            PositionType pt(0,0,0);
//            ColorType color(0,0,0,0.01);
            marker.points.emplace_back(pt);
            marker.points.emplace_back(pt);
//            marker.colors.emplace_back(color);
//            marker.colors.emplace_back(color);
            marker.color_type.a = 0.01;
            marker.color_type.a = 0.01;
        }
    }

    inline void drawVelArrow(const Object::Ptr &obj, Marker &marker, float alpha = 1.) {
        marker.type = MarkerType::LINE_LIST;
        marker.action = ActionType::ADD;
        marker.scale_type.x = 0.1;
        marker.scale_type.y = 0.1;
        marker.scale_type.z = 0.1;
        marker.points.clear();
        marker.colors.clear();

        auto vel = obj->core_infos_.velocity;
        if (vel.norm() < 0.1f) {
            return;  // 静止物体不绘制方向
        }

        // 非车辆
        std::set<ObjectType> object_without_wheels = {ObjectType::UNKNOW, ObjectType::CONE,
                                                      ObjectType::PED, ObjectType::BIC};

        const auto &center = obj->core_infos_.center;
        const auto &size = obj->core_infos_.size;
        auto direction = obj->core_infos_.direction;
        RsBBox box(center, size, direction);
        float box_size = box.volume();
        if (box_size > 0) {
            marker.color_type.a = alpha;

            // 如果障碍物不为自行车、车辆，则按照运动方向绘制
            if (object_without_wheels.find(obj->core_infos_.type) !=
                object_without_wheels.end()) {
                direction = vel;
                direction.normalize();
            }

            float length = size.x;
            if (direction.dot(vel) < 0) {
                direction.x = -direction.x;
                direction.y = -direction.y;
                direction.z = direction.z;
            }

            if (vel.norm() < 5.f) {
                length *= 0.5f;
            }

            PositionType pt_start;
            PositionType pt_end;
            pt_start.x = obj->core_infos_.center.x;
            pt_start.y = obj->core_infos_.center.y;
            pt_start.z = obj->core_infos_.center.z;

            pt_end.x = pt_start.x + direction.x * length;
            pt_end.y = pt_start.y + direction.y * length;
            pt_end.z = pt_start.z;

            marker.points.emplace_back(pt_start);
            marker.points.emplace_back(pt_end);
        } else {
            marker.color_type.a = 0.01;
            PositionType pt;
            pt.x = obj->core_infos_.center.x;
            pt.y = obj->core_infos_.center.y;
            pt.z = obj->core_infos_.center.z;
            marker.points.emplace_back(pt);
            marker.points.emplace_back(pt);
//            RINFO << "marker id = " << marker.id << " box size = " << box_size
//            << " size.x = " << box.size.x << " size.y = " << box.size.y << " size.z = " << box.size.z;
        }

//        if (marker.color_type.a < 0.01) {
//            RINFO << "------marker_id = " << marker.id << " color_type.a = " << marker.color_type.a;
//        }
    }
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_RVIZ_DISPLAY_EXTERNAL_COMMON_BASE_MARKER_PUB_H_
