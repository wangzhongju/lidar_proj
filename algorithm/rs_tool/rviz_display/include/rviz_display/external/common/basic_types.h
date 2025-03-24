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

#ifndef RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_COMMON_BASIC_TYPES_H_
#define RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_COMMON_BASIC_TYPES_H_

#include "rs_perception/common/external/common.h"

namespace robosense {
namespace perception {

struct ColorType {
    explicit ColorType(const float &r_ = 0., const float &g_ = 0., const float &b_ = 0., const float &a_ = 1.) {
        r = r_;
        g = g_;
        b = b_;
        a = a_;
    }

    float r, g, b, a;
};

struct ScaleType {
    explicit ScaleType(const float &x_ = 1., const float &y_ = 1., const float &z_ = 1.) {
        x = x_;
        y = y_;
        z = z_;
    }

    float x, y, z;
};

struct PositionType {
    explicit PositionType(const float &x_ = 0., const float &y_ = 0., const float &z_ = 0.) {
        x = x_;
        y = y_;
        z = z_;
    }

    float x, y, z;
};

struct OrientationType {
    explicit OrientationType(const float &x_ = 0., const float &y_ = 0., const float &z_ = 0., const float &w_ = 1.) {
        x = x_;
        y = y_;
        z = z_;
        w = w_;
    }

    float x, y, z, w;
};

enum class MarkerType {
    ARROW = 0u,
    CUBE = 1u,
    SPHERE = 2u,
    CYLINDER = 3u,
    LINE_STRIP = 4u,
    LINE_LIST = 5u,
    CUBE_LIST = 6u,
    SPHERE_LIST = 7u,
    POINTS = 8u,
    TEXT_VIEW_FACING = 9u,
    MESH_RESOURCE = 10u,
    TRIANGLE_LIST = 11u,
};

enum class ActionType {
    ADD = 0u,
    MODIFY = 1u,
    DELETE = 2u,
    DELETEALL = 3u,
};

struct Marker {
    ColorType color_type;
    ScaleType scale_type;
    std::string ns = "";
    int id = 0;
    std::string frame_id = "";
    MarkerType type;
    ActionType action;
    PositionType position;
    OrientationType orientation;
    std::string text = "";
    std::vector<ColorType> colors;
    std::vector<PositionType> points;
};

enum class Color {
    Lavender = 0,  // 淡紫色
    Yellow = 1,  // 黄色
    CyanBlue = 2,  // 青色
    Red = 3,  // 红色
    Blue = 4,  // 蓝色
    Green = 5,  // 绿色
    DEFAULT = 6,  // 默认
};

inline ColorType genColor(const Color &c) {
    if (Color::Lavender == c) {
        return ColorType(0.6, 0.5, 0.6);
    } else if (Color::Yellow == c) {
        return ColorType(1., 1., 0.);
    } else if (Color::CyanBlue == c) {
        return ColorType(0., 1., 1.);
    } else if (Color::Red == c) {
        return ColorType(0.7, 0., 0.);
    } else if (Color::Blue == c) {
        return ColorType(0., 0., 1.);
    } else if (Color::Green == c) {
        return ColorType(0.4, 0.74, 0.18);
    } else {
        return ColorType(0., 0., 0., 0.);
    }
}

enum class DisplayMode {
    ORIGIN = 0,
    EFFICIENT = 1,
};

// DisplayMode and mapping
const std::map<DisplayMode, std::string> kDisplayMode2NameMap = {
{DisplayMode::ORIGIN,    "ORIGIN"},
{DisplayMode::EFFICIENT, "EFFICIENT"},
};

const std::map<std::string, DisplayMode> kDisplayMode2TypeMap = {
{"ORIGIN",    DisplayMode::ORIGIN},
{"EFFICIENT", DisplayMode::EFFICIENT},
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_COMMON_BASIC_TYPES_H_
