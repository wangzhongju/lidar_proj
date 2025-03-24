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









// Copyright (c) 2019 Uber Technologies, Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#ifndef XVIZMETABUILDER_H
#define XVIZMETABUILDER_H
//
// Metadata: {
//              version: string
//              streams: map<string, StreamMetadata>
//              cameras: map<string, CameraInfo>
//              stream_aliases: map<string, string>
//              ui_config: map<string, UIPanelInfo>
//              log_info: LogInfo
//           }
//
// StreamMetadata {
//                  source: string
//                  units: string
//                  scaler_type: "float"/"int32"/"string"/"bool"
//                  primitive_type: "circle"/"image"/"point"/"polygon"/"polygin"/"stadium"/"text"
//                  ui_primitive_type: "treetable"
//                  annotation_type: "visual"
//                  stream_style: StreamStyleValue
//                  style_class: list<StyleClass>
//                  coordinate: "geographic"/"identity"/"dynamic"/"vehicle_relative"
//                  tranform: list<float, 16>
//                  transform_callback: string
//                }
//
// CameraInfo {
//              human_name: string
//              source: string
//              vehicle_position: list<float, 3>
//              vehicle_orientation: list<float, 9>
//              pixel_width: double
//              pixel_height: double
//              retification_projection: list<float, 9>
//              gl_projection: list<float, 16>
//            }
//
// UIPanelInfo {
//                name: string
//                need_streams: list<string>
//                config: google.protobuf.Struct
//             }
//
// LogInfo {
//           double: start_time
//           double: end_time
//         }
//
// StreamStyleValue {
//                     fill_color: bytes => (r, g, b) / (r, g, b, a)
//                     stroke_color: bytes => (r, g, b) / (r, g, b, a)
//                     stroke_width: float
//                     radius: float
//                     text_size: float
//                     text_rotation: float
//                     text_anchor: TextAnchor
//                     text_baseline: TextAligmentBaseline
//                     height: float
//                     radius_min_pixels: uint32
//                     radius_max_pixels: uint32
//                     stroke_width_min_pixels: uint32
//                     stroke_width_max_pixels: uint32
//                     opacity: float
//                     stroked: bool
//                     filled: bool
//                     extruded: bool
//                     radius_pixels: uint32
//                     font_weight: uint32
//                     font_family: string
//                     point_color_mode: PointCloudMode
//                     point_color_domain: list<float>
//                  }
//
// StyleClass {
//               name: string
//               style: StyleObjectValue
//            }
//
// StyleObjectValue:{
//                    fill_color: bytes => (r, g, b) / (r, g, b, a)
//                    stroke_color: btyes => (r, g, b) / (r, g, b, a)
//                    stroke_width: float
//                    radius: float
//                    text_size: float
//                    text_rotation: float: angle in degree
//                    text_anchor: TextAnchor
//                    text_baseline: TextAlignmentBaseline
//                    height: float
//                  }
//
// PointCloudMode: {
//                   "ELEVATION" / "DISTANCE_TO_VEHICLE" / "DEFAULT"
//                 }
//
// TextAnchor: {
//               START / MIDDLE / END
//             }
//
// TextAlignmentBaseline: {
//                           TOP / CENTER / BOTTOM
//                        }
//
// #include <Eigen/Core>
// #include <Eigen/Geometry>
#include "xviz/uicommon.h"
#include "xviz/xvizcommon.h"

namespace robosense
{
namespace perception
{
class xvizMetaBuilder
{
public:
    typedef std::shared_ptr<xvizMetaBuilder> Ptr;
    typedef std::shared_ptr<const xvizMetaBuilder> ConstPtr;

public:
    xvizMetaBuilder();

    ~xvizMetaBuilder();

public:
    void stream(const std::string stream_id);

    void loginfo(const Json::Value &log_info);

    void uiConfig(const Json::Value &ui_config);

    void category(const std::string &cate);

    void type(const std::string &t);

    void source(const std::string &src);

    void unit(const std::string &u);

    void coordinate(const std::string &coor);

    void transformMatrix(const Json::Value &matrix);

    void pose(const Json::Value &position, const Json::Value &orientation);

    void streamStyle(const Json::Value &style);

    void classStyle(const std::string &name, const Json::Value &style);

    void flush();

    Json::Value getMetadata();

private:
    std::string _streamId;
    std::string _category;
    std::string _type;
    Json::Value _log_info;
    Json::Value _ui_config;
    Json::Value _metadata;
    Json::Value _streams;
    Json::Value _matrix_transform;
    Json::Value _pose_transform;
};

} // namespace perception
} // namespace robosense

#endif // XVIZMETABUILDER_H
