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
#ifndef XVIZPRIMITIVEBUILDER_H
#define XVIZPRIMITIVEBUILDER_H
//
// map<stream_id, PrimitiveState>
//
// PrimitiveState: list<polygon> / list<polyline> / list<text> / list<circle> / list<point> / list<stadium> / list<image>
//
//
// e.g.
// primitives: {
//      stream_id0: {AAAAA},
//      stream_id1: {BBBBB}...
//  }
//
//  "/lidar/points": {
//  "points": [{
//      "colors": "#/accessors/0",
//      "points": "#/accessors/1",
//      "base": {
//          "object_id": "b7785f99-b783-4d3f-b452-a1ef466fe8f8"
//      }
//  }]
//
// textAnchor: {
//                 enum: {Start = 1, Middle = 2, End = 3}
//             }
//
// TextAligmentBaseline: {
//                         enum: {Top= 1, Center = 2, Bottom = 3}
//                       }
//
// StyleObjectValue: {
//                       fill_color: bytes
//                       stroke_color: bytes
//                       stroke_width: float
//                       radius: float
//                       text_size: float
//                       text_rotation: float: in degree
//                       text_anchor: textAnchor
//                       text_baseline: TextAligmentBaseline
//                       height: float
//                   }
//
// PrimitiveBase: {
//                  object_id: string
//                  classes: list<string>
//                  style: StyleObjectValue, optional
//                }
//
// circle: {
//          base: PrimitiveBase
//          center: list<float>
//          radius: float
//         }
//
// Image: {
//          base: PrimitiveBase
//          position: list<float>
//          data: list<bytes>
//          width_px: uint32
//          height_px: uint32
//        }
// Point: {
//          base: PrimitiveBase
//          points: list<float>
//          colors: list<bytes> ==> list<<r, g, b>> / list<<r, g, b, a>>
//        }
// Polygon: {
//            base: PrimitiveBase
//            vertices: list<float> ==> list<<x, y, z>>
//          }
// Polygin: {
//             base: PrimitiveBase
//             vertices: list<float> ==> list<<x, y, z>>
//          }
// Stadium: {
//             base: PrimitiveBase
//             start: list<float>
//             end: list<float>
//             radius: float
//          }
// Text: {
//         base: PrimitiveBase
//         position: list<float, 3>
//         text: string
//       }
//

#include "xviz/xvizbasebuilder.h"

namespace robosense
{
namespace perception
{
class XVIZPrimitiveBuilder : public XVIZBaseBuilder
{
public:
    typedef std::shared_ptr<XVIZPrimitiveBuilder> Ptr;
    typedef std::shared_ptr<const XVIZPrimitiveBuilder> ConstPtr;

public:
    XVIZPrimitiveBuilder(const std::string &category,
                         const Json::Value &metadata,
                         XVIZValidator::Ptr validator);

    virtual ~XVIZPrimitiveBuilder();

public:
    void image(const std::string &data);

    void dimensions(int width, int height);

    void polygon(const Json::Value &vertices);

    void polyline(const Json::Value &vertices);

    void point(const std::string colors, const std::string points);

    void circle(const Json::Value &position, const double radius);

    void stadium(const Json::Value &position, const float radius);

    void text(const Json::Value &position, const std::string &message);

    void identity(const std::string object_id);

    void classes(const Json::Value &class_list);

    void style(const Json::Value &style);

    void flush();

    Json::Value getData();

    void reset();

private:
    // primitive element
    Json::Value _circle;
    Json::Value _image;
    Json::Value _polygon;
    Json::Value _polyline;
    Json::Value _point;
    Json::Value _stadium;
    Json::Value _text;

    // base
    std::string _identity;
    Json::Value _classes;
    Json::Value _style;

    // primitive
    Json::Value _primitive;

    std::string _type;
};
} // namespace perception
} // namespace robosense
#endif // XVIZPRIMITIVEBUILDER_H
