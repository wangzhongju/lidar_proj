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
#ifndef XVIZFUTUREINSTANCE_H
#define XVIZFUTUREINSTANCE_H
//
// map<stream_id, FutureInstance>
//    FutureInstance: {
//                      timestamps: list<double>
//                      primitives: list<PrimitiveState>
//                     }
//
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
// Not Used Now
//

#include "xviz/xvizbasebuilder.h"

namespace robosense
{
namespace perception
{
class xvizFutureInstance : public XVIZBaseBuilder
{
public:
    typedef std::shared_ptr<xvizFutureInstance> Ptr;
    typedef std::shared_ptr<const xvizFutureInstance> ConstPtr;

public:
    xvizFutureInstance(const std::string &category, const Json::Value &metadata, const XVIZValidator::Ptr validator);

    virtual ~xvizFutureInstance();

public:
    void timestamps(const Json::Value &timestamps);

    void image(const std::string &data);

    void dimensions(int width, int height);

    void polygon(const Json::Value &vertices);

    void polyline(const Json::Value &vertices);

    void point(const std::string &points, const std::string &colors);

    void point(const Json::Value &points, const Json::Value &colors);

    void circle(const Json::Value &position, const double radius);

    void stadium(const Json::Value &position, const float radius);

    void text(const Json::Value &position, const std::string &message);

    // base
    void identity(const std::string object_id);

    void classes(const Json::Value &class_list);

    void style(const Json::Value &style);

    // Flush Single Primitive Item
    void singleFlush();

    // Flush Single Timestamp Json
    void flushTimestamp();

    // Flush Total Stream
    void flush();

    Json::Value getData();

    void reset();

private:
    Json::Value _timestamps;

    Json::Value _circle;
    Json::Value _circles;
    Json::Value _image;
    Json::Value _images;
    Json::Value _polygon;
    Json::Value _polygons;
    Json::Value _polyline;
    Json::Value _polylines;
    Json::Value _point;
    Json::Value _points;
    Json::Value _stadium;
    Json::Value _stadiums;
    Json::Value _text;
    Json::Value _texts;

    std::string _identity;
    Json::Value _classes;
    Json::Value _style;

    Json::Value _primitive;

    std::string _type;

    Json::Value _future_instances;
};
} // namespace perception
} // namespace robosense

#endif // XVIZFUTUREINSTANCE_H
