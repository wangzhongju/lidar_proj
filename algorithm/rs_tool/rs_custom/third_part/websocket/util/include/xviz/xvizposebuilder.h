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
#ifndef XVIZPOSEBUILDER_H
#define XVIZPOSEBUILDER_H
//
// map<stream_id, Pose>
//
//  Pose = {timestamp,
//          MapOrigin,
//          poisiton,
//          orientation}
// ===> MapOrigin = { longtitude: double
//                    latitude: double
//                    altitude: double
//                  }
// ===> position: list<double, 3>
// ===> orientation: list<double, 3>
//
// e.g.
// poses": {
// "/vehicle_pose": {
//     "timestamp": 1317042272.349,
//     "mapOrigin": {
//         "longitude": 8.4228850417969,
//         "latitude": 49.011212804408,
//         "altitude": 112.83492279053
//     },
//     "position": [0, 0, 0],
//     "orientation": [0.022447, 0.00001, -1.2219096732051]
// }
// }
//

#include "xviz/xvizbasebuilder.h"

namespace robosense
{
namespace perception
{
class XVIZPoseBuilder : public XVIZBaseBuilder
{
public:
    typedef std::shared_ptr<XVIZPoseBuilder> Ptr;
    typedef std::shared_ptr<const XVIZPoseBuilder> ConstPtr;

public:
    XVIZPoseBuilder(const std::string &category, const Json::Value &metadata, XVIZValidator::Ptr validator);

    virtual ~XVIZPoseBuilder();

public:
    void mapOrigin(const double longitude, const double latitude, const double altitude);

    void position(const double x, const double y, const double z);

    void orientation(const double roll, const double pitch, const double yaw);

    void timestamp(const double timestamp);

    double timestamp();

    Json::Value getData();

    void flush();

    void reset();

public:
    Json::Value _map_origin;
    Json::Value _position;
    Json::Value _orientation;
    double _timestamp;
    bool _is_timestamp;

    Json::Value _poses;
};

} // namespace perception
} // namespace robosense

#endif // XVIZPOSEBUILDER_H
