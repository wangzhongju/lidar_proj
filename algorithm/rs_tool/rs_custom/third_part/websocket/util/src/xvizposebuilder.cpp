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
#include "xviz/xvizposebuilder.h"

namespace robosense
{
namespace perception
{

XVIZPoseBuilder::XVIZPoseBuilder(const std::string &category, const Json::Value &metadata, XVIZValidator::Ptr validator)
    : XVIZBaseBuilder(category, metadata, validator)
{
    reset();
}

XVIZPoseBuilder::~XVIZPoseBuilder()
{
    // TODO...
}

void XVIZPoseBuilder::mapOrigin(const double longitude, const double latitude, const double altitude)
{
    _map_origin[xvizMapOriginFieldName::longtitudeFieldName] = longitude;
    _map_origin[xvizMapOriginFieldName::latitudeFieldName] = latitude;
    _map_origin[xvizMapOriginFieldName::altitudeFieldName] = altitude;
}

void XVIZPoseBuilder::position(const double x, const double y, const double z)
{
    if (_position.size() == 0)
    {
        _position.append(x);
        _position.append(y);
        _position.append(z);
    }
    else
    {
        _position[0] = x;
        _position[1] = y;
        _position[2] = z;
    }
}

void XVIZPoseBuilder::orientation(const double roll, const double pitch, const double yaw)
{
    if (_orientation.size() == 0)
    {
        _orientation.append(roll);
        _orientation.append(pitch);
        _orientation.append(yaw);
    }
    else
    {
        _orientation[0] = roll;
        _orientation[1] = pitch;
        _orientation[2] = yaw;
    }
}

void XVIZPoseBuilder::timestamp(const double timestamp)
{
    _timestamp = timestamp;
    _is_timestamp = true;
}

double XVIZPoseBuilder::timestamp()
{
    return _timestamp;
}

Json::Value XVIZPoseBuilder::getData()
{
    if (!_streamId.empty() && _is_timestamp)
    {
        flush();
    }

    return _poses;
}

void XVIZPoseBuilder::flush()
{
    if (!_streamId.empty() && _is_timestamp)
    {
        Json::Value data;

        if (_is_timestamp)
        {
            data[xvizPoseFieldName::timestampFieldName] = _timestamp;
        }

        if (!_map_origin.isNull())
        {
            data[xvizPoseFieldName::mapOriginFieldName] = _map_origin;
        }

        if (!_position.isNull())
        {
            data[xvizPoseFieldName::positionFieldName] = _position;
        }

        if (!_orientation.isNull())
        {
            data[xvizPoseFieldName::orientationFieldName] = _orientation;
        }

        if (_is_timestamp && (!_map_origin.isNull() || !_position.isNull() || !_orientation.isNull()))
        {
            _poses[_streamId] = data;
        }
    }
}

void XVIZPoseBuilder::reset()
{
    XVIZBaseBuilder::reset();
    _is_timestamp = false;
}

} // namespace perception
} // namespace robosense
