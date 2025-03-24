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
#include "xviz/xvizmetabuilder.h"

namespace robosense
{
namespace perception
{

xvizMetaBuilder::xvizMetaBuilder()
{
    _metadata["version"] = "2.0.0";
    _metadata["streams"] = Json::Value(Json::ValueType::objectValue);
}

xvizMetaBuilder::~xvizMetaBuilder()
{
    // TODO...
}

void xvizMetaBuilder::stream(const std::string stream_id)
{
    if (!_streamId.empty())
    {
        flush();
    }

    _streamId = stream_id;
}

void xvizMetaBuilder::loginfo(const Json::Value &log_info)
{
    _log_info = log_info;
}

void xvizMetaBuilder::uiConfig(const Json::Value &ui_config)
{
    _ui_config = ui_config;
}

void xvizMetaBuilder::category(const std::string &cate)
{
    _streams["category"] = cate;
    _category = cate;
}

void xvizMetaBuilder::type(const std::string &t)
{
    _type = t;
}

void xvizMetaBuilder::source(const std::string &src)
{
    _streams["source"] = src;
}

void xvizMetaBuilder::unit(const std::string &u)
{
    _streams["units"] = u;
}

void xvizMetaBuilder::coordinate(const std::string &coor)
{
    _streams["coordinate"] = coor;
}

void xvizMetaBuilder::transformMatrix(const Json::Value &matrix)
{
    _matrix_transform = matrix;
}

void xvizMetaBuilder::pose(const Json::Value &position, const Json::Value &orientation)
{
#if 0 
    Eigen::Translation3d translate(position[0].asDouble(),
                                   position[1].asDouble(),
                                   position[2].asDouble());

    Eigen::AngleAxisd rotation_x(orientation[0].asDouble() / 180.0 * M_PI, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotation_y(orientation[1].asDouble() / 180.0 * M_PI, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotation_z(orientation[2].asDouble() / 180.0 * M_PI, Eigen::Vector3d::UnitZ());

    Eigen::Matrix4d transform = (translate * rotation_x * rotation_y * rotation_z).matrix();

    _pose_transform = Json::Value();
    for (int i = 0; i < 16; ++i)
    {
        _pose_transform.append(transform(i));
    }
#else 
    std::vector<float> transform(16, 0); 
    transform[0] = transform[5] = transform[10] = transform[15] = 1; 

    _pose_transform = Json::Value();
    for (int i = 0; i < 16; ++i)
    {
        _pose_transform.append(transform[i]);
    }
#endif 
}

void xvizMetaBuilder::streamStyle(const Json::Value &style)
{
    _streams["stream_style"] = style;
}

void xvizMetaBuilder::classStyle(const std::string &name, const Json::Value &style)
{
    Json::Value obj;

    obj["name"] = name;
    obj["style"] = style;

    Json::Value styles;
    if (_streams.isMember("style_classes"))
    {
        styles = _streams["style_classes"];
        styles.append(obj);

        _streams["style_classes"] = styles;
    }
    else
    {
        styles.append(obj);
        _streams["style_classes"] = styles;
    }
}

void xvizMetaBuilder::flush()
{
    if (!_streamId.empty())
    {
        Json::Value streamData = _streams;
        if (!_matrix_transform.isNull() && !_pose_transform.isNull())
        { //
            return;
        }

        else if (!_matrix_transform.isNull())
        {
            streamData["transform"] = _matrix_transform;
            _matrix_transform = Json::Value();
        }
        else if (!_pose_transform.isNull())
        {
            streamData["transform"] = _pose_transform;
            _pose_transform = Json::Value();
        }

        std::string lowerPrimitive = xvizCategory::primitive;
        std::string lowerFutureInstance = xvizCategory::future_instance;
        std::string lowerVariable = xvizCategory::variable;
        std::string lowerTimeseries = xvizCategory::time_series;
        std::transform(lowerPrimitive.begin(), lowerPrimitive.end(), lowerPrimitive.begin(), ::tolower);
        std::transform(lowerFutureInstance.begin(), lowerFutureInstance.end(), lowerFutureInstance.begin(), ::tolower);
        std::transform(lowerVariable.begin(), lowerVariable.end(), lowerVariable.begin(), ::tolower);
        std::transform(lowerTimeseries.begin(), lowerTimeseries.end(), lowerTimeseries.begin(), ::tolower);

        if (_category == lowerPrimitive || _category == lowerFutureInstance)
        {
            streamData["primitive_type"] = _type;
        }
        else if (_category == lowerVariable || _category == lowerTimeseries)
        {
            streamData["scalar_type"] = _type;
        }
        Json::Value metaDataStreams = _metadata["streams"];

        metaDataStreams[_streamId] = streamData;

        _metadata["streams"] = metaDataStreams;
        _streams = Json::Value();
        _streamId.clear();
    }
}

Json::Value xvizMetaBuilder::getMetadata()
{
    if (!_streamId.empty())
    {
        flush();
    }

    _metadata["log_info"] = _log_info;
    _metadata["ui_config"] = _ui_config;

    return _metadata;
}

} // namespace perception
} // namespace robosense
