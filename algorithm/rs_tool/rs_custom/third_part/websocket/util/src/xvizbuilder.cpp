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
#include "xviz/xvizbuilder.h"
namespace robosense
{
namespace perception
{
xvizBuilder::xvizBuilder(const Json::Value &metadata, XVIZValidator::Ptr validator)
{
    _metadata = metadata;
    _validator = validator;
    _updateType = "snapshot";
}

xvizBuilder::~xvizBuilder()
{
    // TODO...
}

void xvizBuilder::persistent()
{
    _updateType = "PERSISTENT";
}

xvizFutureInstance::Ptr xvizBuilder::futureInstance(const std::string &id)
{
    xvizFutureInstance::Ptr builder;

    std::map<std::string, XVIZBaseBuilder::Ptr>::iterator iterMap = _mapBuilder.find(xvizCategory::future_instance);
    if (iterMap == _mapBuilder.end())
    {
        builder.reset(new xvizFutureInstance(xvizCategory::future_instance, _metadata, _validator));
        _mapBuilder.insert(std::pair<std::string, XVIZBaseBuilder::Ptr>(xvizCategory::future_instance, builder));
    }
    else
    {
        builder = dyn_cast<xvizFutureInstance>(iterMap->second);
    }

    builder->streamId(id);

    return builder;
}

XVIZLinkBuilder::Ptr xvizBuilder::link(const std::string &id)
{
    XVIZLinkBuilder::Ptr builder;
    std::map<std::string, XVIZBaseBuilder::Ptr>::iterator iterMap = _mapBuilder.find(xvizCategory::link);
    if (iterMap == _mapBuilder.end())
    {
        builder.reset(new XVIZLinkBuilder(xvizCategory::link, _metadata, _validator));
        _mapBuilder.insert(std::pair<std::string, XVIZBaseBuilder::Ptr>(xvizCategory::link, builder));
    }
    else
    {
        builder = dyn_cast<XVIZLinkBuilder>(iterMap->second);
    }

    builder->streamId(id);

    return builder;
}

XVIZPoseBuilder::Ptr xvizBuilder::pose(const std::string &id)
{
    XVIZPoseBuilder::Ptr builder;

    std::map<std::string, XVIZBaseBuilder::Ptr>::iterator iterMap = _mapBuilder.find(xvizCategory::pose);
    if (iterMap == _mapBuilder.end())
    {
        builder.reset(new XVIZPoseBuilder(xvizCategory::pose, _metadata, _validator));
        _mapBuilder.insert(std::pair<std::string, XVIZBaseBuilder::Ptr>(xvizCategory::pose, builder));
    }
    else
    {
        builder = dyn_cast<XVIZPoseBuilder>(iterMap->second);
    }

    builder->streamId(id);

    return builder;
}

XVIZVariableBuilder::Ptr xvizBuilder::variable(const std::string &id)
{
    XVIZVariableBuilder::Ptr builder;

    std::map<std::string, XVIZBaseBuilder::Ptr>::iterator iterMap = _mapBuilder.find(xvizCategory::variable);
    if (iterMap == _mapBuilder.end())
    {
        builder.reset(new XVIZVariableBuilder(xvizCategory::variable, _metadata, _validator));
        _mapBuilder.insert(std::pair<std::string, XVIZBaseBuilder::Ptr>(xvizCategory::variable, builder));
    }
    else
    {
        builder = dyn_cast<XVIZVariableBuilder>(iterMap->second);
    }

    builder->streamId(id);

    return builder;
}

XVIZPrimitiveBuilder::Ptr xvizBuilder::primitive(const std::string &id)
{
    XVIZPrimitiveBuilder::Ptr builder;

    std::map<std::string, XVIZBaseBuilder::Ptr>::iterator iterMap = _mapBuilder.find(xvizCategory::primitive);
    if (iterMap == _mapBuilder.end())
    {
        builder.reset(new XVIZPrimitiveBuilder(xvizCategory::primitive, _metadata, _validator));
        _mapBuilder.insert(std::pair<std::string, XVIZBaseBuilder::Ptr>(xvizCategory::primitive, builder));
    }
    else
    {
        builder = dyn_cast<XVIZPrimitiveBuilder>(iterMap->second);
    }

    builder->streamId(id);

    return builder;
}

xvizUiPrimitiveBuilder::Ptr xvizBuilder::uiprimitive(const std::string &id)
{
    xvizUiPrimitiveBuilder::Ptr builder;

    std::map<std::string, XVIZBaseBuilder::Ptr>::iterator iterMap = _mapBuilder.find(xvizCategory::ui_primitive);
    if (iterMap == _mapBuilder.end())
    {
        builder.reset(new xvizUiPrimitiveBuilder(xvizCategory::ui_primitive, _metadata, _validator));
        _mapBuilder.insert(std::pair<std::string, XVIZBaseBuilder::Ptr>(xvizCategory::ui_primitive, builder));
    }
    else
    {
        builder = dyn_cast<xvizUiPrimitiveBuilder>(iterMap->second);
    }

    builder->streamId(id);

    return builder;
}

XVIZTimeSeriesBuilder::Ptr xvizBuilder::timeSeries(const std::string &id)
{
    XVIZTimeSeriesBuilder::Ptr builder;

    std::map<std::string, XVIZBaseBuilder::Ptr>::iterator iterMap = _mapBuilder.find(xvizCategory::time_series);
    if (iterMap == _mapBuilder.end())
    {
        builder.reset(new XVIZTimeSeriesBuilder(xvizCategory::time_series, _metadata, _validator));
        _mapBuilder.insert(std::pair<std::string, XVIZBaseBuilder::Ptr>(xvizCategory::time_series, builder));
    }
    else
    {
        builder = dyn_cast<XVIZTimeSeriesBuilder>(iterMap->second);
    }

    builder->streamId(id);

    return builder;
}

// Make
Json::Value xvizBuilder::getMessage()
{
    _xvizMessage = Json::Value();

    _xvizMessage["update_type"] = _updateType;

    Json::Value _xvizData;
    for (std::map<std::string, XVIZBaseBuilder::Ptr>::iterator iter = _mapBuilder.begin();
         iter != _mapBuilder.end(); ++iter)
    {
        const std::string &category = iter->first;
        Json::Value xvizData = iter->second->getData();

        if (category == xvizCategory::variable)
        {
            _xvizData[xvizCategoryFieldName::variableFieldName] = xvizData;
        }
        else if (category == xvizCategory::time_series)
        {
            _xvizData[xvizCategoryFieldName::time_seriesFieldName] = xvizData;
        }
        else if (category == xvizCategory::pose)
        {
            // timestamp
            XVIZPoseBuilder::Ptr xvizPosePtr = dyn_cast<XVIZPoseBuilder>(iter->second);
            _xvizData[xvizPoseFieldName::timestampFieldName] = xvizPosePtr->timestamp();
            _xvizData[xvizCategoryFieldName::poseFieldName] = xvizData;
        }
        else if (category == xvizCategory::link)
        {
            _xvizData[xvizCategoryFieldName::linkFieldName] = xvizData;
        }
        else if (category == xvizCategory::primitive)
        {
            _xvizData[xvizCategoryFieldName::primitiveFieldName] = xvizData;
        }
        else if (category == xvizCategory::ui_primitive)
        {
            _xvizData[xvizCategoryFieldName::ui_primitiveFieldName] = xvizData;
        }
        else if (category == xvizCategory::future_instance)
        {
            _xvizData[xvizCategoryFieldName::future_instanceFieldName] = xvizData;
        }
    }

    Json::Value updates;
    updates.append(_xvizData);

    _xvizMessage["updates"] = updates;

    return _xvizMessage;
}
} // namespace perception
} // namespace robosense
