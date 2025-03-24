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
#include "xviz/xviztimeseriesbuilder.h"
namespace robosense
{
namespace perception
{
XVIZTimeSeriesBuilder::XVIZTimeSeriesBuilder(const std::string &category, const Json::Value &metadata, XVIZValidator::Ptr validator)
    : XVIZBaseBuilder(category, metadata, validator)
{
    reset();
}

XVIZTimeSeriesBuilder::~XVIZTimeSeriesBuilder()
{
    // TODO...
}

void XVIZTimeSeriesBuilder::identity(const std::string &identifier)
{
    _identifier = identifier;
}

void XVIZTimeSeriesBuilder::values(const Json::Value &streams, const Json::Value &values, const std::string &valueTypeName)
{
    _streams = streams;
    _values = values;
    _valueTypeName = valueTypeName;
}

void XVIZTimeSeriesBuilder::timestamp(const double timestamp)
{
    _timestamp = timestamp;
    _is_timestamp = true;
}

void XVIZTimeSeriesBuilder::flush()
{
    if (!_streamId.empty() && _is_timestamp)
    {
        Json::Value data;
        if (!_identifier.empty())
        {
            data[xvizTimeSeriesFieldName::objectIdFieldName] = _identifier;
        }

        if (_streams.isNull() || _values.isNull() || _streams.size() != _values.size() || _valueTypeName.empty())
        {
            return;
        }

        data[xvizTimeSeriesFieldName::streamsFieldName] = _streams;

        Json::Value value;
        value[_valueTypeName] = _values;
        data[xvizTimeSeriesFieldName::valuesFieldName] = value;

        data[xvizTimeSeriesFieldName::timestampFieldName] = _timestamp;

        _timeSeries.append(data);

        resetData();
    }
}

Json::Value XVIZTimeSeriesBuilder::getData()
{
    if (!_streamId.empty() && _is_timestamp)
    {
        flush();
    }

    return _timeSeries;
}

void XVIZTimeSeriesBuilder::reset()
{
    XVIZBaseBuilder::reset();

    _streams = Json::Value();
    _values = Json::Value();
    _valueTypeName = "";
    _identifier = "";
    _is_timestamp = false;
}

void XVIZTimeSeriesBuilder::resetData()
{
    _streams = Json::Value();
    _values = Json::Value();
    _valueTypeName = "";
}
} // namespace perception
} // namespace robosense
