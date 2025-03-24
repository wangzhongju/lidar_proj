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
#include "xviz/xvizfutureinstance.h"

namespace robosense
{
namespace perception
{
xvizFutureInstance::xvizFutureInstance(const std::string &category, const Json::Value &metadata, const XVIZValidator::Ptr validator)
    : XVIZBaseBuilder(category, metadata, validator)
{
    reset();
}

xvizFutureInstance::~xvizFutureInstance()
{
    // TODO...
}

void xvizFutureInstance::timestamps(const Json::Value &timestamps)
{
    _timestamps = timestamps;
}

void xvizFutureInstance::image(const std::string &data)
{
    _image[xvizPrimitiveImageFieldName::dataFieldName] = data;
    _type = xvizPrimitiveType::image;
}

void xvizFutureInstance::dimensions(int width, int height)
{
    _image[xvizPrimitiveImageFieldName::widthPxFieldName] = width;
    _image[xvizPrimitiveImageFieldName::heightPxFieldName] = height;
    _type = xvizPrimitiveType::image;
}

void xvizFutureInstance::polygon(const Json::Value &vertices)
{
    _polygon[xvizPrimitivePolygonFieldName::verticesFieldName] = vertices;
    _type = xvizPrimitiveType::polygon;
}

void xvizFutureInstance::polyline(const Json::Value &vertices)
{
    _polyline[xvizPrimitivePolyginFieldName::verticesFieldName] = vertices;
    _type = xvizPrimitiveType::polyline;
}

void xvizFutureInstance::point(const std::string &points, const std::string &colors)
{
    _point[xvizPrimitivePointFieldName::pointsFieldName] = points;
    _point[xvizPrimitivePointFieldName::colorsFieldName] = colors;
    _type = xvizPrimitiveType::point;
}

void xvizFutureInstance::point(const Json::Value &points, const Json::Value &colors)
{
    _point[xvizPrimitivePointFieldName::pointsFieldName] = points;
    _point[xvizPrimitivePointFieldName::colorsFieldName] = colors;

    _type = xvizPrimitiveType::point;
}

void xvizFutureInstance::circle(const Json::Value &position, const double radius)
{
    _circle[xvizPrimitiveCircleFieldName::centerFieldName] = position;
    _circle[xvizPrimitiveCircleFieldName::radiusFieldName] = radius;
    _type = xvizPrimitiveType::circle;
}

void xvizFutureInstance::stadium(const Json::Value &position, const float radius)
{
    _stadium[xvizPrimitiveStadiumFieldName::startFieldName] = position[0];
    _stadium[xvizPrimitiveStadiumFieldName::endFieldName] = position[1];
    _stadium[xvizPrimitiveStadiumFieldName::radiusFieldName] = radius;

    _type = xvizPrimitiveType::stadium;
}

void xvizFutureInstance::text(const Json::Value &position, const std::string &message)
{
    _text[xvizPrimitiveTextFieldName::positionFieldName] = position;
    _text[xvizPrimitiveTextFieldName::textFieldName] = message;

    _type = xvizPrimitiveType::text;
}

// base
void xvizFutureInstance::identity(const std::string object_id)
{
    _identity = object_id;
}

void xvizFutureInstance::classes(const Json::Value &class_list)
{
    _classes = class_list;
}

void xvizFutureInstance::style(const Json::Value &style)
{
    _style = style;
}

// Flush Single Primitive Item
void xvizFutureInstance::singleFlush()
{
    if (!_type.empty() && !_streamId.empty())
    {
        // base
        Json::Value base;
        bool isHasBase = !(_identity.empty());

        if (isHasBase)
        {
            base[xvizPrimitiveBaseFieldName::objectIdFieldName] = _identity;
            _identity = "";
        }

        if (!_classes.empty())
        {
            base[xvizPrimitiveBaseFieldName::classesFieldName] = _classes;
            _classes = Json::Value();
        }

        if (!_style.empty() && isHasBase)
        {
            base[xvizPrimitiveBaseFieldName::styleFieldName] = _style;
            _style = Json::Value();
        }

        if (_type == xvizPrimitiveType::circle)
        {
            if (isHasBase)
            {
                _circle[xvizPrimitiveCircleFieldName::baseFieldName] = base;
            }

            _circles.append(_circle);
        }
        else if (_type == xvizPrimitiveType::image)
        {
            if (isHasBase)
            {
                _image[xvizPrimitiveImageFieldName::baseFieldName] = base;
            }
            _images.append(_image);
        }
        else if (_type == xvizPrimitiveType::polygon)
        {
            if (isHasBase)
            {
                _polygon[xvizPrimitivePolygonFieldName::baseFieldName] = base;
            }
            _polygons.append(_polygon);
        }
        else if (_type == xvizPrimitiveType::polyline)
        {
            if (isHasBase)
            {
                _polyline[xvizPrimitivePolyginFieldName::baseFieldName] = base;
            }
            _polylines.append(_polyline);
        }
        else if (_type == xvizPrimitiveType::point)
        {
            if (isHasBase)
            {
                _point[xvizPrimitivePointFieldName::baseFieldName] = base;
            }
            _points.append(_point);
        }
        else if (_type == xvizPrimitiveType::stadium)
        {
            if (isHasBase)
            {
                _stadium[xvizPrimitiveStadiumFieldName::baseFieldName] = base;
            }
            _stadiums.append(_stadium);
        }
        else if (_type == xvizPrimitiveType::text)
        {
            if (isHasBase)
            {
                _text[xvizPrimitiveTextFieldName::baseFieldName] = base;
            }
            _texts.append(_text);
        }

        _type = std::string();
    }
}

// Flush Single Timestamp Json
void xvizFutureInstance::flushTimestamp()
{
    if (!_streamId.empty())
    {
        Json::Value items;
        if (!_circles.isNull())
        {
            items[xvizPrimitiveFieldName::circleFieldName] = _circles;
            _circles = Json::Value();
        }

        if (!_images.isNull())
        {
            items[xvizPrimitiveFieldName::imageFieldName] = _images;
            _images = Json::Value();
        }

        if (!_polygons.isNull())
        {
            items[xvizPrimitiveFieldName::polygonFieldName] = _polygons;
            _polygons = Json::Value();
        }

        if (!_polylines.isNull())
        {
            items[xvizPrimitiveFieldName::polylineFieldName] = _polylines;
            _polylines = Json::Value();
        }

        if (!_points.isNull())
        {
            items[xvizPrimitiveFieldName::pointFieldName] = _points;
            _points = Json::Value();
        }

        if (!_stadiums.isNull())
        {
            items[xvizPrimitiveFieldName::stadiumFieldName] = _stadiums;
            _stadiums = Json::Value();
        }

        if (!_texts.isNull())
        {
            items[xvizPrimitiveFieldName::textFieldName] = _texts;
            _texts = Json::Value();
        }

        if (!items.isNull())
        {
            _primitive.append(items);
        }
    }
}

// Flush Total Stream
void xvizFutureInstance::flush()
{
    if (!_streamId.empty())
    {
        Json::Value instances;

        instances[xvizFutureInstanceFieldName::timestampsFieldName] = _timestamps;
        instances[xvizCategoryFieldName::primitiveFieldName] = _primitive;

        _future_instances[_streamId] = instances;

        _streamId = std::string();
        _primitive = Json::Value();
    }
}

Json::Value xvizFutureInstance::getData()
{
    if (!_streamId.empty())
    {
        flush();
    }

    return _future_instances;
}

void xvizFutureInstance::reset()
{
    XVIZBaseBuilder::reset();

    _future_instances = Json::Value();
    _timestamps = Json::Value();
    _type = "";
}

} // namespace perception
} // namespace robosense
