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
#include "xviz/xvizprimitivebuilder.h"
namespace robosense
{
namespace perception
{
XVIZPrimitiveBuilder::XVIZPrimitiveBuilder(const std::string &category,
                                           const Json::Value &metadata,
                                           XVIZValidator::Ptr validator) : XVIZBaseBuilder(category, metadata, validator)
{
    reset();
}

XVIZPrimitiveBuilder::~XVIZPrimitiveBuilder()
{
    // TODO..
}

void XVIZPrimitiveBuilder::image(const std::string &data)
{
    _image[xvizPrimitiveImageFieldName::dataFieldName] = data;
    _type = xvizPrimitiveType::image;
}

void XVIZPrimitiveBuilder::dimensions(int width, int height)
{
    _image[xvizPrimitiveImageFieldName::widthPxFieldName] = width;
    _image[xvizPrimitiveImageFieldName::heightPxFieldName] = height;
    _type = xvizPrimitiveType::image;
}

void XVIZPrimitiveBuilder::polygon(const Json::Value &vertices)
{
    _polygon[xvizPrimitivePolygonFieldName::verticesFieldName] = vertices;
    _type = xvizPrimitiveType::polygon;
}

void XVIZPrimitiveBuilder::polyline(const Json::Value &vertices)
{
    _polyline[xvizPrimitivePolyginFieldName::verticesFieldName] = vertices;
    _type = xvizPrimitiveType::polyline;
}

void XVIZPrimitiveBuilder::point(const std::string colors, const std::string points)
{
    _point[xvizPrimitivePointFieldName::colorsFieldName] = colors;
    _point[xvizPrimitivePointFieldName::pointsFieldName] = points;
    _type = xvizPrimitiveType::point;
}

void XVIZPrimitiveBuilder::circle(const Json::Value &position, const double radius)
{
    _circle[xvizPrimitiveCircleFieldName::centerFieldName] = position;
    _circle[xvizPrimitiveCircleFieldName::radiusFieldName] = radius;

    _type = xvizPrimitiveType::circle;
}

void XVIZPrimitiveBuilder::stadium(const Json::Value &position, const float radius)
{
    _stadium[xvizPrimitiveStadiumFieldName::startFieldName] = position[0];
    _stadium[xvizPrimitiveStadiumFieldName::endFieldName] = position[1];
    _stadium[xvizPrimitiveStadiumFieldName::radiusFieldName] = radius;

    _type = xvizPrimitiveType::stadium;
}

void XVIZPrimitiveBuilder::text(const Json::Value &position, const std::string &message)
{
    _text[xvizPrimitiveTextFieldName::positionFieldName] = position;
    _text[xvizPrimitiveTextFieldName::textFieldName] = message;

    _type = xvizPrimitiveType::text;
}

void XVIZPrimitiveBuilder::identity(const std::string object_id)
{
    _identity = object_id;
}

void XVIZPrimitiveBuilder::classes(const Json::Value &class_list)
{
    _classes = class_list;
}

void XVIZPrimitiveBuilder::style(const Json::Value &style)
{
    _style = style;
}

void XVIZPrimitiveBuilder::flush()
{
    if (!_type.empty() && !_streamId.empty())
    {
        Json::Value primitiveObject;
        if (!_primitive.isMember(_streamId))
        {
            Json::Value streamArray;
            _primitive[_streamId] = streamArray;
        }
        primitiveObject = _primitive[_streamId];

        std::string primitiveKey = _type + "s";
        Json::Value primitiveArray;
        if (!primitiveObject.isMember(primitiveKey))
        {
            primitiveObject[primitiveKey] = primitiveArray;
        }
        primitiveArray = primitiveObject[primitiveKey];

        Json::Value base;
        bool isHasBase = !(_identity.empty());

        if (isHasBase)
        {
            base[xvizPrimitiveBaseFieldName::objectIdFieldName] = _identity;
            _identity = "";
        }

        if (!_classes.isNull() && isHasBase)
        {
            base[xvizPrimitiveBaseFieldName::classesFieldName] = _classes;
            _classes = Json::Value();
        }

        if (!_style.isNull() && isHasBase)
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
            primitiveArray.append(_circle);
        }
        else if (_type == xvizPrimitiveType::image)
        {
            if (isHasBase)
            {
                _image[xvizPrimitiveImageFieldName::baseFieldName] = base;
            }
            primitiveArray.append(_image);
        }
        else if (_type == xvizPrimitiveType::polygon)
        {
            if (isHasBase)
            {
                _polygon[xvizPrimitivePolygonFieldName::baseFieldName] = base;
            }
            primitiveArray.append(_polygon);
        }
        else if (_type == xvizPrimitiveType::polyline)
        {
            if (isHasBase)
            {
                _polyline[xvizPrimitivePolyginFieldName::baseFieldName] = base;
            }
            primitiveArray.append(_polyline);
        }
        else if (_type == xvizPrimitiveType::point)
        {
            if (isHasBase)
            {
                _point[xvizPrimitivePointFieldName::baseFieldName] = base;
            }
            primitiveArray.append(_point);
        }
        else if (_type == xvizPrimitiveType::stadium)
        {
            if (isHasBase)
            {
                _stadium[xvizPrimitiveStadiumFieldName::baseFieldName] = base;
            }
            primitiveArray.append(_stadium);
        }
        else if (_type == xvizPrimitiveType::text)
        {
            if (isHasBase)
            {
                _text[xvizPrimitiveTextFieldName::baseFieldName] = base;
            }
            primitiveArray.append(_text);
        }
        primitiveObject[primitiveKey] = primitiveArray;

        _primitive[_streamId] = primitiveObject;

        _type.clear();
    }
}

Json::Value XVIZPrimitiveBuilder::getData()
{
    if (!_streamId.empty())
    {
        flush();
    }
    return _primitive;
}

void XVIZPrimitiveBuilder::reset()
{
    XVIZBaseBuilder::reset();
    _type = "";
}
} // namespace perception
} // namespace robosense
