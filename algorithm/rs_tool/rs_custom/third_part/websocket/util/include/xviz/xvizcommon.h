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
#ifndef XVIZCOMMON_H
#define XVIZCOMMON_H
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include "jsoncpp/json/json.h"

namespace robosense
{
namespace perception
{
class xvizCategory
{
public:
    static const std::string annotation;
    static const std::string future_instance;
    static const std::string pose;
    static const std::string primitive;
    static const std::string ui_primitive;
    static const std::string time_series;
    static const std::string variable;
    static const std::string link;
};

class xvizCategoryFieldName
{
public:
    static const std::string annotationFieldName;
    static const std::string future_instanceFieldName;
    static const std::string poseFieldName;
    static const std::string primitiveFieldName;
    static const std::string ui_primitiveFieldName;
    static const std::string time_seriesFieldName;
    static const std::string variableFieldName;
    static const std::string linkFieldName;
};

class xvizPrimitiveType
{
public:
    static const std::string circle;
    static const std::string image;
    static const std::string point;
    static const std::string polygon;
    static const std::string polyline;
    static const std::string stadium;
    static const std::string text;
};

// PrimitiveStyle
class xvizPrimitiveStyleFieldName
{
public:
    static const std::string fillColorFieldName;
    static const std::string strokeColorFieldName;
    static const std::string strokeWidthFieldName;
    static const std::string radiusFieldName;
    static const std::string textSizeFieldName;
    static const std::string textRotationFieldName;
    static const std::string textAnchorFieldName;
    static const std::string textBaselineFieldName;
    static const std::string heightFieldName;
};

// primitiveBase
class xvizPrimitiveBaseFieldName
{
public:
    static const std::string objectIdFieldName;
    static const std::string classesFieldName;
    static const std::string styleFieldName;
};

class xvizPrimitiveFieldName
{
public:
    static const std::string circleFieldName;
    static const std::string imageFieldName;
    static const std::string pointFieldName;
    static const std::string polygonFieldName;
    static const std::string polylineFieldName;
    static const std::string stadiumFieldName;
    static const std::string textFieldName;
};

// primitiveCircle
class xvizPrimitiveCircleFieldName
{
public:
    static const std::string baseFieldName;
    static const std::string centerFieldName;
    static const std::string radiusFieldName;
};

// primitiveImage
class xvizPrimitiveImageFieldName
{
public:
    static const std::string baseFieldName;
    static const std::string positionFieldName;
    static const std::string dataFieldName;
    static const std::string widthPxFieldName;
    static const std::string heightPxFieldName;
};

// primitiveBase
class xvizPrimitivePointFieldName
{
public:
    static const std::string baseFieldName;
    static const std::string pointsFieldName;
    static const std::string colorsFieldName;
};

// primitivePolygon
class xvizPrimitivePolygonFieldName
{
public:
    static const std::string baseFieldName;
    static const std::string verticesFieldName;
};

// primitivePolygin
class xvizPrimitivePolyginFieldName
{
public:
    static const std::string baseFieldName;
    static const std::string verticesFieldName;
};

// primitiveStadium
class xvizPrimitiveStadiumFieldName
{
public:
    static const std::string baseFieldName;
    static const std::string startFieldName;
    static const std::string endFieldName;
    static const std::string radiusFieldName;
};

// primitiveText
class xvizPrimitiveTextFieldName
{
public:
    static const std::string baseFieldName;
    static const std::string positionFieldName;
    static const std::string textFieldName;
};

// uiPrimitive
class xvizUiPrimitiveFieldName
{
public:
    static const std::string uiprimitiveFieldName;
    static const std::string treetableFieldName;
    static const std::string columnsFieldName;
    static const std::string nodesFieldName;
};

class xvizUiPrimitiveColumnFieldName
{
public:
    static const std::string displayTextFieldName;
    static const std::string typeFieldName;
    static const std::string unitFieldName;
};

class xvizUiPrimitiveNodeFieldName
{
public:
    static const std::string id;
    static const std::string parent;
    static const std::string values;
};

class xvizValuesFieldName
{
public:
    static const std::string stringFieldName;
    static const std::string int32FieldName;
    static const std::string doubleFieldName;
    static const std::string boolFieldName;
};

class xvizMapOriginFieldName
{
public:
    static const std::string longtitudeFieldName;
    static const std::string latitudeFieldName;
    static const std::string altitudeFieldName;
};

class xvizPoseFieldName
{
public:
    static const std::string timestampFieldName;
    static const std::string mapOriginFieldName;
    static const std::string positionFieldName;
    static const std::string orientationFieldName;
};

class xvizTimeSeriesFieldName
{
public:
    static const std::string timestampFieldName;
    static const std::string streamsFieldName;
    static const std::string valuesFieldName;
    static const std::string objectIdFieldName;
};

class xvizVariableFieldName
{
public:
    static const std::string baseFieldName;
    static const std::string objectIdFieldName;
    static const std::string valuesFieldName;
};

class xvizFutureInstanceFieldName
{
public:
    static const std::string timestampsFieldName;
};

} // namespace perception
} // namespace robosense
#endif // XVIZCOMMON_H
