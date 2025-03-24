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
#include "xviz/xvizcommon.h"

namespace robosense
{
namespace perception
{

const std::string xvizCategory::annotation = "ANNOTATION";
const std::string xvizCategory::future_instance = "FUTURE_INSTANCE";
const std::string xvizCategory::pose = "POSE";
const std::string xvizCategory::primitive = "PRIMITIVE";
const std::string xvizCategory::ui_primitive = "UI_PRIMITIVE";
const std::string xvizCategory::time_series = "TIME_SERIES";
const std::string xvizCategory::variable = "VARIABLE";
const std::string xvizCategory::link = "LINK";

const std::string xvizCategoryFieldName::annotationFieldName = "annotations";
const std::string xvizCategoryFieldName::future_instanceFieldName = "future_instances";
const std::string xvizCategoryFieldName::poseFieldName = "poses";
const std::string xvizCategoryFieldName::primitiveFieldName = "primitives";
const std::string xvizCategoryFieldName::ui_primitiveFieldName = "ui_primitives";
const std::string xvizCategoryFieldName::time_seriesFieldName = "time_series";
const std::string xvizCategoryFieldName::variableFieldName = "variables";
const std::string xvizCategoryFieldName::linkFieldName = "links";

const std::string xvizPrimitiveType::circle = "circle";
const std::string xvizPrimitiveType::image = "image";
const std::string xvizPrimitiveType::point = "point";
const std::string xvizPrimitiveType::polygon = "polygon";
const std::string xvizPrimitiveType::polyline = "polyline";
const std::string xvizPrimitiveType::stadium = "stadium";
const std::string xvizPrimitiveType::text = "text";

// PrimitiveStyle

const std::string xvizPrimitiveStyleFieldName::fillColorFieldName = "fill_color";
const std::string xvizPrimitiveStyleFieldName::strokeColorFieldName = "stroke_color";
const std::string xvizPrimitiveStyleFieldName::strokeWidthFieldName = "stroke_width";
const std::string xvizPrimitiveStyleFieldName::radiusFieldName = "radius";
const std::string xvizPrimitiveStyleFieldName::textSizeFieldName = "text_size";
const std::string xvizPrimitiveStyleFieldName::textRotationFieldName = "text_rotation";
const std::string xvizPrimitiveStyleFieldName::textAnchorFieldName = "text_anchor";
const std::string xvizPrimitiveStyleFieldName::textBaselineFieldName = "text_baseline";
const std::string xvizPrimitiveStyleFieldName::heightFieldName = "height";

// primitiveBase

const std::string xvizPrimitiveBaseFieldName::objectIdFieldName = "object_id";
const std::string xvizPrimitiveBaseFieldName::classesFieldName = "classes";
const std::string xvizPrimitiveBaseFieldName::styleFieldName = "style";

const std::string xvizPrimitiveFieldName::circleFieldName = "circles";
const std::string xvizPrimitiveFieldName::imageFieldName = "images";
const std::string xvizPrimitiveFieldName::pointFieldName = "points";
const std::string xvizPrimitiveFieldName::polygonFieldName = "polygons";
const std::string xvizPrimitiveFieldName::polylineFieldName = "polylines";
const std::string xvizPrimitiveFieldName::stadiumFieldName = "stadiums";
const std::string xvizPrimitiveFieldName::textFieldName = "texts";

// primitiveCircle

const std::string xvizPrimitiveCircleFieldName::baseFieldName = "base";
const std::string xvizPrimitiveCircleFieldName::centerFieldName = "center";
const std::string xvizPrimitiveCircleFieldName::radiusFieldName = "radius";

// primitiveImage
const std::string xvizPrimitiveImageFieldName::baseFieldName = "base";
const std::string xvizPrimitiveImageFieldName::positionFieldName = "position";
const std::string xvizPrimitiveImageFieldName::dataFieldName = "data";
const std::string xvizPrimitiveImageFieldName::widthPxFieldName = "width_px";
const std::string xvizPrimitiveImageFieldName::heightPxFieldName = "height_px";

// primitiveBase
const std::string xvizPrimitivePointFieldName::baseFieldName = "base";
const std::string xvizPrimitivePointFieldName::pointsFieldName = "points";
const std::string xvizPrimitivePointFieldName::colorsFieldName = "colors";

// primitivePolygon
const std::string xvizPrimitivePolygonFieldName::baseFieldName = "base";
const std::string xvizPrimitivePolygonFieldName::verticesFieldName = "vertices";

// primitivePolygin
const std::string xvizPrimitivePolyginFieldName::baseFieldName = "base";
const std::string xvizPrimitivePolyginFieldName::verticesFieldName = "vertices";

// primitiveStadium
const std::string xvizPrimitiveStadiumFieldName::baseFieldName = "base";
const std::string xvizPrimitiveStadiumFieldName::startFieldName = "start";
const std::string xvizPrimitiveStadiumFieldName::endFieldName = "end";
const std::string xvizPrimitiveStadiumFieldName::radiusFieldName = "radius";

// primitiveText
const std::string xvizPrimitiveTextFieldName::baseFieldName = "base";
const std::string xvizPrimitiveTextFieldName::positionFieldName = "position";
const std::string xvizPrimitiveTextFieldName::textFieldName = "text";

// uiPrimitive
const std::string xvizUiPrimitiveFieldName::uiprimitiveFieldName = "ui_primitives";
const std::string xvizUiPrimitiveFieldName::treetableFieldName = "treetable";
const std::string xvizUiPrimitiveFieldName::columnsFieldName = "columns";
const std::string xvizUiPrimitiveFieldName::nodesFieldName = "nodes";

const std::string xvizUiPrimitiveColumnFieldName::displayTextFieldName = "display_text";
const std::string xvizUiPrimitiveColumnFieldName::typeFieldName = "type";
const std::string xvizUiPrimitiveColumnFieldName::unitFieldName = "unit";

const std::string xvizUiPrimitiveNodeFieldName::id = "id";
const std::string xvizUiPrimitiveNodeFieldName::parent = "parent";
const std::string xvizUiPrimitiveNodeFieldName::values = "values";

const std::string xvizValuesFieldName::stringFieldName = "strings";
const std::string xvizValuesFieldName::int32FieldName = "int32s";
const std::string xvizValuesFieldName::doubleFieldName = "doubles";
const std::string xvizValuesFieldName::boolFieldName = "bools";

const std::string xvizMapOriginFieldName::longtitudeFieldName = "longitude";
const std::string xvizMapOriginFieldName::latitudeFieldName = "latitude";
const std::string xvizMapOriginFieldName::altitudeFieldName = "altitude";

const std::string xvizPoseFieldName::timestampFieldName = "timestamp";
const std::string xvizPoseFieldName::mapOriginFieldName = "mapOrigin";
const std::string xvizPoseFieldName::positionFieldName = "position";
const std::string xvizPoseFieldName::orientationFieldName = "orientation";

const std::string xvizTimeSeriesFieldName::timestampFieldName = "timestamp";
const std::string xvizTimeSeriesFieldName::streamsFieldName = "streams";
const std::string xvizTimeSeriesFieldName::valuesFieldName = "values";
const std::string xvizTimeSeriesFieldName::objectIdFieldName = "object_id";

const std::string xvizVariableFieldName::baseFieldName = "base";
const std::string xvizVariableFieldName::objectIdFieldName = "object_id";
const std::string xvizVariableFieldName::valuesFieldName = "values";

const std::string xvizFutureInstanceFieldName::timestampsFieldName = "timestamps";

} // namespace perception
} // namespace robosense
