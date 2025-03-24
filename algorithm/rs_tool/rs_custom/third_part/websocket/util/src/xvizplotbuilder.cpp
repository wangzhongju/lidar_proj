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
#include "xviz/xvizplotbuilder.h"

namespace robosense
{
namespace perception
{
xvizPlotBuilder::xvizPlotBuilder() : xvizBaseUIBuilder("plot")
{
    reset();
}

xvizPlotBuilder::~xvizPlotBuilder()
{
    // TODO...
}

void xvizPlotBuilder::independentVariable(const std::string &ui_independentVariable)
{
    _independentVariable = ui_independentVariable;
    _is_independentVariable = true;
}

void xvizPlotBuilder::dependentVariables(const Json::Value &ui_denpendentVariables)
{
    _dependentVariables = ui_denpendentVariables;
}

void xvizPlotBuilder::regions(const Json::Value &ui_regions)
{
    _regions = ui_regions;
    _is_regions = true;
}

void xvizPlotBuilder::description(const std::string &ui_description)
{
    _description = ui_description;
    _is_description = true;
}

void xvizPlotBuilder::title(const std::string &ui_title)
{
    _title = ui_title;
    _is_title = true;
}

Json::Value xvizPlotBuilder::getUI()
{
    if (!(_is_independentVariable || _is_regions))
    {
        return Json::Value();
    }

    Json::Value obj = xvizBaseUIBuilder::getUI();

    if (_is_independentVariable)
    {
        obj["independentVariable"] = _independentVariable;
        obj["dependentVariables"] = _dependentVariables;
    }

    if (_is_regions)
    {
        obj["regions"] = _regions;
    }

    if (_is_title)
    {
        obj["title"] = _title;
    }

    if (_is_description)
    {
        obj["description"] = _description;
    }

    return obj;
}

void xvizPlotBuilder::reset()
{
    xvizBaseUIBuilder::reset();

    _is_independentVariable = false;
    _is_regions = false;
    _is_description = false;
    _is_title = false;

    _independentVariable = "";
    _dependentVariables = Json::Value();
    _title = "";
    _description = "";
    _regions = Json::Value();
}

} // namespace perception
} // namespace robosense
