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
#include "xviz/xvizpanelbuilder.h"

namespace robosense
{
namespace perception
{
xvizPanelBuilder::xvizPanelBuilder() : xvizBaseUIBuilder("panel")
{
    reset();
}

xvizPanelBuilder::~xvizPanelBuilder()
{
    // TODO..
}

void xvizPanelBuilder::name(const std::string &ui_name)
{
    _name = ui_name;
}

void xvizPanelBuilder::layout(const std::string &ui_layout)
{
    _layout = ui_layout;
    _is_layout = true;
}

void xvizPanelBuilder::interactions(const Json::Value &ui_interactions)
{
    _interactions = ui_interactions;
    _is_interactions = true;
}

Json::Value xvizPanelBuilder::getUI()
{
    if (_name.empty())
    {
        return Json::Value();
    }

    Json::Value obj = xvizBaseUIBuilder::getUI();

    obj["name"] = _name;

    if (_is_layout)
    {
        obj["layout"] = _layout;
    }

    if (_is_interactions)
    {
        obj["interactions"] = _interactions;
    }

    return obj;
}

void xvizPanelBuilder::reset()
{
    xvizBaseUIBuilder::reset();

    _is_layout = false;
    _is_interactions = false;
    _name = "";
    _layout = "";
    _interactions = Json::Value();
}
} // namespace perception
} // namespace robosense
