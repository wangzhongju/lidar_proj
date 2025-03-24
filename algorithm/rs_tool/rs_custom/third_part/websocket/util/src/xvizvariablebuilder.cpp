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
#include "xviz/xvizvariablebuilder.h"

namespace robosense
{
namespace perception
{
XVIZVariableBuilder::XVIZVariableBuilder(const std::string &category,
                                         const Json::Value &metadata,
                                         XVIZValidator::Ptr validator) : XVIZBaseBuilder(category, metadata, validator)
{
    reset();
}

XVIZVariableBuilder::~XVIZVariableBuilder()
{
    // TODO..
}

void XVIZVariableBuilder::identity(const std::string &identifier)
{
    _identifier = identifier;
}

void XVIZVariableBuilder::values(Json::Value &values, const std::string valuesFieldName)
{
    _values = values;
    _fieldName = valuesFieldName;
}

void XVIZVariableBuilder::flush()
{
    if (!_identifier.empty() && !_streamId.empty())
    {
        Json::Value objBase;
        objBase[xvizVariableFieldName::objectIdFieldName] = _identifier;

        Json::Value variable;
        variable[xvizVariableFieldName::baseFieldName] = objBase;

        Json::Value values;
        values[_fieldName] = _values;
        variable[xvizVariableFieldName::valuesFieldName] = values;

        _variables.append(variable);

        reset();
    }
}

Json::Value XVIZVariableBuilder::getData()
{
    if (!_streamId.empty())
    {
        flush();
    }
    return _variables;
}

void XVIZVariableBuilder::reset()
{
    XVIZBaseBuilder::reset();

    _identifier = "";
    _values = Json::Value();
}
} // namespace perception
} // namespace robosense
