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
#ifndef XVIZBUILDER_H
#define XVIZBUILDER_H
#include "xviz/xvizmetabuilder.h"
#include "xviz/xvizfutureinstance.h"
#include "xviz/xvizlinkbuilder.h"
#include "xviz/xvizposebuilder.h"
#include "xviz/xvizprimitivebuilder.h"
#include "xviz/xviztimeseriesbuilder.h"
#include "xviz/xvizuiprimitivebuilder.h"
#include "xviz/xvizvariablebuilder.h"
namespace robosense
{
namespace perception
{
class xvizBuilder
{
public:
    typedef std::shared_ptr<xvizBuilder> Ptr;
    typedef std::shared_ptr<const xvizBuilder> ConstPtr;

public:
    xvizBuilder(const Json::Value &metadata, XVIZValidator::Ptr validator);

    ~xvizBuilder();

public:
    void persistent();

    xvizFutureInstance::Ptr futureInstance(const std::string &id);

    XVIZLinkBuilder::Ptr link(const std::string &id);

    XVIZPoseBuilder::Ptr pose(const std::string &id);

    XVIZVariableBuilder::Ptr variable(const std::string &id);

    XVIZPrimitiveBuilder::Ptr primitive(const std::string &id);

    xvizUiPrimitiveBuilder::Ptr uiprimitive(const std::string &id);

    XVIZTimeSeriesBuilder::Ptr timeSeries(const std::string &id);

    // Make
    Json::Value getMessage();

private:
    template <typename T>
    std::shared_ptr<typename std::enable_if<(!std::is_same<XVIZBaseBuilder, T>::value) && (std::is_base_of<XVIZBaseBuilder, T>::value), T>::type>
    dyn_cast(std::shared_ptr<XVIZBaseBuilder> ptr)
    {
        return std::static_pointer_cast<T>(ptr);
    }

private:
    Json::Value _metadata;
    XVIZValidator::Ptr _validator;
    std::map<std::string, XVIZBaseBuilder::Ptr> _mapBuilder;
    Json::Value _xvizMessage;
    std::string _updateType;
};
} // namespace perception
} // namespace robosense
#endif // XVIZBUILDER_H
