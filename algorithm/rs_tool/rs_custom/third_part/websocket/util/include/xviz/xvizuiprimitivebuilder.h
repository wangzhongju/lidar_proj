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
#ifndef XVIZUIPRIMITIVEBUILDER_H
#define XVIZUIPRIMITIVEBUILDER_H
//
// map<stream_id, UIPrimitiveState>
//
// UIPrimitiveState: {
//                      treetable: TreeTable
//                   }
// TreeTable: {
//              columns: list<TreeTableColumn>
//              nodes: list<TreeTableNode>
//            }
// TreeTableColumn: {
//                     display_text: string
//                     type: "int32"/"double"/"string"/boolean"
//                     uint: string
//                  }
// TreeTableNode: {
//                   id: int32
//                    parent: int32
//                    column_values: list<string>
//                }
//

#include "xviz/xvizbasebuilder.h"
namespace robosense
{
namespace perception
{
class xvizUiPrimitiveBuilder : public XVIZBaseBuilder
{
public:
    typedef std::shared_ptr<xvizUiPrimitiveBuilder> Ptr;
    typedef std::shared_ptr<const xvizUiPrimitiveBuilder> ConstPtr;

public:
    xvizUiPrimitiveBuilder(const std::string &category, const Json::Value &metadata, XVIZValidator::Ptr validator);

    virtual ~xvizUiPrimitiveBuilder();

public:
    void columns(const std::string &text, const std::string type, const std::string &unit);

    void nodes(const int id, const int parent, const Json::Value &values);

    void flush();

    Json::Value getData();

    void reset();

private:
    Json::Value _column;
    Json::Value _node;

    Json::Value _treetablecolumns;
    Json::Value _treetablenodes;

    Json::Value _ui_primitive;

    Json::Value _ui_primitives;
};

} // namespace perception
} // namespace robosense

#endif // XVIZUIPRIMITIVEBUILDER_H
