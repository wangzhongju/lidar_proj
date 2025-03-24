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
#ifndef RS_PERCEPTION_LIDAR_MAP_FILTER_EXTERNAL_MAP_FILTER_H
#define RS_PERCEPTION_LIDAR_MAP_FILTER_EXTERNAL_MAP_FILTER_H
#include "rs_perception/lidar/map_filter/external/base_map_filter.h"
// #include "rs_beta/map_filter/include/rs_perception/lidar/map_filter/external/base_map_filter.h"

namespace robosense {
namespace perception {
namespace lidar {

class MOD_PUBLIC MapFilter : public BaseMapFilter {
public:
    using Ptr = std::shared_ptr<MapFilter>;

    // entrance of initialize of map filter
    // input: yaml node
    void init(const RsYamlNode& config_node) override;

    // entrance of map filter
    // input: robosense perception message struct.
    // output: void. the result will be recorded in "msg_ptr".
    void perception(const RsLidarFrameMsg::Ptr& msg_ptr) override;

    // entrance of getting the result from map filter
    // input: an "Any::Ptr"
    // output: void. the result is in "any"
    void getResult(Any::Ptr& any) override;

private:
    std::string name() {
        return "MapFilter";
    }
    const std::string strategy = "MoleMapFilter";
    RsYamlNode config_node_;
    BaseMapFilter::Ptr impl_ptr_;

};
}   // namespace lidar
}   // namespace perception
}   // namespace robosense
#endif  // RS_PERCEPTION_LIDAR_MAP_FILTER_EXTERNAL_MAP_FILTER_H
