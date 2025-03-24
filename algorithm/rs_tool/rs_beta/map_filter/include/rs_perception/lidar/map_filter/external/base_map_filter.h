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
#ifndef RS_PERCEPTION_LIDAR_MAP_FILTER_EXTERNAL_BASE_MAP_FILTER_H
#define RS_PERCEPTION_LIDAR_MAP_FILTER_EXTERNAL_BASE_MAP_FILTER_H

#include "rs_common/external/basic_type/rs_point.h"
#include "rs_common/external/register/register.h"
#include "rs_perception/common/external/base_component/base_component.h"
#include "rs_perception/lidar/map_filter/external/common/rs_hd_map.h"
// #include "rs_beta/map_filter/include/rs_perception/lidar/map_filter/external/common/rs_hd_map.h"

namespace robosense {
namespace perception {
namespace lidar {

struct MOD_PUBLIC StaticMsg {
public:
    int laneIdx = -1;
    int vel_num = 0;
    int direction = 0;
    double start_time = 0.0;
    double end_time = 0.0;
};

class MOD_PUBLIC MapFilterMsg {
public:
    using Ptr = std::shared_ptr<MapFilterMsg>;
    std::vector<Object::Ptr> objects;
    std::map<int, std::vector<EventType>> obj_evt_map;
    std::map<int, bool> jam_map;
    std::map<int, std::map<int, StaticMsg>> count_map;
    bool filter_obj = false;
    bool event_detect = false;
};

class MOD_PUBLIC BaseMapFilter: public BaseComponent {
public:
    using Ptr = std::shared_ptr<BaseMapFilter>;

    virtual void init(const RsYamlNode& config_node) = 0;

    virtual void perception(const RsLidarFrameMsg::Ptr& msg_ptr) = 0;

    virtual void getResult(Any::Ptr& any) = 0;

    virtual ~BaseMapFilter() = default;

protected:
    void clean() {
        res_ptr_->objects.clear();
        res_ptr_->jam_map.clear();
        res_ptr_->obj_evt_map.clear();
    }
    MapFilterMsg::Ptr res_ptr_;
};

RS_REGISTER_REGISTERER(BaseMapFilter);
#define RS_REGISTER_MAP_FILTER(name) \
RS_REGISTER_CLASS(BaseMapFilter, name)

}  // namespace lidar
}  // namespace perception
}  // namespace robosense
#endif  // RS_PERCEPTION_LIDAR_MAP_FILTER_EXTERNAL_BASE_MAP_FILTER_H
