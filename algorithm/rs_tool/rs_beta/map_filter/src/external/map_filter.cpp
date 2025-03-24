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
#include "rs_perception/lidar/map_filter/external/map_filter.h"
#include "rs_perception/lidar/map_filter/internal/mole/mole_map_filter.h"
//#include <rs_beta/map_filter/include/rs_perception/lidar/map_filter/external/map_filter.h>
// #include "rs_beta/map_filter/include/rs_perception/lidar/map_filter/external/map_filter.h"
// #include "rs_beta/map_filter/include/rs_perception/lidar/map_filter/internal/mole/mole_map_filter.h"

namespace robosense {
namespace perception {
namespace lidar {
void MapFilter::init(const RsYamlNode &config_node) {
    config_node_ = config_node;

    std::string tmp_strategy = strategy;
    rsYamlRead(config_node_, "strategy", tmp_strategy);


    if (!BaseMapFilterRegisterer::IsValid(tmp_strategy)) {
        tmp_strategy = strategy;
    }
    if (!BaseMapFilterRegisterer::IsValid(tmp_strategy)) {
        RERROR << name() << ": init strategy " << tmp_strategy << " failed!";
        RS_THROW("init register class error!");
    }

    impl_ptr_.reset(BaseMapFilterRegisterer::getInstanceByName(tmp_strategy));
    impl_ptr_->init(config_node_);
}

void MapFilter::perception(const RsLidarFrameMsg::Ptr &msg_ptr) {
    impl_ptr_->perception(msg_ptr);
}

void MapFilter::getResult(Any::Ptr &any) {
    impl_ptr_->getResult(any);
}

}   // namespace lidar
}   // namespace perception
}   // namespace robosense
