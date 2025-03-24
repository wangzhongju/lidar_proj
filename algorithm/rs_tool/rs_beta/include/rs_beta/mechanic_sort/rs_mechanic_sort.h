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
#ifndef RS_BETA_MECHANIC_SORT_RS_MECHANIC_SORT_H
#define RS_BETA_MECHANIC_SORT_RS_MECHANIC_SORT_H

#include "rs_perception/common/external/rs_config_manager.h"
#include "rs_common/external/basic_type/rs_point_cloud.h"

namespace robosense {

class RsMechanicSort {
public:
    using Ptr = std::shared_ptr<RsMechanicSort>;

    explicit RsMechanicSort(const std::string& frame_id) {
        frame_id_ = frame_id;
    }

    void sort(const RsPointCloudGPT::Ptr& in_cloud_ptr, std::vector<int>& sort_2_in_indice);

    std::string name() {
        return frame_id_ + "/RsMechanicSort";
    }
private:
    std::string frame_id_ = "";
    bool init_ = false;
    std::vector<std::pair<int, float> > angle_vec_;
};

}   // namespace robosense

#endif  // RS_BETA_MECHANIC_SORT_RS_MECHANIC_SORT_H
