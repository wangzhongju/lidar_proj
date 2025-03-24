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

#include "rs_perception/perception/external/subs/v2r_perception.h"

namespace robosense {
namespace perception {

void V2rPerception::init(const RsYamlNode& config_node) {
    initConfig(config_node);
    initV2rConfig(config_node);
}

void V2rPerception::perception(const RsPerceptionMsg::Ptr& msg_ptr) {
    frame_idx_++;
    if (frame_idx_ > std::numeric_limits<int32_t >::max() - 10) {
        frame_idx_ = 0;
    }
    msg_ptr->tic_toc.ticReceive(frame_idx_);
    initMsg(msg_ptr);
    // update pose and gps
    for (auto itr = msg_ptr->sub_lidar_msgs_map.begin(); itr != msg_ptr->sub_lidar_msgs_map.end(); ++itr) {
        itr->second->updateGlobalAxis(v2r_global_pose_);
        itr->second->gps_origin = gps_origin_;
    }
    process();

    RDEBUGSP << name() << ": cost time " << (getTime() - msg_ptr_->tic_toc.receive_time) * 1000. << " ms.";
    msg_ptr_->tic_toc.ticSend();
}

void V2rPerception::start() {
    pipeline_thread_worker_ptr_.reset(new PipelineThreadWorker<RsPerceptionMsg::Ptr>);
    pipeline_thread_worker_ptr_->bind([this](const RsPerceptionMsg::Ptr& msg_ptr){
        this->perception(msg_ptr);
        for (auto &cb : this->perception_cb_list_) {
            cb(msg_ptr);
        }
    });
    pipeline_thread_worker_ptr_->start();
}

void V2rPerception::addData(const RsPerceptionMsg::Ptr& msg_ptr) {
    pipeline_thread_worker_ptr_->add(msg_ptr);
}

void V2rPerception::regPerceptionCallback(const std::function<void(const RsPerceptionMsg::Ptr& msg_ptr)>& cb) {
    std::lock_guard<std::mutex> lg(mx_perception_cb_);
    perception_cb_list_.emplace_back(cb);
}

RS_REGISTER_PERCEPTION(V2rPerception);

}  // namespace perception
}  // namespace robosense
