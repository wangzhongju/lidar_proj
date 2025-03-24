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
#ifndef RS_LOCALIZATION_INTERNAL_ROBOSENSE_PROTO_PROTO_MSG_TRANSLATOR_H
#define RS_LOCALIZATION_INTERNAL_ROBOSENSE_PROTO_PROTO_MSG_TRANSLATOR_H

#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_PROTO_FOUND

#include "rs_localization/msg/vehiclestate_msg.h"
#include "Proto_msg.Loc.pb.h"
#include "rs_localization/status_macro.h"

namespace robosense {
namespace localization {
/************************************************************************/
/**Translation functions between Robosense message and Protobuf message**/
/************************************************************************/

inline void toProtoMsg(const VehicleStateMsg& rs_msg, Proto_msg::VehicleStateMsg& proto_msg) {
    // timestamp
    proto_msg.set_timestamp(rs_msg.timestamp);

    // seq
    proto_msg.set_seq(rs_msg.seq);

    // frame_id
    proto_msg.set_frame_id(rs_msg.frame_id);

    // parent_frame_id
    proto_msg.set_parent_frame_id(rs_msg.parent_frame_id);

    // status
    proto_msg.set_status(rs_msg.status);

    // origin
    Proto_msg::Vector1d *proto_origin_ptr = proto_msg.mutable_origin();
    size_t origin_nums = rs_msg.origin.size();
    for (size_t i = 0; i < origin_nums; i++) {
        proto_origin_ptr->add_data(rs_msg.origin[i]);
    }

    // fix
    Proto_msg::Vector1d *proto_fix_ptr = proto_msg.mutable_fix();
    size_t fix_nums = rs_msg.fix.size();
    for (size_t i = 0; i < fix_nums; i++) {
        proto_fix_ptr->add_data(rs_msg.fix[i]);
    }

    // pos
    Proto_msg::Vector1d *proto_pos_ptr = proto_msg.mutable_pos();
    size_t pos_nums = rs_msg.pos.size();
    for (size_t i = 0; i < pos_nums; i++) {
        proto_pos_ptr->add_data(rs_msg.pos[i]);
    }

    // pos_cov
    Proto_msg::Vector1d *proto_pos_cov_ptr = proto_msg.mutable_pos_cov();
    size_t pos_cov_nums = rs_msg.pos_cov.size();
    for (size_t i = 0; i < pos_cov_nums; i++) {
        proto_pos_cov_ptr->add_data(rs_msg.pos_cov[i]);
    }

    // orien
    Proto_msg::Vector1d *proto_orien_ptr = proto_msg.mutable_orien();
    size_t orien_nums = rs_msg.orien.size();
    for (size_t i = 0; i < orien_nums; i++) {
        proto_orien_ptr->add_data(rs_msg.orien[i]);
    }

    // orien_cov
    Proto_msg::Vector1d *proto_orien_cov_ptr = proto_msg.mutable_orien_cov();
    size_t orien_cov_nums = rs_msg.orien_cov.size();
    for (size_t i = 0; i < orien_cov_nums; i++) {
        proto_orien_cov_ptr->add_data(rs_msg.orien_cov[i]);
    }

    // angular_vel
    Proto_msg::Vector1d *proto_angular_vel_ptr = proto_msg.mutable_angular_vel();
    size_t angular_vel_nums = rs_msg.angular_vel.size();
    for (size_t i = 0; i < angular_vel_nums; i++) {
        proto_angular_vel_ptr->add_data(rs_msg.angular_vel[i]);
    }

    // angular_vel_cov
    Proto_msg::Vector1d *proto_angular_vel_cov_ptr = proto_msg.mutable_angular_vel_cov();
    size_t angular_vel_cov_nums = rs_msg.angular_vel_cov.size();
    for (size_t i = 0; i < angular_vel_cov_nums; i++) {
        proto_angular_vel_cov_ptr->add_data(rs_msg.angular_vel_cov[i]);
    }

    // linear_vel
    Proto_msg::Vector1d *proto_linear_vel_ptr = proto_msg.mutable_linear_vel ();
    size_t linear_vel_nums = rs_msg.linear_vel .size();
    for (size_t i = 0; i < linear_vel_nums; i++) {
        proto_linear_vel_ptr->add_data(rs_msg.linear_vel [i]);
    }

    // linear_vel_cov
    Proto_msg::Vector1d *proto_linear_vel_cov_ptr = proto_msg.mutable_linear_vel_cov();
    size_t linear_vel_cov_nums = rs_msg.linear_vel_cov.size();
    for (size_t i = 0; i < linear_vel_cov_nums; i++) {
        proto_linear_vel_cov_ptr->add_data(rs_msg.linear_vel_cov[i]);
    }

    // acc
    Proto_msg::Vector1d *proto_acc_ptr = proto_msg.mutable_acc();
    size_t acc_nums = rs_msg.acc.size();
    for (size_t i = 0; i < acc_nums; i++) {
        proto_acc_ptr->add_data(rs_msg.acc[i]);
    }

    // acc_cov
    Proto_msg::Vector1d *proto_acc_cov_ptr = proto_msg.mutable_acc_cov();
    size_t acc_cov_nums = rs_msg.acc_cov.size();
    for (size_t i = 0; i < acc_cov_nums; i++) {
        proto_acc_cov_ptr->add_data(rs_msg.acc_cov[i]);
    }
}

}  // namespace localization
}  // namespace robosense

#endif  // RS_PROTO_FOUND
#endif  // RS_LOCALIZATION_INTERNAL_ROBOSENSE_PROTO_PROTO_MSG_TRANSLATOR_H
