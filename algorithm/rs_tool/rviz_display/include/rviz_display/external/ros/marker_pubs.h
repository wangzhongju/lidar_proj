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

#ifndef RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_MARKER_PUBS_H_
#define RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_MARKER_PUBS_H_

#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND

#include "rviz_display/external/common/base_marker_pub.h"
#include "rviz_display/external/ros/ros_common.h"

namespace robosense {
namespace perception {

struct MarkerPubOptions {
    BasePubOptions base_options;
    std::vector<std::string> pub_keys;
};

class MarkerPubs : public BaseMarkerPub {
public:
    using Ptr = std::shared_ptr<MarkerPubs>;

    MarkerPubs() {
        nh_ptr_.reset(new ros::NodeHandle);
    }

    // load configures from yaml and init marker display function.
    // marker include cube/arrow/box line etc.
    // input: yaml node
    inline void init(const MarkerPubOptions &options) {
        base_options_ = options.base_options;

        marker_array_ptr_.reset(new ROS_MARKER_ARRAY);
        perception_pub_ = nh_ptr_->advertise<ROS_MARKER_ARRAY>
        (base_options_.pre_fix + "perception_info_rviz", 1, true);

        genFunc(options.pub_keys);
    }

    // entrance of display the marker
    // input: robosense perception message struct
    // output: void. the result will be published into ros space.
    inline void display(const RsPerceptionMsg::Ptr &msg_ptr) override {
        for (const auto &func : func_set_) {
            func(msg_ptr);
        }

        marker_array_ptr_->markers.clear();
        ros::Time time(msg_ptr->rs_lidar_result_ptr->timestamp);
        for (auto itr = marker_list_map_.begin(); itr != marker_list_map_.end(); ++itr) {
            for (const auto &marker : itr->second) {
                ROS_MARKER ros_marker;
                transMarker(marker, ros_marker);
                ros_marker.header.stamp = time;
                marker_array_ptr_->markers.push_back(ros_marker);
            }
        }

        perception_pub_.publish(*marker_array_ptr_);
    }

private:
    void genOrientation(Marker &marker, const float &direction = 0) override {
        ROS_MARKER ros_marker;
        tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, direction);
        tf::quaternionTFToMsg(quat, ros_marker.pose.orientation);
        marker.orientation.x = ros_marker.pose.orientation.x;
        marker.orientation.y = ros_marker.pose.orientation.y;
        marker.orientation.z = ros_marker.pose.orientation.z;
        marker.orientation.w = ros_marker.pose.orientation.w;
    }

    inline std::string name() override {
        return "MarkerPubs";
    }

    std::unique_ptr<ros::NodeHandle> nh_ptr_;

    ROS_MARKER_ARRAY::Ptr marker_array_ptr_;
    ros::Publisher perception_pub_;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_ROS_FOUND

#endif  // RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_MARKER_PUBS_H_
