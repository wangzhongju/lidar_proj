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
#include "rs_localization/sender/internal/robosense_ros/ros_sender.h"

#ifdef RS_ROS_FOUND
#include "rs_localization/sender/internal/robosense_ros/ros_msg_translator.h"
#include "rs_sensor/lidar/msg/ros_msg_translator.h"
#include "rs_localization/status_macro.h"
namespace robosense {
namespace localization {


int RosLocalizationSender::init(const RsYamlNode &config_node,
                                const std::shared_ptr<LocalizationInterface> &localization_ptr) {
    localization_ptr_ = localization_ptr;
    params.load(config_node);
    // params.log(name());
    initRosPublisher(params);

    return 0;
}

void RosLocalizationSender::send() {
    // 地图发送一次
    if (params.send_map_ros_)
    {
        localization::GridMap grid_map;
        lidar::LidarPointCloudMsg pointcloud_map;
        nav_msgs::OccupancyGrid grid_ros;
        if (localization_ptr_->getMap(grid_map) == common::ErrCode_Success)
        {
            toRosMsg(grid_map, grid_ros);
            localization_grid_map_pub_.publish(grid_ros);
        }
        if (localization_ptr_->getMap(pointcloud_map) == common::ErrCode_Success)
        {
            sensor_msgs::PointCloud2 ros_msg_;
            pcl::toROSMsg(toRosMsg(pointcloud_map), ros_msg_);
            localization_pointcloud_map_pub_.publish(ros_msg_);
        }
    }
    while (params.start_flag_) {
        const double total_duration = 1.0 / params.localization_freq_ * 1000000.0;
        auto status = localization_ptr_->getModuleStatus().localization_status;
        auto start_time = getTime();
        static uint32_t prev_seq = 0;
        if (status != loc_status::LOC_LOST)
        {
            VehicleStateMsg cur_state;
            if (localization_ptr_->getVehicleState(cur_state) == common::ErrCode_Success)
            {
                if(cur_state.seq == prev_seq) {
                    continue;
                }
                pubLocalizationRos(cur_state);
                prev_seq = cur_state.seq;
            }
        }
        auto end_time = getTime();
        auto exec_duration = (end_time - start_time) * 1000000;
        if (total_duration > exec_duration)
        {
            usleep(total_duration - exec_duration);
        }
    }
}

void RosLocalizationSender::pubLocalizationRos(const VehicleStateMsg &vstate) {
    if (params.send_path_ros_)
    {
        toRosMsg(vstate, car_path_);
        localization_path_pub_.publish(car_path_);
    }
    /**** Pub Pos ****/
    if (params.send_pos_ros_)
    {
        nav_msgs::Odometry ros_odom_msg;
        sensor_msgs::NavSatFix ros_fix_msg;
        toRosMsg(vstate, ros_odom_msg, ros_fix_msg);
        localization_pos_pub_.publish(ros_odom_msg);
        localization_fix_pub_.publish(ros_fix_msg);
    }

    /**** Pub TF ****/
    tf_broadcaster_ptr_->sendTransform(toRosTf(vstate));
}
void RosLocalizationSender::start() {
    params.start_flag_ = true;
    auto func = [this]() {
        while (params.start_flag_)
        {
            localization::LocalizationInterface::ModuleStatus status = localization_ptr_->getModuleStatus();
            std_msgs::UInt32 msg;
            msg.data = status.status_int;
            localization_status_pub_.publish(msg);
            sleep(1);
        }
    };
    std::thread t(func);
    t.detach();
    localization_sender_thread_ = std::thread(std::bind(&RosLocalizationSender::send, this));
}

void RosLocalizationSender::stop() {
    params.start_flag_ = false;
    if(localization_sender_thread_.joinable()) {
        localization_sender_thread_.join();
    }
}

RS_REGISTER_LOCALIZATION_SENDER(RosLocalizationSender)

}
}
#endif  // RS_ROS_FOUND