/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
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

#pragma once

#include "rs_dependence/rs_dependence_manager.h"
#ifdef RS_ROS_FOUND

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <pcl_ros/point_cloud.h>
#include "rs_common/external/msg/lidar_point_cloud_msg.h"
#include "rs_sensor/lidar/msg/ros_msg/lidar_scan_ros.h"
#include "rs_driver/msg/packet_msg.h"
#include "rs_driver/msg/scan_msg.h"

namespace robosense {
namespace lidar {
/*Packet Length*/
#ifndef RSLIDAR_PKT_LEN
#define RSLIDAR_PKT_LEN 1248
#endif
/************************************************************************/
/**Translation functions between RoboSense message and ROS message**/
/************************************************************************/
inline pcl::PointCloud<pcl::PointXYZI> toRosMsg(const LidarPointCloudMsg &rs_msg) {
    pcl::PointCloud<pcl::PointXYZI> ros_msg;
    const auto& scan_ptr = rs_msg.point_cloud_ptr;
    ros_msg.resize(scan_ptr->points.size());
    ros_msg.height = scan_ptr->height;
    ros_msg.width = scan_ptr->width;
    if (g_point_type == PointType::POINTXYZI) {
        memcpy(ros_msg.points.data(), scan_ptr->points.data(), sizeof(RsPoint) * scan_ptr->points.size());
    } else {
        for (size_t i = 0; i < scan_ptr->points.size(); ++i) {
            const auto& scan_pt = scan_ptr->points[i];
            auto& pt = ros_msg.points[i];
            pt.x = scan_pt.x;
            pt.y = scan_pt.y;
            pt.z = scan_pt.z;
            pt.intensity = scan_pt.intensity;
        }
    }
    ros_msg.header.stamp = rs_msg.timestamp;
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    return std::move(ros_msg);
}

inline LidarPointCloudMsg toRsMsg(const pcl::PointCloud<pcl::PointXYZI> &ros_msg,
                                  std::string frame_id = "/rslidar_points") {
    LidarPointCloudMsg rs_msg;
    LidarPointCloudMsg::PointCloud *ptr_tmp = new LidarPointCloudMsg::PointCloud();
    ptr_tmp->points.resize(ros_msg.size());
    ptr_tmp->height = ros_msg.height;
    ptr_tmp->width = ros_msg.width;
    rs_msg.height = ros_msg.height;
    rs_msg.width = ros_msg.width;
    if (g_point_type == PointType::POINTXYZI) {
        memcpy(ptr_tmp->points.data(), ros_msg.points.data(), sizeof(RsPoint) * ros_msg.points.size());
    } else {
        for (size_t i = 0; i < ptr_tmp->points.size(); ++i) {
            auto& scan_pt = ptr_tmp->points[i];
            const auto& pt = ros_msg.points[i];
            scan_pt.x = pt.x;
            scan_pt.y = pt.y;
            scan_pt.z = pt.z;
            scan_pt.intensity = pt.intensity;
        }
    }
    rs_msg.seq = ros_msg.header.seq;
    ros::Time time = pcl_conversions::fromPCL(ros_msg.header.stamp);
    rs_msg.timestamp = time.toSec();
    rs_msg.parent_frame_id = ros_msg.header.frame_id;
    rs_msg.frame_id = frame_id;
    rs_msg.point_cloud_ptr.reset(ptr_tmp);
    return rs_msg;
}

inline PacketMsg toRsMsg(const rslidar_msgs::rslidarPacket &ros_msg) {
    PacketMsg rs_msg;
    for (size_t i = 0; i < RSLIDAR_PKT_LEN; i++) {
        rs_msg.packet[i] = std::move(ros_msg.data[i]);
    }
    return std::move(rs_msg);
}

inline rslidar_msgs::rslidarPacket toRosMsg(const PacketMsg &rs_msg) {
    rslidar_msgs::rslidarPacket ros_msg;
    for (size_t i = 0; i < rs_msg.packet.size(); i++) {
        ros_msg.data[i] = std::move(rs_msg.packet[i]);
    }
    return std::move(ros_msg);
}

inline ScanMsg toRsMsg(const rslidar_msgs::rslidarScan &ros_msg) {
    ScanMsg rs_msg;
    rs_msg.seq = ros_msg.header.seq;
    rs_msg.timestamp = ros_msg.header.stamp.toSec();
    rs_msg.frame_id = ros_msg.header.frame_id;

    for (uint32_t i = 0; i < ros_msg.packets.size(); i++) {
        PacketMsg tmp = toRsMsg(ros_msg.packets[i]);
        rs_msg.packets.emplace_back(std::make_shared<PacketMsg>(tmp));
    }
    return std::move(rs_msg);
}

inline rslidar_msgs::rslidarScan toRosMsg(const ScanMsg &rs_msg) {
    rslidar_msgs::rslidarScan ros_msg;
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.frame_id;
    ros_msg.header.seq = rs_msg.seq;
    for (uint32_t i = 0; i < rs_msg.packets.size(); i++) {
        rslidar_msgs::rslidarPacket tmp = toRosMsg(*rs_msg.packets[i]);
        ros_msg.packets.emplace_back(std::move(tmp));
    }
    return std::move(ros_msg);
}

inline std_msgs::Time toRosMsg(const CameraTrigger &rs_msg) {
    std_msgs::Time ros_msg;
    ros_msg.data = ros_msg.data.fromSec(rs_msg.second);
    return ros_msg;
}

}  // namespace lidar
}  // namespace robosense
#endif  // RS_ROS_FOUND
