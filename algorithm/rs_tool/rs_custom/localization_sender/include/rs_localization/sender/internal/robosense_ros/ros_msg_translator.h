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
#ifndef RS_LOCALIZATION_SENDER_INTERNAL_ROBOSENSE_ROS_ROS_MSG_TRANSLATOR_H
#define RS_LOCALIZATION_SENDER_INTERNAL_ROBOSENSE_ROS_ROS_MSG_TRANSLATOR_H
#include <cmath>
#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "rs_localization/msg/vehiclestate_msg.h"
#include "rs_localization/msg/grid_map_msg.h"

namespace robosense {
namespace localization {
/************************************************************************/
/**Translation functions between Robosense message and ROS message**/
/************************************************************************/

inline std::array<double, 4> toQuaternion(const std::array<double, 3> &origin)
{
    std::array<double, 4> result{};
    double half_row = 0.5 * origin[0];
    double half_pitch = 0.5 * origin[1];
    double half_yaw = 0.5 * origin[2];
    result[0] = std::sin(half_row) * std::cos(half_pitch) * std::cos(half_yaw) - std::cos(half_row) * std::sin(half_pitch) * std::sin(half_yaw); //x
    result[1] = std::cos(half_row) * std::sin(half_pitch) * std::cos(half_yaw) + std::sin(half_row) * std::cos(half_pitch) * std::sin(half_yaw); //y
    result[2] = std::cos(half_row) * std::cos(half_pitch) * std::sin(half_yaw) - std::sin(half_row) * std::sin(half_pitch) * std::cos(half_yaw); //z
    result[3] = std::cos(half_row) * std::cos(half_pitch) * std::cos(half_yaw) + std::sin(half_row) * std::sin(half_pitch) * std::sin(half_yaw); //w
    return std::move(result);
}
inline std::array<double, 3> toEuler(const std::array<double, 4> &origin)
{
    std::array<double, 3> result{};
    double x = origin[0];
    double y = origin[1];
    double z = origin[2];
    double w = origin[3];
    result[0] = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)); //row
    result[1] = std::asin(2 * (w * y - z * x));                           // pitch
    result[2] = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)); //yaw
    return std::move(result);
}

inline void toRosMsg(const VehicleStateMsg& rs_msg, nav_msgs::Odometry& nav_odom, sensor_msgs::NavSatFix& nav_fix)
{
    nav_odom.header.stamp.fromSec(rs_msg.timestamp);
    nav_odom.header.frame_id = rs_msg.parent_frame_id;
    nav_odom.header.seq = rs_msg.seq;
    nav_odom.child_frame_id = rs_msg.frame_id;
    nav_odom.pose.pose.position.x = rs_msg.pos[0];
    nav_odom.pose.pose.position.y = rs_msg.pos[1];
    nav_odom.pose.pose.position.z = rs_msg.pos[2];
    std::array<double, 4> quaternion = toQuaternion(rs_msg.orien);
    nav_odom.pose.pose.orientation.x = quaternion[0];
    nav_odom.pose.pose.orientation.y = quaternion[1];
    nav_odom.pose.pose.orientation.z = quaternion[2];
    nav_odom.pose.pose.orientation.w = quaternion[3];
    nav_odom.pose.covariance[0] = rs_msg.pos_cov[0];
    nav_odom.pose.covariance[1] = rs_msg.pos_cov[1];
    nav_odom.pose.covariance[2] = rs_msg.pos_cov[2];
    nav_odom.pose.covariance[6] = rs_msg.pos_cov[3];
    nav_odom.pose.covariance[7] = rs_msg.pos_cov[4];
    nav_odom.pose.covariance[8] = rs_msg.pos_cov[5];
    nav_odom.pose.covariance[12] = rs_msg.pos_cov[6];
    nav_odom.pose.covariance[13] = rs_msg.pos_cov[7];
    nav_odom.pose.covariance[14] = rs_msg.pos_cov[8];
    nav_odom.pose.covariance[21] = rs_msg.orien_cov[0];
    nav_odom.pose.covariance[22] = rs_msg.orien_cov[1];
    nav_odom.pose.covariance[23] = rs_msg.orien_cov[2];
    nav_odom.pose.covariance[27] = rs_msg.orien_cov[3];
    nav_odom.pose.covariance[28] = rs_msg.orien_cov[4];
    nav_odom.pose.covariance[29] = rs_msg.orien_cov[5];
    nav_odom.pose.covariance[33] = rs_msg.orien_cov[6];
    nav_odom.pose.covariance[34] = rs_msg.orien_cov[7];
    nav_odom.pose.covariance[35] = rs_msg.orien_cov[8];
    nav_odom.twist.covariance[0] = rs_msg.linear_vel_cov[0];
    nav_odom.twist.covariance[1] = rs_msg.linear_vel_cov[1];
    nav_odom.twist.covariance[2] = rs_msg.linear_vel_cov[2];
    nav_odom.twist.covariance[6] = rs_msg.linear_vel_cov[3];
    nav_odom.twist.covariance[7] = rs_msg.linear_vel_cov[4];
    nav_odom.twist.covariance[8] = rs_msg.linear_vel_cov[5];
    nav_odom.twist.covariance[12] = rs_msg.linear_vel_cov[6];
    nav_odom.twist.covariance[13] = rs_msg.linear_vel_cov[7];
    nav_odom.twist.covariance[14] = rs_msg.linear_vel_cov[8];
    nav_odom.twist.covariance[21] = rs_msg.acc_cov[0];
    nav_odom.twist.covariance[22] = rs_msg.acc_cov[1];
    nav_odom.twist.covariance[23] = rs_msg.acc_cov[2];
    nav_odom.twist.covariance[27] = rs_msg.acc_cov[3];
    nav_odom.twist.covariance[28] = rs_msg.acc_cov[4];
    nav_odom.twist.covariance[29] = rs_msg.acc_cov[5];
    nav_odom.twist.covariance[33] = rs_msg.acc_cov[6];
    nav_odom.twist.covariance[34] = rs_msg.acc_cov[7];
    nav_odom.twist.covariance[35] = rs_msg.acc_cov[8];
    nav_odom.twist.twist.linear.x = rs_msg.linear_vel[0];
    nav_odom.twist.twist.linear.y = rs_msg.linear_vel[1];
    nav_odom.twist.twist.linear.z = rs_msg.linear_vel[2];
    nav_odom.twist.twist.angular.x = rs_msg.angular_vel[0];
    nav_odom.twist.twist.angular.y = rs_msg.angular_vel[1];
    nav_odom.twist.twist.angular.z = rs_msg.angular_vel[2];

    nav_fix.header = nav_odom.header;
    nav_fix.header.frame_id = "/world";

    nav_fix.longitude = rs_msg.fix[0];
    nav_fix.latitude = rs_msg.fix[1];
    nav_fix.altitude = rs_msg.fix[2];
}

inline geometry_msgs::TransformStamped toRosTf(const VehicleStateMsg& rs_msg)
{
    geometry_msgs::TransformStamped tf;
    tf.header.stamp.fromSec(rs_msg.timestamp);
    tf.header.frame_id = rs_msg.parent_frame_id;
    tf.header.seq = rs_msg.seq;
    tf.child_frame_id = rs_msg.frame_id;
    tf.transform.translation.x = rs_msg.pos[0];
    tf.transform.translation.y = rs_msg.pos[1];
    tf.transform.translation.z = rs_msg.pos[2];

    std::array<double, 4> quaternion = toQuaternion(rs_msg.orien);
    tf.transform.rotation.x = quaternion[0];
    tf.transform.rotation.y = quaternion[1];
    tf.transform.rotation.z = quaternion[2];
    tf.transform.rotation.w = quaternion[3];

    return tf;
}

inline void toRosMsg(const VehicleStateMsg& rs_msg, nav_msgs::Path& path)
{
    geometry_msgs::PoseStamped posestamp;
    posestamp.header.stamp.fromSec(rs_msg.timestamp);
    posestamp.header.seq = rs_msg.seq;
    posestamp.header.frame_id = rs_msg.parent_frame_id;
    posestamp.pose.position.x = rs_msg.pos[0];
    posestamp.pose.position.y = rs_msg.pos[1];
    posestamp.pose.position.z = rs_msg.pos[2];
    std::array<double, 4> quaternion = toQuaternion(rs_msg.orien);
    posestamp.pose.orientation.x = quaternion[0];
    posestamp.pose.orientation.y = quaternion[1];
    posestamp.pose.orientation.z = quaternion[2];
    posestamp.pose.orientation.w = quaternion[3];
    path.header.stamp.fromSec(rs_msg.timestamp);
    path.header.frame_id = rs_msg.parent_frame_id;
    path.header.seq = rs_msg.seq;
    path.poses.push_back(posestamp);
    if (path.poses.size() > 3000)
    {
        path.poses.erase(path.poses.begin(), path.poses.begin() + (path.poses.size() - 3000));
    }
}

inline void toRosMsg(const GridMap& map, nav_msgs::OccupancyGrid& occu_grid)
{
    occu_grid.header.frame_id = map.frame_id;
    occu_grid.info.resolution = map.resolution;
    occu_grid.info.width = map.width;
    occu_grid.info.height = map.height;
    occu_grid.info.origin.position.x = map.origin.at(0);
    occu_grid.info.origin.position.y = map.origin.at(1);
    occu_grid.info.origin.orientation.x = map.orientation.at(0);
    occu_grid.info.origin.orientation.y = map.orientation.at(1);
    occu_grid.info.origin.orientation.z = map.orientation.at(2);
    occu_grid.info.origin.orientation.w = map.orientation.at(3);

    for (auto& grid : map.grids)
        occu_grid.data.emplace_back(grid);
}

}  // namespace localization
}  // namespace robosense

#endif  // RS_ROS_FOUND
#endif  // RS_LOCALIZATION_SENDER_INTERNAL_ROBOSENSE_ROS_ROS_MSG_TRANSLATOR_H
