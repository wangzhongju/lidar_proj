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

#pragma once

#include "rs_dependence/rs_dependence_manager.h"
#include "rs_common/external/util/rs_util.h"
#ifdef RS_ROS_FOUND

#include <nav_msgs/Odometry.h>
#include "rs_common/external/msg/odom_msg.h"

namespace robosense {
namespace odom {
/************************************************************************/
/**Translation functions between Robosense message and ROS message**/
/************************************************************************/

inline OdomMsg toRsMsg(const nav_msgs::Odometry &ros_msg) {
    OdomMsg rs_msg;
    rs_msg.seq = ros_msg.header.seq;
    rs_msg.timestamp = ros_msg.header.stamp.toSec();
    rs_msg.parent_frame_id = ros_msg.header.frame_id;
    rs_msg.frame_id = "/odom";
    rs_msg.linear_vel[0] = ros_msg.twist.twist.linear.x;
    rs_msg.linear_vel[1] = ros_msg.twist.twist.linear.y;
    rs_msg.linear_vel[2] = ros_msg.twist.twist.linear.z;
    rs_msg.angular_vel[0] = ros_msg.twist.twist.angular.x;
    rs_msg.angular_vel[1] = ros_msg.twist.twist.angular.y;
    rs_msg.angular_vel[2] = ros_msg.twist.twist.angular.z;
    rs_msg.position[0] = ros_msg.pose.pose.position.x;
    rs_msg.position[1] = ros_msg.pose.pose.position.y;
    rs_msg.position[2] = ros_msg.pose.pose.position.z;

    std::array<double,4> q{ros_msg.pose.pose.orientation.x, ros_msg.pose.pose.orientation.y, ros_msg.pose.pose.orientation.z,ros_msg.pose.pose.orientation.w};
    rs_msg.orientation = robosense::toEuler(q);
    return rs_msg;
}

inline nav_msgs::Odometry toRosMsg(const OdomMsg &rs_msg) {
    nav_msgs::Odometry ros_msg;
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    ros_msg.twist.twist.linear.x = rs_msg.linear_vel[0];
    ros_msg.twist.twist.linear.y = rs_msg.linear_vel[1];
    ros_msg.twist.twist.linear.z = rs_msg.linear_vel[2];
    ros_msg.twist.twist.angular.x = rs_msg.angular_vel[0];
    ros_msg.twist.twist.angular.y = rs_msg.angular_vel[1];
    ros_msg.twist.twist.angular.z = rs_msg.angular_vel[2];
    ros_msg.pose.pose.position.x = rs_msg.position[0];
    ros_msg.pose.pose.position.y = rs_msg.position[1];
    ros_msg.pose.pose.position.z = rs_msg.position[2];

    std::array<double,4> q = robosense::toQuaternion(rs_msg.orientation);
    ros_msg.pose.pose.orientation.x = q[0];
    ros_msg.pose.pose.orientation.y = q[1];
    ros_msg.pose.pose.orientation.z = q[2];
    ros_msg.pose.pose.orientation.w = q[3];
    return ros_msg;
}

}  // namespace odom
}  // namespace robosense
#endif  // RS_ROS_FOUND
