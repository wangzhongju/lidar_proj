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
#ifdef RS_ROS_FOUND

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include "rs_common/external/msg/gnss_msg.h"

namespace robosense {
namespace gnss {
/************************************************************************/
/**Translation functions between Robosense message and ROS message**/
/************************************************************************/

inline GnssMsg toRsMsg(const sensor_msgs::NavSatFix &ros_msg) {
    GnssMsg rs_msg;
    rs_msg.seq = ros_msg.header.seq;
    rs_msg.timestamp = ros_msg.header.stamp.toSec();
    rs_msg.parent_frame_id = ros_msg.header.frame_id;
    rs_msg.frame_id = "/gnss";
    rs_msg.status = ros_msg.status.status;
    rs_msg.pos[0] = ros_msg.latitude;
    rs_msg.pos[1] = ros_msg.longitude;
    rs_msg.pos[2] = ros_msg.altitude;
    return rs_msg;
}

inline sensor_msgs::NavSatFix toRosMsg(const GnssMsg &rs_msg) {
    sensor_msgs::NavSatFix ros_msg;
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    ros_msg.status.status = rs_msg.status;
    ros_msg.latitude = rs_msg.pos[0];
    ros_msg.longitude = rs_msg.pos[1];
    ros_msg.altitude = rs_msg.pos[2];
    return ros_msg;
}

inline nav_msgs::Odometry toRosMsgOdom(const GnssMsg &rs_msg) {
    nav_msgs::Odometry ros_msg;
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(rs_msg.orien[0], rs_msg.orien[1],
                                                                                  rs_msg.orien[2]);
    ros_msg.pose.pose.orientation = odom_quat;
    ros_msg.twist.twist.linear.x = rs_msg.linear_vel[0];
    ros_msg.twist.twist.linear.y = rs_msg.linear_vel[1];
    ros_msg.twist.twist.linear.z = rs_msg.linear_vel[2];
    return ros_msg;
}

}  // namespace gnss
}  // namespace robosense
#endif  // RS_ROS_FOUND
