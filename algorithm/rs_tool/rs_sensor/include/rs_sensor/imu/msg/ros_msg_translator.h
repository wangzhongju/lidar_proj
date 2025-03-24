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
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include "rs_common/external/msg/imu_msg.h"

namespace robosense {
namespace imu {

/************************************************************************/
/**Translation functions between Robosense message and ROS message**/
/************************************************************************/

inline ImuMsg toRsMsg(const sensor_msgs::Imu &ros_msg) {
    ImuMsg rs_msg;
    geometry_msgs::Quaternion orientation = ros_msg.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    rs_msg.orien[0] = roll;
    rs_msg.orien[1] = pitch;
    rs_msg.orien[2] = yaw;

    rs_msg.acc[0] = ros_msg.linear_acceleration.x;
    rs_msg.acc[1] = ros_msg.linear_acceleration.y;
    rs_msg.acc[2] = ros_msg.linear_acceleration.z;

    rs_msg.angular_vel[0] = ros_msg.angular_velocity.x;
    rs_msg.angular_vel[1] = ros_msg.angular_velocity.y;
    rs_msg.angular_vel[2] = ros_msg.angular_velocity.z;

    rs_msg.timestamp = ros_msg.header.stamp.toSec();
    rs_msg.parent_frame_id = ros_msg.header.frame_id;
    rs_msg.frame_id = "/imu";
    rs_msg.seq = ros_msg.header.seq;
    return rs_msg;
}

inline sensor_msgs::Imu toRosMsg(const ImuMsg &rs_msg) {
    sensor_msgs::Imu ros_msg;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(rs_msg.orien[0], rs_msg.orien[1],
                                                                                  rs_msg.orien[2]);

    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    ros_msg.angular_velocity.x = static_cast<double>(rs_msg.angular_vel[0]);
    ros_msg.angular_velocity.y = static_cast<double>(rs_msg.angular_vel[1]);
    ros_msg.angular_velocity.z = static_cast<double>(rs_msg.angular_vel[2]);
    ros_msg.linear_acceleration.x = static_cast<double>((rs_msg.acc[0]));
    ros_msg.linear_acceleration.y = static_cast<double>((rs_msg.acc[1]));
    ros_msg.linear_acceleration.z = static_cast<double>((rs_msg.acc[2]));
    ros_msg.orientation = odom_quat;
    return ros_msg;
}

}  // namespace imu
}  // namespace robosense
#endif  // RS_ROS_FOUND
