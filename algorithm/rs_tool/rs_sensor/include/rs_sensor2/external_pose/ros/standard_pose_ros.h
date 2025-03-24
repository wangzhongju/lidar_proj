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
#ifndef RS_SENSEOR2_EXTERNAL_ROS_STANDARD_POSE_ROS_H
#define RS_SENSEOR2_EXTERNAL_ROS_STANDARD_POSE_ROS_H

#include "rs_common/external/common.h"
#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND
#include "rs_sensor2/common/base_sensor.h"
#include "rs_sensor2/common/data_type/can_data.h"

namespace robosense {
namespace sensor {

class StandardPoseSensor : public BaseSensor {
public:
    using Ptr = std::shared_ptr<StandardPoseSensor>;

    explicit StandardPoseSensor() {
        init_origin = false;
        pose_msg_ptr_.reset(new CanData);
    }

    void init(const RsYamlNode &config_node) override {
        nh_ptr_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
        RsYamlNode calibration_node, odom_node;
        rsYamlSubNode(config_node, "calibration", calibration_node);
        rsYamlSubNode(calibration_node, "odometry", odom_node);
        rsYamlRead(odom_node, "frame_id", odom_frame_id);

        RsYamlNode pose_node;
        rsYamlSubNode(config_node, "external_pose", pose_node);
        std::string ros_recv_topic_ = "/pose_topic";
        rsYamlRead(pose_node, "ros_recv_topic", ros_recv_topic_);
        pose_msg_sub_ = nh_ptr_->subscribe(ros_recv_topic_, 1,
                                         &StandardPoseSensor::pose_msg_callback, this);
    }

    void start() override {
    }

    void stop() override {
    }

    void regRecvCallback(const std::function<void(const AnySensor::Ptr &)> &cb) override {
        std::unique_lock<std::mutex> lock(mutex_);
        syn_cb_list_.emplace_back(cb);
    }

private:
    inline void recvCallback(const CanData::Ptr &msgPtr) {
        AnySensor::Ptr sensor_ptr(new AnySensor(AnySensorType::CAN, msgPtr));
        sensor_ptr->setTimestamp(msgPtr->timestamp);
        sensor_ptr->setFrameID(odom_frame_id);
        for (size_t i = 0; i < this->syn_cb_list_.size(); ++i) {
            this->syn_cb_list_[i](sensor_ptr);
        }
    }

    inline void pose_msg_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
        pose_msg_ptr_->timestamp = msg->header.stamp.toSec();

        Eigen::RowVector3f translation;
        translation(0) = msg->pose.position.x;
        translation(1) = msg->pose.position.y;
        translation(2) = msg->pose.position.z;

        const auto& o_x = msg->pose.orientation.x;
        const auto& o_y = msg->pose.orientation.y;
        const auto& o_z = msg->pose.orientation.z;
        const auto& o_w = msg->pose.orientation.w;
        Eigen::Quaternionf quat(o_w, o_x, o_y, o_z);
        Eigen::Matrix3f rotation = quat.toRotationMatrix();

        if (!init_origin) {
            original_rotation = rotation;
            original_translation = translation;
            init_origin = true;
        } else {
            Eigen::Matrix3f odo_rotation = original_rotation.inverse() * rotation;
            Eigen::Vector3f odo_translation = 
                original_rotation.inverse() * ((translation - original_translation).transpose());

            pose_msg_ptr_->pose_ptr->x = odo_translation(0);
            pose_msg_ptr_->pose_ptr->y = odo_translation(1);
            pose_msg_ptr_->pose_ptr->z = odo_translation(2);
            pose_msg_ptr_->pose_ptr->roll = std::atan2(odo_rotation(2, 1), odo_rotation(2, 2));
            pose_msg_ptr_->pose_ptr->pitch = std::atan2(-odo_rotation(2, 0), std::sqrt(odo_rotation(2, 1) * odo_rotation(2, 1) 
                + odo_rotation(2, 2) * odo_rotation(2, 2)));
            pose_msg_ptr_->pose_ptr->yaw = std::atan2(odo_rotation(1, 0), odo_rotation(0, 0));
        }
        recvCallback(pose_msg_ptr_);
    }

    std::unique_ptr<ros::NodeHandle> nh_ptr_;
    ros::Subscriber pose_msg_sub_;
    CanData::Ptr pose_msg_ptr_;
    bool init_origin;
    Eigen::Matrix3f original_rotation;
    Eigen::RowVector3f original_translation;
    std::string odom_frame_id = "odometry";
    std::mutex mutex_;
    std::vector<std::function<void(const AnySensor::Ptr &msg_ptr)> > syn_cb_list_;
};

RS_REGISTER_SENSOR(StandardPoseSensor)

}   // namespace sensor
}   // namespace robosense

#endif  // RS_ROS_FOUND
#endif  //RS_SENSEOR2_EXTERNAL_ROS_STANDARD_POSE_ROS_H

