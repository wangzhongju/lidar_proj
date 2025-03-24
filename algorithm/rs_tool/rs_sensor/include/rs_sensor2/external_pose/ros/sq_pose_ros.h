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
#ifndef RS_SENSEOR2_EXTERNAL_ROS_SQ_POSE_ROS_H
#define RS_SENSEOR2_EXTERNAL_ROS_SQ_POSE_ROS_H

#include "rs_common/external/common.h"
#include "rs_common/external/util/rs_gps_translator.h"
#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND
#include "rs_sensor2/common/base_sensor.h"
#include "rs_sensor2/common/data_type/can_data.h"
#include "rs_sensor2/external_pose/msg/Pos320Nav.h"

namespace robosense {
namespace sensor {

class SqPoseSensor : public BaseSensor {
public:
    using Ptr = std::shared_ptr<SqPoseSensor>;

    explicit SqPoseSensor() {
        pose_msg_ptr_.reset(new CanData);
        gpsTranslator.reset(new RsGpsTranslator);
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
                                         &SqPoseSensor::pose_msg_callback, this);
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

    inline void pose_msg_callback(const perception_msg::Pos320Nav::ConstPtr& msgPtr) {
        pose_msg_ptr_->timestamp = msgPtr->header.stamp.toSec();
        pose_msg_ptr_->pose_ptr->roll = msgPtr->roll / 180.0 * RS_M_PI;
        pose_msg_ptr_->pose_ptr->pitch = msgPtr->pitch / 180.0 * RS_M_PI;
        pose_msg_ptr_->pose_ptr->yaw = msgPtr->yaw;
        if (msgPtr->yaw >= 0.0 && msgPtr->yaw <= 270.0) {
            pose_msg_ptr_->pose_ptr->yaw = (90.0 - msgPtr->yaw) / 180.0 * RS_M_PI;
        } else {
            pose_msg_ptr_->pose_ptr->yaw = (450.0 - msgPtr->yaw) / 180.0 * RS_M_PI;
        }
        if (pose_msg_ptr_->pose_ptr->yaw < -RS_M_PI) {
            pose_msg_ptr_->pose_ptr->yaw = pose_msg_ptr_->pose_ptr->yaw + 2 * RS_M_PI;
        } else if (pose_msg_ptr_->pose_ptr->yaw > RS_M_PI) {
            pose_msg_ptr_->pose_ptr->yaw = pose_msg_ptr_->pose_ptr->yaw - 2 * RS_M_PI;
        }

        RsVector3d cur_gps(msgPtr->longitude, msgPtr->latitude, msgPtr->altitude);
        RsVector3d xyz;
        if (!init_gps_origin_) {
            gpsTranslator->setGpsOrigin(cur_gps);
            init_gps_origin_ = true;
        } else {
            gpsTranslator->gps2xyz(cur_gps,xyz);
        }
        pose_msg_ptr_->pose_ptr->x = xyz.x;
        pose_msg_ptr_->pose_ptr->y = xyz.y;
        pose_msg_ptr_->pose_ptr->z = xyz.z;
        recvCallback(pose_msg_ptr_);
    }

    std::unique_ptr<ros::NodeHandle> nh_ptr_;
    ros::Subscriber pose_msg_sub_;
    CanData::Ptr pose_msg_ptr_;
    RsGpsTranslator::Ptr gpsTranslator;
    bool init_gps_origin_;
    std::string odom_frame_id = "odometry";
    std::mutex mutex_;
    std::vector<std::function<void(const AnySensor::Ptr &msg_ptr)> > syn_cb_list_;
};

RS_REGISTER_SENSOR(SqPoseSensor)

}   // namespace sensor
}   // namespace robosense

#endif  // RS_ROS_FOUND
#endif  //RS_SENSEOR2_EXTERNAL_ROS_SQ_POSE_ROS_H

