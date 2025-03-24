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
#ifndef RS_SENSOR_CAN_LXONE_ROS_CAN_LXONE_ROS_H
#define RS_SENSOR_CAN_LXONE_ROS_CAN_LXONE_ROS_H

#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND

#include "rs_sensor2/can/common/can_frame.h"
#include "rs_sensor2/can/common/can_parser.h"
#include "rs_sensor2/can/lxone/lx_can_frame_parse.h"
#include "rs_sensor2/can/msg/can_msg.h"
#include "rs_sensor2/common/base_sensor.h"
#include "rs_common/external/synchronize/rs_synchronizer.h"

namespace robosense {
namespace sensor {

class CanLxoneRosSensor : public BaseSensor {
public:
    using Ptr = std::shared_ptr<CanLxoneRosSensor>;

    explicit CanLxoneRosSensor() {
        can_msg_ptr_.reset(new CanData);
    }

    void init(const RsYamlNode &config_node) override {
        nh_ptr_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
        ros::NodeHandle nh_;
        std::string odom_frame_id = "odometry";
        RsYamlNode calibration_node, odom_node;
        rsYamlSubNode(config_node, "calibration", calibration_node);
        rsYamlSubNode(calibration_node, "odometry", odom_node);
        rsYamlRead(odom_node, "frame_id", odom_frame_id);

        RsYamlNode can_node;
        rsYamlSubNode(config_node, "can", can_node);
        std::string ros_recv_topic_ = "/can_data";
        rsYamlRead(can_node, "ros_recv_topic", ros_recv_topic_);
        can_msg_sub = nh_ptr_->subscribe("/can_data", 1,
                                                           &CanLxoneRosSensor::can_msg_callback, this);

        RsSynchronizerOptions init_options;
        init_options.frame_id_vec.resize(2);
        init_options.frame_id_vec[0] = "speed";
        init_options.frame_id_vec[1] = "yawrate";
        syn_ptr_.reset(new RsSynchronizer(init_options));
        {
            auto func = [this, odom_frame_id](const std::vector<SynSensor::Ptr> &msg_ptr_vec) {
                double time = std::min(msg_ptr_vec[0]->getTimestamp(), msg_ptr_vec[1]->getTimestamp());
                float yaw_rate = 0.;
                float speed = 0.;
                for (size_t i = 0; i < msg_ptr_vec.size(); ++i) {
                    if (msg_ptr_vec[i]->getFrameID() == "speed") {
                        speed = msg_ptr_vec[i]->getSensor<SpeedMsg>().speed;
                    }
                    if (msg_ptr_vec[i]->getFrameID() == "yawrate") {
                        yaw_rate = msg_ptr_vec[i]->getSensor<YawMsg>().yaw_rate;
                    }
                }
                if (this->can_msg_ptr_->timestamp < 0) {
                    this->can_msg_ptr_->timestamp = time;
                } else {
                    double delta_time = time - this->can_msg_ptr_->timestamp;
                    const auto &pose_ptr = this->can_msg_ptr_->pose_ptr;
                    float cur_yaw = pose_ptr->yaw + yaw_rate * delta_time;
                    pose_ptr->x =
                    pose_ptr->x + speed * delta_time * std::cos(pose_ptr->yaw + (cur_yaw - pose_ptr->yaw) / 2.);
                    pose_ptr->y =
                    pose_ptr->y + speed * delta_time * std::sin(pose_ptr->yaw + (cur_yaw - pose_ptr->yaw) / 2.);
                    pose_ptr->yaw = cur_yaw;
                    this->can_msg_ptr_->timestamp = time;
                }

                CanData::Ptr msg_ptr(new CanData);
                *msg_ptr->pose_ptr = *(this->can_msg_ptr_->pose_ptr);
                msg_ptr->timestamp = this->can_msg_ptr_->timestamp;
                AnySensor::Ptr sensor_ptr(new AnySensor(AnySensorType::CAN, msg_ptr));
                sensor_ptr->setTimestamp(time);
                sensor_ptr->setFrameID(odom_frame_id);
                for (size_t i = 0; i < this->syn_cb_list_.size(); ++i) {
                    this->syn_cb_list_[i](sensor_ptr);
                }
            };
            syn_ptr_->regSynCallback(func);
        }
    }

    inline void start() override {
        syn_ptr_->start();
    }

    inline void stop() override {
        syn_ptr_->stop();
    }

    inline void regRecvCallback(const std::function<void(const AnySensor::Ptr &)> &cb) override {
        std::unique_lock<std::mutex> lock(mutex_);
        syn_cb_list_.emplace_back(cb);
    }

private:
    std::string name() {
        return "CanLxoneRosSensor";
    }

    std::unique_ptr<ros::NodeHandle> nh_ptr_;
    ros::Subscriber can_msg_sub;
    struct YawMsg {
        float yaw_rate = 0.;
    };
    struct SpeedMsg {
        float speed = 0.;
    };

    inline void recvCallback(const Basic_Msg::Ptr &msgPtr) {
        const std::string msgName = msgPtr->msgName();
        RTRACE << name() << ": Can Ros Receive msgName = " << msgName;

        if (msgName == "ESP_0x225") {
            LX_CAN::ESP_0x225 *pMsg = dynamic_cast<LX_CAN::ESP_0x225 *>(msgPtr.get());
            if (pMsg != nullptr) {
                float yawrate = pMsg->ACU_YawRate;
                yawrate = yawrate / 180. * RS_M_PI;
                while (yawrate < -RS_M_PI) {
                    yawrate += 2 * RS_M_PI;
                }
                while (yawrate > RS_M_PI) {
                    yawrate -= 2 * RS_M_PI;
                }

                YawMsg yaw_msg;
                yaw_msg.yaw_rate = yawrate;
                SynSensor::Ptr sensor_ptr(new SynSensor(SynSensorType::ANY, yaw_msg));
                sensor_ptr->setTimestamp(msgPtr->getTimestamp());
                sensor_ptr->setFrameID("yawrate");
                syn_ptr_->addData(sensor_ptr);
            }
        } else if (msgName == "ESP_0x295") {
            LX_CAN::ESP_0x295 *pMsg = dynamic_cast<LX_CAN::ESP_0x295 *>(msgPtr.get());
            if (pMsg != nullptr) {
                float speed = pMsg->ESP_VehicleSpeed / 3.6;

                SpeedMsg speed_msg;
                speed_msg.speed = speed;
                SynSensor::Ptr sensor_ptr(new SynSensor(SynSensorType::ANY, speed_msg));
                sensor_ptr->setFrameID("speed");
                sensor_ptr->setTimestamp(msgPtr->getTimestamp());
                syn_ptr_->addData(sensor_ptr);
            }
        }
    }

    inline void can_msg_callback(const perception_msg::can_msg &data) {
        long canId = data.can_id.data;
        unsigned int size = data.can_dlc.data;
        std::vector<unsigned char> canData;
        canData.resize(size);
        memcpy(canData.data(), data.can_data.data(), canData.size() * sizeof(unsigned char));

        RSCanFrame rs_frame = can_frame_parse(canId, canData.size(), canData.data());
        Basic_Msg::Ptr msgPtr = LX_CAN::can_lx_frame_process(rs_frame);
        msgPtr->setTimestamp(data.header.stamp.toSec());
        recvCallback(msgPtr);
    }

    RsSynchronizer::Ptr syn_ptr_;
    CanData::Ptr can_msg_ptr_;

    std::mutex mutex_;
    std::vector<std::function<void(const AnySensor::Ptr &msg_ptr)> > syn_cb_list_;

};

RS_REGISTER_SENSOR(CanLxoneRosSensor)

}  // namespace sensor
}  // namespace robosense

#endif  // RS_ROS_FOUND

#endif  // RS_SENSOR_CAN_LXONE_ROS_CAN_LXONE_ROS_H
