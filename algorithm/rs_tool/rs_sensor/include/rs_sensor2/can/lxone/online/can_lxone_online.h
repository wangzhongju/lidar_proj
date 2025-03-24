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
#ifndef RS_SENSOR2_CAN_LXONE_CAN_LXONE_ONLINE_H
#define RS_SENSOR2_CAN_LXONE_CAN_LXONE_ONLINE_H

#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_KVASER_CAN_FOUND

#include "rs_sensor2/common/base_sensor.h"
#include "rs_common/external/synchronize/rs_synchronizer.h"
#include "rs_sensor2/can/lxone/lx_can_frame_parse.h"
#include "rs_sensor2/can/common/io/kvaser_io.h"
#include "rs_sensor2/can/common/config/kvaser_config.h"
#include "rs_sensor2/can/common/can_parser.h"

#ifdef RS_ROS_FOUND

#include "rs_sensor2/can/msg/can_msg.h"

#endif  // RS_ROS_FOUND

namespace robosense {
namespace sensor {

class CanLxoneOnlineSensor : public BaseSensor {
public:
    using Ptr = std::shared_ptr<CanLxoneOnlineSensor>;

    explicit CanLxoneOnlineSensor() {
        can_msg_ptr_.reset(new CanData);
    }

    void init(const RsYamlNode &config_node) override {
        std::string odom_frame_id = "odometry";
        RsYamlNode calibration_node, odom_node;
        rsYamlSubNode(config_node, "calibration", calibration_node);
        rsYamlSubNode(calibration_node, "odometry", odom_node);
        rsYamlRead(odom_node, "frame_id", odom_frame_id);

        RsYamlNode can_node;
        rsYamlSubNode(config_node, "can", can_node);
        rsYamlRead(can_node, "send_ros_msg", send_ros_msg_);
        std::string ros_send_topic_ = "/can_data";
        rsYamlRead(can_node, "ros_send_topic", ros_send_topic_);

        if (send_ros_msg_) {
#ifdef RS_ROS_FOUND
            nh_ptr_.reset(new ros::NodeHandle);
            can_msg_pub_ = nh_ptr_->advertise<perception_msg::can_msg>(ros_send_topic_ , 1);
#endif  // RS_ROS_FOUND
        }

        RsSynchronizerOptions init_options;
        init_options.frame_id_vec.resize(2);
        init_options.frame_id_vec[0] = "speed";
        init_options.frame_id_vec[1] = "yawrate";
        syn_ptr_.reset(new RsSynchronizer(init_options));
        {
            auto func = [this, odom_frame_id](const std::vector<SynSensor::Ptr>& msg_ptr_vec){
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
                    const auto& pose_ptr = this->can_msg_ptr_->pose_ptr;
                    float cur_yaw = pose_ptr->yaw + yaw_rate * delta_time;
                    pose_ptr->x = pose_ptr->x + speed * delta_time * std::cos(pose_ptr->yaw + (cur_yaw - pose_ptr->yaw) / 2.);
                    pose_ptr->y = pose_ptr->y + speed * delta_time * std::sin(pose_ptr->yaw + (cur_yaw - pose_ptr->yaw) / 2.);
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

        canIoPtr_.reset(new RSKvaserCanIO());

        int kvaserChannle = 0;
        rsYamlRead(config_node, "kvaser_channel", kvaserChannle);

        RSKvaserCanConfig::Ptr kvaserCanConfigPtr(new RSKvaserCanConfig);
        RsYamlNode kvaser_nodes;

        RSSingleKvaserCanConfig singleChannleConfig;
        singleChannleConfig.kvaserChannle = kvaserChannle;
        singleChannleConfig.kvaserOpenMode = canOPEN_EXCLUSIVE;
        singleChannleConfig.kvaserFreq = canBITRATE_500K;
        singleChannleConfig.kvaserControlMode = canDRIVER_NORMAL;
        singleChannleConfig.affinity_id = 4;
        kvaserCanConfigPtr->kvaserCanConfigs.push_back(singleChannleConfig);

        basicConfigPtr_ = kvaserCanConfigPtr;

        // 注册回调函数
        auto exceptCb = std::bind(&CanLxoneOnlineSensor::exceptCallback, this, std::placeholders::_1, std::placeholders::_2);
        canIoPtr_->can_register_exception(exceptCb);
        auto recvNativeCb = std::bind(&CanLxoneOnlineSensor::recvNativeCallback, this, std::placeholders::_1, std::placeholders::_2,
                                      std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
        canIoPtr_->can_register_callback(recvNativeCb);

        int ret = canIoPtr_->can_init(basicConfigPtr_);

        if (ret != 0) {
            RERROR << name() << ": Initial Can Failed !";
            RS_THROW("init can failed!");
        }
    }

    inline void start() override {
        syn_ptr_->start();
        int ret = canIoPtr_->can_start();
        if (ret != 0) {
            RERROR << name() << ": Start Can Failed !";
            RS_THROW("can start failed!");
        }
    }

    inline void stop() override {
        syn_ptr_->stop();
        canIoPtr_->can_stop();
    }

    inline void regRecvCallback(const std::function<void(const AnySensor::Ptr &)> &cb) override {
        std::unique_lock<std::mutex> lock(mutex_);
        syn_cb_list_.emplace_back(cb);
    }

private:
    std::string name() {
        return "CanLxoneOnlineSensor";
    }

    bool send_ros_msg_ = false;
#ifdef RS_ROS_FOUND
    std::unique_ptr<ros::NodeHandle> nh_ptr_;
    ros::Publisher can_msg_pub_;
#endif  // RS_ROS_FOUND
    struct YawMsg {
        float yaw_rate = 0.;
    };
    struct SpeedMsg {
        float speed = 0.;
    };

    inline void recvCallback(const Basic_Msg::Ptr &msgPtr) {
        if (isAbnormal_ == true) {
            RERROR << name() << ": Can IO Exception ... ";
            return;
        }
        if (msgPtr == nullptr) {
            RERROR << name() << "rec empty message";
            return;
        }

        const std::string msgName = msgPtr->msgName();
        RTRACE << name() << ": Can Receive msgName = " << msgName;

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
                sensor_ptr->setTimestamp(getTime());
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
                sensor_ptr->setTimestamp(getTime());
                syn_ptr_->addData(sensor_ptr);
            }
        }
    }

    inline void recvNativeCallback(const std::string& vehicleType, long canId, 
                const std::vector<unsigned char>& canData, unsigned int canFlag, unsigned long canStamp) {
        std::lock_guard<std::mutex> lg(canMtx_);
        if (isAbnormal_ == true) {
            RERROR << name() << ": Can IO Exception ... ";
            return;
        }

#ifdef RS_ROS_FOUND 
        perception_msg::can_msg msg;
        if(isUseCanStamp_) {
            msg.header.stamp.fromNSec(canStamp);
        } else {
            msg.header.stamp.fromSec(getTime());
        }
        msg.header.seq = canFrameSeq_;
        msg.header.frame_id = canFrameId_;

        msg.vehicle_type.data = vehicleType;
        msg.can_id.data = canId;
        msg.can_dlc.data = canData.size();
        memcpy(msg.can_data.data(), canData.data(), canData.size() * sizeof(unsigned char));
        can_msg_pub_.publish(msg);

        RSCanFrame rs_frame = can_frame_parse(canId, canData.size(), canData.data());
        Basic_Msg::Ptr msgPtr = LX_CAN::can_lx_frame_process(rs_frame);
        recvCallback(msgPtr);
#endif // RS_ROS_FOUND
    }

    inline void exceptCallback(const int errNo, const std::string &errMsg) {
        RERROR << name() << ": errNo = " << errNo << ", errMsg = " << errMsg;

        std::lock_guard<std::mutex> lg(canMtx_);
        isAbnormal_ = true;
    }

    std::mutex canMtx_;
    bool isAbnormal_ = false;
    RSCanIOBasic::Ptr canIoPtr_;
    RSCanConfigBasic::Ptr basicConfigPtr_;
    int canFrameSeq_ = 0; 
    std::string canFrameId_ = "/can";
    // = true, use can's timestamp; = false, use system's timestamp, default is use system timestamp 
    bool isUseCanStamp_ = false;  
    
    RsSynchronizer::Ptr syn_ptr_;
    CanData::Ptr can_msg_ptr_;

    std::mutex mutex_;
    std::vector<std::function<void(const AnySensor::Ptr &msg_ptr)> > syn_cb_list_;
};

RS_REGISTER_SENSOR(CanLxoneOnlineSensor)

}  // namespace sensor
}  // namespace robosense

#endif  // RS_KVASER_CAN_FOUND

#endif  // RS_SENSOR2_CAN_LXONE_CAN_LXONE_ONLINE_H
