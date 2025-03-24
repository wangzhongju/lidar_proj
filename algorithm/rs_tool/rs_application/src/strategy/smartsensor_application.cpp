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

#include "rs_application/strategy/smartsensor_application.h"

namespace robosense {

void SmartSensorApplication::init(const RsYamlNode& config) {
    initSensorManager(config);
    initPreprocessing(config);

    RsYamlNode general_node;
    rsYamlSubNode(config, "general", general_node);
    rsYamlRead(general_node, "run_perception", run_perception_);
    rsYamlRead(general_node, "run_rviz_display", run_rviz_display_);
    rsYamlRead(general_node, "run_communication", run_communication_);
    if (run_perception_) {
        initPerception(config);
    }
}

void SmartSensorApplication::initSensorManager(const RsYamlNode& config) {
    RsYamlNode sensor_config_node;
    rsYamlSubNode(config, "sensor", sensor_config_node);
    sensor_ptr_.reset(new sensor::SensorManager);
    sensor_ptr_->init(sensor_config_node);

    sensor_2_ptr_.reset(new sensor::SensorManager2);
    sensor_2_ptr_->init(sensor_config_node);
}

void SmartSensorApplication::initPreprocessing(const RsYamlNode& config) {
    RsYamlNode preprocessing_node;
    rsYamlSubNode(config, "preprocessing", preprocessing_node);
    preprocessing_ptr_.reset(new preprocessing::RsPreprocessing);
    preprocessing_ptr_->init(preprocessing_node);

    if (preprocessing_ptr_->hasLidar()) {
        auto func = [this](const lidar::LidarPointCloudMsg &lidar_msg_ptr) {
            RTRACE << name() << ": get lidar data of " << lidar_msg_ptr.frame_id;

            const auto& cloud_ptr = lidar_msg_ptr.point_cloud_ptr;
            RsPointCloudGPT::Ptr scan_ptr(new RsPointCloudGPT);
            scan_ptr->height = cloud_ptr->height;
            scan_ptr->width = cloud_ptr->width;
            scan_ptr->resize(cloud_ptr->height * cloud_ptr->width);
            memcpy(scan_ptr->points.data(), cloud_ptr->points.data(),
                   cloud_ptr->height * cloud_ptr->width * sizeof(RsPoint));

            SynSensor::Ptr sensor_ptr(new SynSensor(SynSensorType::LIDAR, scan_ptr));
            sensor_ptr->setFrameID(lidar_msg_ptr.frame_id);
            sensor_ptr->setTimestamp(lidar_msg_ptr.timestamp);

            this->preprocessing_ptr_->addData(sensor_ptr);
        };
        sensor_ptr_->regRecvCallback(func);
    }

    if (preprocessing_ptr_->hasOdometry()) {
        auto func = [this](const sensor::AnySensor::Ptr &sensor_ptr) {
            RTRACE << name() << ": get odometry data!";
            if (sensor_ptr->getSensorType() == sensor::AnySensorType::CAN) {
                RsPose::Ptr pose_ptr(new RsPose);
                *pose_ptr = *(sensor_ptr->getSensor<sensor::CanData::Ptr>()->pose_ptr);
                SynSensor::Ptr syn_sensor_ptr(new SynSensor(SynSensorType::POSE, pose_ptr));
                syn_sensor_ptr->setFrameID(sensor_ptr->getFrameID());
                syn_sensor_ptr->setTimestamp(sensor_ptr->getTimestamp());
                this->preprocessing_ptr_->addData(syn_sensor_ptr);
            }
        };
        sensor_2_ptr_->regRecvCallback(func);
    }

    RsYamlNode result_sender_node;
    rsYamlSubNode(preprocessing_node, "result_sender", result_sender_node);
    send_cloud_.reset(new preprocessing::CloudSender);
    send_cloud_->init(result_sender_node, preprocessing_ptr_);
    auto send_lidar = [this](const std::vector<SynSensor::Ptr> msg_ptr_vec) {
        if (this->preprocessing_ptr_->getFusionData() != nullptr) {
            this->send_cloud_->addData(this->preprocessing_ptr_->getFusionData());
        }
    };
    preprocessing_ptr_->regSynCallback(send_lidar);
}

void SmartSensorApplication::initPerception(const RsYamlNode& config) {
    RsYamlNode perception_node;
    rsYamlSubNode(config, "perception", perception_node);
    perception_ptr_.reset(new perception::Perception);
    perception_ptr_->init(perception_node);
    auto func = [this](const std::vector<SynSensor::Ptr>& msg_ptr_vec){
        RTRACE << name() << ": sychronize succeed!";
        const auto& calib_map = this->preprocessing_ptr_->getSensorCalib();
        perception::RsPerceptionMsg::Ptr msg_ptr(new perception::RsPerceptionMsg);
        for (size_t i = 0; i < msg_ptr_vec.size(); ++i) {
            const auto& frame_id = msg_ptr_vec[i]->getFrameID();
            if (calib_map.find(frame_id) == calib_map.end()) {
                continue;
            }
            if (calib_map.at(frame_id).sensor_type == SynSensorType::LIDAR) {
                perception::RsLidarFrameMsg::Ptr lidar_msg_ptr(new perception::RsLidarFrameMsg);
                lidar_msg_ptr->frame_id = frame_id;
                lidar_msg_ptr->timestamp = msg_ptr_vec[i]->getTimestamp();
                lidar_msg_ptr->scan_ptr = msg_ptr_vec[i]->getSensor<RsPointCloudGPT::Ptr>();
                msg_ptr->sub_lidar_msgs_map[frame_id] = lidar_msg_ptr;
            }
        }
        if (preprocessing_ptr_->hasOdometry()) {
            RsPose::Ptr pose_ptr;
            for (size_t i = 0; i < msg_ptr_vec.size(); ++i) {
                const auto& frame_id = msg_ptr_vec[i]->getFrameID();
                if (calib_map.find(frame_id) == calib_map.end()) {
                    continue;
                }
                if (calib_map.at(frame_id).sensor_type == SynSensorType::POSE) {
                    pose_ptr = msg_ptr_vec[i]->getSensor<RsPose::Ptr>();
                    break;
                }
            }
            for (auto itr = msg_ptr->sub_lidar_msgs_map.begin(); itr != msg_ptr->sub_lidar_msgs_map.end(); ++itr) {
                *itr->second->global_pose_ptr = *pose_ptr;
            }
        }
        this->perception_ptr_->addData(msg_ptr);
    };
    preprocessing_ptr_->regSynCallback(func);

    addFuncInPerception(perception_node);

    if (run_communication_) {
        RsYamlNode communication_node;
        rsYamlSubNode(config, "communication", communication_node);
        sender_ptr_.reset(new perception::Sender);
        sender_ptr_->init(communication_node);

        auto func_communication = [this](const perception::RsPerceptionMsg::Ptr& msg_ptr) {
            this->sender_ptr_->send(msg_ptr);
        };
        perception_ptr_->regPerceptionCallback(func_communication);
    }
}

void SmartSensorApplication::start() {
    sensor_ptr_->start();
    sensor_2_ptr_->start();
    preprocessing_ptr_->start();
    if (run_perception_) {
        perception_ptr_->start();
#ifdef RS_ROS_FOUND
        if (run_rviz_display_) {
            rviz_display_ptr_->start();
        }
#endif  // RS_ROS_FOUND
    }
    send_cloud_->start();
}

void SmartSensorApplication::stop() {
    sensor_ptr_->stop();
    sensor_2_ptr_->stop();
    preprocessing_ptr_->stop();
    if (run_perception_) {
        perception_ptr_->stop();
#ifdef RS_ROS_FOUND
        if (run_rviz_display_) {
            rviz_display_ptr_->stop();
        }
#endif  // RS_ROS_FOUND
    }
    send_cloud_->stop();
}

RS_REGISTER_APPLICATION(SmartSensorApplication)

}  // namespace robosense
