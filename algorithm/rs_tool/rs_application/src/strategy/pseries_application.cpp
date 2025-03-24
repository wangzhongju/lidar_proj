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

#include "rs_application/strategy/pseries_application.h"

namespace robosense {

void PseriesApplication::init(const RsYamlNode& config) {
    initSensorManager(config);
    initPreprocessing(config);

    RsYamlNode general_node;
    rsYamlSubNode(config, "general", general_node);
    rsYamlRead(general_node, "run_perception", run_perception_);
    rsYamlRead(general_node, "run_rviz_display", run_rviz_display_);
    rsYamlRead(general_node, "run_communication", run_communication_);
    rsYamlRead(general_node, "run_localization", run_localization_);
#ifdef COMPILE_LOCALIZATION
    if (run_localization_) {
        initLocalization(config);
    }
#endif  // COMPILE_LOCALIZATION
    if (run_perception_) {
        initPerception(config);
    }
}

void PseriesApplication::initSensorManager(const RsYamlNode& config) {
    RsYamlNode sensor_config_node;
    rsYamlSubNode(config, "sensor", sensor_config_node);
    sensor_ptr_.reset(new sensor::SensorManager);
    sensor_ptr_->init(sensor_config_node);

    sensor_2_ptr_.reset(new sensor::SensorManager2);
    sensor_2_ptr_->init(sensor_config_node);
}

void PseriesApplication::initPreprocessing(const RsYamlNode& config) {
    RsYamlNode preprocessing_node;
    rsYamlSubNode(config, "preprocessing", preprocessing_node);
    preprocessing_ptr_.reset(new preprocessing::RsPreprocessing);
    preprocessing_ptr_->init(preprocessing_node);

    if (preprocessing_ptr_->hasLidar()) {
        auto func = [this](const lidar::LidarPointCloudMsg &lidar_msg_ptr) {
            RTRACE << name() << ": get lidar data of " << lidar_msg_ptr.frame_id;
            RsPointCloudGPT::Ptr scan_ptr(new RsPointCloudGPT);
            scan_ptr->height = lidar_msg_ptr.height;
            scan_ptr->width = lidar_msg_ptr.width;
            scan_ptr->resize(lidar_msg_ptr.height * lidar_msg_ptr.width);
            memcpy(scan_ptr->points.data(), lidar_msg_ptr.point_cloud_ptr->points.data(),
                   lidar_msg_ptr.height * lidar_msg_ptr.width * sizeof(RsPoint));

            SynSensor::Ptr sensor_ptr(new SynSensor(SynSensorType::LIDAR, scan_ptr));
            sensor_ptr->setFrameID(lidar_msg_ptr.frame_id);
            sensor_ptr->setTimestamp(lidar_msg_ptr.timestamp);
            this->preprocessing_ptr_->addData(sensor_ptr);
        };
        sensor_ptr_->regRecvCallback(func);
    }
    if (preprocessing_ptr_->hasOdometry() && !run_localization_) {
        auto func = [this](const sensor::AnySensor::Ptr & sensor_ptr) {
            if (sensor_ptr->getSensorType() == sensor::AnySensorType::CAN) {
                RTRACE << name() << ": get odometry data!";
                RsPose::Ptr pose_ptr(new RsPose);
                *pose_ptr = *(sensor_ptr->getSensor<sensor::CanData::Ptr>()->pose_ptr);
                SynSensor::Ptr syn_sensor_ptr(new SynSensor(SynSensorType::POSE, pose_ptr));
                syn_sensor_ptr->setTimestamp(sensor_ptr->getTimestamp());
                syn_sensor_ptr->setFrameID(sensor_ptr->getFrameID());
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

void PseriesApplication::initPerception(const RsYamlNode& config) {
    RsYamlNode perception_node;
    rsYamlSubNode(config, "perception", perception_node);
    perception_ptr_.reset(new perception::Perception);
    perception_ptr_->init(perception_node);
    const auto calib_map = preprocessing_ptr_->getSensorCalib();
    for(auto itr = calib_map.begin(); itr != calib_map.end(); ++itr) {
        RINFO << "frame id " << itr->first;
        RINFO << itr->second.sensor_to_vehicle_pose;
    }

    if (preprocessing_ptr_->hasOdometry() && !run_localization_) {
        // 使用can pose
        auto func_with_odometry = [this](const std::vector<SynSensor::Ptr>& msg_ptr_vec) {
            RTRACE << name() << ": sychronize succeed!";
            const auto& calib_map = this->preprocessing_ptr_->getSensorCalib();
            perception::RsPerceptionMsg::Ptr msg_ptr(new perception::RsPerceptionMsg);
            RsPose::Ptr pose_ptr;
            for (size_t i = 0; i < msg_ptr_vec.size(); ++i) {
                const auto& frame_id = msg_ptr_vec[i]->getFrameID();
                if (calib_map.find(frame_id) == calib_map.end()) {
                    continue;
                }
                if (calib_map.at(frame_id).sensor_type == SynSensorType ::LIDAR) {
                    perception::RsLidarFrameMsg::Ptr lidar_msg_ptr(new perception::RsLidarFrameMsg);
                    lidar_msg_ptr->frame_id = frame_id;
                    lidar_msg_ptr->timestamp = msg_ptr_vec[i]->getTimestamp();
                    lidar_msg_ptr->scan_ptr = msg_ptr_vec[i]->getSensor<RsPointCloudGPT::Ptr>();
                    msg_ptr->sub_lidar_msgs_map[frame_id] = lidar_msg_ptr;
                }
                if (calib_map.at(frame_id).sensor_type == SynSensorType::POSE) {
                    pose_ptr = msg_ptr_vec[i]->getSensor<RsPose::Ptr>();
                }
            }

            for (auto itr = msg_ptr->sub_lidar_msgs_map.begin(); itr != msg_ptr->sub_lidar_msgs_map.end(); ++itr) {
                *itr->second->global_pose_ptr = *pose_ptr;
            }
            this->perception_ptr_->addData(msg_ptr);
        };
        this->preprocessing_ptr_->regSynCallback(func_with_odometry);
    }
    else if (run_localization_) {
        // 使用定位 global pose
#ifdef COMPILE_LOCALIZATION
        auto func_with_loc = [this](const std::vector<SynSensor::Ptr>& msg_ptr_vec) {
            RTRACE << name() << ": sychronize succeed!";
            const auto& calib_map = this->preprocessing_ptr_->getSensorCalib();
            perception::RsPerceptionMsg::Ptr msg_ptr(new perception::RsPerceptionMsg);
            RsPose::Ptr pose_ptr(new RsPose);
            RsVector3d gps_origin_;
            double latest_time = this->preprocessing_ptr_->getMainData()->timestamp;
            for (size_t i = 0; i < msg_ptr_vec.size(); ++i) {
                const auto& frame_id = msg_ptr_vec[i]->getFrameID();
                if (calib_map.find(frame_id) == calib_map.end()) {
                    continue;
                }
                if (calib_map.at(frame_id).sensor_type == SynSensorType ::LIDAR) {
                    perception::RsLidarFrameMsg::Ptr lidar_msg_ptr(new perception::RsLidarFrameMsg);
                    lidar_msg_ptr->frame_id = frame_id;
                    lidar_msg_ptr->timestamp = msg_ptr_vec[i]->getTimestamp();
                    lidar_msg_ptr->scan_ptr = msg_ptr_vec[i]->getSensor<RsPointCloudGPT::Ptr>();
                    msg_ptr->sub_lidar_msgs_map[frame_id] = lidar_msg_ptr;
                    latest_time = lidar_msg_ptr->timestamp;
                }
            }

            auto status_ = this->localization_ptr_->getModuleStatus().localization_status;

            if (status_ == localization::loc_status::LOC_NORMAL) {
                localization::VehicleStateMsg cur_state;
                if (this->localization_ptr_->getVehicleState(cur_state, latest_time) == common::ErrCode_Success) {
                    pose_ptr->x = static_cast<float >(cur_state.pos[0]);
                    pose_ptr->y = static_cast<float >(cur_state.pos[1]);
                    pose_ptr->z = static_cast<float >(cur_state.pos[2]);
                    pose_ptr->roll = static_cast<float >(cur_state.orien[0]);
                    pose_ptr->pitch = static_cast<float >(cur_state.orien[1]);
                    pose_ptr->yaw = static_cast<float >(cur_state.orien[2]);
                    gps_origin_.x = cur_state.origin[0];
                    gps_origin_.y = cur_state.origin[1];
                    gps_origin_.z = cur_state.origin[2];
                }
            }

            for (auto itr = msg_ptr->sub_lidar_msgs_map.begin(); itr != msg_ptr->sub_lidar_msgs_map.end(); ++itr) {
                *itr->second->global_pose_ptr = *pose_ptr;
                itr->second->gps_origin = gps_origin_;
            }
            this->perception_ptr_->addData(msg_ptr);
        };
        this->preprocessing_ptr_->regSynCallback(func_with_loc);
#endif  // COMPILE_LOCALIZATION
    }
    else {
        // 不使用 global pose
        auto func = [this](const std::vector<SynSensor::Ptr>& msg_ptr_vec){
            RTRACE << name() << ": sychronize succeed!";
            const auto& calib_map = this->preprocessing_ptr_->getSensorCalib();
            perception::RsPerceptionMsg::Ptr msg_ptr(new perception::RsPerceptionMsg);

            for (size_t i = 0; i < msg_ptr_vec.size(); ++i) {
                const auto& frame_id = msg_ptr_vec[i]->getFrameID();
                if (calib_map.at(frame_id).sensor_type == SynSensorType ::LIDAR) {
                    perception::RsLidarFrameMsg::Ptr lidar_msg_ptr(new perception::RsLidarFrameMsg);
                    lidar_msg_ptr->frame_id = frame_id;
                    lidar_msg_ptr->timestamp = msg_ptr_vec[i]->getTimestamp();
                    lidar_msg_ptr->scan_ptr = msg_ptr_vec[i]->getSensor<RsPointCloudGPT::Ptr>();
                    msg_ptr->sub_lidar_msgs_map[frame_id] = lidar_msg_ptr;
                }
            }
            this->perception_ptr_->addData(msg_ptr);
        };
        preprocessing_ptr_->regSynCallback(func);
    }

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
#ifdef COMPILE_LOCALIZATION
void PseriesApplication::initLocalization(const RsYamlNode &config) {
    std::string path = (std::string)RELEASE_PROJECT_PATH + "/config/system_config";
    RsYamlNode localization_node, localization_common_node;
    rsYamlSubNode(config, "localization", localization_node);
    rsYamlSubNode(localization_node, "common", localization_common_node);
    bool use_fusion_cloud = false;
    pose_guess.reset(new RsPose);
    rsYamlRead(localization_common_node, "use_fusion_cloud", use_fusion_cloud);
    rsYamlRead(localization_common_node, "localization_mode", localization_mode_);
    rsYamlRead(localization_common_node, "pose_guess_0", pose_guess->x);
    rsYamlRead(localization_common_node, "pose_guess_1", pose_guess->y);
    rsYamlRead(localization_common_node, "pose_guess_2", pose_guess->z);
    if (use_fusion_cloud) {
        preprocessing_ptr_->setLidarDataFusion(use_fusion_cloud);
    }

    localization_ptr_.reset(localization::LocalizationFactory::create());
    localization_ptr_->regExceptionCallback(
    std::bind(&PseriesApplication::localExceptionCallback, this, std::placeholders::_1));
    localization_ptr_->init(path, localization_node);
    localization_ptr_->setCalibrationMap(this->preprocessing_ptr_->getSensorCalib());

    auto func_lidar = [this, use_fusion_cloud](const std::vector<SynSensor::Ptr> msg_ptr_vec) {
        if (use_fusion_cloud) {
            this->localization_ptr_->lidarCallback(*this->preprocessing_ptr_->getFusionData());
        } else {
            this->localization_ptr_->lidarCallback(*this->preprocessing_ptr_->getMainData());
        }
    };
    preprocessing_ptr_->regSynCallback(func_lidar);
    auto func_imu = [this](const imu::ImuMsg &imu_msg) {
        RTRACE << name() << ": get imu data of " << imu_msg.frame_id;
        this->localization_ptr_->imuCallback(imu_msg);
    };
    sensor_ptr_->regRecvCallback(func_imu);
    auto func_gnss = [this](const gnss::GnssMsg &gnss_msg) {
        RTRACE << name() << ": get gnss data of " << gnss_msg.frame_id;
        this->localization_ptr_->gnssCallback(gnss_msg);
    };
    sensor_ptr_->regRecvCallback(func_gnss);
    auto func_odom = [this](const odom::OdomMsg &odom_msg) {
        RTRACE << name() << ": get odom data of " << odom_msg.frame_id;
        this->localization_ptr_->odomCallback(odom_msg);
    };
    sensor_ptr_->regRecvCallback(func_odom);

    RsYamlNode result_sender_node;
    rsYamlSubNode(localization_node, "result_sender", result_sender_node);

    localization_sender_ptr_.reset(new localization::LocalizationSender);
    localization_sender_ptr_->init(result_sender_node, localization_ptr_);

}
#endif  // COMPILE_LOCALIZATION

void PseriesApplication::start() {
    sensor_ptr_->start();
    sensor_2_ptr_->start();
    preprocessing_ptr_->start();
    if (run_perception_) {
        perception_ptr_->start();
#ifdef RS_ROS_FOUND
        if (run_rviz_display_) {
            rviz_display_ptr_->start();
        }
#endif   // RS_ROS_FOUND
    }
    send_cloud_->start();
#ifdef COMPILE_LOCALIZATION
    if (run_localization_) {
        localization_ptr_->start(localization_mode_, pose_guess);
        localization_sender_ptr_->start();
    }
#endif  // COMPILE_LOCALIZATION
}

void PseriesApplication::stop() {
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
#ifdef COMPILE_LOCALIZATION
    if (run_localization_) {
        localization_ptr_->stop();
        localization_sender_ptr_->stop();
    }
#endif  // COMPILE_LOCALIZATION
}

RS_REGISTER_APPLICATION(PseriesApplication)

}  // namespace robosense
