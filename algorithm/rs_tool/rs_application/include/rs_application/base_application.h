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

#ifndef RS_APPLICATION_BASE_APPLICATION_H_
#define RS_APPLICATION_BASE_APPLICATION_H_

#include "rs_common/external/common.h"

#include "rs_sensor/manager.h"
#include "rs_sensor2/sensor_manager_2.h"
#include "rs_preprocessing/external/preprocessing.h"
#include "rs_perception/perception/external/perception.h"
#include "rs_perception/communication/external/sender.h"
#include "rs_application/utils/perception_save_result.h"
#include "rviz_display/external/rviz_display.h"
#include "rs_preprocessing/sender/external/cloud_sender.h"

namespace robosense {

class BaseRsApplication {
public:
    using Ptr = std::shared_ptr<BaseRsApplication>;

    virtual ~BaseRsApplication() = default;

    virtual void init(const RsYamlNode& config) = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

protected:
    virtual void initSensorManager(const RsYamlNode& config) = 0;
    virtual void initPreprocessing(const RsYamlNode& config) = 0;
    virtual void initPerception(const RsYamlNode& config) = 0;

    std::unique_ptr<sensor::SensorManager2> sensor_2_ptr_;
    std::unique_ptr<sensor::SensorManager> sensor_ptr_;
    preprocessing::RsPreprocessing::Ptr preprocessing_ptr_;
    perception::Perception::Ptr perception_ptr_;
    perception::Sender::Ptr sender_ptr_;
    PerceptionSaveResult::Ptr save_result_ptr_;
    preprocessing::CloudSender::Ptr send_cloud_ = nullptr;

#ifdef RS_ROS_FOUND
    perception::RvizDisplay::Ptr rviz_display_ptr_;
#endif  // RS_ROS_FOUND

    bool run_perception_ = false;
    bool run_rviz_display_ = true;
    bool run_communication_ = false;
    bool is_v2r = false;
    RsPose::Ptr V2r_global_pose_;

    // init save result function and rviz display function according to the configures.
    // input: yaml node
    inline void addFuncInPerception(const RsYamlNode& perception_node) {
        RsYamlNode save_result_node;
        rsYamlSubNode(perception_node, "save_result", save_result_node);
        save_result_ptr_.reset(new PerceptionSaveResult);
        save_result_ptr_->init(save_result_node);

        auto func_save = [this](const perception::RsPerceptionMsg::Ptr& msg_ptr) {
            this->save_result_ptr_->save(msg_ptr);
        };
        perception_ptr_->regPerceptionCallback(func_save);
#ifdef RS_ROS_FOUND
        if (run_rviz_display_) {
            RsYamlNode rviz_display_node;
            rsYamlSubNode(perception_node, "rviz", rviz_display_node);
            rviz_display_ptr_.reset(new perception::RvizDisplay);
            if (is_v2r) {
                addV2rglobalPose(perception_node);
                rviz_display_ptr_->addV2rPose(V2r_global_pose_);
            }
            rviz_display_ptr_->init(rviz_display_node);

            auto func_rviz = [this](const perception::RsPerceptionMsg::Ptr& msg_ptr) {
                this->rviz_display_ptr_->addData(msg_ptr);
            };
            perception_ptr_->regPerceptionCallback(func_rviz);
        }
#endif  // RS_ROS_FOUND
    }
    // this function can deliver the global pose to rviz display function
    // when select "V2R" strategy.
    // input: yaml node
    inline void addV2rglobalPose(const RsYamlNode& config_node) {
        RsYamlNode v2r_node;
        rsYamlSubNode(config_node, "V2rPerception", v2r_node);
        // init global pose
        RsYamlNode global_pose_node;
        rsYamlSubNode(v2r_node, "global_pose", global_pose_node);
        V2r_global_pose_.reset(new RsPose);
        V2r_global_pose_->load(global_pose_node);
    }
};

RS_REGISTER_REGISTERER(BaseRsApplication);
#define RS_REGISTER_APPLICATION(name)  \
RS_REGISTER_CLASS(BaseRsApplication, name)

}  // namespace robosense

#endif  // RS_APPLICATION_BASE_APPLICATION_H_
