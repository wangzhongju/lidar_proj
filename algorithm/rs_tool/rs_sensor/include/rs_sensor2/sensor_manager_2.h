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
#ifndef RS_SENSOR2_SENSOR_MANAGER_2_H
#define RS_SENSOR2_SENSOR_MANAGER_2_H

#include "rs_sensor2/common/base_sensor.h"

namespace robosense {
namespace sensor {

class SensorManager2 {
public:
    using Ptr = std::shared_ptr<SensorManager2>;

    explicit SensorManager2() = default;

    inline void init(const RsYamlNode &config_node) {

        {  // can
            RsYamlNode can_node;
            bool flag = rsYamlSubNode(config_node, "can", can_node);
            if (flag) {
                std::string msg_source = "";
                std::string tag = "";
                rsYamlRead(can_node, "msg_source", msg_source);
                rsYamlRead(can_node, "tag", tag);

                std::string strategy = "Can" + tag + "" + msg_source + "Sensor";
                if (!BaseSensorRegisterer::IsValid(strategy)) {
                    RERROR << name() << ": parse strategy " << strategy << " not registered!";
                    RS_THROW("no register!");
                }
                RTRACE << name() << ": parse strategy " << strategy << " init succeed!";
                BaseSensor::Ptr impl_ptr;
                impl_ptr.reset(BaseSensorRegisterer::getInstanceByName(strategy));
                impl_ptr->init(config_node);
                sensor_ptr_vec.push_back(impl_ptr);
            }
        }

        {
            // external pose
            RsYamlNode pose_node;
            bool flag = rsYamlSubNode(config_node, "external_pose", pose_node);
            if (flag) {
                std::string msg_source = "";
                rsYamlRead(pose_node, "msg_source", msg_source);
                std::string strategy = msg_source + "PoseSensor";
                if (!BaseSensorRegisterer::IsValid(strategy)) {
                    RERROR << name() << ": pose strategy " << strategy << " not registered!";
                    RS_THROW("no register!");
                }
                RTRACE << name() << ": pose strategy " << strategy << " init succeed!";
                BaseSensor::Ptr impl_ptr;
                impl_ptr.reset(BaseSensorRegisterer::getInstanceByName(strategy));
                impl_ptr->init(config_node);
                sensor_ptr_vec.push_back(impl_ptr);
            }
        }
    }

    inline void start() {
        for (size_t i = 0; i < sensor_ptr_vec.size(); ++i) {
            sensor_ptr_vec[i]->start();
        }
    }

    inline void stop() {
        for (size_t i = 0; i < sensor_ptr_vec.size(); ++i) {
            sensor_ptr_vec[i]->stop();
        }
    }

    inline void regRecvCallback(const std::function<void(const AnySensor::Ptr &)> &cb) {
        for (size_t i = 0; i < sensor_ptr_vec.size(); ++i) {
            sensor_ptr_vec[i]->regRecvCallback(cb);
        }
    }

private:

    inline std::string name() {
        return "SensorManager2";
    }

    std::vector<BaseSensor::Ptr> sensor_ptr_vec;

};

}  // namespace sensor
}  // namespace robosense

#endif  // RS_SENSOR2_SENSOR_MANAGER_2_H
