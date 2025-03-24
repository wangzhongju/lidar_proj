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
#ifndef RS_SENSOR2_COMMON_BASE_SENSOR_H
#define RS_SENSOR2_COMMON_BASE_SENSOR_H

#include <memory>
#include <functional>
#include "rs_common/external/basic_type/rs_yaml_node.h"
#include "rs_common/external/register/register.h"
#include "rs_sensor2/common/any_sensor.h"

namespace robosense {
namespace sensor {

class BaseSensor {
public:
    using Ptr = std::shared_ptr<BaseSensor>;

    virtual ~BaseSensor() = default;

    virtual void init(const RsYamlNode &config_node) = 0;

    virtual void start() = 0;

    virtual void stop() = 0;

    virtual void regRecvCallback(const std::function<void(const AnySensor::Ptr &)> &cb) = 0;
};

RS_REGISTER_REGISTERER(BaseSensor);
#define RS_REGISTER_SENSOR(name)  \
RS_REGISTER_CLASS(BaseSensor, name)

}  // namespace sensor
}  // namespace robosense

#endif  // RS_SENSOR2_COMMON_BASE_SENSOR_H
