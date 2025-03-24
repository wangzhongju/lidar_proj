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

#ifndef RS_APPLICATION_STRATEGY_SMARTSENSOR_APPLICATION_H_
#define RS_APPLICATION_STRATEGY_SMARTSENSOR_APPLICATION_H_

#include "rs_application/base_application.h"

namespace robosense {

class SmartSensorApplication: public BaseRsApplication {
public:
    using Ptr = std::shared_ptr<SmartSensorApplication>;

    ~SmartSensorApplication() {
        stop();
    }

    // load configures from yaml and init the perception strategy.
    // input: yaml node
    void init(const RsYamlNode& config) override;

    // start the thread and wait for data.
    void start() override;

    // stop the thread.
    void stop() override;

private:
    std::string name() {
        return "SmartSensorApplication";
    }
    // load configures from yaml and init the sensor.
    // input: yaml node
    void initSensorManager(const RsYamlNode& config) override;

    // load configures from yaml and init data preprocessing method.
    // Mainly process the data parsed from the sensor, including data synchronization, update timestamp, etc.
    // input: yaml node
    void initPreprocessing(const RsYamlNode& config) override;

    // load configures from yaml and init the detail of perception.
    // input: yaml node
    void initPerception(const RsYamlNode& config) override;
};

}  // namespace robosense

#endif  // RS_APPLICATION_STRATEGY_SMARTSENSOR_APPLICATION_H_
