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
#ifndef RS_SENSOR2_COMMON_ANY_SENSOR_H
#define RS_SENSOR2_COMMON_ANY_SENSOR_H

#include <map>
#include "rs_sensor2/common/data_type/can_data.h"
#include "rs_common/external/rs_logger.h"
#include "rs_common/external/rs_exceptions.h"
#include "rs_common/external/rs_any.h"

namespace robosense {
namespace sensor {

enum class AnySensorType {
    CAN = 1,
};

/**
 * AnySensorType and typeid map
 */
const std::map<AnySensorType, const char *> kAnySensorType2TypeIdMap = {
{AnySensorType::CAN, typeid(CanData::Ptr).name()},
};

struct AnySensor {
    using Ptr = std::shared_ptr<AnySensor>;

    template<typename T>
    explicit AnySensor(const AnySensorType &type_, const T &any_) {
        if (kAnySensorType2TypeIdMap.at(type_) != typeid(any_).name()) {
            RERROR << name() << ": Construct value not fit AnySensorType !";
            RS_THROW("construct error!");
        }
        type = type_;
        msg_ptr.reset(new Any(any_));
    }

    std::string name() {
        return "AnySensor";
    }

    inline const double &getTimestamp() const {
        return stamp;
    }

    inline const std::string &getFrameID() const {
        return frame_id;
    }

    inline const AnySensorType &getSensorType() const {
        return type;
    }

    template<typename T>
    inline T getSensor() {
        return *msg_ptr->AnyCast<T>();
    }

    inline void setTimestamp(const double &stamp_) {
        stamp = stamp_;
    }

    inline void setFrameID(const std::string &frame_id_) {
        frame_id = frame_id_;
    }

private:

    AnySensorType type;
    double stamp;
    std::string frame_id;
    Any::Ptr msg_ptr = nullptr;
};

}  // namespace sensor
}  // namespace robosense

#endif  // RS_SENSOR2_COMMON_ANY_SENSOR_H
