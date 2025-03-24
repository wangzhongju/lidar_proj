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
#ifndef RS_SENSOR_CAN_COMMON_BASE_BASE_CAN_MSG_H
#define RS_SENSOR_CAN_COMMON_BASE_BASE_CAN_MSG_H

#include <string>
#include <memory>

namespace robosense {
namespace sensor {

class Basic_Msg {
public:
    using Ptr = std::shared_ptr<Basic_Msg>;
    using ConstPtr = std::shared_ptr<const Basic_Msg>;

public:
    Basic_Msg() {
        Msg_Name = "Basic_Msg";
        Msg_Id = 0;
        timestamp = 0.0;
    }

    virtual ~Basic_Msg() {
    }

public:
    std::string msgName() {
        return Msg_Name;
    }

    unsigned int msgId() {
        return Msg_Id;
    }

    inline const double &getTimestamp() const {
        return timestamp;
    }

    inline void setTimestamp(const double& stamp_) {
        timestamp = stamp_;
    }

public:
    // 解析Can的标准帧
    virtual void parseSFF(const unsigned char *data) {
        //
    }

protected:
    std::string Msg_Name;
    double timestamp;
    unsigned int Msg_Id;
};

}  // namespace sensor
}  // namespace robosense

#endif  // RS_SENSOR_CAN_COMMON_BASE_BASE_CAN_MSG_H
