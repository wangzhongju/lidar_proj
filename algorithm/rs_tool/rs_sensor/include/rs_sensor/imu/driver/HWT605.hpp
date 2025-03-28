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
#pragma once

#include "rs_sensor/imu/adapter/driver_adapter.hpp"
#include <iomanip>

namespace robosense {
namespace imu {
const double g_ = 9.8;
const int msg_len = 3;
const int acc_scale = 4;
const int gyr_scale = 500;
const int angle_scale = 180;
const double deg2rad = 0.01745329;
const int max_range = 32768;
struct Time {
    int year;
    int month;
    int day;
    int hour;
    int miniute;
    int sec;
};

struct Acc {
    double acc_x;
    double acc_y;
    double acc_z;
    double T;
};

struct Gyr {
    double gyr_x;
    double gyr_y;
    double gyr_z;
};

struct Eular_Angle {
    double roll;
    double pitch;
    double yaw;
};

struct Mag {
    double h_x;
    double h_y;
    double h_z;
};

struct Press_Height {
    double presure;
    double height;
};

struct Gps {
    double Longitude;
    double Latitude;
};

struct Quad {
    double q0, q1, q2, q3;
};

struct ImuData {
    Time time_;
    Acc acc_;
    Gyr gyr_;
    Eular_Angle eular_angle_;
    Mag mag_;
    Press_Height ph_;
    Gps gps_;
    Quad quad_;

    ImuData operator+(const ImuData &b) {
        ImuData temp_;
        temp_.acc_.acc_x = this->acc_.acc_x + b.acc_.acc_x;
        temp_.acc_.acc_y = this->acc_.acc_y + b.acc_.acc_y;
        temp_.acc_.acc_z = this->acc_.acc_z + b.acc_.acc_z - g_;

        temp_.gyr_.gyr_x = this->gyr_.gyr_x + b.gyr_.gyr_x;
        temp_.gyr_.gyr_y = this->gyr_.gyr_y + b.gyr_.gyr_y;
        temp_.gyr_.gyr_z = this->gyr_.gyr_z + b.gyr_.gyr_z;

        temp_.eular_angle_.roll = this->eular_angle_.roll + b.eular_angle_.roll;
        temp_.eular_angle_.pitch = this->eular_angle_.pitch + b.eular_angle_.pitch;
        temp_.eular_angle_.yaw = this->eular_angle_.yaw + b.eular_angle_.yaw;
        return temp_;
    }

    ImuData operator-(const ImuData &b) {
        ImuData temp_;
        temp_.acc_.acc_x = this->acc_.acc_x - b.acc_.acc_x;
        temp_.acc_.acc_y = this->acc_.acc_y - b.acc_.acc_y;
        temp_.acc_.acc_z = this->acc_.acc_z - b.acc_.acc_z;

        temp_.gyr_.gyr_x = this->gyr_.gyr_x - b.gyr_.gyr_x;
        temp_.gyr_.gyr_y = this->gyr_.gyr_y - b.gyr_.gyr_y;
        temp_.gyr_.gyr_z = this->gyr_.gyr_z - b.gyr_.gyr_z;

        temp_.eular_angle_.roll = this->eular_angle_.roll - b.eular_angle_.roll;
        temp_.eular_angle_.pitch = this->eular_angle_.pitch - b.eular_angle_.pitch;
        temp_.eular_angle_.yaw = this->eular_angle_.yaw - b.eular_angle_.yaw;
        return temp_;
    }

    ImuData operator=(int n) {
        ImuData temp_;
        temp_.acc_.acc_x = n;
        temp_.acc_.acc_y = n;
        temp_.acc_.acc_z = n;

        temp_.gyr_.gyr_x = n;
        temp_.gyr_.gyr_y = n;
        temp_.gyr_.gyr_z = n;

        temp_.eular_angle_.roll = n;
        temp_.eular_angle_.pitch = n;
        temp_.eular_angle_.yaw = n;
        return temp_;
    }

    ImuData operator/(int n) {
        ImuData temp_;
        temp_.acc_.acc_x = this->acc_.acc_x / n;
        temp_.acc_.acc_y = this->acc_.acc_y / n;
        temp_.acc_.acc_z = this->acc_.acc_z / n;

        temp_.gyr_.gyr_x = this->gyr_.gyr_x / n;
        temp_.gyr_.gyr_y = this->gyr_.gyr_y / n;
        temp_.gyr_.gyr_z = this->gyr_.gyr_z / n;

        temp_.eular_angle_.roll = this->eular_angle_.roll / n;
        temp_.eular_angle_.pitch = this->eular_angle_.pitch / n;
        temp_.eular_angle_.yaw = this->eular_angle_.yaw / n;
        return temp_;
    }
};

typedef ImuData YJ901_Data;
}  // namespace imu
namespace imu {
class ImuHWT605 : virtual public DriverAdapter {
public:
    ImuHWT605() = default;

    ~ImuHWT605() = default;

private:
    void prepareMsg();

    int checkMarkBit();

    bool autoConnect();

    void analysis(std::vector<uint8_t> buf_);

    void decode_msg(int num, std::vector<uint8_t> buf_rec_);

    robosense::imu::YJ901_Data yj9_data_;

    void translation_4d(std::array<int16_t, 4> &vec_hec) {
        for (int i = 0; i < 4; i++) {
            if (vec_hec[i] & 0x8000) {
                vec_hec[i] -= 1;
                vec_hec[i] = ~vec_hec[i];
                vec_hec[i] &= 0x7fff;
                vec_hec[i] = -vec_hec[i];
            }
        }
    }

    void translation_3d(std::array<int16_t, 3> &vec_hec) {
        for (int i = 0; i < 3; i++) {
            if (vec_hec[i] & 0x8000) {
                vec_hec[i] -= 1;
                vec_hec[i] = ~vec_hec[i];
                vec_hec[i] &= 0x7fff;
                vec_hec[i] = -vec_hec[i];
            }
        }
    }

    void translation_2d(std::array<int32_t, 2> &vec_hec) {
        for (int i = 0; i < 2; i++) {
            if (vec_hec[i] & 0x80000000) {
                vec_hec[i] -= 1;
                vec_hec[i] = ~vec_hec[i];
                vec_hec[i] &= 0x7fffffff;
                vec_hec[i] = -vec_hec[i];
            }
        }
    }
};

#define HWT605_MARKBYTE 0x55

inline void ImuHWT605::prepareMsg() {
    using namespace robosense::common;
    std::vector<uint8_t> buf_rec;
    int count = (int) imu_ser_->serialRead(buf_rec, msg_len * 11 - 1, imu_parameter_.timeout);
    if (count < 0) {
        reportError(ErrCode_ImuDriverInterrupt);
        self_state_ = CHECK_CONNECTION;
    }

    decode_msg(count, buf_rec);

    yj9_data_ = yj9_data_;
    imu_seq_++;
    ImuMsg imu_msg;
    imu_msg.orien[0] = yj9_data_.eular_angle_.roll / 180.0 * M_PI;
    imu_msg.orien[1] = yj9_data_.eular_angle_.pitch / 180.0 * M_PI;
    imu_msg.orien[2] = yj9_data_.eular_angle_.yaw / 180.0 * M_PI;
    imu_msg.acc[0] = yj9_data_.acc_.acc_x;
    imu_msg.acc[1] = yj9_data_.acc_.acc_y;
    imu_msg.acc[2] = yj9_data_.acc_.acc_z;
    imu_msg.angular_vel[0] = yj9_data_.gyr_.gyr_x;
    imu_msg.angular_vel[1] = yj9_data_.gyr_.gyr_y;
    imu_msg.angular_vel[2] = yj9_data_.gyr_.gyr_z;
    imu_msg.timestamp = getTime();
    imu_msg.seq = imu_seq_;
    imu_msg.frame_id = imu_parameter_.frame_id;
    imu_msg.parent_frame_id = imu_msg.frame_id;
    if (std::abs(imu_msg.angular_vel[2]) > imu_parameter_.warning_gyro_z) {
        reportError(ErrCode_ImuDriverGyroztoolarge);
    }  // Gyro Z is too large!

    runCallBack(imu_msg);
}

inline int ImuHWT605::checkMarkBit() {
    std::vector<uint8_t> buf;
    if ((int) imu_ser_->serialRead(buf, 1, imu_parameter_.timeout) < 0) {
        reportError(common::ErrCode_ImuDriverConnectfail);
        return 0;
    }
    if (buf[0] == (uint8_t) (HWT605_MARKBYTE))
        return 2;
    return 1;
}

inline bool ImuHWT605::autoConnect() {
    auto x = [&]() {
        for (int j = 0; j < 32; j++) {
            std::vector<uint8_t> buf;
            if (imu_ser_->serialRead(buf, 1, imu_parameter_.timeout) < 0) {
                imu_ser_->closePort();
                return false;
            }
            if (buf[0] != (uint8_t) (HWT605_MARKBYTE)) {
                continue;
            } else {
                return true;
            }
        }
        imu_ser_->closePort();
        return false;
    };
    return imu_ser_->autoConnect(x, imu_ser_, imu_parameter_.baudrate);
}

inline void ImuHWT605::decode_msg(int count, std::vector<uint8_t> buf_rec) {
    std::vector<uint8_t> buf_decode;
    for (int j = 0; j < (count + 1) / 11; j++) {
        if (j == 0) {
            buf_decode.emplace_back(0x55);
            for (int k = 0; k < 10; k++) {
                buf_decode.emplace_back(buf_rec[k] & 0xff);
            }
        } else {
            for (int k = j * 11 - 1; k < (j + 1) * 11 - 1; k++) {
                buf_decode.emplace_back(buf_rec[k] & 0xff);
            }
        }
        analysis(buf_decode);

        buf_rec.clear();
        buf_decode.clear();
    }
}

inline void ImuHWT605::analysis(std::vector<uint8_t> buf_) {
    if (buf_[0] != 0x55) {
        return;
    }
    switch (buf_[1]) {
        // Time out;
        case 0x50: {
            yj9_data_.time_.year = buf_[2];
            yj9_data_.time_.month = buf_[3];
            yj9_data_.time_.day = buf_[4];
            yj9_data_.time_.hour = buf_[5];
            yj9_data_.time_.miniute = buf_[6];
            yj9_data_.time_.sec = buf_[7];
            break;
        }
            // Acc out;
        case 0x51: {
            std::array<int16_t, 3> acc_hex;
            acc_hex[0] = (buf_[3] << 8 | buf_[2]);
            acc_hex[1] = (buf_[5] << 8 | buf_[4]);
            acc_hex[2] = (buf_[7] << 8 | buf_[6]);
            translation_3d(acc_hex);
            yj9_data_.acc_.acc_x = (double) acc_hex[0] / max_range * acc_scale * g_;
            yj9_data_.acc_.acc_y = (double) acc_hex[1] / max_range * acc_scale * g_;
            yj9_data_.acc_.acc_z = (double) acc_hex[2] / max_range * acc_scale * g_;
            break;
        }
            // Gyr out;
        case 0x52: {
            std::array<int16_t, 3> gyr_hex;
            gyr_hex[0] = (buf_[3] << 8 | buf_[2]);
            gyr_hex[1] = (buf_[5] << 8 | buf_[4]);
            gyr_hex[2] = (buf_[7] << 8 | buf_[6]);
            translation_3d(gyr_hex);
            yj9_data_.gyr_.gyr_x = (double) gyr_hex[0] / max_range * gyr_scale * deg2rad;
            yj9_data_.gyr_.gyr_y = (double) gyr_hex[1] / max_range * gyr_scale * deg2rad;
            yj9_data_.gyr_.gyr_z = (double) gyr_hex[2] / max_range * gyr_scale * deg2rad;
            break;
        }
            // Angle out
        case 0x53: {
            std::array<int16_t, 3> angle_hex;
            angle_hex[0] = (buf_[3] << 8 | buf_[2]);
            angle_hex[1] = (buf_[5] << 8 | buf_[4]);
            angle_hex[2] = (buf_[7] << 8 | buf_[6]);
            translation_3d(angle_hex);
            yj9_data_.eular_angle_.roll = (double) angle_hex[0] / max_range * angle_scale;
            yj9_data_.eular_angle_.pitch = (double) angle_hex[1] / max_range * angle_scale;
            yj9_data_.eular_angle_.yaw = (double) angle_hex[2] / max_range * angle_scale;
            break;
        }
            // Mag out
        case 0x54: {
            std::array<int16_t, 3> mag_hex;
            mag_hex[0] = (buf_[3] << 8 | buf_[2]);
            mag_hex[1] = (buf_[5] << 8 | buf_[4]);
            mag_hex[2] = (buf_[7] << 8 | buf_[6]);
            translation_3d(mag_hex);
            yj9_data_.mag_.h_x = (double) mag_hex[0];
            yj9_data_.mag_.h_y = (double) mag_hex[1];
            yj9_data_.mag_.h_z = (double) mag_hex[2];
            break;
        }
            // pressure & height out
        case 0x56: {
            std::array<int32_t, 2> ph_hex;
            ph_hex[0] = ((buf_[5] << 24) | (buf_[4] << 16) | (buf_[3] << 8) | buf_[2]);
            ph_hex[1] = ((buf_[9] << 24) | (buf_[8] << 16) | (buf_[7] << 8) | buf_[6]);
            translation_2d(ph_hex);
            yj9_data_.ph_.presure = (double) ph_hex[0];
            yj9_data_.ph_.height = (double) ph_hex[1] / 100;
            break;
        }
            // Gps out;
        case 0x57: {
            std::array<int32_t, 2> gps;
            gps[0] = ((buf_[5] << 24) | (buf_[4] << 16) | (buf_[3] << 8) | buf_[2]);
            gps[1] = ((buf_[9] << 24) | (buf_[8] << 16) | (buf_[7] << 8) | buf_[6]);
            translation_2d(gps);
            yj9_data_.gps_.Longitude = (double) gps[0] / 10000000;
            yj9_data_.gps_.Latitude = (double) gps[1] / 10000000;
            break;
        }
        case 0x59: {
            std::array<int16_t, 4> qua_hex;
            qua_hex[0] = (buf_[3] << 8 | buf_[2]);
            qua_hex[1] = (buf_[5] << 8 | buf_[4]);
            qua_hex[2] = (buf_[7] << 8 | buf_[6]);
            qua_hex[3] = (buf_[9] << 8 | buf_[8]);
            translation_4d(qua_hex);
            yj9_data_.quad_.q3 = (double) qua_hex[0] / max_range;
            yj9_data_.quad_.q0 = (double) qua_hex[1] / max_range;
            yj9_data_.quad_.q1 = (double) qua_hex[2] / max_range;
            yj9_data_.quad_.q2 = (double) qua_hex[3] / max_range;
            break;
        }
    }
}

}  // namespace imu
}  // namespace robosense
