#pragma once

#include "controlcan.h"
#include "rs_common/external/common.h"

struct canbusData {
    std::string frame_id;
    uint32_t id;
    std::array<uint8_t, 8> data;
};

using std::mutex;
using std::unique_lock;
using std::vector;

namespace robosense {
namespace odom {
class CanBridge {
public:
    CanBridge(unsigned int can_index);

    ~CanBridge();

    int Read(vector <canbusData> &frames);

    void CanOpen(int timeing0, int timing1);

private:
    VCI_BOARD_INFO board_info_;

    void ReceiveFunc();

    void CanClose();

    std::shared_ptr <std::thread> thread_ptr_;
    bool run_flag_;
    unsigned int can_index_;
    unsigned int device_index_;
    vector <canbusData> can_buf_;
    canbusData can_tmp_;
    mutex can_mutex_sync_;
};

using std::mutex;
using std::string;
using std::to_string;
using std::vector;

inline CanBridge::CanBridge(unsigned int can_index) : run_flag_(false), can_index_(0), device_index_(0) {
    can_index_ = can_index;
}

inline CanBridge::~CanBridge() {
    CanClose();
}

inline int CanBridge::Read(vector <canbusData> &frames) {
    frames.clear();
    {
        unique_lock <mutex> lock(can_mutex_sync_);
        for (auto it : can_buf_) {
            frames.push_back(it);
        }
        can_buf_.clear();
    }
    return frames.size();
}

inline void CanBridge::ReceiveFunc() {
    int receive_length;
    VCI_CAN_OBJ receive_buffer[2500];

    while (run_flag_) {
        if ((receive_length = VCI_Receive(VCI_USBCAN2, device_index_, can_index_, receive_buffer, 1000, 100)) > 0) {
            for (int i = 0; i < receive_length; i++) {
                can_tmp_.frame_id = "/can";
                can_tmp_.id = receive_buffer[i].ID;
                for (int j = 0; j < receive_buffer[i].DataLen; j++) {
                    can_tmp_.data[j] = receive_buffer[i].Data[j];
                }
                {
                    unique_lock <mutex> lock(can_mutex_sync_);
                    can_buf_.emplace_back(can_tmp_);
                }
            }
        }
    }
    pthread_exit(0);
}

inline void CanBridge::CanOpen(int timeing0, int timeing1) {
    if (run_flag_) {
        return;
    }

    if (VCI_OpenDevice(VCI_USBCAN2, device_index_, 0) == 1) {
        RTRACE << "CanBus: Open device success";
    } else {
        RERROR << "CanBus: Open device fail";
        exit(-1);
    }
    if (VCI_ReadBoardInfo(VCI_USBCAN2, device_index_, &board_info_) == 1) {
        RTRACE << "CanBus: Get VCI_ReadBoardInfo success!";

        std::string output_string("");
        for (int i = 0; i < 20; i++) {
            output_string += board_info_.str_Serial_Num[i];
        }

        RTRACE << "CanBus: Serial_Num: " << output_string;
        output_string.clear();

        for (int i = 0; i < 10; i++) {
            output_string += board_info_.str_hw_Type[i];
        }

        RTRACE << "CanBus: hw_Type:" << output_string;
    }

    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 1;
    config.Mode = 0;

    /*500 Kbps  0x00  0x1C*/
    config.Timing0 = timeing0;
    config.Timing1 = timeing1;

    if (VCI_InitCAN(VCI_USBCAN2, device_index_, can_index_, &config) != 1) {
        RERROR << "CanBus: init CAN error";
        exit(-1);
        VCI_CloseDevice(VCI_USBCAN2, device_index_);
    }

    if (VCI_StartCAN(VCI_USBCAN2, device_index_, can_index_) != 1) {
        RERROR << "CanBus: Start CAN error";
        exit(-1);
        VCI_CloseDevice(VCI_USBCAN2, device_index_);
    }

    run_flag_ = true;
    thread_ptr_.reset(new std::thread([this]() { ReceiveFunc(); }));
}

inline void CanBridge::CanClose() {
    run_flag_ = false;
    thread_ptr_->join();
    VCI_CloseDevice(VCI_USBCAN2, device_index_);
}
}  // namespace odom
}  // namespace robosense
