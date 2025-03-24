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
#ifndef RS_SENSOR_CAN_COMMON_IO_KVASER_IO_H
#define RS_SENSOR_CAN_COMMON_IO_KVASER_IO_H

#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_KVASER_CAN_FOUND

#include <thread>
#include <condition_variable>
#include <list>
#include "rs_common/external/rs_logger.h"
#include "rs_sensor2/can/common/config/kvaser_config.h"
#include "rs_sensor2/can/common/base/base_can_io.h"

namespace robosense {
namespace sensor {

// Kvaser Can Channel IO 
class RSKvaserChannelIo {
public:
    using Ptr = std::shared_ptr<RSKvaserChannelIo>;
    using ConstPtr = std::shared_ptr<const RSKvaserChannelIo>;

public:
    int can_init(const RSSingleKvaserCanConfig &singleKvaserCanConfig) {
        _singleKvaserCanConfig = singleKvaserCanConfig;
        _hnd = canOpenChannel(singleKvaserCanConfig.kvaserChannle, singleKvaserCanConfig.kvaserOpenMode);
        if (_hnd < 0) {
            return -3;
        }

        canStatus status;

        status = canSetBusParams(_hnd, singleKvaserCanConfig.kvaserFreq, 0, 0, 0, 0, 0);
        if (status != canOK) {
            return -4;
        }

        status = canSetBusOutputControl(_hnd, singleKvaserCanConfig.kvaserControlMode);

        if (status != canOK) {
            return -5;
        }

        status = canBusOn(_hnd);

        if (status != canOK) {
            return -7;
        }

        return 0;
    }

    bool can_is_open() {
        return (_hnd != -1);
    }

    int can_start() {
        if (!can_is_open()) {
            return -1;
        }

        thread_start();

        return 0;
    }

    void can_stop() {
        thread_stop();
        can_close();
    }

    void register_can_callback(const ChannelCanReceiveCallback &channelCanCallback) {
        _channelCanCallback = channelCanCallback;
    }

    void register_can_exception(const CanExceptCallback &execptCallback) {
        _exceptCallback = execptCallback;
    }

private:
    void can_worker() {
        canStatus status;
        while (_workerIsStop == false && can_is_open()) {
            long canId;
            unsigned char canBuffer[64] = {0};
            unsigned int canDlc;
            unsigned int canFlag;
            unsigned long canStamp;
            // canRead();
            unsigned int timeout = 100;
            status = canReadWait(_hnd, &canId, &canBuffer, &canDlc, &canFlag, &canStamp, timeout);

            if (status == canERR_NOMSG) {
                continue;
            } else if (status == canOK) {
                if (_channelCanCallback != nullptr) {
                    std::vector<unsigned char> buffer(canDlc, '\0');
                    memcpy(buffer.data(), canBuffer, canDlc);
                    _channelCanCallback(_singleKvaserCanConfig.vehicleType, canId, buffer, canFlag, canStamp);
                }
            } else {
                if (_exceptCallback != nullptr) {
                    int errNo = -1;
                    std::string errMsg = "CAN Read Error";
                    _exceptCallback(errNo, errMsg);
                }
                break;
            }
        }
        _workerIsStop = true;
        can_close();
    }

    void can_close() {
        if (can_is_open()) {
            canBusOff(_hnd);
            canClose(_hnd);
            _hnd = -1;
        }
    }

    void thread_start() {
        _workerIsStop = false;

        _workerThread.reset(new std::thread(&RSKvaserChannelIo::can_worker, this));

        if (_singleKvaserCanConfig.affinity_id >= 0) {
            int num_cpus = std::thread::hardware_concurrency();
            if (_singleKvaserCanConfig.affinity_id >= num_cpus) {
                RERROR << "set cpu_affinity id larger than num of cpu!";
                RERROR << "set cpu_affinity id is " << _singleKvaserCanConfig.affinity_id << " num of cpu is "
                       << num_cpus;
                exit(-1);
            }
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(_singleKvaserCanConfig.affinity_id, &cpuset);
            int rc = pthread_setaffinity_np(_workerThread->native_handle(), sizeof(cpu_set_t), &cpuset);
            if (rc != 0) {
                RERROR << "Error calling pthread_setaffinity_np: " << rc;
                exit(-1);
            }
        }
    }

    void thread_stop() {
        if (_workerIsStop == false) {
            _workerIsStop = true;

            if (_workerThread != nullptr) {
                if (_workerThread->joinable()) {
                    _workerThread->join();
                }
            }
        }
    }

private:
    std::shared_ptr<std::thread> _workerThread;
    bool _workerIsStop;
    canHandle _hnd;
    ChannelCanReceiveCallback _channelCanCallback;
    CanExceptCallback _exceptCallback;
    RSSingleKvaserCanConfig _singleKvaserCanConfig;
};

class RSKvaserCanIO : public RSCanIOBasic {
public:
    using Ptr = std::shared_ptr<RSKvaserCanIO>;
    using ConstPtr = std::shared_ptr<const RSKvaserCanIO>;

private:
    class RSChannelCanData {
    public:
        std::string vehicleType; 
        long canId;
        std::vector<unsigned char> buffer;
        unsigned int canFlag;
        unsigned long canStamp;
    };

public:
    RSKvaserCanIO() : _hnd(-1), _workerIsStop(true) {
        // TODO...
    }

    virtual ~RSKvaserCanIO() {
        can_stop();
    }

public:
    virtual int can_init(const RSCanConfigBasic::Ptr &basicCanConfigPtr) {
        if (basicCanConfigPtr == nullptr) {
            return -1;
        }

        if (basicCanConfigPtr->canConfigName() != "KVASER_CAN_CONFIG") {
            return -2;
        }

        _kvaserCanConfigPtr = std::dynamic_pointer_cast<RSKvaserCanConfig>(basicCanConfigPtr);

        canInitializeLibrary();

        auto channelCallback = std::bind(&RSKvaserCanIO::local_callback, this, std::placeholders::_1,
                                         std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
        auto channelExceptCallback = std::bind(&RSKvaserCanIO::local_callback_except, this, std::placeholders::_1,
                                               std::placeholders::_2);

        _kvaserChannelCanIos.clear();
        for (size_t i = 0; i < _kvaserCanConfigPtr->kvaserCanConfigs.size(); ++i) {
            RSKvaserChannelIo::Ptr kvaserChannelIoPtr(new RSKvaserChannelIo());

            int ret = kvaserChannelIoPtr->can_init(_kvaserCanConfigPtr->kvaserCanConfigs[i]);

            if (ret != 0) {
                return -3;
            }

            kvaserChannelIoPtr->register_can_callback(channelCallback);

            kvaserChannelIoPtr->register_can_exception(channelExceptCallback);

            _kvaserChannelCanIos.push_back(kvaserChannelIoPtr);
        }

        return 0;
    }

    virtual bool can_is_open() {
        for (size_t i = 0; i < _kvaserChannelCanIos.size(); ++i) {
            if (!_kvaserChannelCanIos[i]->can_is_open()) {
                return false;
            }
        }
        return true;
    }

    virtual int can_start() {
        if (!can_is_open()) {
            return -1;
        }

        try {
            _workerIsStop = false;
            _workerThread.reset(new std::thread(&RSKvaserCanIO::work_thread, this));

            if (_kvaserCanConfigPtr->kvaserCanConfigs[0].affinity_id >= 0) {
                int num_cpus = std::thread::hardware_concurrency();
                if (_kvaserCanConfigPtr->kvaserCanConfigs[0].affinity_id >= num_cpus) {
                    RERROR << "set cpu_affinity id larger than num of cpu!";
                    RERROR << "set cpu_affinity id is " << _kvaserCanConfigPtr->kvaserCanConfigs[0].affinity_id
                           << " num of cpu is " << num_cpus;
                    exit(-1);
                }
                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                CPU_SET(_kvaserCanConfigPtr->kvaserCanConfigs[0].affinity_id, &cpuset);
                int rc = pthread_setaffinity_np(_workerThread->native_handle(), sizeof(cpu_set_t), &cpuset);
                if (rc != 0) {
                    RERROR << "Error calling pthread_setaffinity_np: " << rc;
                    exit(-1);
                }
            }
        }
        catch (...) {
            return -2;
        }

        for (size_t i = 0; i < _kvaserChannelCanIos.size(); ++i) {
            int ret = _kvaserChannelCanIos[i]->can_start();

            if (ret != 0) {
                return -3;
            }
        }

        return 0;
    }

    virtual void can_stop() {
        for (size_t i = 0; i < _kvaserChannelCanIos.size(); ++i) {
            _kvaserChannelCanIos[i]->can_stop();
        }

        if (_workerThread != nullptr) {
            _workerIsStop = true;
            _receiveCanCond.notify_all();

            if (_workerThread->joinable()) {
                _workerThread->join();
            }
        }
    }

private:
//    void can_frame_process(unsigned int canId, unsigned int canLen, unsigned char *canBuffer, unsigned int canFlag,
//                           unsigned int canStamp) {
//        Basic_Msg::Ptr msgPtr = can_frame_parse(canId, canLen, canBuffer, canFlag, canStamp);
//
//        if (msgPtr == nullptr) {
//            return;
//        }
//
//        run_callback(msgPtr);
//    }
//
//    Basic_Msg::Ptr
//    can_frame_parse(unsigned int canId, unsigned int canLen, unsigned char *canBuffer, unsigned int canFlag,
//                    unsigned int canStamp) {
//
//        if(_canParseCallback == nullptr){
//            return nullptr;
//        }
//
//        RSCanFrame rs_frame;
//
//        if (canLen <= 8) {
//            can_frame frame;
//            frame.can_id = canId;
//            frame.can_dlc = canLen;
//            memcpy(frame.data, canBuffer, canLen);
//
//            rs_frame = RSCanFrame(CAN_PROTOCOL_TYPE::PROTOCOL_TYPE_CAN);
//            rs_frame.update_can_frame(frame);
//        } else {
//            canfd_frame frame;
//            frame.can_id = canId;
//            frame.len = canLen;
//            memcpy(frame.data, canBuffer, canLen);
//
//            rs_frame = RSCanFrame(CAN_PROTOCOL_TYPE::PROTOCOL_TYPE_CANFD);
//            rs_frame.update_canfd_frame(frame);
//        }
//
//        return _canParseCallback(rs_frame);
//    }
//
//    void run_callback(const Basic_Msg::Ptr &msgPtr) {
//        std::lock_guard<std::mutex> lg(_rcvMtx);
//        for (size_t i = 0; i < _rcvCallbacks.size(); ++i) {
//            if (_rcvCallbacks[i] == nullptr) {
//                continue;
//            }
//            _rcvCallbacks[i](msgPtr);
//        }
//    }

    void run_native_callback(const RSChannelCanData& data){
        std::lock_guard<std::mutex> lg(_rcvNativeMtx); 
        for(size_t i = 0; i < _rcvNativeCallbacks.size(); ++i){
            if(_rcvNativeCallbacks[i] == nullptr){
                continue; 
            }
            _rcvNativeCallbacks[i](data.vehicleType, data.canId, data.buffer, data.canFlag, data.canStamp); 
        }
    }

    void run_callback_except(const int errNo, const std::string &errMsg) {
        std::lock_guard<std::mutex> lg(_excMtx);
        for (size_t i = 0; i < _excCallbacks.size(); ++i) {
            if (_excCallbacks[i] == nullptr) {
                continue;
            }
            _excCallbacks[i](errNo, errMsg);
        }
    }

    void
    local_callback(const std::string& vehicleType, long canId, const std::vector<unsigned char> &buffer, unsigned int canFlag, unsigned long canStamp) {
        std::lock_guard<std::mutex> lg(_receiveCanMtx);
        RSChannelCanData data;
        data.vehicleType = vehicleType; 
        data.canId = canId;
        data.buffer = buffer;
        data.canFlag = canFlag;
        data.canStamp = canStamp;
        _receiveCanBuffer.push_back(data);
        _receiveCanCond.notify_one();
    }

    void local_callback_except(const int errNo, const std::string &errMsg) {
        run_callback_except(errNo, errMsg);
    }

    void work_thread() {
        while (_workerIsStop == false) {
            RSChannelCanData data;
            {
                std::unique_lock<std::mutex> lg(_receiveCanMtx);
                _receiveCanCond.wait(lg, [&] { return (_workerIsStop || _receiveCanBuffer.size() > 0); });

                if (_workerIsStop == true) {
                    return;
                }

                auto iterList = _receiveCanBuffer.begin();
                data = *iterList;
                _receiveCanBuffer.erase(iterList);
            }
            // Step1: Can Native Data Return  
            run_native_callback(data); 

            // Step2: Parse Can Data If Register Can Data Parser 
//            can_frame_process(data.canId, data.buffer.size(), data.buffer.data(), data.canFlag, data.canStamp);
        }
    }

private:
    bool _workerIsStop;
    canHandle _hnd;
    std::shared_ptr<std::thread> _workerThread;
    std::mutex _receiveCanMtx;
    std::condition_variable _receiveCanCond;
    std::list<RSChannelCanData> _receiveCanBuffer;
    RSKvaserCanConfig::Ptr _kvaserCanConfigPtr;
    std::vector<RSKvaserChannelIo::Ptr> _kvaserChannelCanIos;
};

}  // namespace sensor
}  // namespace robosense

#endif // RS_KVASER_CAN_FOUND

#endif  // RS_SENSOR_CAN_COMMON_IO_KVASER_IO_H
