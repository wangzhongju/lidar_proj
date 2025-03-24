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
#ifndef RS_SENSOR_CAN_LXONE_ONLINE_LX_CAN_MSG_H
#define RS_SENSOR_CAN_LXONE_ONLINE_LX_CAN_MSG_H

#include "rs_sensor2/can/common/base/base_can_msg.h"

namespace robosense {
namespace sensor {
namespace LX_CAN {

class ESP_0x295 : public Basic_Msg {
public:
    using Ptr = std::shared_ptr<ESP_0x295>;
    using ConstPtr = std::shared_ptr<const ESP_0x295>;

public:
    ESP_0x295() {
        Msg_Name = "ESP_0x295";
        Msg_Id = 0x295; // 十进制： 661
    }

    virtual ~ESP_0x295() {
        // TODO...
    }

public:
    virtual void parseSFF(const unsigned char *data) {
        ESP_OffRoadModeStatus = data[0] >> 5;
        ESP_EBDCtrlActive = data[0] >> 6;
        ESP_VehicleSpeedValid = data[0] >> 7;
        ESP_VehicleSpeed = ((((unsigned short int) (data[0] & 0x1F)) << 8) + data[1]) * 0.05625;
        ESP_ESPOffLamp = data[2] & 0x01;
        ESP_ESPLampInfo = (data[2] & 0x02) >> 1;
        ESP_ESPFault = (data[2] & 0x04) >> 2;
        ESP_ABSCtrlActive = (data[2] & 0x08) >> 3;
        ESP_ABSFault = (data[2] & 0x10) >> 4;
        ESP_VDCCtrlActive = (data[2] & 0x20) >> 5;
        ESP_TCSCtrlActive = (data[2] & 0x40) >> 6;
        ESP_HBBIntervention = (data[2] & 0x80) >> 7;
        ESP_HBBStatus = data[3] & 0x03;
        ESP_HHCCtrlActive = (data[3] & 0x04) >> 2;
        ESP_HHCAvailable = (data[3] & 0x08) >> 3;
        ESP_AVHCtrlActive = (data[3] & 0x10) >> 4;
        ESP_AVHStandby = (data[3] & 0x20) >> 5;
        ESP_AVHAvailable = (data[3] & 0x40) >> 6;
        ESP_CDPCtrlActive = (data[3] & 0x80) >> 7;
        ESP_CDPAvailable = data[4] & 0x01;
        ESP_HDCCtrlActive = (data[4] & 0x06) >> 1;
        ESP_HDCAvailable = (data[4] & 0x08) >> 3;
        ESP_EBDFault = data[5] & 0x01;
        ESP_EPBCtrlActive = (data[5] & 0x06) >> 1;
        ESP_VehicleStandStill = (data[5] & 0x18) >> 3;
        ESP_EPBFault = (data[5] & 0x20) >> 5;
        ESP_NoPressBrakeWarning = (data[5] & 0x40) >> 6;
        ESP_BrakeLightStatus = (data[5] & 0x80) >> 7;
        ESP_HBACtrlActive = data[6] & 0x01;
        ESP_HAZEmergencyBrakeLight = (data[6] & 0x02) >> 1;
        ESP_BrakeSystemFaultWarningLamp = (data[6] & 0x0C) >> 2;
        ESP_GeneralStatus_RollingCounter = (data[6] & 0xF0) >> 4;
        ESP_GeneralStatus_Checksum = data[7];
    }

public:
    unsigned char ESP_OffRoadModeStatus;            //  : 5|1@0+ (1,0) [0|1] ""  GW_CH
    unsigned char ESP_EBDCtrlActive;                //  : 6|1@0+ (1,0) [0|1] ""  VCU_CH,ADAS
    unsigned char ESP_VehicleSpeedValid;            //  : 7|1@0+ (1,0) [0|1] ""  VCU_CH,ADAS,ACU,EPS,ECSS,GW_CH
    float ESP_VehicleSpeed;                         //  : 4|13@0+ (0.05625,0) [0|460.6875] "km/h"  VCU_CH,ADAS,ACU,EPS,ECSS,GW_CH
    unsigned char ESP_ESPOffLamp;                   //  : 16|1@0+ (1,0) [0|1] ""  VCU_CH,ADAS,GW_CH
    unsigned char ESP_ESPLampInfo;                  // : 17|1@0+ (1,0) [0|1] ""  ADAS,EPS,GW_CH
    unsigned char ESP_ESPFault;                     //  : 18|1@0+ (1,0) [0|1] ""  VCU_CH,ADAS,GW_CH
    unsigned char ESP_ABSCtrlActive;                //  : 19|1@0+ (1,0) [0|1] ""  VCU_CH,ADAS,EPS,ECSS
    unsigned char ESP_ABSFault;                     // : 20|1@0+ (1,0) [0|1] ""  VCU_CH,EPS,GW_CH
    unsigned char ESP_VDCCtrlActive;                // : 21|1@0+ (1,0) [0|1] ""  VCU_CH,ADAS,ECSS
    unsigned char ESP_TCSCtrlActive;                //  : 22|1@0+ (1,0) [0|1] ""  VCU_CH,ADAS,ECSS
    unsigned char ESP_HBBIntervention;              //  : 23|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_HBBStatus;                    //  : 25|2@0+ (1,0) [0|3] "" Vector__XXX
    unsigned char ESP_HHCCtrlActive;                // : 26|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_HHCAvailable;                 //  : 27|1@0+ (1,0) [0|1] ""  VCU_CH
    unsigned char ESP_AVHCtrlActive;                // : 28|1@0+ (1,0) [0|1] ""  VCU_CH,GW_CH
    unsigned char ESP_AVHStandby;                   //  : 29|1@0+ (1,0) [0|1] ""  VCU_CH,GW_CH
    unsigned char ESP_AVHAvailable;                 //  : 30|1@0+ (1,0) [0|1] ""  VCU_CH,GW_CH
    unsigned char ESP_CDPCtrlActive;                //  : 31|1@0+ (1,0) [0|1] ""  VCU_CH,ADAS
    unsigned char ESP_CDPAvailable;                 // : 32|1@0+ (1,0) [0|1] ""  VCU_CH,ADAS
    unsigned char ESP_HDCCtrlActive;                // : 34|2@0+ (1,0) [0|3] ""  VCU_CH,ADAS,GW_CH
    unsigned char ESP_HDCAvailable;                 //  : 35|1@0+ (1,0) [0|1] ""  ADAS
    unsigned char ESP_EBDFault;                     //  : 40|1@0+ (1,0) [0|1] ""  VCU_CH,ADAS,GW_CH
    unsigned char ESP_EPBCtrlActive;                // : 42|2@0+ (1,0) [0|3] ""  VCU_CH,ADAS,GW_CH
    unsigned char ESP_VehicleStandStill;            // : 44|2@0+ (1,0) [0|3] ""  ADAS
    unsigned char ESP_EPBFault;                     //  : 45|1@0+ (1,0) [0|1] ""  VCU_CH,ADAS,GW_CH
    unsigned char ESP_NoPressBrakeWarning;          // : 46|1@0+ (1,0) [0|1] ""  GW_CH
    unsigned char ESP_BrakeLightStatus;             // : 47|1@0+ (1,0) [0|1] ""  GW_CH
    unsigned char ESP_HBACtrlActive;                // : 48|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_HAZEmergencyBrakeLight;       // : 49|1@0+ (1,0) [0|1] ""  GW_CH
    unsigned char ESP_BrakeSystemFaultWarningLamp;  // : 51|2@0+ (1,0) [0|3] ""  GW_CH
    unsigned char ESP_GeneralStatus_RollingCounter; // : 55|4@0+ (1,0) [0|15] ""  VCU_CH,ADAS,ACU,EPS,ECSS,GW_CH
    unsigned char ESP_GeneralStatus_Checksum;       // : 63|8@0+ (1,0) [0|255] ""  VCU_CH,ADAS,ACU,EPS,GW_CH
};

class ESP_0x225 : public Basic_Msg {
public:
    using Ptr = std::shared_ptr<ESP_0x225>;
    using ConstPtr = std::shared_ptr<const ESP_0x225>;

public:
    ESP_0x225() {
        Msg_Name = "ESP_0x225";
        Msg_Id = 0x225; // 十进制: 549
    }

    virtual ~ESP_0x225() {
        // TODO...
    }

public:
    virtual void parseSFF(const unsigned char *data) {
        ACU_LateralAcc = ((((unsigned short int) (data[0])) << 8) + data[1]) * 0.0002 + (-5);
        ACU_LateralSensorState = data[2] & 0x03;
        ACU_YawRate = ((((unsigned short int) (data[3])) << 8) + data[4]) * 0.01 + (-300);
        ACU_YawRateSensorState = data[5] & 0x03;
        ACU_LateralAccandYawRate_Rolling = (data[6] & 0xF0) >> 4;
        ACU_LateralAccandYawRate_Checksu = data[7];
    }

public:
    float ACU_LateralAcc;                           // : 7|16@0+ (0.0002,-5) [-5|5] "g"  VCU_CH,ESP,ADAS,GW_CH
    unsigned char ACU_LateralSensorState;           // : 17|2@0+ (1,0) [0|3] ""  VCU_CH,ESP,ADAS,GW_CH
    float ACU_YawRate;                              // : 31|16@0+ (0.01,-300) [-300|300] "\A1\E3/s"  VCU_CH,ESP,ADAS,GW_CH
    unsigned char ACU_YawRateSensorState;           // : 41|2@0+ (1,0) [0|3] ""  VCU_CH,ESP,ADAS,GW_CH
    unsigned char ACU_LateralAccandYawRate_Rolling; // : 55|4@0+ (1,0) [0|15] ""  VCU_CH,ESP,ADAS,GW_CH
    unsigned char ACU_LateralAccandYawRate_Checksu; // : 63|8@0+ (1,0) [0|255] ""  VCU_CH,ESP,ADAS,GW_CH
};

class ESP_0x155 : public Basic_Msg {
public:
    using Ptr = std::shared_ptr<ESP_0x155>;
    using ConstPtr = std::shared_ptr<const ESP_0x155>;

public:
    ESP_0x155() {
        Msg_Name = "ESP_0x155";
        Msg_Id = 0x155; // 十进制: 341
    }

    virtual ~ESP_0x155() {
        // TODO...
    }

public:
    virtual void parseSFF(const unsigned char *data) {
        EPS_SteeringAngle = ((((short int) (data[0])) << 8) + data[1]) * 0.1;
        EPS_SteeringAngleValid = data[2] >> 4;
        EPS_Calibration = data[2] >> 6;
        EPS_SteeringAngleSpeed = ((((unsigned short int) (data[2] & 0x0F)) << 8) + data[3]) + (-2048);
        EPS_GeneralStatus_RollingCounter = data[6] >> 4;
        EPS_GeneralStatus_Checksum = data[7];
    }

public:
    float EPS_SteeringAngle;                        // : 7|16@0- (0.1,0) [-780|779.9] "\A1\E3"  VCU_CH,ESP,ADAS,GW_CH
    unsigned char EPS_SteeringAngleValid;           // : 20|1@0+ (1,0) [0|1] ""  VCU_CH,ESP,ADAS,GW_CH
    unsigned char EPS_Calibration;                  //: 22|1@0+ (1,0) [0|1] ""  ESP,ADAS
    short int EPS_SteeringAngleSpeed;               // : 19|12@0+ (1,-2048) [-2048|2046] "deg/s"  VCU_CH,ESP,ADAS
    unsigned char EPS_GeneralStatus_RollingCounter; // : 55|4@0+ (1,0) [0|15] ""  VCU_CH,ESP,ADAS,GW_CH
    unsigned char EPS_GeneralStatus_Checksum;       // : 63|8@0+ (1,0) [0|255] ""  VCU_CH,ESP,ADAS,GW_CH
};

class ESP_0x46F : public Basic_Msg {
public:
    using Ptr = std::shared_ptr<ESP_0x46F>;
    using ConstPtr = std::shared_ptr<const ESP_0x46F>;

public:
    ESP_0x46F() {
        Msg_Name = "ESP_0x46F";
        Msg_Id = 0x46F; // 十进制: 1135
    }

    virtual ~ESP_0x46F() {
        // TODO...
    }

public:
    virtual void parseSFF(const unsigned char *data) {
        EPS_SourceAddress = data[0];
    }

public:
    unsigned char EPS_SourceAddress; // : 7|8@0+ (1,0) [0|255] ""  ESP,Ibooster,ADAS,ACU,GW_CH,EGW_CH
};

class ESP_0x655 : public Basic_Msg {
public:
    using Ptr = std::shared_ptr<ESP_0x655>;
    using ConstPtr = std::shared_ptr<const ESP_0x655>;

public:
    ESP_0x655() {
        Msg_Name = "ESP_0x655";
        Msg_Id = 0x655; // 十进制: 1621
    }

    virtual ~ESP_0x655() {
        // TODO...
    }

public:
    virtual void parseSFF(const unsigned char *data) {
        ESP_PumpMotorControl_C160071 = data[0] & 0x01;
        ESP_WhlSpdSensorFLMultiFailures_ = (data[0] & 0x02) >> 1;
        ESP_WhlSpdSensorFLShortGND_C1602 = (data[0] & 0x04) >> 2;
        ESP_WhlSpdSensorFLOpen_C160313 = (data[0] & 0x08) >> 3;
        ESP_WhlSpdSensorFLShortUbatt_C16 = (data[0] & 0x10) >> 4;
        ESP_WhlSpdSensorFRMultiFailures_ = (data[0] & 0x20) >> 5;
        ESP_WhlSpdSensorFRShortGND_C1606 = (data[0] & 0x40) >> 6;
        ESP_WhlSpdSensorFROpen_C160713 = (data[0] & 0x80) >> 7;

        ESP_WhlSpdSensorFRShortUbatt_C16 = data[1] & 0x01;
        ESP_WhlSpdSensorRLMultiFailures_ = (data[1] & 0x02) >> 1;
        ESP_WhlSpdSensorRLShortGND_C160A = (data[1] & 0x04) >> 2;
        ESP_WhlSpdSensorRLOpen_C160B13 = (data[1] & 0x08) >> 3;
        ESP_WhlSpdSensorRLShortUbatt_C16 = (data[1] & 0x10) >> 4;
        ESP_WhlSpdSensorRRMultiFailures_ = (data[1] & 0x20) >> 5;
        ESP_WhlSpdSensorRRShortGND_C160E = (data[1] & 0x40) >> 6;
        ESP_WhlSpdSensorRROpen_C160F13 = (data[1] & 0x80) >> 7;

        ESP_WhlSpdSensorRRShortUbatt_C16 = data[2] & 0x01;
        ESP_StrngWhlSensorFlt_C161128 = (data[2] & 0x02) >> 1;
        ESP_LateralAccSensorSigFlt_C1612 = (data[2] & 0x04) >> 2;
        ESP_LongitudinalAccSnsorSigFlt_C = (data[2] & 0x08) >> 3;
        ESP_YRSSigFlt_C161429 = (data[2] & 0x10) >> 4;
        ESP_ABSESPImplausibleCtrl_C16150 = (data[2] & 0x20) >> 5;
        ESP_PATASwitchFailure_C161609 = (data[2] & 0x40) >> 6;
        ESP_WhlSpdSensorGeneralFailure_C = (data[2] & 0x80) >> 7;

        ESP_VvlRelayFailure_C161804 = data[3] & 0x01;
        ESP_VvlGeneralFailure_C161904 = (data[3] & 0x02) >> 1;
        ESP_VaraintCodeFault_C161A00 = (data[3] & 0x04) >> 2;
        ESP_CtrlModuleComBusOff_U007388 = (data[3] & 0x08) >> 3;
        ESP_CANBusOffError_U007488 = (data[3] & 0x10) >> 4;
        ESP_LostComWithVCU_U160087 = (data[3] & 0x20) >> 5;
        ESP_LostComWithADAS_U160187 = (data[3] & 0x40) >> 6;
        ESP_LostComWithACU_U160287 = (data[3] & 0x80) >> 7;

        ESP_LostComWithEPS_U160387 = data[4] & 0x01;
        ESP_LostComWithMCUF_U160487 = (data[4] & 0x02) >> 1;
        ESP_LostComWithMCUR_U160587 = (data[4] & 0x04) >> 2;
        ESP_LostComWithGW_U160687 = (data[4] & 0x08) >> 3;
        ESP_LostComWithIPC_U160787 = (data[4] & 0x10) >> 4;
        ESP_BatVolHigh_U160817 = (data[4] & 0x20) >> 5;
        ESP_BatVolLow_U160916 = (data[4] & 0x40) >> 6;
        ESP_ECUHardwareError_U160A04 = (data[4] & 0x80) >> 7;

        ESP_ECUSoftwareError_U160B04 = data[5] & 0x01;
        ESP_CANHardwareError_U160C04 = (data[5] & 0x02) >> 1;
        ESP_InvalidSigRxfromVCU_C161B86 = (data[5] & 0x04) >> 2;
        ESP_InvalidSigRxfromADAS_C161C86 = (data[5] & 0x08) >> 3;
        ESP_InvalidSigRxfromACU_C161D86 = (data[5] & 0x10) >> 4;
        ESP_InvalidSigRxfromEPS_C161E86 = (data[5] & 0x20) >> 5;
        ESP_InvalidSigRxfromMCUF_C161F86 = (data[5] & 0x40) >> 6;
        ESP_InvalidSigRxfromMCUR_C162086 = (data[5] & 0x80) >> 7;

        ESP_InvalidSigRxfromGW_C162186 = data[6] & 0x01;
        ESP_InvalidSigRxfromIPC_C162286 = (data[6] & 0x02) >> 1;
        ESP_BLSSigPlausibility_C162302 = (data[6] & 0x04) >> 2;
        ESP_Overheatfault_C162402 = (data[6] & 0x08) >> 3;
        ESP_hydraulicunitisworn_C162502 = (data[6] & 0x10) >> 4;
        ESP_PressuresensorfaultLine_C162 = (data[6] & 0x20) >> 5;
        ESP_PressuresensorfaultSig_C1627 = (data[6] & 0x40) >> 6;
        ESP_ReverseGearabnormal_C162802 = (data[6] & 0x80) >> 7;

        ESP_SASCalibrationError_C162946 = data[7] & 0x01;
        ESP_SystemInitnotComplete_C162A0 = (data[7] & 0x02) >> 1;
        ESP_VacuumsensorFailure_C162B01 = (data[7] & 0x04) >> 2;
        ESP_Wheelspeedsensorfrontleft_C1 = (data[7] & 0x08) >> 3;
        ESP_Wheelspeedsensorfrontright_C = (data[7] & 0x10) >> 4;
        ESP_Wheelspeedsensorrearleft_C16 = (data[7] & 0x20) >> 5;
        ESP_Wheelspeedsensorrearright_C1 = (data[7] & 0x40) >> 6;
        ESP_EPBLActuatorElectricalFailur = (data[7] & 0x80) >> 7;
    }

public:
    unsigned char ESP_PumpMotorControl_C160071;     //  : 0|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorFLMultiFailures_; //  : 1|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorFLShortGND_C1602; // : 2|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorFLOpen_C160313;   // : 3|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorFLShortUbatt_C16; // : 4|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorFRMultiFailures_; // : 5|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorFRShortGND_C1606; // : 6|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorFROpen_C160713;   // : 7|1@0+ (1,0) [0|1] "" Vector__XXX

    unsigned char ESP_WhlSpdSensorFRShortUbatt_C16; // : 8|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorRLMultiFailures_; // : 9|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorRLShortGND_C160A; // : 10|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorRLOpen_C160B13;   // : 11|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorRLShortUbatt_C16; // : 12|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorRRMultiFailures_; // : 13|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorRRShortGND_C160E; // : 14|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorRROpen_C160F13;   // : 15|1@0+ (1,0) [0|1] "" Vector__XXX

    unsigned char ESP_WhlSpdSensorRRShortUbatt_C16; // : 16|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_StrngWhlSensorFlt_C161128;    // : 17|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_LateralAccSensorSigFlt_C1612; // : 18|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_LongitudinalAccSnsorSigFlt_C; // : 19|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_YRSSigFlt_C161429;            // : 20|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_ABSESPImplausibleCtrl_C16150; // : 21|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_PATASwitchFailure_C161609;    // : 22|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WhlSpdSensorGeneralFailure_C; // : 23|1@0+ (1,0) [0|1] "" Vector__XXX

    unsigned char ESP_VvlRelayFailure_C161804;     // : 24|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_VvlGeneralFailure_C161904;   // : 25|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_VaraintCodeFault_C161A00;    // : 26|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_CtrlModuleComBusOff_U007388; // : 27|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_CANBusOffError_U007488;      // : 28|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_LostComWithVCU_U160087;      // : 29|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_LostComWithADAS_U160187;     // : 30|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_LostComWithACU_U160287;      // : 31|1@0+ (1,0) [0|1] "" Vector__XXX

    unsigned char ESP_LostComWithEPS_U160387;   // : 32|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_LostComWithMCUF_U160487;  // : 33|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_LostComWithMCUR_U160587;  // : 34|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_LostComWithGW_U160687;    // : 35|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_LostComWithIPC_U160787;   // : 36|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_BatVolHigh_U160817;       // : 37|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_BatVolLow_U160916;        // : 38|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_ECUHardwareError_U160A04; // : 39|1@0+ (1,0) [0|1] "" Vector__XXX

    unsigned char ESP_ECUSoftwareError_U160B04;     // : 40|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_CANHardwareError_U160C04;     // : 41|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_InvalidSigRxfromVCU_C161B86;  // : 42|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_InvalidSigRxfromADAS_C161C86; // : 43|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_InvalidSigRxfromACU_C161D86;  // : 44|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_InvalidSigRxfromEPS_C161E86;  // : 45|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_InvalidSigRxfromMCUF_C161F86; // : 46|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_InvalidSigRxfromMCUR_C162086; // : 47|1@0+ (1,0) [0|1] "" Vector__XXX

    unsigned char ESP_InvalidSigRxfromGW_C162186;   // : 48|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_InvalidSigRxfromIPC_C162286;  // : 49|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_BLSSigPlausibility_C162302;   // : 50|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_Overheatfault_C162402;        // : 51|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_hydraulicunitisworn_C162502;  // : 52|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_PressuresensorfaultLine_C162; // : 53|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_PressuresensorfaultSig_C1627; // : 54|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_ReverseGearabnormal_C162802;  // : 55|1@0+ (1,0) [0|1] "" Vector__XXX

    unsigned char ESP_SASCalibrationError_C162946;  // : 56|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_SystemInitnotComplete_C162A0; // : 57|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_VacuumsensorFailure_C162B01;  // : 58|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_Wheelspeedsensorfrontleft_C1; // : 59|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_Wheelspeedsensorfrontright_C; // : 60|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_Wheelspeedsensorrearleft_C16; // : 61|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_Wheelspeedsensorrearright_C1; // : 62|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_EPBLActuatorElectricalFailur; // : 63|1@0+ (1,0) [0|1] "" Vector__XXX
};

class ESP_0x675 : public Basic_Msg {
public:
    using Ptr = std::shared_ptr<ESP_0x675>;
    using ConstPtr = std::shared_ptr<const ESP_0x675>;

public:
    ESP_0x675() {
        Msg_Name = "ESP_0x675";
        Msg_Id = 0x675; // 十进制: 1653
    }

    virtual ~ESP_0x675() {
        // TODO...
    }

public:
    virtual void parseSFF(const unsigned char *data) {
        ESP_EPBLActuatorFailure_C163101 = data[0] & 0x01;
        ESP_EPBRActuatorElectricalFailur = (data[0] & 0x02) >> 1;
        ESP_EPBRightActuatorFailure_C163 = (data[0] & 0x04) >> 2;
        ESP_ElectricalFailureonEPBActuat = (data[0] & 0x08) >> 3;
        ESP_EPBButtonCirMalFunction_C163 = (data[0] & 0x10) >> 4;
        ESP_EPBLActuatorOverCurrent_C163 = (data[0] & 0x20) >> 5;
        ESP_EPBRActuatorOverCurrent_C163 = (data[0] & 0x40) >> 6;
        ESP_PBCShadowMemoryError_C163801 = (data[0] & 0x80) >> 7;

        ESP_APBButtonBlocked_C163971 = data[1] & 0x01;
        ESP_UnexpectedActuatorModesatIni = (data[1] & 0x02) >> 1;
        ESP_MotorStallDetectedLeftMoC_C1 = (data[1] & 0x04) >> 2;
        ESP_NoMotorStartDetectedRightMoC = (data[1] & 0x08) >> 3;
        ESP_InsufficientForceIncreaseDet = (data[1] & 0x10) >> 4;
        ESP_SuspiciousMotorCharacteristi = (data[1] & 0x20) >> 5;
        ESP_NoForceIncreaseDetectedRight = (data[1] & 0x40) >> 6;
        ESP_ExcessiveFrictionDetectedRig = (data[1] & 0x80) >> 7;

        ESP_EmergencyReleaseDetectedRigh = data[2] & 0x01;
        ESP_MotorStallDetectedRightMoC_C = (data[2] & 0x02) >> 1;
        ESP_ImplausibleActuatorModes_C16 = (data[2] & 0x04) >> 2;
        ESP_HPSnotRealized_C164400 = (data[2] & 0x08) >> 3;
        ESP_NoMotorStartDetectedLeftMoC_ = (data[2] & 0x10) >> 4;
        ESP_InsufficientForceIncrea_0000 = (data[2] & 0x20) >> 5;
        ESP_SuspiciousMotorCharacte_0000 = (data[2] & 0x40) >> 6;
        ESP_NoForceIncreaseDetectedLeftM = (data[2] & 0x80) >> 7;

        ESP_ExcessiveFrictionDetectedLef = data[3] & 0x01;
        ESP_EmergencyReleaseDetectedLeft = (data[3] & 0x02) >> 1;
        ESP_LossComWithIB_U160D87 = (data[3] & 0x04) >> 2;
        ESP_InvalidSigRxfromIB_U160E86 = (data[3] & 0x08) >> 3;
    }

public:
    unsigned char ESP_EPBLActuatorFailure_C163101;  // : 0|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_EPBRActuatorElectricalFailur; // : 1|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_EPBRightActuatorFailure_C163; // : 2|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_ElectricalFailureonEPBActuat; // : 3|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_EPBButtonCirMalFunction_C163; // : 4|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_EPBLActuatorOverCurrent_C163; // : 5|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_EPBRActuatorOverCurrent_C163; // : 6|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_PBCShadowMemoryError_C163801; // : 7|1@0+ (1,0) [0|1] "" Vector__XXX

    unsigned char ESP_APBButtonBlocked_C163971;     // : 8|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_UnexpectedActuatorModesatIni; // : 9|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_MotorStallDetectedLeftMoC_C1; // : 10|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_NoMotorStartDetectedRightMoC; // : 11|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_InsufficientForceIncreaseDet; // : 12|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_SuspiciousMotorCharacteristi; // : 13|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_NoForceIncreaseDetectedRight; // : 14|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_ExcessiveFrictionDetectedRig; // : 15|1@0+ (1,0) [0|1] "" Vector__XXX

    unsigned char ESP_EmergencyReleaseDetectedRigh; // : 16|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_MotorStallDetectedRightMoC_C; // : 17|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_ImplausibleActuatorModes_C16; // : 18|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_HPSnotRealized_C164400;       // : 19|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_NoMotorStartDetectedLeftMoC_; // : 20|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_InsufficientForceIncrea_0000; // : 21|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_SuspiciousMotorCharacte_0000; // : 22|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_NoForceIncreaseDetectedLeftM; // : 23|1@0+ (1,0) [0|1] "" Vector__XXX

    unsigned char ESP_ExcessiveFrictionDetectedLef; // : 24|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_EmergencyReleaseDetectedLeft; // : 25|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_LossComWithIB_U160D87;        // : 26|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_InvalidSigRxfromIB_U160E86;   // : 27|1@0+ (1,0) [0|1] "" Vector__XXX
};

class ESP_0x7CF : public Basic_Msg {
public:
    using Ptr = std::shared_ptr<ESP_0x7CF>;
    using ConstPtr = std::shared_ptr<const ESP_0x7CF>;

public:
    ESP_0x7CF() {
        Msg_Name = "ESP_0x7CF";
        Msg_Id = 0x7CF; // 十进制: 1999
    }

    virtual ~ESP_0x7CF() {
        // TODO...
    }

public:
    virtual void parseSFF(const unsigned char *data) {
        ESP_DiagRespByte0 = data[0];
        ESP_DiagRespByte1 = data[1];
        ESP_DiagRespByte2 = data[2];
        ESP_DiagRespByte3 = data[3];
        ESP_DiagRespByte4 = data[4];
        ESP_DiagRespByte5 = data[5];
        ESP_DiagRespByte6 = data[6];
        ESP_DiagRespByte7 = data[7];
    }

public:
    unsigned char ESP_DiagRespByte0; // : 7|8@0+ (1,0) [0|255] ""  GW_CH,EGW_CH
    unsigned char ESP_DiagRespByte1; // : 15|8@0+ (1,0) [0|255] ""  GW_CH,EGW_CH
    unsigned char ESP_DiagRespByte2; // : 23|8@0+ (1,0) [0|255] ""  GW_CH,EGW_CH
    unsigned char ESP_DiagRespByte3; // : 31|8@0+ (1,0) [0|255] ""  GW_CH,EGW_CH
    unsigned char ESP_DiagRespByte4; // : 39|8@0+ (1,0) [0|255] ""  GW_CH,EGW_CH
    unsigned char ESP_DiagRespByte5; // : 47|8@0+ (1,0) [0|255] ""  GW_CH,EGW_CH
    unsigned char ESP_DiagRespByte6; // : 55|8@0+ (1,0) [0|255] ""  GW_CH,EGW_CH
    unsigned char ESP_DiagRespByte7; //: 63|8@0+ (1,0) [0|255] ""  GW_CH,EGW_CH
};

class ESP_0x460 : public Basic_Msg {
public:
    using Ptr = std::shared_ptr<ESP_0x460>;
    using ConstPtr = std::shared_ptr<const ESP_0x460>;

public:
    ESP_0x460() {
        Msg_Name = "ESP_0x460";
        Msg_Id = 0x460; // 十进制: 1120
    }

    virtual ~ESP_0x460() {
        // TODO...
    }

public:
    virtual void parseSFF(const unsigned char *data) {
        ESP_SourceAddress = data[0];
        ESP_RepeatMessageReq = data[1] & 0x01;
        ESP_ActiveWakeupStatus = data[1] & 0x10 >> 4;
        ESP_NMStatus = data[2] & 0x01;
        ESP_WakeupReasons = data[3];
        ESP_StaywakeReasons1_PowerStatus = data[4] & 0x01;
        ESP_StaywakeReasons2_Diagnosis = (data[4] & 0x02) >> 1;
    }

public:
    unsigned char ESP_SourceAddress;                //  : 7|8@0+ (1,0) [0|255] ""  Ibooster,ADAS,ACU,EPS,GW_CH,EGW_CH
    unsigned char ESP_RepeatMessageReq;             // : 8|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_ActiveWakeupStatus;           //  : 12|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_NMStatus;                     //  : 16|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_WakeupReasons;                // : 31|8@0+ (1,0) [0|255] "" Vector__XXX
    unsigned char ESP_StaywakeReasons1_PowerStatus; // : 32|1@0+ (1,0) [0|1] "" Vector__XXX
    unsigned char ESP_StaywakeReasons2_Diagnosis;   //  : 33|1@0+ (1,0) [0|1] "" Vector__XXX
};

}  // namespace LX_CAN
}  // namespace sensor
}  // namespace robosense

#endif  // RS_SENSOR_CAN_LXONE_ONLINE_LX_CAN_MSG_H
