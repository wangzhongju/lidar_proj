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
// 接口卡类型定义
#define VCI_PCI5121 1
#define VCI_PCI9810 2
#define VCI_USBCAN1 3
#define VCI_USBCAN2 4
#define VCI_PCI9820 5
#define VCI_CAN232  6
#define VCI_PCI5110 7
#define VCI_CANLite 8
#define VCI_ISA9620 9
#define VCI_ISA5420 10

// CAN错误码
#define ERR_CAN_OVERFLOW 0x0001  //  CAN控制器内部FIFO溢出
#define ERR_CAN_ERRALARM 0x0002     // CAN控制器错误报警
#define ERR_CAN_PASSIVE  0x0004     //  CAN控制器消极错误
#define ERR_CAN_LOSE     0x0008     //  CAN控制器仲裁丢失
#define ERR_CAN_BUSERR   0x0010     //  CAN控制器总线错误

// 通用错误码
#define ERR_DEVICEOPENED   0x0100  //  设备已经打开
#define ERR_DEVICEOPEN     0x0200  //  打开设备错误
#define ERR_DEVICENOTOPEN  0x0400  //  设备没有打开
#define ERR_BUFFEROVERFLOW 0x0800  //  缓冲区溢出
#define ERR_DEVICENOTEXIST 0x1000  //  此设备不存在
#define ERR_LOADKERNELDLL  0x2000  //  装载动态库失败
#define ERR_CMDFAILED      0x4000  //  执行命令失败错误码
#define ERR_BUFFERCREATE   0x8000  //  内存不足


// 函数调用返回状态值
#define    STATUS_OK  1
#define STATUS_ERR 0

using USHORT = uint16_t;
using BYTE = unsigned char;
using CHAR = char;
using UCHAR = unsigned char;
using UINT = unsigned int;
using DWORD = unsigned int;
using PVOID = void *;
using ULONG = unsigned int;

#define LPVOID void*
#define BOOL BYTE
#define TRUE 1
#define FALSE 0


#if 1
// 1.ZLGCAN系列接口卡信息的数据类型。
typedef struct _VCI_BOARD_INFO {
    USHORT hw_Version;
    USHORT fw_Version;
    USHORT dr_Version;

    USHORT in_Version;
    USHORT irq_Num;
    BYTE can_Num;
    CHAR str_Serial_Num[20];
    CHAR str_hw_Type[40];
    USHORT Reserved[4];
} VCI_BOARD_INFO, *PVCI_BOARD_INFO;

// 2.定义CAN信息帧的数据类型。
typedef struct _VCI_CAN_OBJ {
    UINT ID;
    UINT TimeStamp;
    BYTE TimeFlag;
    BYTE SendType;
    BYTE RemoteFlag;  // 是否是远程帧
    BYTE ExternFlag;  // 是否是扩展帧
    BYTE DataLen;
    BYTE Data[8];
    BYTE Reserved[3];
} VCI_CAN_OBJ, *PVCI_CAN_OBJ;

// 3.定义CAN控制器状态的数据类型。
typedef struct _VCI_CAN_STATUS {
    UCHAR ErrInterrupt;
    UCHAR regMode;
    UCHAR regStatus;
    UCHAR regALCapture;
    UCHAR regECCapture;
    UCHAR regEWLimit;
    UCHAR regRECounter;
    UCHAR regTECounter;
    DWORD Reserved;
} VCI_CAN_STATUS, *PVCI_CAN_STATUS;

// 4.定义错误信息的数据类型。
typedef struct _ERR_INFO {
    UINT ErrCode;
    BYTE Passive_ErrData[3];
    BYTE ArLost_ErrData;
} VCI_ERR_INFO, *PVCI_ERR_INFO;

// 5.定义初始化CAN的数据类型
typedef struct _INIT_CONFIG {
    DWORD AccCode;
    DWORD AccMask;
    DWORD Reserved;
    UCHAR Filter;
    UCHAR Timing0;
    UCHAR Timing1;
    UCHAR Mode;
} VCI_INIT_CONFIG, *PVCI_INIT_CONFIG;

#ifdef __cplusplus
extern "C" {
#endif


DWORD VCI_FindUsbDevice(PVCI_BOARD_INFO pInfo);
DWORD VCI_FindUsbDevice111(PVCI_BOARD_INFO pInfo);
DWORD VCI_ConnectDevice(DWORD DeviceType, DWORD DeviceInd);

DWORD VCI_OpenDevice(DWORD DeviceType, DWORD DeviceInd, DWORD Reserved);
DWORD VCI_CloseDevice(DWORD DeviceType, DWORD DeviceInd);
DWORD VCI_InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig);

DWORD VCI_ReadBoardInfo(DWORD DeviceType, DWORD DeviceInd, PVCI_BOARD_INFO pInfo);
DWORD VCI_ReadErrInfo(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_ERR_INFO pErrInfo);
DWORD VCI_ReadCANStatus(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_STATUS pCANStatus);

DWORD VCI_GetReference(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, DWORD RefType, PVOID pData);
DWORD VCI_SetReference(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, DWORD RefType, PVOID pData);

ULONG VCI_GetReceiveNum(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);
DWORD VCI_ClearBuffer(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);

DWORD VCI_StartCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);
DWORD VCI_ResetCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);

ULONG VCI_Transmit(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pSend, UINT Len);
ULONG VCI_Receive(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pReceive, UINT Len, int WaitTime);

#ifdef __cplusplus
}
#endif
#endif


