#ifndef __MAIN_H
#define __MAIN_H

#include "gd32f4xx.h"

#define MODULE_ENB_DEBUG_PRINT
//系统支持的网口数量，SW_PORTS_NUMBER = 外接网口数量 + 1(此为MCU连接交换机的Port0,外接网口从Port1开始)
#define SW_PORTS_NUMBER                     6
#define SW_SOFTWARE_VERSION                 "V1.1 Build "__DATE__" "    //不大于26 byte
#define SW_HARDWARE_VERSION                 "ROB042_MB201_V1.0"         //不大于20 byte
#define SW_DEV_NAME                         "ROB042_MB201"              //不大于16 byte

#define DEV_NAME                            "desc"
#define MAC_SNAME                           "mac"
#define IP_SNAME                            "ip"
#define NETMASK_SNAME                       "nm"
#define GATEWAY_SNAME                       "gw"
#define SW_SNAME                            "sw_ver"
#define HW_SNAME                            "hw_ver"
#define PVL_STATE                           "pvl_s"
#define PVLT_GR                             "tb"
#define CHECK_CRC                           "ck_crc"

#define CRC16_TABLE_CODE                    5
#define FILE_MAX_BYTE                       256

#define RMII_MODE
//#define MII_MODE

#ifdef  MII_MODE
#define PHY_CLOCK_MCO
#endif

void MX_FREERTOS_Init(void);
void StartDefaultTask(void const * argument);
void run_application_loop(void);
void set_reboot(uint16_t time);
void reboot(void);
int sys_default(void);
void StartDefaultTask(void const * argument);
#endif /* __MAIN_H */


