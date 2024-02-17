//——————————————————————————————————————————————————————————————————————————————
// Description: CAN Driver Implementation
// Author: Adam Saiyad, Julius Calzada (julius.jai@gmail.com)
// Revision: v1.2.8
// 06.28.2022: Migration to ESP32 Arduino environment
// 10.03.2022: Integrated earlyversion.c, can-bridge-firmware.h & nissan-can-structs.h
// 10.28.2022: Separated LEAF from ENV200
// 10.31.2022: Improved Web Request Processing
// 11.11.2022: Created separate cpp files for can_bridge_manager common, leaf and env200
// 11.18.2022: Added CURRENT_CONTROL_ENABLED equivalent to CHARGECURRENT from leaf-can-bridge-3-port-master project
// 11.26.2022: Integrated configurable parameters for: BATTERY_SAVER_ENABLED/DISABLED, GLIDE_IN_DRIVE_ENABLED/DISABLED, 
// 12.02.2022: Updated CHARGECURRENT implementation for ID0x54B using CurrentControl Web parameters
// 12.04.2022: Merging of Inverter Upgrade based on https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/can-bridge-inverter.c
// 12.06.2022: Updated Charge Current logic - 1) Start conditions are charging state and fan speed; 2) Display kW for 15sec and revert to SOC
// 12.31.2022: Fix charge current functions Fix regen power and motor power Fix Glide and drive add code 
//——————————————————————————————————————————————————————————————————————————————

//——————————————————————————————————————————————————————————————————————————————
// IMPORTANT:
// 1. Download ACAN2515 library from:
//    https://www.arduino.cc/reference/en/libraries/acan2515/
// 2. Refer link on how to install the library:
//    https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries
//——————————————————————————————————————————————————————————————————————————————

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <ACAN2515.h>
#include "canframe.h"
#include "config.h"

void CAN_Init(void);

#ifdef CAN_CH0_ENABLED
bool CAN0_Transmit(CANMessage frame);
bool CAN0_NewFrameIsAvailable(void);
void CAN0_ReadNewFrame(CANMessage &frame);
#endif //CAN_CH0_ENABLED

#ifdef CAN_CH1_ENABLED
bool CAN1_Transmit(CANMessage frame);
bool CAN1_NewFrameIsAvailable(void);
void CAN1_ReadNewFrame(CANMessage &frame);
#endif //CAN_CH1_ENABLED

#ifdef CAN_CH2_ENABLED
void CAN2_Init(void);
void CAN2_onReceive(int packetSize);
bool CAN2_Transmit(can_frame tx_frame);
#endif //CAN_CH2_ENABLED

#endif //CAN_DRIVER_H
//——————————————————————————————————————————————————————————————————————————————
