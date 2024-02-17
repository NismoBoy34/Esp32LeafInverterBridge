//——————————————————————————————————————————————————————————————————————————————
// Description: 3-bus transparent CAN bridge with Battery upgrade SW for Nissan env200
// Author: Adam Saiyad, Julius Calzada (julius.jai@gmail.com)
// Revision: v1.2.8
// 06.28.2022: Migration to ESP32 Arduino environment
// 10.03.2022: Integrated earlyversion.c, can-bridge-firmware.h & nissan-can-structs.h
// 10.25.2022: v1.1.4: Integrated API for OTA to allow runtime configurability of vehicle parameters
// 10.28.2022: Separated LEAF from ENV200
// 10.31.2022: Improved Web Request Processing
// 11.10.2022: Completed dynamic configurations for the following: LEAF_2011, LEAF_2014, E_NV_200, BATTERY_SWAP & BATTERY_SAVER
// 11.11.2022: Created separate cpp files for can_bridge_manager common, leaf and env200
// 11.18.2022: Added CURRENT_CONTROL_ENABLED equivalent to CHARGECURRENT from leaf-can-bridge-3-port-master project
// 11.26.2022: Integrated configurable parameters for: BATTERY_SAVER_ENABLED/DISABLED, GLIDE_IN_DRIVE_ENABLED/DISABLED, 
// 12.02.2022: Updated CHARGECURRENT implementation for ID0x54B using CurrentControl Web parameters
// 12.04.2022: Merging of Inverter Upgrade based on https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/can-bridge-inverter.c
// 12.06.2022: Updated Charge Current logic - 1) Start conditions are charging state and fan speed; 2) Display kW for 15sec and revert to SOC
// 12.31.2022: Fix charge current functions Fix regen power and motor power Fix Glide and drive add code 
//——————————————————————————————————————————————————————————————————————————————

#ifndef CAN_BRIDGE_MANAGER_COMMON_H
#define CAN_BRIDGE_MANAGER_COMMON_H

#include "canframe.h"
#include "config.h"

//function prototypes
void hw_init(void);

void SID_to_str(char * str, uint32_t num);
void canframe_to_str(char * str, can_frame_t frame);

void buffer_send_can0(can_frame_t frame);
#ifdef CAN_CH0_ENABLED
void buffer_check_can0(void);
bool direct_send_can0(can_frame_t frame);
#endif //CAN_CH0_ENABLED

void buffer_send_can1(can_frame_t frame);
#ifdef CAN_CH1_ENABLED
void buffer_check_can1(void);
bool direct_send_can1(can_frame_t frame);
#endif //CAN_CH1_ENABLED

void buffer_send_can2(can_frame_t frame);
#ifdef CAN_CH2_ENABLED
void buffer_check_can2(void);
bool direct_send_can2(can_frame_t frame);
#endif //#ifdef CAN_CH2_ENABLED

void Schedule_Buffer_Check_CAN(void);

#endif //CAN_BRIDGE_MANAGER_COMMON_H
