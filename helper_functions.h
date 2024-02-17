//——————————————————————————————————————————————————————————————————————————————
// Description: Helper function for string format & calibration data
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
#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <Arduino.h>
#include "canframe.h"

void uint32_to_str(char * str, uint32_t num);
uint8_t ReadCalibrationByte( uint8_t index );
void int_to_5digit(int num, char * buffer);
void int_to_4digit(int num, char * buffer);
void int_to_3digit(int num, char * buffer);
void int_to_4digit_nodec(int num, char * buffer);
void int_to_hex(char * str, int num);
void calc_crc8(can_frame_t *frame);
void calc_sum4(can_frame_t *frame);
void convert_5bc_to_array(volatile Leaf_2011_5BC_message * src, uint8_t * dest);
void convert_array_to_5bc(volatile Leaf_2011_5BC_message * dest, uint8_t * src);
void convert_5c0_to_array(volatile Leaf_2011_5C0_message * src, uint8_t * dest);

void TIMER_Start(void);
bool TIMER_Expired(uint32_t durationInSec);
void TIMER_Count(void);

#endif //HELPER_FUNCTIONS_H
