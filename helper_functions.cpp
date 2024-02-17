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

//——————————————————————————————————————————————————————————————————————————————
#include <Arduino.h>
#include "helper_functions.h"
#include "canframe.h"

//——————————————————————————————————————————————————————————————————————————————
uint8_t	crctable[256] = {0,133,143,10,155,30,20,145,179,54,60,185,40,173,167,34,227,102,108,233,120,253,247,114,80,213,223,90,203,78,68,193,67,198,204,73,216,93,87,210,240,117,127,250,107,238,228,97,160,37,47,170,59,190,180,49,19,150,156,25,136,13,7,130,134,3,9,140,29,152,146,23,53,176,186,63,174,43,33,164,101,224,234,111,254,123,113,244,214,83,89,220,77,200,194,71,197,64,74,207,94,219,209,84,118,243,249,124,237,104,98,231,38,163,169,44,189,56,50,183,149,16,26,159,14,139,129,4,137,12,6,131,18,151,157,24,58,191,181,48,161,36,46,171,106,239,229,96,241,116,126,251,217,92,86,211,66,199,205,72,202,79,69,192,81,212,222,91,121,252,246,115,226,103,109,232,41,172,166,35,178,55,61,184,154,31,21,144,1,132,142,11,15,138,128,5,148,17,27,158,188,57,51,182,39,162,168,45,236,105,99,230,119,242,248,125,95,218,208,85,196,65,75,206,76,201,195,70,215,82,88,221,255,122,112,245,100,225,235,110,175,42,32,165,52,177,187,62,28,153,147,22,135,2,8,141};

//——————————————————————————————————————————————————————————————————————————————
//print standard ID (11-bit) to string
//——————————————————————————————————————————————————————————————————————————————
void SID_to_str(char * str, uint32_t num){
  uint8_t tmp;
	str += 2;
	tmp = num & 0xF;
	if(tmp > 9){ *str-- = 55 + tmp; } else { *str-- = 48 + tmp; }
	tmp = (num & 0xF0) >> 4;
	if(tmp > 9){ *str-- = 55 + tmp; } else { *str-- = 48 + tmp; }
	num >>= 8;
	tmp = num & 0xF;
	if(tmp > 9){ *str = 55 + tmp; } else { *str = 48 + tmp; }
}

//——————————————————————————————————————————————————————————————————————————————
//print uint32 to hex value in a string
//——————————————————————————————————————————————————————————————————————————————
void uint32_to_str(char * str, uint32_t num){
	num = num & 0x1FFFFFFF;
	uint8_t tmp;
	str += 7;
	for(uint8_t i = 0; i < 4; i++){
		tmp = num & 0xF;
		if(tmp > 9){ *str-- = 55 + tmp; } else { *str-- = 48 + tmp; }
		tmp = (num & 0xF0) >> 4;
		if(tmp > 9){ *str-- = 55 + tmp; } else { *str-- = 48 + tmp; }
		num >>= 8;
	}
}

//——————————————————————————————————————————————————————————————————————————————
//CAN frame to string translation
//——————————————————————————————————————————————————————————————————————————————
void canframe_to_str(char * str, can_frame_t frame){
	uint8_t tmp;
	for(uint8_t i = 0; i < frame.can_dlc; i++){
		tmp = (frame.data[i] & 0xF0) >> 4;
		if(tmp > 9){ *str++ = 55 + tmp; } else { *str++ = 48 + tmp; }
		tmp = frame.data[i] & 0xF;
		if(tmp > 9){ *str++ = 55 + tmp; } else { *str++ = 48 + tmp; }
	}
}

//——————————————————————————————————————————————————————————————————————————————
//Integer to hex translation
//——————————————————————————————————————————————————————————————————————————————
void int_to_hex(char * str, int num){
	uint8_t tmp;
	tmp = (num & 0xF0) >> 4;
	if(tmp > 9){ *str++ = 55 + tmp; } else { *str++ = 48 + tmp; }
	tmp = num & 0xF;
	if(tmp > 9){ *str++ = 55 + tmp; } else { *str++ = 48 + tmp; }
}

//——————————————————————————————————————————————————————————————————————————————
//recalculates the CRC-8 with 0x85 poly
//——————————————————————————————————————————————————————————————————————————————
void calc_crc8(can_frame_t *frame){
	uint8_t crc = 0;
	for(uint8_t i = 0; i < 7; i++){
		crc = crctable[(crc ^ ((int) (*frame).data[i])) % 256];
	}
	(*frame).data[7] = crc;
}

//——————————————————————————————————————————————————————————————————————————————
//calc_sum4
//——————————————————————————————————————————————————————————————————————————————
void calc_sum4(can_frame_t *frame){
	uint8_t sum = 0;
	for(uint8_t i = 0; i < 7; i++){
		sum += (*frame).data[i] >> 4;
		sum += (*frame).data[i] & 0xF;
	}
	sum = (sum + 2) & 0xF;
	(*frame).data[7] = ((*frame).data[7] & 0xF0) + sum;
}

//——————————————————————————————————————————————————————————————————————————————
//int_to_3digit
//——————————————————————————————————————————————————————————————————————————————
void int_to_3digit(int num, char * buffer){
	if(num < 0) num = -num;
	uint16_t res = 100;
	for(uint8_t i = 0; i < 3; i++){
		*buffer++ = 48 + (num / res);
		num = num % res;
		res /= 10;
	}
}

//——————————————————————————————————————————————————————————————————————————————
//int_to_4digit
//——————————————————————————————————————————————————————————————————————————————
void int_to_4digit(int num, char * buffer){
	if(num < 0) num = -num;
	uint16_t res = 1000;
	for(uint8_t i = 0; i < 4; i++){
		if(i == 3) *buffer++ = 46;
		*buffer++ = 48 + (num / res);
		num = num % res;
		res /= 10;
	}
}

//——————————————————————————————————————————————————————————————————————————————
//int_to_4digit_nodec
//——————————————————————————————————————————————————————————————————————————————
void int_to_4digit_nodec(int num, char * buffer){
	if(num < 0) num = -num;
	uint16_t res = 1000;
	for(uint8_t i = 0; i < 4; i++){
		*buffer++ = 48 + (num / res);
		num = num % res;
		res /= 10;
	}
}

//——————————————————————————————————————————————————————————————————————————————
//int_to_5digit
//——————————————————————————————————————————————————————————————————————————————
void int_to_5digit(int num, char * buffer){
	uint16_t res = 10000;
	for(uint8_t i = 0; i < 5; i++){
		*buffer++ = 48 + (num / res);
		num = num % res;
		res /= 10;
	}
}

//——————————————————————————————————————————————————————————————————————————————
//reads signature bytes
//ToDo: To confirm the usage of this function. Currently not used.
//——————————————————————————————————————————————————————————————————————————————
void NVM_GetGUID(uint8_t * b) {
	enum {
		LOTNUM0=8,  // Lot Number Byte 0, ASCII
		LOTNUM1,    // Lot Number Byte 1, ASCII
		LOTNUM2,    // Lot Number Byte 2, ASCII
		LOTNUM3,    // Lot Number Byte 3, ASCII
		LOTNUM4,    // Lot Number Byte 4, ASCII
		LOTNUM5,    // Lot Number Byte 5, ASCII
		WAFNUM =16, // Wafer Number
		COORDX0=18, // Wafer Coordinate X Byte 0
		COORDX1,    // Wafer Coordinate X Byte 1
		COORDY0,    // Wafer Coordinate Y Byte 0
		COORDY1,    // Wafer Coordinate Y Byte 1
	};
	b[ 0]=ReadCalibrationByte(LOTNUM0);
	b[ 1]=ReadCalibrationByte(LOTNUM1);
	b[ 2]=ReadCalibrationByte(LOTNUM2);
	b[ 3]=ReadCalibrationByte(LOTNUM3);
	b[ 4]=ReadCalibrationByte(LOTNUM4);
	b[ 5]=ReadCalibrationByte(LOTNUM5);
	b[ 6]=ReadCalibrationByte(WAFNUM );
	b[ 7]=ReadCalibrationByte(COORDX0);
	b[ 8]=ReadCalibrationByte(COORDX1);
	b[ 9]=ReadCalibrationByte(COORDY0);
	b[10]=ReadCalibrationByte(COORDY1);
}

void convert_5bc_to_array(volatile Leaf_2011_5BC_message * src, uint8_t * dest){
  dest[0] = (uint8_t) (src->LB_CAPR >> 2);
  dest[1] = (uint8_t) (((src->LB_CAPR << 6) & 0xC0) | ((src->LB_FULLCAP >> 4) & 0x1F));
  dest[2] = (uint8_t) (((src->LB_FULLCAP << 4) & 0xF0) | ((src->LB_CAPSEG) & 0x0F));
  dest[3] = (uint8_t) (src->LB_AVET);
  dest[4] = (uint8_t) (((src->LB_SOH << 1) & 0xFE) | ((src->LB_CAPSW) & 1));
  dest[5] = (uint8_t) (((src->LB_RLIMIT << 5) & 0xE0) | ((src->LB_CAPBALCOMP << 2) & 4) | ((src->LB_RCHGTCON >> 3) & 3));
  dest[6] = (uint8_t) (((src->LB_RCHGTCON << 5) & 0xE0) | ((src->LB_RCHGTIM >> 8) & 0x1F));
  dest[7] = (uint8_t) (src->LB_RCHGTIM);
}

void convert_array_to_5bc(volatile Leaf_2011_5BC_message * dest, uint8_t * src){
  dest->LB_CAPR = (src[0] << 2) | (src[1] & 0xC0 >> 6);
}

void convert_5c0_to_array(volatile Leaf_2011_5C0_message * src, uint8_t * dest){
  dest[0] = (src->LB_HIS_DATA_SW << 6) | src->LB_HIS_HLVOL_TIMS;
  dest[1] = src->LB_HIS_TEMP_WUP << 1;
  dest[2] = src->LB_HIS_TEMP << 1;
  dest[3] = src->LB_HIS_INTG_CUR;
  dest[4] = src->LB_HIS_DEG_REGI << 1;
  dest[5] = src->LB_HIS_CELL_VOL << 2;
  dest[7] = src->LB_DTC;
}

//——————————————————————————————————————————————————————————————————————————————
//Read Non volatile calibration data
//ToDo: To confirm the usage of this function. Currently NVM data storage.
//——————————————————————————————————————————————————————————————————————————————
uint8_t ReadCalibrationByte( uint8_t index ){
	uint8_t result = 0u;
	return result;
}

//——————————————————————————————————————————————————————————————————————————————
//Global Timer Function in Seconds
//——————————————————————————————————————————————————————————————————————————————
static uint32_t globalTimer = 0;
void TIMER_Start(void)
{
  globalTimer = 0;
}

bool TIMER_Expired(uint32_t durationInSec)
{
    bool retValue = false;
    if(globalTimer >= durationInSec)
    {
       retValue = true;
    }

    return retValue;
}

void TIMER_Count(void)
{
  if(globalTimer < UINT32_MAX)
  {
    globalTimer++;
  }  
}
