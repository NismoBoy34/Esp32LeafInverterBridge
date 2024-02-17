//——————————————————————————————————————————————————————————————————————————————
// Description: CAN message frame format
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

#ifndef CANFRAME_H
#define CANFRAME_H

#include <Arduino.h>

typedef struct can_frame can_frame_t;

/**
 * struct can_frame - basic CAN frame structure
 * @can_id:  CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
 * @can_dlc: frame payload length in byte (0 .. 8) aka data length code
 *           N.B. the DLC field from ISO 11898-1 Chapter 8.4.2.3 has a 1:1
 *           mapping of the 'data length code' to the real payload length
 * @__pad:   padding
 * @__res0:  reserved / padding
 * @__res1:  reserved / padding
 * @data:    CAN frame payload (up to 8 byte)
 */
#define CAN_MAX_DLEN		8

struct can_frame{
	uint32_t	can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t		can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */        
	uint8_t		data[CAN_MAX_DLEN];
};


/*
** Added from nissan-can-structs.h
*/

typedef struct {
	int			UNKNOWN_BYTE0:8;
	int			UNKNOWN_BYTE1:8;
	int			UNKNOWN_BYTE2:8;
	int			UNKNOWN_BYTE3:8;
	int			PRUN:2;
	int			UNKNOWN_BYTE4:3;
	int			BTONFN:1;
	int			UNKNOWN_BYTE45:3;
	int			RLYP:1;
	int			UNKNOWN_BYTE5:6;
	int			CRC8:8;
} Leaf_2011_1D4_message;

typedef struct {
	int			UNKNOWN_BYTE0:8;
	int			UNKNOWN_BYTE1:8;
	int			UNKNOWN_BYTE2:5;
	int			CANMASK:1;
	int			UNKNOWN_BYTE2_2:2;
	int			VCM_WakeUpSleepCmd:2;
	int			UNKNOWN_BYTE3:6;
	int			UNKNOWN_BYTE4:8;
	int			UNKNOWN_BYTE5:8;
} Leaf_2011_50B_message;

typedef struct {
	int			UNKNOWN_BYTE0:8;
	int			UNKNOWN_BYTE1:8;
	int			UNKNOWN_BYTE2:8;
	int			UNKNOWN_BYTE3:6;
	int			PRUN:2;
	int			ALU_Q_LBC:8;
	int			CRC8:8;
} Leaf_2011_50C_message;

typedef struct {
	int			TCSOC:1;
	int			UNKNOWN_BYTE0:7;
	int			UNKNOWN_BYTE1:8;
	int			UNKNOWN_BYTE2:2;
	int			CHG_STA_RQ:2;
	int			UNKNOWN_BYTE2_2:4;
	int			UNKNOWN_BYTE3:8;
	int			UNKNOWN_BYTE4:8;
	int			UNKNOWN_BYTE5:8;
	int			UNKNOWN_BYTE6:6;
	int			MPRUN:2;
	int			CRC8:8;
} Leaf_2011_1F2_message;

typedef struct {
	int			LB_CURRENT:11;
	int			LB_FAIL:2;
	int			LB_STATUS:3;
	int			LB_VOLTAGE:10;
	int			LB_FRLYON:1;
	int			LB_FCHGEND:1;
	int			LB_INTERLOCK:1;
	int			LB_POUT_STATUS:1;
	int			ZEROS:16;
	int			ZEROS_2:6;
	int			MPR1DB:2;
	int			CRC8:8;
} Leaf_2011_1DB_message;

typedef struct {
	int			LB_POUT:10;
	int			LB_PIN:10;
	int			LB_BPCMAX:10;
	int			LB_PIN_STATUS:2;
	int			LB_BPCUPRATE:3;
	int			LB_CODECON:3;
	int			LB_CODE1:8;
	int			LB_CODE2:8;
	int			MPR1DC:2;
	int			CRC8:8;
} Leaf_2011_1DC_message;

typedef struct {
	int			LB_SOC:10;
	int			ZEROS_BYTE1:6;
	int			LB_ALU_ANSWER:8;
	int			LB_IRSEN_VOL:10;
	int			ZEROS_BYTE5:5;
	int			LB_IRSEN:1;
	int			LB_EMPTY:1;
	int			LB_REFUSE:2;
	int			ZEROS_BYTE6:2;
	int			MPR55B:2;
	int			CRC8:8;
} Leaf_2011_55B_message;

typedef struct {
	int			LB_CAPR:10;
	int			LB_FULLCAP:10;
	int			LB_CAPSEG:4;
	int			LB_AVET:8;
	int			LB_SOH:7;
	int			LB_CAPSW:1;
	int			LB_RLIMIT:3;
	int			SPACER:2;
	int			LB_CAPBALCOMP:1;
	int			LB_RCHGTCON:5;
	int			LB_RCHGTIM:13;
} Leaf_2011_5BC_message;

typedef struct {
	int			LB_HIS_DATA_SW:2;
	int			SPACER_1:2;
	int			LB_HIS_HLVOL_TIMS:4;
	int			LB_HIS_TEMP_WUP:7;
	int			SPACER_2:1;
	int			LB_HIS_TEMP:7;
	int			SPACER_3:1;
	int			LB_HIS_INTG_CUR:8;
	int			LB_HIS_DEG_REGI:7;
	int			SPACER_4:1;
	int			LB_HIS_CELL_VOL:6;
	int			SPACER_5:10;
	int			LB_DTC:8;
} Leaf_2011_5C0_message;

typedef struct {
	Leaf_2011_1D4_message CAN_1D4;
	Leaf_2011_50B_message CAN_50B;
	Leaf_2011_50C_message CAN_50C;
	Leaf_2011_1F2_message CAN_1F2;
	Leaf_2011_1DB_message CAN_1DB;
	Leaf_2011_1DC_message CAN_1DC;
	Leaf_2011_55B_message CAN_55B;
} Leaf_2011_state;

typedef struct {
	uint16_t	temp_neg_25[16];
	uint16_t	temp_neg_20[16];
	uint16_t	temp_neg_10[16];
	uint16_t	temp_0[16];
	uint16_t	temp_10[16];
	uint16_t	temp_20[16];
	uint16_t	temp_25[16];
	uint16_t	temp_35[16];
	uint16_t	temp_45[16];
	uint16_t	temp_60[16];
} Gen4_battery_voltage;

typedef struct {
	uint8_t		run_state;				//what the car is doing at the moment
	uint16_t	HV_bus_voltage;			//high voltage bus voltage
	uint8_t		HV_bat_temperature_bars; //main battery temperature
	uint8_t		HV_bat_temperature;
	uint16_t	max_QC_power;
	uint8_t		max_charge_percentage;	//to which percentage should the car charge
	uint8_t		charge_percentage;		//how full is the combined battery capacity?
	uint8_t		regen;					//regeneration strength (0-f)
	uint8_t		HV_bat_current;			//current in main battery
	} car_state;
	
#define		CAR_OFF				0
#define		CAR_IDLE			1
#define		CAR_DRIVING			2
#define		CAR_SLOW_CHARGING	3
#define		CAR_FAST_CHARGING	4

#endif //CANFRAME_H
