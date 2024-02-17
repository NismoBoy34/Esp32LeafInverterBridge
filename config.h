//——————————————————————————————————————————————————————————————————————————————
// Description: Global configuration
// Author: Adam Saiyad, Julius Calzada (julius.jai@gmail.com)
// Revision: v1.2.8
// 06.28.2022: Migration to ESP32 Arduino environment
// 10.03.2022: Integrated earlyversion.c, can-bridge-firmware.h & nissan-can-structs.h
// 10.25.2022: v1.1.4: Integrated API for Web configuration to allow runtime configurability of vehicle parameters
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

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Comment this out when module is used without Serial Port connected to avoid infinite loop during Serial initialization
//#define SERIAL_DEBUG_MONITOR
//#define DEBUG_NVM_PREFERENCE
//#define DEBUG_WEB_PROCESSING
//#define DEBUG_WEB_SOCKET

//——————————————————————————————————————————————————————————————————————————————
// Vehicle selection 
// Requirement: Uncomment the target vehicle; Comment the unused vehicle.
//              Default if LEAF Vehicle.
//——————————————————————————————————————————————————————————————————————————————
#define CAN_BRIDGE_FOR_LEAF
//#define CAN_BRIDGE_FOR_ENV200 

//——————————————————————————————————————————————————————————————————————————————
// CAN Polling Time Config
// Requirement: Select the target polling time by assigning T_POLLING with T_POLLING_VALUE_xxxx.
//              Default is 1 millisecond.
//——————————————————————————————————————————————————————————————————————————————
#define T_POLLING_VALUE_100US   (100)   //100 microsecond
#define T_POLLING_VALUE_1MS     (1000)  //1000 microsecond or 1 millisecond
#define T_POLLING_VALUE_10MS    (10000) //10 millisecond

#define T_POLLING               (T_POLLING_VALUE_100US)

//——————————————————————————————————————————————————————————————————————————————
// LEAF Testing Conditions  
//——————————————————————————————————————————————————————————————————————————————
//Requirement: Un-comment below definition if ID Translation is required
#define LEAF_TRANSLATION_ENABLED

//Requirement: Un-comment below definition if ID Translation is required
//#define LEAF_BLACKLISTING_ENABLED

//Requirement: Un-comment below definition if Brutforce is required
//#define LEAF_BRUTEFORCE_UPGRADE
//#define EXTRAGIDS  320 //65 used for 30kWh bruteforce upgrade

//——————————————————————————————————————————————————————————————————————————————
// Transmit Buffer Size
//——————————————————————————————————————————————————————————————————————————————
#define TXBUFFER_SIZE	32

//——————————————————————————————————————————————————————————————————————————————
// CAN Channel Assignments
//——————————————————————————————————————————————————————————————————————————————
#define CAN_CHANNEL_0  (0U)
#define CAN_CHANNEL_1  (1U)
#define CAN_CHANNEL_2  (2U)

//Port Enabling/Disabling
//#define CAN_CH0_ENABLED
#define CAN_CH1_ENABLED
#define CAN_CH2_ENABLED

#if !defined (CAN_CH2_ENABLED)
  #error "CAN_CH2_ENABLED must be defined. Internal CAN controller must be paired with external CAN controller."
#endif

#if !defined (CAN_CH0_ENABLED) && !defined (CAN_CH1_ENABLED)
  #error "Either CAN_CH0_ENABLED or CAN_CH1_ENABLED must be defined. 1 external CAN controller must be paired with CAN 2."
#endif 

//---Start of earlyversion.c

/* Choose your vehicle */

//-------Deprecated!!!!-----Choose from OTA configurable Vehicle Selections---
//#define E_NV_200				//Nissan e-NV200 or e-NV200 EVALIA
//#define LEAF_2011				//Nissan Leaf 2010-2013 (light interior)
#define LEAF_2014				//Nissan Leaf 2013-2017 (dark interior, old exterior style, 24/30kWh battery)
//#define LEAF_2018				//Nissan Leaf 2017- (new exterior style, 40 or 60kWh battery)

//---NEW--Choose from OTA configurable Vehicle Selections
#define NISSAN_LEAF_2010_to_2019()    (Vehicle_Selection == Vehicle_Selection_Nissan_LEAF_2010_2019) //None will be removed
#define NISSAN_ENV200()               (Vehicle_Selection == Vehicle_Selection_Nissan_ENV200) //Equals to #define E_NV_200     
#define NISSAN_ZE0_2011_2012()        (Vehicle_Selection == Vehicle_Selection_ZE0_2011_2012) //Equals to #define LEAF_2011
#define NISSAN_AZE0_2013_2017()       (Vehicle_Selection == Vehicle_Selection_AZE0_2013_2017) //Equals to #define LEAF_2014
#define NISSAN_ZE1_2018_2022()        (Vehicle_Selection == Vehicle_Selection_ZE1_2018_2022) //Equals to #define LEAF_2018 

#define NISSAN_LEAF_CONFIG()         ( NISSAN_LEAF_2010_to_2019() || NISSAN_ZE0_2011_2012() || NISSAN_AZE0_2013_2017() || NISSAN_ZE1_2018_2022() ) 

/* Choose the modification */

//-------Deprecated!!!!-----Choose from OTA configurable Battery Selection---
//#define BATTERY_SWAP			//battery-swapped vehicle
//#define BATTERY_24KWH
//#define BATTERY_30KWH
//#define BATTERY_40KWH
//#define BATTERY_62KWH

//---NEW--Choose from OTA configurable Battery Selection
/////#define BATTERY_30KWH()       (Battery_Selection == Battery_Selection_30Kw)
//#define BATTERY_40KWH()       (Battery_Selection == Battery_Selection_40Kwh)
//#define BATTERY_62KWH()       (Battery_Selection == Battery_Selection_62Kwh)
//#define BATTERY_BRUTE_FORCE() (Battery_Selection == Battery_Selection_BruteForce_30KWh_24KWH_LBC) // Is this referring to BATTERY_SWAP???

//#define BATTERY_SWAP_ENABLED()  ( BATTERY_24KWH() || BATTERY_30KWH() || BATTERY_40KWH() || BATTERY_62KWH() )



/* Quick-charging defines */
//#define MIN_QC_POWER 40			//in 0.025kW steps, so 40=1kW
//#define MAX_VOLTAGE_QC 395  //max voltage for quick charging. Only extender packs can handle 404V, safer for lower voltage for smaller packs

//---End of earlyversion.c

//---Start of can-bridge-firmware.h

//Leaf-specific defines
#define CHARGING_QUICK_START    0x40
#define CHARGING_QUICK          0xC0
#define CHARGING_QUICK_END      0xE0
#define CHARGING_SLOW           0x20
#define CHARGING_IDLE           0x60
#define QUICK_CHARGE            0b00000
//#define NORMAL_CHARGE_200V_100  0b01001
//#define NORMAL_CHARGE_200V_80   0b01010
//#define NORMAL_CHARGE_100V_100  0b10001
//#define NORMAL_CHARGE_100V_80   0b10010

//---End of can-bridge-firmware.h

// More OTA configurable options

//--------------------------------------
// Inverter Upgrade 110Kw/160Kw
//--------------------------------------
#define INVERTER_UPGRADE_EM57_MOTOR_WITH_110KW()    (Inverter_Upgrade_110Kw_160Kw == Inverter_Upgrade_EM57_Motor_with_110Kw_inverter)
#define INVERTER_UPGRADE_EM57_MOTOR_WITH_160KW()    (Inverter_Upgrade_110Kw_160Kw == Inverter_Upgrade_EM57_Motor_with_160Kw_Inverter)
#define INVERTER_UPGRADE_DISABLED()                 (Inverter_Upgrade_110Kw_160Kw == Inverter_Upgrade_Disabled)

#define INVERTER_UPGRADE_ENABLED()                  (INVERTER_UPGRADE_EM57_MOTOR_WITH_110KW() || INVERTER_UPGRADE_EM57_MOTOR_WITH_160KW())

//--------------------------------------
// BatterySaver  
//--------------------------------------
//#define BATTERY_SAVER_50_PERCENT()  (Battery_Saver == BatterySaver_50percent)
//#define BATTERY_SAVER_60_PERCENT()  (Battery_Saver == BatterySaver_60percent)
//#define BATTERY_SAVER_80_PERCENT()  (Battery_Saver == BatterySaver_80percent)
//#define BATTERY_SAVER_DISABLED()    (Battery_Saver == BatterySaver_Disabled)

//#define BATTERY_SAVER_ENABLED()     ( BATTERY_SAVER_50_PERCENT() || BATTERY_SAVER_60_PERCENT() || BATTERY_SAVER_80_PERCENT() )

//--------------------------------------
// Glide in Drive
//--------------------------------------
//#define GLIDE_IN_DRIVE_ENABLED()    (Glide_In_Drive == Glide_In_Drive_Enabled)
//#define GLIDE_IN_DRIVE_DISABLED()   (Glide_In_Drive == Glide_In_Drive_Disabled)


//--------------------------------------
// CurrentControl
//--------------------------------------
//#define CURRENT_CONTROL_1p0_KW()            (Current_Control == CurrentControl_1p0_kW)
//#define CURRENT_CONTROL_2p0_KW()            (Current_Control == CurrentControl_2p0_kW)
//#define CURRENT_CONTROL_3p0_KW()            (Current_Control == CurrentControl_3p0_kW)
//#define CURRENT_CONTROL_4p0_KW()            (Current_Control == CurrentControl_4p0_kW)
//#define CURRENT_CONTROL_6p0_KW()            (Current_Control == CurrentControl_6p0_kW)
//#define CURRENT_CONTROL_UNRESTRICTED_KW()   (Current_Control == CurrentControl_Unrestricted_kW)

//#define CURRENT_CONTROL_ENABLED()           (CURRENT_CONTROL_1p0_KW() || CURRENT_CONTROL_2p0_KW() || CURRENT_CONTROL_3p0_KW() || CURRENT_CONTROL_4p0_KW() || CURRENT_CONTROL_6p0_KW() || CURRENT_CONTROL_UNRESTRICTED_KW())

//--------------------------------------
//  OTA API Defines and externs
//--------------------------------------
void Set_Vehicle_Selection(String setValue);
extern uint8_t Vehicle_Selection;

void Set_Inverter_Upgrade_110Kw_160Kw(String setValue);
extern uint8_t Inverter_Upgrade_110Kw_160Kw;

//void Set_Battery_Selection(String setValue);
//extern uint8_t Battery_Selection;

//void Set_Battery_Saver(String setValue);
//extern uint8_t Battery_Saver;

//void Set_Glide_In_Drive(String setValue);
//extern uint8_t Glide_In_Drive;

//void Set_Current_Control(String setValue);
//extern uint8_t Current_Control;

//Web Request Manager
void WebRequestProcessing(const String data);

//--------------------------------------
// Vehicle Selection
//--------------------------------------
#define Vehicle_Selection_Nissan_LEAF_2010_2019 (0)
#define Vehicle_Selection_Nissan_ENV200         (1)
#define Vehicle_Selection_ZE0_2011_2012         (2)
#define Vehicle_Selection_AZE0_2013_2017        (3)
#define Vehicle_Selection_ZE1_2018_2022         (4)
//--------------------------------------

//--------------------------------------
// Inverter Upgrade 110Kw/160Kw
//--------------------------------------
#define Inverter_Upgrade_EM57_Motor_with_110Kw_inverter (0)
#define Inverter_Upgrade_EM57_Motor_with_160Kw_Inverter (1)
#define Inverter_Upgrade_Disabled                       (2)

//--------------------------------------
// Battery Selection
//--------------------------------------
//#define Battery_Selection_24Kwh                       (0)
//#define Battery_Selection_30Kw                        (1)
//#define Battery_Selection_40Kwh                       (2)
//#define Battery_Selection_62Kwh                       (3)
//#define Battery_Selection_BruteForce_30KWh_24KWH_LBC  (4)

//--------------------------------------
// BatterySaver  
//--------------------------------------
//#define BatterySaver_50percent  (0)
//#define BatterySaver_60percent  (1)
//#define BatterySaver_80percent  (2)
//#define BatterySaver_Disabled   (3)

//--------------------------------------
// Glide in Drive
//--------------------------------------
//#define Glide_In_Drive_Enabled  (0)
//#define Glide_In_Drive_Disabled (1)

//--------------------------------------
// CurrentControl
//--------------------------------------
//#define CurrentControl_1p0_kW           (0)
//#define CurrentControl_2p0_kW           (1)
//#define CurrentControl_3p0_kW           (2)
//#define CurrentControl_4p0_kW           (3)
//#define CurrentControl_6p0_kW           (4)
//#define CurrentControl_Unrestricted_kW  (5)

//--------------------------------------
// From leaf-can-bridge-3-port-master
//--------------------------------------
//#define CHARGECURRENT        //if defined, enables setting current limiters while car is charging
//#define RAPIDGATEDODGER      //if defined, enables throttling of DC fastcharging via HVAC controls
//#define DISABLE_REGEN_IN_DRIVE    //For maximum hypermiling. When you shift from D->N, the regen gets disabled in D. Do another shift to N to re-enable regen.


//--------------------------------------
// Inverter Testing only
//--------------------------------------
#define MESSAGE_0x11A
#define MESSAGE_0x1D4
#define MESSAGE_0x1DA //batcom
#define MESSAGE_0x284 //batcom
#define MESSAGE_0x50C //batcom
//#define MESSAGE_0x1F2
#define MESSAGE_0x603 // batcom
//#define MESSAGE_0x55B //batcom


#endif //CONFIG_H
