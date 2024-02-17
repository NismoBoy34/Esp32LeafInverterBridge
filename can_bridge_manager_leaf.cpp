//Can 1 = Inverter buffer_send_can1
//Can 2 = Vcm Side  buffer_send_can2
//——————————————————————————————————————————————————————————————————————————————
// Description: 3-bus transparent CAN bridge with Battery upgrade SW for Nissan env200
// Author: Adam Saiyad, Julius Calzada (julius.jai@gmail.com)
// Revision: v1.1.3//added updates for quick charge 70% and battery pairing glitch p3102 error persistent cleared
// Revision: v1.1.4 // Adding of webserver and OTA eeprom saving functions 
// Revision: v1.2.8
// 06.28.2022: Migration to ESP32 Arduino environment
// 10.03.2022: Integrated earlyversion.c, can-bridge-firmware.h & nissan-can-structs.h
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

#include <Arduino.h>
#include "can_bridge_manager_common.h"
#include "can_bridge_manager_leaf.h"
#include "can_bridge_manager_leaf_inverter_upgrade.h"
#include "can_driver.h"
#include "helper_functions.h"
#include "config.h"

#if defined(CAN_BRIDGE_FOR_LEAF)
  static volatile  uint8_t   repeat_can        = 1;  //repeat CAN1 to CAN2 and vice versa (transparent bridge mode)
#endif

volatile  uint16_t   torqueDemand    = 0; //NM * 4
volatile  uint16_t   torqueResponse  = 0; //NM * 2
volatile  uint16_t   VCMtorqueDemand = 0; //NM * 4
volatile  uint16_t   current         = 0;
volatile  uint16_t  battery_soc    = 0;
volatile  uint8_t   shift_state     = 0;
volatile  uint8_t   charging_state      = 0;
volatile  uint8_t   eco_screen        = 0;


#define REGEN_TUNING_ENABLED

#define TORQUE_MULTIPLIER_110 0.9    //(1.28 is too small, results in 118kW and 1.37 results in 122kW)
#define TORQUE_MULTIPLIER_160 1.6    //For a more aggressive pedal feel, multipliers between 1.4 and 1.6 can be used

#define REGEN_MULTIPLIER 1.10

/* Do not make any changes to the rows below unless you are sure what you are doing */

#define SHIFT_DRIVE   4
#define SHIFT_ECO     5
#define SHIFT_REVERSE 2
#define SHIFT_NEUTRAL 3
#define SHIFT_PARK    0

#define   SHIFT_P   0x00
#define   SHIFT_D   0x40 //Modified, no longer following mux standard
#define   SHIFT_N   0x30
#define   SHIFT_R   0x20
#define   SHIFT_ECO 0x50

#define ECO_OFF      1
#define ECO_ON       2

#define CHARGING_QUICK_START    0x40
#define CHARGING_QUICK          0xC0
#define CHARGING_QUICK_END      0xE0
#define CHARGING_SLOW           0x20
#define CHARGING_IDLE           0x60


// CAN messages used for deleting P3197 [EV/HEV] & P318E [MOTOR CONTROL] DTC (Send 4B9)
static  can_frame_t inv_4B9_message = {.can_id = 0x4B9, .can_dlc = 1, .data = {0x40}};
volatile  uint8_t   content_4B9     = 0x40;

// Sending 355 does not fix any DTC (but probably good to send it anyway)
// Sending 625 removes U215B [HV BATTERY]
// Sending 5C5 (355 at 40ms) removes U214E [HV BATTERY] and U1000 [MOTOR CONTROL] 
// Sending 3B8 and 5EB removes U1000 and P318E [HV BATTERY]
  static volatile  uint8_t   PRUN10MS      = 0;
  
  static    can_frame_t inv_605_message = {.can_id = 0x605, .can_dlc = 1, .data = {0x00}};
  static    can_frame_t inv_607_message = {.can_id = 0x607, .can_dlc = 1, .data = {0x00}};

  //This message is 40kWh specific (maybe on
  static  can_frame_t inv_1C2_message = {.can_id = 0x1C2, .can_dlc = 1, .data = {0x50}};
  static volatile  uint8_t   content_1C2     = 0x50;

  static  can_frame_t inv_108_message = {.can_id = 0x108, .can_dlc = 3, .data = {0x00,0x00,0x00}};
  static volatile  uint8_t   content_108_1     = 0x00;
  static volatile  uint8_t   content_108_2     = 0x00;
  static    uint8_t   lookuptable_crc_108[16] = {0x00,0x85,0x8F,0x0A,0x9B,0x1e,0x14,0x91,0xb3,0x36,0x3c,0xb9,0x28,0xad,0xa7,0x22};

  //volatile  can_frame_t inv_1CB_message = {.can_id = 0x1cb, .can_dlc = 7, .data = {0x00,0x00,0x00,0x02,0x60,0x00,0x62}}; //Actual content
volatile  can_frame_t inv_1CB_message = {.can_id = 0x1cb, .can_dlc = 7, .data = {0x00,0x09,0xFF,0xCE,0x10,0x8b,0xe7}}; //Startup sequence

//  This message is not needed if you have a 62kWh pack (1ED), but probably good to send it towards the 160kW inverter
  static  can_frame_t inv_1ED_message = {.can_id = 0x1ED, .can_dlc = 3, .data = {0xFF,0xe0,0x68}};
  static volatile  uint8_t   content_1ED_1   = 0xe0;
  static volatile  uint8_t   content_1ED_2   = 0x68;

  //Static for now, content unknown and changes. Snapshot from power run
  static    can_frame_t inv_355_message = {.can_id = 0x355, .can_dlc = 8, .data = {0x14,0x0a,0x13,0x97,0x10,0x00,0x40,0x00}};
  static volatile  uint8_t   ticker40ms    = 0;
  static  can_frame_t inv_5CD_message = {.can_id = 0x5CD, .can_dlc = 5, .data = {0x7a,0x06,0xf5,0x1F,0xC0}}; 
  static volatile  uint8_t   content_5CD   = 0;
  static volatile  uint8_t   flipFlop    = 0;                    
  static volatile  uint8_t   ticker100ms   = 0;
  static  can_frame_t inv_3B8_message = {.can_id = 0x3B8, .can_dlc = 5, .data = {0x7F,0xE8,0x01,0x07,0xFF}}; //Data from idle
  static volatile  uint8_t   content_3B8   = 0;
  static volatile  uint8_t   flip_3B8    = 0;
  
  //Content does not change
  static    can_frame_t inv_625_message = {.can_id = 0x625, .can_dlc = 6, .data = {0x02,0x00,0xff,0x1d,0x20,0x00}};
  static    can_frame_t inv_5EC_message = {.can_id = 0x5EC, .can_dlc = 1, .data = {0x00}};
  static    can_frame_t inv_5C5_message = {.can_id = 0x5C5, .can_dlc = 8, .data = {0x40,0x01,0x2F,0x5E,0x00,0x00,0x00,0x00}};
  static    can_frame_t inv_5EB_message = {.can_id = 0x5EB, .can_dlc = 8, .data = {0xE0,0x0F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}};

void LEAF_CAN_Bridge_Manager_Init(void)
{
  
}

//——————————————————————————————————————————————————————————————————————————————
// [LEAF] CAN Bridge Main Handler
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_BRIDGE_FOR_LEAF
void LEAF_CAN_Bridge_Manager(void){

  //--- Monitor CAN0 & CAN1 receptions
  #ifdef CAN_CH0_ENABLED
  if(true == CAN0_NewFrameIsAvailable()) {
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.println("CAN0 Message is received");
    #endif //#ifdef SERIAL_DEBUG_MONITOR
    
    CANMessage ch0_rxdata;
    can_frame_t ch0_frame;
    uint16_t ch0_i;
    CAN0_ReadNewFrame(ch0_rxdata);
    ch0_frame.can_id = ch0_rxdata.id;
    ch0_frame.can_dlc = ch0_rxdata.len;
    for(ch0_i=0; ch0_i<ch0_frame.can_dlc; ch0_i++) {
      ch0_frame.data[ch0_i] = ch0_rxdata.data[ch0_i];
    }
    
    //Call CAN0 event handler
    //if( INVERTER_UPGRADE_ENABLED() )
  //  {
      LEAF_CAN_Handler(CAN_CHANNEL_0, ch0_frame);
   // }
    //else
   // {
      //Not supported for now
   // }
  }
  #endif //CAN_CH0_ENABLED


  #ifdef CAN_CH1_ENABLED
  if(true == CAN1_NewFrameIsAvailable()) {
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.println("CAN1 Message is received");
    #endif //#ifdef SERIAL_DEBUG_MONITOR

    CANMessage ch1_rxdata;
    can_frame_t ch1_frame;
    uint16_t ch1_i;
    CAN1_ReadNewFrame(ch1_rxdata);
    ch1_frame.can_id = ch1_rxdata.id;
    ch1_frame.can_dlc = ch1_rxdata.len;
    for(ch1_i=0; ch1_i<ch1_frame.can_dlc; ch1_i++) {
      ch1_frame.data[ch1_i] = ch1_rxdata.data[ch1_i];
    }
    
    //Call CAN1 event handler
   // if( INVERTER_UPGRADE_ENABLED() )
   // {
      LEAF_CAN_Handler(CAN_CHANNEL_1, ch1_frame );  
  //  }
   // else
    //{
      //Not supported for now
  //  }
  }
  #endif //CAN_CH1_ENABLED  

}
#endif //#ifdef CAN_BRIDGE_FOR_LEAF

//——————————————————————————————————————————————————————————————————————————————
// [LEAF] CAN handler - evaluates received data, tranlate and transmits to the other CAN bus
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_BRIDGE_FOR_LEAF
void LEAF_CAN_Handler(uint8_t can_bus, can_frame_t new_rx_frame){  
  
	can_frame_t frame;
  //int16_t temp = 0;

	memcpy(&frame, &new_rx_frame, sizeof(new_rx_frame));

	//Debugging format
	//if CAN_CHANNEL 0 -> "0|   |..."
	//if CAN_CHANNEL 1 -> "1|   |..."
	//if CAN_CHANNEL 2 -> "2|   |..."
	char strbuf[] = "0|   |                \n"; 
	if(CAN_CHANNEL_1 == can_bus){ strbuf[0] = '1'; }
	if(CAN_CHANNEL_2 == can_bus){ strbuf[0] = '2'; }

	//Evaluate according to received ID
	#ifdef LEAF_TRANSLATION_ENABLED    
	switch(frame.can_id){

    case 0x5A9:
        
        //eco_screen = (frame.data[0] & 0x03);
        //eco_screen = (frame.data[0] & 0x01);
       // eco_screen = (frame.data[0] & 0x02 >> 1);
    
    break;

    
    #ifdef MESSAGE_0x11A
    case 0x11A: //store shifter status
      switch(frame.data[0] & 0xF0){
        case 0x20:
          shift_state = SHIFT_REVERSE;
        break;
        case 0x30:
          shift_state = SHIFT_NEUTRAL;
        break;
        case 0x40:         
        shift_state = SHIFT_DRIVE;
        break;
        case 0x00:
          shift_state = SHIFT_PARK;
        break;        
        default:
          shift_state = SHIFT_PARK;
        break;
      }
    break;
    #endif //#ifdef MESSAGE_0x11A
      

                  
// ------ debug for eco shift switch
 
      //case 0x1DB:                
      //frame.data[4] = (shift_state+50) ; //SOC% will show the RAW can value for the shifter                       
      //calc_crc8(&frame);
      //break;
      case 0x1DB:                
      if(eco_screen == ECO_ON){ 
          frame.data[4] = 99; //99% soc displayed
      } 
      if(eco_screen == ECO_OFF){ 
          frame.data[4] = 11; //11% soc displayed
      }                                                   
      calc_crc8(&frame);
      break;
//---------------------End of debug       
      
    #ifdef MESSAGE_0x1D4
    case 0x1D4: //VCM request signal     
      torqueDemand = ((frame.data[2] << 8) | frame.data[3]); //Requested torque is 12-bit long signed.
      //torqueDemand = (torqueDemand & 0xFFF0) >> 4; //take out only 12 bits (remove 4)
      //VCMtorqueDemand = torqueDemand; //Store the original VCM demand value
      VCMtorqueDemand = (torqueDemand >> 4); //Store the original VCM demand value (ignoring sign, just the raw NM demand)
                    
        //if (shift_state != SHIFT_DRIVE || (torqueDemand < 2048 && eco_screen == ECO_ON)) break; //Stop modifying message if: Not in drive OR requesting power in ECO mode
          if (shift_state != SHIFT_DRIVE || eco_screen == ECO_ON) break; //Stop modifying message if: Not in drive OR ECO mode is ON
          
        if((frame.data[2] & 0x80)){ //Message is signed, we are requesting regen
            #ifndef REGEN_TUNING_ENABLED
          break; //We are demanding regen and regen tuning is not on, abort modification!
          #endif
          torqueDemand = ~torqueDemand; //2S complement
          torqueDemand = (torqueDemand >> 4);
          torqueDemand = (torqueDemand * REGEN_MULTIPLIER);
          torqueDemand = (torqueDemand << 4);
          torqueDemand = ~torqueDemand; //2S complement
                  
          frame.data[2] = torqueDemand >> 8; //Slap it back into whole 2nd frame
          frame.data[3] = (torqueDemand & 0x00F0);
        }
      else{
        torqueDemand = (torqueDemand >> 4);
        if( INVERTER_UPGRADE_EM57_MOTOR_WITH_110KW() )                   
        {
            torqueDemand = (torqueDemand * TORQUE_MULTIPLIER_110);
        }              
        if( INVERTER_UPGRADE_EM57_MOTOR_WITH_160KW() )
        {
          torqueDemand = (torqueDemand * TORQUE_MULTIPLIER_160);
        }   
        torqueDemand = (torqueDemand << 4); //Shift back the 4 removed bits 
        frame.data[2] = torqueDemand >> 8; //Slap it back into whole 2nd frame
        frame.data[3] = (torqueDemand & 0x00F0);       
      }
      calc_crc8(&frame); 
    break;
    #endif //#ifdef MESSAGE_0x1D4

    #ifdef MESSAGE_0x1DA
    case 0x1DA: //motor response also needs to be modified      
      //torqueResponse = (int16_t) (((frame.data[2] & 0x07) << 8) | frame.data[3]);
      //torqueResponse = (torqueResponse & 0b0000011111111111); //only take out 11bits, no need to shift
        torqueResponse = (((frame.data[2] & 0x07) << 8) | frame.data[3]);
        torqueResponse = (torqueResponse & 0x7FF); //only take out 11bits, no need to shift
        
        if (shift_state != SHIFT_DRIVE || eco_screen == ECO_ON) break; //Stop modifying message if: Not in drive OR ECO mode is ON

        if (frame.data[2] & 0x04){ //We are Regen braking
          #ifndef REGEN_TUNING_ENABLED
          break; //We are demanding regen and regen tuning is not on, abort modification!
          #endif
          torqueResponse = (VCMtorqueDemand*0.5); //Fool VCM that response is exactly the same as demand
          frame.data[2] = ((frame.data[2] & 0xF8) | (torqueResponse >> 8));
          frame.data[3] = (torqueResponse & 0xFF);
        }
        else //We are requesting power in D (ECO OFF)
        {
          torqueResponse = (VCMtorqueDemand*0.5); //Fool VCM that response is exactly the same as demand        
          frame.data[2] = ((frame.data[2] & 0xF8) | (torqueResponse >> 8));
          frame.data[3] = (torqueResponse & 0xFF);
        }

        calc_crc8(&frame);
        break;   
        #endif //#ifdef MESSAGE_0x1DA

    #ifdef MESSAGE_0x284
    case 0x284: //Hacky way of generating missing inverter message 
      //Upon reading VCM originating 0x284 every 20ms, send the missing message(s) to the inverter
          if(charging_state == CHARGING_SLOW){
              break; //abort all message modifications, otherwise we interrupt AC charging on 62kWh LEAFs
        }
  { //Start of function 0x284
      ticker40ms++;
      if(ticker40ms > 1)
      {
        ticker40ms = 0;
          
        if(can_bus == 1)
        {
         // buffer_send_can2(inv_355_message); //40ms
        }
        else
        {
          buffer_send_can1(inv_355_message); //40ms
        }
      }
  }
    break;
    #endif //#ifdef MESSAGE_0x284

    #ifdef MESSAGE_0x50C
    case 0x50C: //Hacky way of generating missing inverter message 
      //Upon reading VCM originating 0x50C every 100ms, send the missing message(s) to the inverter
      //Eliminate the CheckEV light first
        content_4B9++;
        if(content_4B9 > 79)
        {
          content_4B9 = 64;
        }
        inv_4B9_message.data[0] = content_4B9; //64 - 79 (0x40 - 0x4F)

        if(can_bus == 1)
        {
         // buffer_send_can2(inv_4B9_message); //100ms
        }
        else
        {
          buffer_send_can1(inv_4B9_message); //100ms
        }
        
        if(charging_state == CHARGING_SLOW){
          break; //abort all further message modifications, otherwise we interrupt AC charging on 62kWh LEAFs
        }
         
      if(can_bus == 1)
      {
     //  buffer_send_can2(inv_4B9_message); //100ms
      //  buffer_send_can2(inv_625_message); //100ms
     //   buffer_send_can2(inv_5C5_message); //100ms
      //  buffer_send_can2(inv_3B8_message); //100ms
      } 
      else 
      {
        buffer_send_can1(inv_4B9_message); //100ms
        buffer_send_can1(inv_625_message); //100ms
        buffer_send_can1(inv_5C5_message); //100ms
        buffer_send_can1(inv_3B8_message); //100ms
      }
        
      content_3B8++;
      if(content_3B8 > 14)
      {
        content_3B8 = 0;
      }
      inv_3B8_message.data[2] = content_3B8; //0 - 14 (0x00 - 0x0E)
        
      if(flip_3B8)
      {
        flip_3B8 = 0;
        inv_3B8_message.data[1] = 0xC8;
      }
      else
      {
        flip_3B8 = 1;
        inv_3B8_message.data[1] = 0xE8;
      }
              
      ticker100ms++; //500ms messages go here
      if(ticker100ms > 4)
      {
        ticker100ms = 0;
        if(can_bus == 1)
        {
        //  buffer_send_can2(inv_5EC_message); //500ms
         // buffer_send_can2(inv_5EB_message); //500ms
        }
        else
        {
          buffer_send_can1(inv_5EC_message); //500ms
          buffer_send_can1(inv_5EB_message); //500ms
        }
          
          
        if(flipFlop == 0)
        {
          flipFlop = 1;
          inv_5CD_message.data[1] = content_5CD;
          if(can_bus == 1)//1000ms messages alternating times
          {
            buffer_send_can2(inv_5CD_message); //1000ms
          }
          else
          {
            buffer_send_can1(inv_5CD_message); //1000ms
          }
          content_5CD = (content_5CD + 4);
          if(content_5CD > 238)
          {
            content_5CD = 2;
          }
        }
        else
        {
          flipFlop = 0;
        }
      }
    break;
    #endif //#ifdef MESSAGE_0x50C

    #ifdef MESSAGE_0x1F2  
    case 0x1F2: //Hacky way of generating missing inverter message
      //Upon reading VCM originating 0x1F2 every 10ms, send the missing message(s) to the inverter
        //charging_state = frame.data[2];
        
         //if(charging_state == CHARGING_SLOW){
         // break; //abort all message modifications, otherwise we interrupt AC charging on 62kWh LEAFs
         //} 
      
      if(can_bus == 1)
      {
        buffer_send_can2(inv_1C2_message);
        buffer_send_can2(inv_108_message);
        buffer_send_can2(inv_1CB_message);
        buffer_send_can2(inv_1ED_message);
      }
      else
      {
        buffer_send_can1(inv_1C2_message);
        buffer_send_can1(inv_108_message);
        buffer_send_can1(inv_1CB_message);
        buffer_send_can1(inv_1ED_message);
      }
      
      PRUN10MS++;
      if (PRUN10MS > 3){
        PRUN10MS = 0;
      }
      
      if (PRUN10MS == 0)
      {
        inv_1CB_message.data[5] = 0x88;
        inv_1CB_message.data[6] = 0xED;
        inv_1ED_message.data[1] = 0xE0;
        inv_1ED_message.data[2] = 0x68;
      }
      else if(PRUN10MS == 1)
      {
        inv_1CB_message.data[5] = 0x89;
        inv_1CB_message.data[6] = 0x68;
        inv_1ED_message.data[1] = 0xE1;
        inv_1ED_message.data[2] = 0xED;
      }
      else if(PRUN10MS == 2)
      {
        inv_1CB_message.data[5] = 0x8A;
        inv_1CB_message.data[6] = 0x62;
        inv_1ED_message.data[1] = 0xE2;
        inv_1ED_message.data[2] = 0xE7;
      }
      else if(PRUN10MS == 3)
      {
        inv_1CB_message.data[5] = 0x8B;
        inv_1CB_message.data[6] = 0xE7;
        inv_1ED_message.data[1] = 0xE3;
        inv_1ED_message.data[2] = 0x62;
      }      
            
      content_1C2++;
      if(content_1C2 > 95)
      {
        content_1C2 = 80;
      }
      
      inv_1C2_message.data[0] = content_1C2; //80 - 95 (0x50 - 0x5F)
      
      content_108_1++;
      if(content_108_1 > 0x0F)
      {
        content_108_1 = 0;
      }
      
      content_108_2 = lookuptable_crc_108[content_108_1];
      
      inv_108_message.data[1] = content_108_1;
      inv_108_message.data[2] = content_108_2;
     
    break;
    #endif //#ifdef MESSAGE_0x1F2

    #ifdef MESSAGE_0x55B
    case 0x55B:
        //Collect SOC%
        main_battery_soc = (frame.data[0] << 2) | ((frame.data[1] & 0xC0) >> 6); 
        main_battery_soc /= 10; //Remove decimals, 0-100 instead of 0-100.0
      break;
      #endif //#ifdef MESSAGE_0x55B
    
    #ifdef MESSAGE_0x603  
    case 0x603:
      //Send new ZE1 wakeup messages, why not
      if(can_bus == 1)
      {
       // buffer_send_can2(inv_605_message);
       // buffer_send_can2(inv_607_message);
      }
      else
      {
        buffer_send_can1(inv_605_message);
        buffer_send_can1(inv_607_message);
      }
    break;
    #endif //#ifdef MESSAGE_0x603
    
    default:
      break;
  }
  
  #endif //#ifdef LEAF_TRANSLATION_ENABLED

  

  //--- Gateway all messages except the unwanted IDs
  //if you enable CAN repeating between bus 1 and 2, we end up here 
  if(repeat_can){
    
    //you can blacklist certain messages or message contents like this, blocking them from both being forwarded and being displayed
/*    uint8_t blacklist = 0;
    #ifdef LEAF_BLACKLISTING_ENABLED 
    switch(frame.can_id){    
      case 0x284:
      case 0x50C:
      case 0x1F2:
      case 0x603:
        blacklist = 1;
      break;
      default:
        blacklist = 0;
      break;
    }
    #endif //#ifdef LEAF_BLACKLISTING_ENABLED
*/
    uint8_t blacklist = 0;
    #ifdef LEAF_BLACKLISTING_ENABLED 
    switch(frame.can_id){      
      default:
        blacklist = 0;
      break;
    }
    #endif //#ifdef LEAF_BLACKLISTING_ENABLED
    
    //ToDo: Must confirm the correct CAN Channels connected to VCM and LBC
    if(!blacklist){
    #if defined (CAN_CH0_ENABLED) //Priority 1: CAN Channel 0 with Channel 2
      if(CAN_CHANNEL_0 == can_bus){
        buffer_send_can2(frame);
      }else if(CAN_CHANNEL_2 == can_bus) {
        buffer_send_can0(frame);
      }
      else{
        #ifdef SERIAL_DEBUG_MONITOR
        Serial.println("CAN Channel 1 is not used for gatewaying");
        #endif //#ifdef SERIAL_DEBUG_MONITOR
      }
    #elif defined (CAN_CH1_ENABLED) //Priority 2: Channel 1 with Channel 2
      if(CAN_CHANNEL_1 == can_bus){
        buffer_send_can2(frame);
      }else if(CAN_CHANNEL_2 == can_bus) {
        buffer_send_can1(frame);
      }
      else{
        #ifdef SERIAL_DEBUG_MONITOR
        Serial.println("CAN Channel 0 is not used for gatewaying");
        #endif //#ifdef SERIAL_DEBUG_MONITOR
      }
    #else
      #error "CAN 0 or CAN 1 must be paired with CAN 2 for Gatewaying."
    #endif
    }
  }
  //--- End of Messages Gateway
      
  //Post processing for debugging
  SID_to_str(strbuf + 2, frame.can_id);
  canframe_to_str(strbuf + 6, frame);
  #ifdef SERIAL_DEBUG_MONITOR
  Serial.println(strbuf);
  #endif //#ifdef SERIAL_DEBUG_MONITOR

}
#endif// CAN_BRIDGE_FOR_LEAF
