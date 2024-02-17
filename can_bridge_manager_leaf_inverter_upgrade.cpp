/*
//——————————————————————————————————————————————————————————————————————————————
// Description: Inverter Upgrade
// Author: Adam Saiyad, Julius Calzada (julius.jai@gmail.com)
// Revision: v1.2.8
// 12.04.2022: Merging of Inverter Upgrade based on https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/can-bridge-inverter.c
// 12.06.2022: Updated Charge Current logic - 1) Start conditions are charging state and fan speed; 2) Display kW for 15sec and revert to SOC
// 12.31.2022: Fix charge current functions Fix regen power and motor power Fix Glide and drive add code 
//——————————————————————————————————————————————————————————————————————————————

#include <Arduino.h>
#include "can_bridge_manager_common.h"
#include "can_bridge_manager_leaf_inverter_upgrade.h"
#include "helper_functions.h"
#include "config.h"

static volatile  uint8_t   repeat_can      = 1;  //repeat CAN1 to CAN2 and vice versa (transparent bridge mode)

static volatile  int16_t   torqueDemand    = 0; //NM * 4
static volatile  int16_t   torqueResponse  = 0; //NM * 2
static volatile  int16_t   VCMtorqueDemand = 0; //NM * 4
static volatile  uint8_t   shift_state     = 0;

#define SHIFT_DRIVE   4
#define SHIFT_ECO     5
#define SHIFT_REVERSE 2
#define SHIFT_NEUTRAL 3
#define SHIFT_PARK    0

// CAN messages used for deleting P3197 [EV/HEV] & P318E [MOTOR CONTROL] DTC (Send 4B9)
static  can_frame_t inv_4B9_message = {.can_id = 0x4B9, .can_dlc = 1, .data = {0x40}};
static volatile  uint8_t   content_4B9     = 0x40;

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
  static  can_frame_t inv_1CB_message = {.can_id = 0x1cb, .can_dlc = 7, .data = {0x00,0x09,0xFF,0xCE,0x10,0x8b,0xe7}}; //Startup sequence

  //This message is not needed if you have a 62kWh pack (1ED), but probably good to send it towards the 160kW inverter
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
    
//——————————————————————————————————————————————————————————————————————————————
// [LEAF] CAN handler - evaluates received data, tranlate and transmits to the other CAN bus
// Separate routine for Inverter Upgrade
//——————————————————————————————————————————————————————————————————————————————
void LEAF_CAN_Handler_Inverter_Upgrade(uint8_t can_bus, can_frame_t new_rx_frame){  
  
	can_frame_t frame;
  int16_t temp = 0;

	memcpy(&frame, &new_rx_frame, sizeof(new_rx_frame));

  //Debugging format
  //if CAN_CHANNEL 0 -> "0|   |..."
  //if CAN_CHANNEL 1 -> "1|   |..."
  //if CAN_CHANNEL 2 -> "2|   |..."
  char strbuf[] = "0|   |                \n"; 
  if(CAN_CHANNEL_1 == can_bus){ strbuf[0] = '1'; }
  if(CAN_CHANNEL_2 == can_bus){ strbuf[0] = '2'; }

	//Evaluate according to received ID 
	switch(frame.can_id)
	{
    case 0x11A: //store shifter status
      switch(frame.data[0] & 0xF0)
      {
        case 0x20:
          shift_state = SHIFT_REVERSE;
        break;
        case 0x30:
          shift_state = SHIFT_NEUTRAL;
        break;
        case 0x40:
          if(frame.data[1] & 0x10){ shift_state = SHIFT_ECO; }
          else { shift_state = SHIFT_DRIVE; }
        break;
        case 0x00:
          shift_state = SHIFT_PARK;
        break;        
        default:
          shift_state = SHIFT_PARK;
        break;
      }
    break;

    case 0x1D4: //VCM request signal     
      torqueDemand = (int16_t) ((frame.data[2] << 8) | frame.data[3]); //Requested torque is 12-bit long signed.
      torqueDemand = (torqueDemand & 0xFFF0) >> 4; //take out only 12 bits (remove 4)
      VCMtorqueDemand = torqueDemand; //Store the original VCM demand value
              
      temp = (torqueDemand & 0b0000100000000000); //extract the 12th bit, this contains signed info.
      if (temp > 0){//check if message is signed
        break; //We are demanding regen, abort modification of message TODO, make better handling of this
      }
        
      if(shift_state == SHIFT_DRIVE){ // Increase power only when in drive AND
        //#ifdef LEAF_110kW
        if( INVERTER_UPGRADE_EM57_MOTOR_WITH_110KW() )
        {
          torqueDemand = torqueDemand*1.32; //Add a multiplier of 1.32
          //(1.28 is too small, results in 118kW and 1.37 results in 122kW)
        }
        //#endif
        
        //#ifdef LEAF_160kW
        if( INVERTER_UPGRADE_EM57_MOTOR_WITH_160KW() )
        {
          torqueDemand = torqueDemand*1.37; //Add a multiplier of 1.37
        }
        //#endif
            
        torqueDemand = (torqueDemand << 4); //Shift back the 4 removed bits 
        frame.data[2] = torqueDemand >> 8; //Slap it back into whole 2nd frame
        frame.data[3] = torqueDemand & 0xF0; //Only high nibble on 3rd frame (0xFF might work if no colliding data)
        calc_crc8(&frame);
      } 
    break;

    case 0x1DA: //motor response also needs to be modified      
      torqueResponse = (int16_t) (((frame.data[2] & 0x07) << 8) | frame.data[3]);
      torqueResponse = (torqueResponse & 0b0000011111111111); //only take out 11bits, no need to shift
        
      temp = (torqueResponse & 0b0000010000000000); //extract the 11th bit, this contains signed info.
      if (temp > 0){ //message is signed
        break;//We are getting regen, abort modification of message
      } 

      if(shift_state == SHIFT_DRIVE){ //modify power response message only when in drive
        if(torqueResponse > 1){ //(90*0.5=45Nm)
          //torqueResponse = torqueResponse*0.77; //Fool VCM that the response is smaller (OLD STRATEGY)
          torqueResponse = (VCMtorqueDemand*0.5); //Fool VCM that response is exactly the same as demand (remove 
            
          frame.data[2] = (torqueResponse >> 8);
          frame.data[3] = (torqueResponse & 0xFF);
          calc_crc8(&frame);
         }
      }
    break;

    case 0x284: //Hacky way of generating missing inverter message 
      //Upon reading VCM originating 0x284 every 20ms, send the missing message(s) to the inverter

      ticker40ms++;
      if(ticker40ms > 1)
      {
        ticker40ms = 0;
          
        if(can_bus == 1)
        {
          buffer_send_can2(inv_355_message); //40ms
        }
        else
        {
          buffer_send_can1(inv_355_message); //40ms
        }
      }
    break;

    case 0x50C: //Hacky way of generating missing inverter message 
      //Upon reading VCM originating 0x50C every 100ms, send the missing message(s) to the inverter
      if(can_bus == 1)
      {
        buffer_send_can2(inv_4B9_message); //100ms
        buffer_send_can2(inv_625_message); //100ms
        buffer_send_can2(inv_5C5_message); //100ms
        buffer_send_can2(inv_3B8_message); //100ms
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
        
      content_4B9++;
      if(content_4B9 > 79)
      {
        content_4B9 = 64;
      }
      inv_4B9_message.data[0] = content_4B9; //64 - 79 (0x40 - 0x4F)
        
      ticker100ms++; //500ms messages go here
      if(ticker100ms > 4)
      {
        ticker100ms = 0;
        if(can_bus == 1)
        {
          buffer_send_can2(inv_5EC_message); //500ms
          buffer_send_can2(inv_5EB_message); //500ms
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
      
    case 0x1F2: //Hacky way of generating missing inverter message
      //Upon reading VCM originating 0x1F2 every 10ms, send the missing message(s) to the inverter
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
      
    case 0x603:
      //Send new ZE1 wakeup messages, why not
      if(can_bus == 1)
      {
        buffer_send_can2(inv_605_message);
        buffer_send_can2(inv_607_message);
      }
      else
      {
        buffer_send_can1(inv_605_message);
        buffer_send_can1(inv_607_message);
      }
    break;

    default:
    break;
  }

  //--- Gateway all messages except the unwanted IDs
  //if you enable CAN repeating between bus 1 and 2, we end up here 
  if(repeat_can){
    
    //you can blacklist certain messages or message contents like this, blocking them from both being forwarded and being displayed
    //Use this to avoid generic transmission at the end of the function.
    //Specific transmission is done inside the switch statement
    uint8_t blacklist = 0;
    switch(frame.can_id)
    {     
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
}
*/
