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
#include "can_driver.h"
#include "helper_functions.h"
#include "config.h"

//——————————————————————————————————————————————————————————————————————————————
// Transmit Buffer Structure
//——————————————————————————————————————————————————————————————————————————————
//Because the MCP25625 transmit buffers seem to be able to corrupt messages (see errata), we're implementing
//our own buffering. This is an array of frames-to-be-sent, FIFO. Messages are appended to buffer_end++ as they
//come in and handled according to buffer_pos until buffer_pos == buffer_end, at which point both pointers reset
//the buffer size should be well in excess of what this device will ever see
static can_frame_t tx0_buffer[TXBUFFER_SIZE];
static uint8_t		tx0_buffer_pos		= 0;
static uint8_t		tx0_buffer_end		= 0;

static can_frame_t tx1_buffer[TXBUFFER_SIZE];
static uint8_t		tx1_buffer_pos		= 0;
static uint8_t		tx1_buffer_end		= 0;

static can_frame_t tx2_buffer[TXBUFFER_SIZE];
static uint8_t   tx2_buffer_pos    = 0;
static uint8_t   tx2_buffer_end    = 0;

//——————————————————————————————————————————————————————————————————————————————
// Hardware initialization
//——————————————————————————————————————————————————————————————————————————————
void hw_init(void){
  CAN_Init();
}

//——————————————————————————————————————————————————————————————————————————————
// Application CAN 0 Transmit Buffer
//——————————————————————————————————————————————————————————————————————————————
void buffer_send_can0(can_frame_t frame){	
	
	// Push to the buffer
	memcpy(&tx0_buffer[tx0_buffer_end], &frame, sizeof(frame));
	
	// Update buffer end counter
	tx0_buffer_end++;
	
	// Checks if end counter does not overflow
	// silently handle buffer overflows
	if(tx0_buffer_end >= TXBUFFER_SIZE){
		tx0_buffer_end = TXBUFFER_SIZE - 1;
		
		#ifdef SERIAL_DEBUG_MONITOR
		Serial.println("Application CAN0 Tx Buffer has overflowed!");
		#endif //#ifdef SERIAL_DEBUG_MONITOR
	}
	
	// Try to empty the buffer
  #ifdef CAN_CH0_ENABLED
	buffer_check_can0();
  #endif //CAN_CH0_ENABLED
}

//——————————————————————————————————————————————————————————————————————————————
// Application CAN0 Transmit Buffer Checking
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH0_ENABLED
void buffer_check_can0(void){
	
	// Checks if buffer is not empty
	if(tx0_buffer_end != tx0_buffer_pos){		

    //noInterrupts(); //disable interrupts
		
		// Check if CAN buffer is empty/ready and send 1 frame
		bool ok = direct_send_can0 (tx0_buffer[tx0_buffer_pos]);
		
		if(ok){
			//Update position if transmitted successfully
			tx0_buffer_pos++;
			//end of buffer, reset
			if(tx0_buffer_pos == tx0_buffer_end){ 
				tx0_buffer_end = 0;
				tx0_buffer_pos = 0;
			}
		}
		else{
			#ifdef SERIAL_DEBUG_MONITOR
			Serial.println("CAN0 Tx Buffer is not ready!");
			#endif //#ifdef SERIAL_DEBUG_MONITOR
		}
	
	  //interrupts(); //re-enable enterrupts
	}
}
#endif //CAN_CH0_ENABLED
//——————————————————————————————————————————————————————————————————————————————
// Application CAN 1 Transmit Buffer
//——————————————————————————————————————————————————————————————————————————————
void buffer_send_can1(can_frame_t frame){	
	
	// Push to the buffer
	memcpy(&tx1_buffer[tx1_buffer_end], &frame, sizeof(frame));
	
	// Update buffer end counter
	tx1_buffer_end++;
	
	// Checks if end counter does not overflow
	// silently handle buffer overflows
	if(tx1_buffer_end >= TXBUFFER_SIZE){
		tx1_buffer_end = TXBUFFER_SIZE - 1;
		
		#ifdef SERIAL_DEBUG_MONITOR
		Serial.println("Application CAN1 Tx Buffer has overflowed!");
		#endif //#ifdef SERIAL_DEBUG_MONITOR
	}
	
	// Try to empty the buffer
  #ifdef CAN_CH1_ENABLED
	//buffer_check_can1();
  #endif //CAN_CH1_ENABLED
}

//——————————————————————————————————————————————————————————————————————————————
// Application CAN1 Transmit Buffer Checking
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH1_ENABLED
void buffer_check_can1(void){
	
	// Checks if buffer is not empty
	if(tx1_buffer_end != tx1_buffer_pos){		

    //noInterrupts(); //disable interrupts
		
		// Check if CAN buffer is empty/ready and send 1 frame
		bool ok = direct_send_can1(tx1_buffer[tx1_buffer_pos]);
   
		if(ok){
			//Update position if transmitted successfully
			tx1_buffer_pos++;
			//end of buffer, reset
			if(tx1_buffer_pos == tx1_buffer_end){ 
				tx1_buffer_end = 0;
				tx1_buffer_pos = 0;
			}
		}
		else{
			#ifdef SERIAL_DEBUG_MONITOR
			Serial.println("CAN1 Tx Buffer is not ready!");
			#endif //#ifdef SERIAL_DEBUG_MONITOR
		}

    //interrupts(); //re-enable enterrupts
	}
}
#endif //CAN_CH1_ENABLED
//——————————————————————————————————————————————————————————————————————————————
// Application CAN 2 Transmit Buffer
//——————————————————————————————————————————————————————————————————————————————
void buffer_send_can2(can_frame_t frame){  
  
  // Push to the buffer
  memcpy(&tx2_buffer[tx2_buffer_end], &frame, sizeof(frame));
  
  // Update buffer end counter
  tx2_buffer_end++;
  
  // Checks if end counter does not overflow
  // silently handle buffer overflows
  if(tx2_buffer_end >= TXBUFFER_SIZE){
    tx2_buffer_end = TXBUFFER_SIZE - 1;
    
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.println("Application CAN2 Tx Buffer has overflowed!");
    #endif //#ifdef SERIAL_DEBUG_MONITOR
  }
  
  // Try to empty the buffer
  #ifdef CAN_CH2_ENABLED
  //buffer_check_can2();
  #endif //CAN_CH2_ENABLED
}

//——————————————————————————————————————————————————————————————————————————————
// Application CAN2 Transmit Buffer Checking
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH2_ENABLED
void buffer_check_can2(void){
  
  // Checks if buffer is not empty
  if(tx2_buffer_end != tx2_buffer_pos){   

    //noInterrupts(); //disable interrupts
    
    // Check if CAN buffer is empty/ready and send 1 frame
    bool ok = direct_send_can2(tx2_buffer[tx2_buffer_pos]);
    
    if(ok){
      //Update position if transmitted successfully
      tx2_buffer_pos++;
      //end of buffer, reset
      if(tx2_buffer_pos == tx2_buffer_end){ 
        tx2_buffer_end = 0;
        tx2_buffer_pos = 0;
      }
    }
    else{
      #ifdef SERIAL_DEBUG_MONITOR
      Serial.println("CAN2 Tx Buffer is not ready!");
      #endif //#ifdef SERIAL_DEBUG_MONITOR
    }
  
    //interrupts(); //re-enable enterrupts
  }
}
#endif //CAN_CH2_ENABLED
//——————————————————————————————————————————————————————————————————————————————
// Common scheduling for Application CAN Transmit Buffer
//——————————————————————————————————————————————————————————————————————————————
void Schedule_Buffer_Check_CAN(void) {
  
  #ifdef CAN_CH0_ENABLED
  buffer_check_can0();
  #endif //CAN_CH0_ENABLED

  #ifdef CAN_CH1_ENABLED
  buffer_check_can1();
  #endif //CAN_CH1_ENABLED

  #ifdef CAN_CH2_ENABLED
  buffer_check_can2();
  #endif //CAN_CH2_ENABLED
}

//——————————————————————————————————————————————————————————————————————————————
// Driver CAN Channel 0 transmission
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH0_ENABLED
bool direct_send_can0(can_frame_t frame){
  CANMessage txdata;
  uint16_t i;

  //noInterrupts(); //disable interrupts
  
  //Assemble tx data according to CANMessage format
  txdata.ext  = false; // Standard ID
  txdata.id   = frame.can_id;
  txdata.len  = frame.can_dlc;
  for(i=0; i<frame.can_dlc; i++) {
    txdata.data[i] = frame.data[i];
  }

  //Pass data to CAN0 transmit 
  bool ok = CAN0_Transmit(txdata);  

  //interrupts(); //re-enable enterrupts
  
  return ok;
  
}
#endif //CAN_CH0_ENABLED
//——————————————————————————————————————————————————————————————————————————————
// Driver CAN Channel 1 transmission
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH1_ENABLED
bool direct_send_can1(can_frame_t frame){
  CANMessage txdata;
  uint16_t i;

  //noInterrupts(); //disable interrupts

  //Assemble tx data according to CANMessage format
  txdata.ext  = false; // Standard ID
  txdata.id   = frame.can_id;
  txdata.len  = frame.can_dlc;
  for(i=0; i<frame.can_dlc; i++) {
    txdata.data[i] = frame.data[i];
  }

  //Pass data to CAN1 transmit 
  bool ok = CAN1_Transmit(txdata);

  //interrupts(); //re-enable enterrupts
  
  return ok;
  
}
#endif //CAN_CH1_ENABLED
//——————————————————————————————————————————————————————————————————————————————
// Driver CAN Channel 2 transmission
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH2_ENABLED
bool direct_send_can2(can_frame_t frame){  

  //noInterrupts(); //disable interrupts
  
  //Pass data to CAN1 transmit 
  bool ok = CAN2_Transmit(frame);

  //interrupts(); //re-enable enterrupts
    
  return ok;
  
}
#endif //CAN_CH2_ENABLED
