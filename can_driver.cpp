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
// 3. For Internal CAN bus, use https://github.com/sandeepmistry/arduino-CAN
//——————————————————————————————————————————————————————————————————————————————

//——————————————————————————————————————————————————————————————————————————————
#include <Arduino.h>
#include <SPI.h>
#include "can_driver.h"
#include "config.h"
#include <CAN.h>
#include "canframe.h"
#include "can_bridge_manager_common.h"
#include "can_bridge_manager_leaf.h"
#include "can_bridge_manager_env200.h"
#include "helper_functions.h"

//——————————————————————————————————————————————————————————————————————————————
// CAN communication using ESP32 and MCP2515 hardware and ACAN2515 Arduino library.
// Interface between ESP32 and MCP2515 is through SPI.
// ESP32 has 2 SPI peripherals called VSPI and HSPI.
// Two SPI objects are created for VSPI and HSPI repectively. <SPI.h> is inherited directly from the Arduino standard library.
// ACAN2515 instantiates CAN object with each corresponding SPI object.
// Note: ACAN2515 is used because SPI can be configured for VSPI and HSPI. Other library is using fixed SPI channel.
//——————————————————————————————————————————————————————————————————————————————

//——————————————————————————————————————————————————————————————————————————————
// CAN0 SPI Configuration
//——————————————————————————————————————————————————————————————————————————————
// ESP32                MCP2515
//——————————————————————————————————————————————————————————————————————————————
// GPIO26:SPI_CLK   ->  PIN13:SCK
// GPIO19:SPI_MISO  <-  PIN15:SO
// GPIO18:SPI_MOSI  ->  PIN14:SI
// GPIO27:SPI_CS    ->  PIN16:CS
// GPIO23:CAN0_INT  <-  PIN10:INT
//——————————————————————————————————————————————————————————————————————————————
static const byte MCP2515_SCK_CAN0  = 26; // SCK input of MCP2515
static const byte MCP2515_MOSI_CAN0 = 19; // SDI input of MCP2515 changesd from 19
static const byte MCP2515_MISO_CAN0 = 18; // SDO output of MCP2515 changed from 18
static const byte MCP2515_CS_CAN0   = 27; // CS input of MCP2515
static const byte MCP2515_INT_CAN0  = 23; // INT output of MCP2515

//——————————————————————————————————————————————————————————————————————————————
// CAN1 SPI Configuration
//——————————————————————————————————————————————————————————————————————————————
// ESP32 MCP2515
//——————————————————————————————————————————————————————————————————————————————
// GPIO26:SPI_CLK   ->  PIN13:SCK
// GPIO19:SPI_MISO  <-  PIN15:SO
// GPIO18:SPI_MOSI  ->  PIN14:SI
// GPIO15:SPI_CS    ->  PIN16:CS
// GPIO22:CAN1_INT  <-  PIN10:INT
//——————————————————————————————————————————————————————————————————————————————
static const byte MCP2515_SCK_CAN1  = 26; // SCK input of MCP2515
static const byte MCP2515_MOSI_CAN1 = 19; // SDI input of MCP2515 cahnged from 19
static const byte MCP2515_MISO_CAN1 = 18; // SDO output of MCP2515 changed from 18
static const byte MCP2515_CS_CAN1   = 15; // CS input of MCP2515
static const byte MCP2515_INT_CAN1  = 22; // INT output of MCP2515

//——————————————————————————————————————————————————————————————————————————————
// MCP2515 Quartz: <ToDo> adapt to your design
//——————————————————————————————————————————————————————————————————————————————
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 8 MHz

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

#ifdef CAN_CH0_ENABLED
SPIClass spi0; 
ACAN2515 can0 (MCP2515_CS_CAN0, spi0, MCP2515_INT_CAN0);
#endif //CAN_CH0_ENABLED

#ifdef CAN_CH1_ENABLED
SPIClass spi1;
ACAN2515 can1 (MCP2515_CS_CAN1, spi1, MCP2515_INT_CAN1);
#endif //CAN_CH1_ENABLED

//——————————————————————————————————————————————————————————————————————————————
//  CAN2 Variables
//——————————————————————————————————————————————————————————————————————————————

//——————————————————————————————————————————————————————————————————————————————
//  CAN Initialization
//——————————————————————————————————————————————————————————————————————————————
void CAN_Init(void) {
  
  //--- Begin SPI
  #ifdef CAN_CH0_ENABLED
  spi0.begin (MCP2515_SCK_CAN0, MCP2515_MISO_CAN0, MCP2515_MOSI_CAN0) ;
  #endif //CAN_CH0_ENABLED

  #ifdef CAN_CH1_ENABLED
  spi1.begin (MCP2515_SCK_CAN1, MCP2515_MISO_CAN1, MCP2515_MOSI_CAN1) ;
  #endif //CAN_CH1_ENABLED
  
  //--- Configure CAN0 and CAN1 settings
  #ifdef CAN_CH0_ENABLED
  ACAN2515Settings can0_settings (QUARTZ_FREQUENCY, 500UL * 1000UL) ; // CAN bit rate 500 kb/s
  #endif //CAN_CH0_ENABLED

  #ifdef CAN_CH1_ENABLED
  ACAN2515Settings can1_settings (QUARTZ_FREQUENCY, 500UL * 1000UL) ; // CAN bit rate 500 kb/s
  #endif //CAN_CH0_ENABLED
  
  //--- Start CAN channels and report CAN parameters if successful
  #ifdef CAN_CH0_ENABLED
  uint16_t errorCode0 = can0.begin (can0_settings, [] { can0.isr () ; }) ;
  #endif //CAN_CH0_ENABLED

  #ifdef CAN_CH1_ENABLED
  uint16_t errorCode1 = can1.begin (can1_settings, [] { can1.isr () ; }) ;
  #endif //CAN_CH1_ENABLED

  #ifdef CAN_CH0_ENABLED
  //--- Report CAN0 configuration in serial monitor
  if (0U == errorCode0) {
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.println("CAN Channel 0 Configuration Succeeded");
    Serial.print("Bit Rate prescaler: ");
    Serial.println(can0_settings.mBitRatePrescaler);
    Serial.print("Propagation Segment: ");
    Serial.println(can0_settings.mPropagationSegment);
    Serial.print("Phase segment 1: ");
    Serial.println(can0_settings.mPhaseSegment1);
    Serial.print("Phase segment 2: ");
    Serial.println(can0_settings.mPhaseSegment2);
    Serial.print("SJW: ");
    Serial.println(can0_settings.mSJW);
    Serial.print("Triple Sampling: ");
    Serial.println(can0_settings.mTripleSampling ? "yes" : "no");
    Serial.print("Actual bit rate: ");
    Serial.print(can0_settings.actualBitRate ());
    Serial.println(" bit/s");
    Serial.print("Exact bit rate ? ");
    Serial.println(can0_settings.exactBitRate () ? "yes" : "no");
    Serial.print("Sample point: ");
    Serial.print(can0_settings.samplePointFromBitStart ());
    Serial.println("%");
    #endif //#ifdef SERIAL_DEBUG_MONITOR
  }else{
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.print("CAN Channel 0 Configuration Failed: Error code 0x");
    Serial.println(errorCode0, HEX);
    #endif //#ifdef SERIAL_DEBUG_MONITOR
  }
  #endif //CAN_CH0_ENABLED

  #ifdef CAN_CH1_ENABLED
  //--- Report CAN1 configuration in serial monitor
  if (0U == errorCode1) {
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.println("CAN Channel 1 Configuration Succeeded");
    Serial.print("Bit Rate prescaler: ");
    Serial.println(can1_settings.mBitRatePrescaler);
    Serial.print("Propagation Segment: ");
    Serial.println(can1_settings.mPropagationSegment);
    Serial.print("Phase segment 1: ");
    Serial.println(can1_settings.mPhaseSegment1);
    Serial.print("Phase segment 2: ");
    Serial.println(can1_settings.mPhaseSegment2);
    Serial.print("SJW: ");
    Serial.println(can1_settings.mSJW);
    Serial.print("Triple Sampling: ");
    Serial.println(can1_settings.mTripleSampling ? "yes" : "no");
    Serial.print("Actual bit rate: ");
    Serial.print(can1_settings.actualBitRate ());
    Serial.println(" bit/s");
    Serial.print("Exact bit rate ? ");
    Serial.println(can1_settings.exactBitRate () ? "yes" : "no");
    Serial.print("Sample point: ");
    Serial.print(can1_settings.samplePointFromBitStart ());
    Serial.println("%");
    #endif //#ifdef SERIAL_DEBUG_MONITOR
  }else{
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.print("CAN Channel 1 Configuration Failed: Error code 0x");
    Serial.println(errorCode1, HEX);
    #endif //#ifdef SERIAL_DEBUG_MONITOR
  }
  #endif //CAN_CH1_ENABLED

  // Internal CAN Controller
  #ifdef CAN_CH2_ENABLED
  CAN2_Init();
  #endif //CAN_CH2_ENABLED
}

//——————————————————————————————————————————————————————————————————————————————
//  CAN0 Transmission Routine
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH0_ENABLED
bool CAN0_Transmit(CANMessage frame) {
  bool ok = can0.tryToSend (frame);    
  if(ok) {
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.print("CAN0 Transmission Succeeded");
    #endif //#ifdef SERIAL_DEBUG_MONITOR
  }else {
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.println("CAN0 Transmission Failed");
    #endif //#ifdef SERIAL_DEBUG_MONITOR 
  }

  return ok;
}
#endif //CAN_CH0_ENABLED

//——————————————————————————————————————————————————————————————————————————————
//  CAN1 Transmission Routine
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH1_ENABLED
bool CAN1_Transmit(CANMessage frame) {
  bool ok = can1.tryToSend (frame);    
  if(ok) {
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.print("CAN1 Transmission Succeeded");
    #endif //#ifdef SERIAL_DEBUG_MONITOR
  }else {
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.println("CAN1 Transmission Failed");
    #endif //#ifdef SERIAL_DEBUG_MONITOR
  }

  return ok;
}
#endif //CAN_CH1_ENABLED

//——————————————————————————————————————————————————————————————————————————————
//  CAN0 Reception Monitor
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH0_ENABLED
bool CAN0_NewFrameIsAvailable(void) {
  return (can0.available());
}
#endif //CAN_CH0_ENABLED

#ifdef CAN_CH0_ENABLED
void CAN0_ReadNewFrame(CANMessage &frame){
  (void)can0.receive(frame); 
}
#endif//CAN_CH0_ENABLED

//——————————————————————————————————————————————————————————————————————————————
//  CAN1 Reception Monitor
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH1_ENABLED
bool CAN1_NewFrameIsAvailable(void) {
  return (can1.available());
}
#endif //CAN_CH1_ENABLED

#ifdef CAN_CH1_ENABLED
void CAN1_ReadNewFrame(CANMessage &frame){
  (void)can1.receive(frame); 
}
#endif //CAN_CH1_ENABLED
  
//——————————————————————————————————————————————————————————————————————————————
//  Using ESP32 (SJA1000) Internal Bus Controller - Initialization 
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH2_ENABLED
void CAN2_Init(void) {
  int result = 0;
  
  // start the CAN bus at 500 kbps
  //result = CAN.begin(1000E3); //Use for esp32UE boards
  result = CAN.begin(500E3); //Use for esp32u boards
  
  if (!result) {
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.println("ESP32 SJA1000 CAN2 Initialization Failed!");
    #endif //SERIAL_DEBUG_MONITOR    
  }
  else{
    // continue the rest of initialization
    // register the receive callback
    CAN.onReceive(CAN2_onReceive);
  }
}
#endif //CAN_CH2_ENABLED
//——————————————————————————————————————————————————————————————————————————————
//  Using ESP32 (SJA1000) Internal Bus Controller - Interrupt Service Routine
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH2_ENABLED
void CAN2_onReceive(int packetSize) {
  unsigned int i = 0;
  can_frame_t rx_frame;

  noInterrupts(); //disable interrupts
  
  //Organize Received Message  
  rx_frame.can_id  = CAN.packetId();
  rx_frame.can_dlc = CAN.packetDlc();  
    
  //Copy CAN buffer data to driver variable
  //Guard innfinite loop by checking counter i reaching to a not logical value
  //Logical value is 1-8
  while(CAN.available() && i < 8) {
    rx_frame.data[i++] = CAN.read();
  }

  //Validate number of recection interrupts
  if(i > 8){
    #ifdef SERIAL_DEBUG_MONITOR
    Serial.println("CAN2 Reception interruption is > 8");
    #endif //#SERIAL_DEBUG_MONITOR
  }
  else {
    //Push to CAN0/CAN1 Tx buffer
    LEAF_CAN_Handler(CAN_CHANNEL_2, rx_frame);
  }
     
  #ifdef SERIAL_DEBUG_MONITOR   
   Serial.print("CAN2 Received:");
   Serial.print(rx_frame.can_id, HEX);
   Serial.print(" | ");
   Serial.print(rx_frame.can_dlc, HEX);
   Serial.print(" | "); 
   
   for(i = 0; i<rx_frame.can_dlc; i++) {
    Serial.print(rx_frame.data[i], HEX);
   }
   Serial.println();
  #endif // SERIAL_DEBUG_MONITOR 

  interrupts(); //re-enable enterrupts

}
#endif //CAN_CH2_ENABLED
//——————————————————————————————————————————————————————————————————————————————
//  Using ESP32 (SJA1000) Internal Bus Controller - CAN Transmit Routine
//——————————————————————————————————————————————————————————————————————————————
#ifdef CAN_CH2_ENABLED
bool CAN2_Transmit(can_frame tx_frame){
  bool ok = true;
  CAN.beginPacket(tx_frame.can_id);
  CAN.write(tx_frame.data, tx_frame.can_dlc);
  CAN.endPacket();

  return ok;
  
}
#endif //CAN_CH2_ENABLED

//——————————————————————————————————————————————————————————————————————————————
