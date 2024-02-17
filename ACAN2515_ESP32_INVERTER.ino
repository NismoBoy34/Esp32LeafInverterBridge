//Inverter Code moved to can_bridge_manager_leaf.cpp to test and functions Please note config.h #define //#define MESSAGE_0x1F2 should be disabled for 62Kwh upgrades
//——————————————————————————————————————————————————————————————————————————————
// Description: CAN translation strategy
// Author: Adam Saiyad, Julius Calzada (julius.jai@gmail.com)
// Revision: v1.3.0
// 06.28.2022: Migration to ESP32 Arduino environment
// 10.03.2022: Integrated earlyversion.c, can-bridge-firmware.h & nissan-can-structs.h
// 10.25.2022: v1.1.4: Integrated API for Web configuration to allow runtime configurability of vehicle parameters
// 10.28.2022: Separated LEAF from ENV200
// 10.31.2022: Improved Web Request Processing
// 11.11.2022: Created separate cpp files for can_bridge_manager common, leaf and env200
// 11.18.2022: Added CURRENT_CONTROL_ENABLED equivalent to CHARGECURRENT from leaf-can-bridge-3-port-master project
// 11.26.2022: Integrated configurable parameters for: BATTERY_SAVER_ENABLED/DISABLED, GLIDE_IN_DRIVE_ENABLED/DISABLED, 
// 12.02.2022: Updated CHARGECURRENT implementation for ID0x54B using CurrentControl Web parameters
// 12.04.2022: Merging of Inverter Upgrade based on https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/can-bridge-inverter.c
// 12.06.2022: Updated Charge Current logic - 1) Start conditions are charging state and fan speed; 2) Display kW for 15sec and revert to SOC
// 12.31.2022: Fix charge current functions Fix regen power and motor power Fix Glide and drive add code All in leaf.cpp
// 15.09.2023: When using esp32 u3 and version 3 chips change Using ESP32 (SJA1000) Internal Bus Controller - Initialization speed to 1000E instead of 500E
//——————————————————————————————————————————————————————————————————————————————

//——————————————————————————————————————————————————————————————————————————————
// IMPORTANT:
// 1. Download ACAN2515 library from:
//    https://www.arduino.cc/reference/en/libraries/acan2515/
// 2. Refer link on how to install the library:
//    https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries
// 3. For Internal CAN bus, use https://github.com/sandeepmistry/arduino-CAN
//
//——————————————————————————————————————————————————————————————————————————————

#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board" 
#endif

//——————————————————————————————————————————————————————————————————————————————

#include <Arduino.h>
#include "can_driver.h"
#include "can_bridge_manager_common.h"
#include "can_bridge_manager_leaf.h"
#include "can_bridge_manager_env200.h"
#include "config.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "SPIFFS.h"
#include "helper_functions.h"

#include <Preferences.h>
Preferences prefs;
void PREF_Init(void);
uint8_t Vehicle_Selection;
uint8_t Inverter_Upgrade_110Kw_160Kw;
uint8_t Battery_Selection;
uint8_t Battery_Saver;
uint8_t Glide_In_Drive;
uint8_t Current_Control;

//——————————————————————————————————————————————————————————————————————————————
// Web Socket Prototypes
//——————————————————————————————————————————————————————————————————————————————
void initWebSocket();
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len);
void notifyClients(String type);
String GetConfigValue(String type);

//——————————————————————————————————————————————————————————————————————————————
// LED Indicator
//——————————————————————————————————————————————————————————————————————————————
#define LED_BUILTIN 2
#define INTERVAL_1SEC     (1000000 / T_POLLING)
uint32_t counter_1sec = 0;

//——————————————————————————————————————————————————————————————————————————————
// Timer Interrupt
//——————————————————————————————————————————————————————————————————————————————
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t timerOsTick = 0;

void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux);
  timerOsTick++;   // Increment the counter and set the time of ISR
  portEXIT_CRITICAL_ISR(&timerMux);  
}  

void initTimer(void) {
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every T_POLLING (refer config.h)  
  timerAlarmWrite(timer, T_POLLING, true);

  // Start an alarm
  timerAlarmEnable(timer);

}

void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Replace with your network credentials
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";
void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
  
//——————————————————————————————————————————————————————————————————————————————
// Arduino Initialization
//——————————————————————————————————————————————————————————————————————————————
void setup () {

  #ifdef SERIAL_DEBUG_MONITOR // Must be disabled during production to avoid infinite loop when Serial Port is not available (refer config.h)
  //--- Start serial debug monitor
  Serial.begin (115200) ;
  //--- Wait for serial monitor to be connected
  while (!Serial) {
    delay (50);
  }
  #endif //#ifdef SERIAL_DEBUG_MONITOR

  //--- Start CAN communication
  hw_init();

  //--- Initialize timer interrupt
  initTimer();

  // initialize internal variable with nvm values
  PREF_Init();

  //--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
  //WiFi.begin(ssid, password);
  initSPIFFS();
  WiFi.softAP("Can-bridgeINV", "Password");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  //Websocket
  initWebSocket();
     
  //Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send(SPIFFS, "/index.html", String(), false);
  });

  server.on(
    "/settings",
    HTTP_POST,
  [](AsyncWebServerRequest * request) {
    if (request->hasParam("body", true)) {
      AsyncWebParameter* p = request->getParam("body", true);
      Serial.println(p->value().c_str());
      request->send(200, "text/plain", "OK");

      //Call routine to process the new web request
      WebRequestProcessing( p->value().c_str() );
    }
    else{
      request->send(200, "text/plain", "Fail");
      }
  });

  server.serveStatic("/static/", SPIFFS, "/static/");
  server.onNotFound(notFound);
  AsyncElegantOTA.begin(&server);
  server.begin();
  Serial.println("[Server] [HTTP] OK");

  LEAF_CAN_Bridge_Manager_Init();
}

//——————————————————————————————————————————————————————————————————————————————
// Main Loop (never ending loop)
//——————————————————————————————————————————————————————————————————————————————
void loop () {
  //---------------------------------------------------------------------------------
  // HIGH PRIORITY TASK (CONSIDERED REAL TIME, BASED ON CAN ISR)
  //---------------------------------------------------------------------------------
  // Important: CAN Reception Interrupt is handled by ACAN2515 library
  // Checking the available messages received at maximum speed (or unconditional) is ok.
  // The control is done by the library (available(), receive() methods)
  // The good thing is the ACAN2515 receive buffer size is 32, therefore it is less likely to have receive overflow
  // ToDo: If needed, we can increase the buffer size higher than 32  
  //#if defined(CAN_BRIDGE_FOR_LEAF)
    if( NISSAN_LEAF_CONFIG() )
    {
		LEAF_CAN_Bridge_Manager();
    }
    //else if( NISSAN_ENV200() )
    {
		//ENV200_CAN_Bridge_Manager();
 //   }
//	else
//	{
		//Not valid vehicle configuration
	}
  //#else
    //#error "Invalid vehicle or target vehicle is not defined!"
  //#endif

  //---------------------------------------------------------------------------------
  // LOW PRIORITY TASK (POLLING)
  //---------------------------------------------------------------------------------
  // This is only used for transmission buffer management and other tasks.
  // Refer T_POLLING for the actual frequency (refer config.h)
  // timerOsTick becomes 1 when 10ms has elapsed. In case of drifting (value > 1), normalize the counter by resetting to 0.
  if(timerOsTick > 0U) {  
     
    timerOsTick = 0U;

    // Timing for Application Tx Buffer handling
    Schedule_Buffer_Check_CAN(); 
    
    //LED indicator every 1 sec
    counter_1sec++;
    if(counter_1sec > INTERVAL_1SEC) {
       counter_1sec = 0;

       digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN));

       TIMER_Count();  
    }    
  }
  AsyncElegantOTA.loop();  
  ws.cleanupClients(); 
}

//——————————————————————————————————————————————————————————————————————————————

//——————————————————————————————————————————————————————————————————————————————
// Storage using Preferences
//——————————————————————————————————————————————————————————————————————————————
void PREF_Init(void)
{
  // Open flash reading
  prefs.begin("ESP32", false);  

  // If stored values exist, copy to internal variables; else assign with 0.
  Vehicle_Selection             = prefs.getUInt("VehSelect", 0);
  Inverter_Upgrade_110Kw_160Kw  = prefs.getUInt("InvUpgrade", 0);
  Battery_Selection             = prefs.getUInt("BattSelect", 0);
  Battery_Saver                 = prefs.getUInt("BattSaver", 0);
  Glide_In_Drive                = prefs.getUInt("GlideDrive", 0);
  Current_Control               = prefs.getUInt("CurrCont", 0);

  // Close the Preferences
  prefs.end();

  #ifdef DEBUG_NVM_PREFERENCE
  Serial.println("\nStored NVM After Reset Values");  
  Serial.println("\n(Stored NVM) Vehicle_Selection = ");
  Serial.print(Vehicle_Selection);
  
  Serial.println("\n(Stored NVM) Inverter_Upgrade_110Kw_160Kw = ");
  Serial.print(Inverter_Upgrade_110Kw_160Kw);
  
  Serial.println("\n(Stored NVM) Battery_Selection = ");
  Serial.print(Battery_Selection);
  
  Serial.println("\n(Stored NVM) Battery_Saver = ");
  Serial.print(Battery_Saver);
  
  Serial.println("\n(Stored NVM) Glide_In_Drive = ");
  Serial.print(Glide_In_Drive);

  Serial.println("\n(Stored NVM) Current_Control = ");
  Serial.print(Current_Control);
  #endif //#ifdef DEBUG_NVM_PREFERENCE
}

//——————————————————————————————————————————————————————————————————————————————

//——————————————————————————————————————————————————————————————————————————————
// OTA API
//——————————————————————————————————————————————————————————————————————————————
void Set_Vehicle_Selection(String setValue)
{

  //-----------------------------------------
  //Input: Radio Button
  //-----------------------------------------
  //name="Vehicle" id="Vehicle1" value="LEAF" 
  //name="Vehicle" id="Vehicle2" value="ENV200"
  //name="Vehicle" id="Vehicle3" value="ZE0"
  //name="Vehicle" id="Vehicle4" value="AZE0"
  //name="Vehicle" id="Vehicle5" value="ZE1"  
    
  //--------------------------------------
  // Output: Vehicle Selection
  //--------------------------------------
  //#define Vehicle_Selection_Nissan_LEAF_2010_2019 (0)
  //#define Vehicle_Selection_Nissan_ENV200         (1)
  //#define Vehicle_Selection_ZE0_2011_2012         (2)
  //#define Vehicle_Selection_AZE0_2013_2017        (3)
  //#define Vehicle_Selection_ZE1_2018_2022         (4)

  // Example: 
  // If OTA "Vehicle Selection" option is "Nissan LEAF 2010-2019"
  // Set_Vehicle_Selection(inputMessage[0]);

  // Open the preferences
  prefs.begin("ESP32", false);
  
  if(setValue == "LEAF")
  {
    Vehicle_Selection = Vehicle_Selection_Nissan_LEAF_2010_2019;
  } 
  else if(setValue == "ENV200")
  {
    Vehicle_Selection = Vehicle_Selection_Nissan_ENV200;
  }
  else if(setValue == "ZE0")
  {
    Vehicle_Selection = Vehicle_Selection_ZE0_2011_2012;
  }
  else if(setValue == "AZE0")
  {
    Vehicle_Selection = Vehicle_Selection_AZE0_2013_2017;
  }
  else if(setValue == "ZE1")
  {
    Vehicle_Selection = Vehicle_Selection_ZE1_2018_2022;
  }
  else
  { 
    Vehicle_Selection = Vehicle_Selection_Nissan_LEAF_2010_2019;    
  }

  //write to flash
  prefs.putUInt("VehSelect", Vehicle_Selection);

  // Close the Preferences
  prefs.end();

  //debug value
  #ifdef DEBUG_NVM_PREFERENCE
  Serial.println("\n(New NVM) Vehicle_Selection = ");
  Serial.print(Vehicle_Selection);
  #endif //#ifdef DEBUG_NVM_PREFERENCE
  
}

void Set_Inverter_Upgrade_110Kw_160Kw(String setValue)
{
  //--------------------------------------
  // Input: Radion Button
  //--------------------------------------
  //name="Inverter" id="Inverter1" value="110"
  //name="Inverter" id="Inverter2" value="160"
  //name="Inverter" id="Inverter3" value="0"
  
  //--------------------------------------
  // Output: Inverter Upgrade 110Kw/160Kw
  //--------------------------------------
  //#define Inverter_Upgrade_EM57_Motor_with_110Kw_inverter (0)
  //#define Inverter_Upgrade_EM57_Motor_with_160Kw_Inverter (1)
  //#define Inverter_Upgrade_Disabled                       (2)

  //Ex:
  // If OTA "Inverter Upgrade 110Kw/160Kw" option is "EM57 Motor with 110Kw inverter"
  // Set_Inverter_Upgrade_110Kw_160Kw(Inverter_Upgrade_EM57_Motor_with_110Kw_inverter);

  // Open the preferences
  prefs.begin("ESP32", false);
  
  if(setValue == "110")
  {    
    Inverter_Upgrade_110Kw_160Kw = Inverter_Upgrade_EM57_Motor_with_110Kw_inverter;
  }
  else if(setValue == "160")
  {
    Inverter_Upgrade_110Kw_160Kw = Inverter_Upgrade_EM57_Motor_with_160Kw_Inverter;
  }
  else if(setValue == "0")
  {
    Inverter_Upgrade_110Kw_160Kw = Inverter_Upgrade_Disabled;
  }
  else
  {   
    Inverter_Upgrade_110Kw_160Kw = Inverter_Upgrade_EM57_Motor_with_110Kw_inverter; 
  }

  //write to flash
  prefs.putUInt("InvUpgrade", Inverter_Upgrade_110Kw_160Kw);
  
  // Close the Preferences
  prefs.end();

  //debug NVM value
  #ifdef DEBUG_NVM_PREFERENCE
  Serial.println("\n(New NVM) Inverter_Upgrade_110Kw_160Kw = ");
  Serial.print(Inverter_Upgrade_110Kw_160Kw);
  #endif //#ifdef DEBUG_NVM_PREFERENCE
}

void Set_Battery_Selection(String setValue)
{
  //--------------------------------------
  // Input: Radio Button
  //--------------------------------------
  //name="Battery" id="Battery1" value="24"
  //name="Battery" id="Battery2" value="30"
  //name="Battery" id="Battery3" value="40"
  //name="Battery" id="Battery4" value="62"
  //name="Battery" id="Battery5" value="-1"  
  
  //--------------------------------------
  // Output: Battery Selection
  //--------------------------------------
  //#define Battery_Selection_24Kwh                       (0)
  //#define Battery_Selection_30Kw                        (1)
  //#define Battery_Selection_40Kwh                       (2)
  //#define Battery_Selection_62Kwh                       (3)
  //#define Battery_Selection_BruteForce_30KWh_24KWH_LBC  (4)

  //Ex:
  // If OTA "Battery Selection" option is "24Kwh"
  // Set_Battery_Selection(Battery_Selection_24Kwh);

  // Open the preferences
  prefs.begin("ESP32", false);
  /*
  if(setValue == "24")
  {
    Battery_Selection = Battery_Selection_24Kwh;
  }
  else if(setValue == "30")
  {
    Battery_Selection = Battery_Selection_30Kw;
  }
  else if(setValue == "40")
  {
    Battery_Selection = Battery_Selection_40Kwh;
  }
  else if(setValue == "62")
  {  
    Battery_Selection = Battery_Selection_62Kwh;
  }
  else if(setValue == "-1")
  {  
    Battery_Selection = Battery_Selection_BruteForce_30KWh_24KWH_LBC;
  }
  else
  {
    Battery_Selection = Battery_Selection_24Kwh;
  }
*/
  // write to flash
  prefs.putUInt("BattSelect", Battery_Selection);

  // Close the Preferences
  prefs.end();

  //debug NVM value
  #ifdef DEBUG_NVM_PREFERENCE
  Serial.println("\n(New NVM) Battery_Selection = ");
  Serial.print(Battery_Selection);
  #endif //#ifdef DEBUG_NVM_PREFERENCE
}

void Set_Battery_Saver(String setValue)
{
  //--------------------------------------
  // Input: Radio Button
  //--------------------------------------
  //name="BatterySaver" id="BatterySaver1" value="50"
  //name="BatterySaver" id="BatterySaver2" value="60"
  //name="BatterySaver" id="BatterySaver3" value="80"
  //name="BatterySaver" id="BatterySaver4" value="0"
    
  //--------------------------------------
  // Output: BatterySaver  
  //--------------------------------------
  //#define BatterySaver_50percent  (0)
  //#define BatterySaver_60percent  (1)
  //#define BatterySaver_80percent  (2)
  //#define BatterySaver_Disabled   (3)

  //Ex:
  //If OTA "BatterySaver" option is "50%"
  // Set_Battery_Saver(BatterySaver_50percent);

  // Open the preferences
  prefs.begin("ESP32", false);
/*
  if(setValue == "50")
  {  
    Battery_Saver = BatterySaver_50percent;
  }
  else if(setValue == "60")
  {
    Battery_Saver = BatterySaver_60percent;
  }
  else if(setValue == "80")
  {
    Battery_Saver = BatterySaver_80percent;
  }
  else if(setValue == "0")
  {   
    Battery_Saver = BatterySaver_Disabled;
  }
  else
  {  
    Battery_Saver = BatterySaver_50percent;    
  }

  //write to flash 
  prefs.putUInt("BattSaver", Battery_Saver);
*/
  // Close the Preferences
  prefs.end();

  //debug NVM value
  #ifdef DEBUG_NVM_PREFERENCE
  Serial.println("\n(New NVM) Battery_Saver = ");
  Serial.print(Battery_Saver);
  #endif //#ifdef DEBUG_NVM_PREFERENCE
}

void Set_Glide_In_Drive(String setValue)
{
  //--------------------------------------
  // Input: Radion Button
  //--------------------------------------  
  //name="Glide" id="Glide1" value="1"
  //name="Glide" id="Glide2" value="0"  
  
  //--------------------------------------
  // Output: Glide in Drive
  //--------------------------------------  
  //#define Glide_In_Drive_Enabled  (0)
  //#define Glide_In_Drive_Disabled (1)

  //Ex:
  // If OTA "Glide in Drive" option is "Enabled"
  // Set_Glide_In_Drive(Glide_In_Drive_Enabled);

  // Open the preferences
  prefs.begin("ESP32", false);
  /*
  if(setValue == "1")
  {    
    Glide_In_Drive = Glide_In_Drive_Enabled;
  }
  else if(setValue == "0")
  {
    Glide_In_Drive = Glide_In_Drive_Disabled;
  }
  else
  {  
    Glide_In_Drive = Glide_In_Drive_Enabled;
  } 

  //write to flash 
  prefs.putUInt("GlideDrive", Glide_In_Drive); 

  // Close the Preferences */
  prefs.end();

  //debug NVM value
  #ifdef DEBUG_NVM_PREFERENCE
  Serial.println("\n(New NVM) Glide_In_Drive = ");
  Serial.print(Glide_In_Drive);
  #endif //DEBUG_NVM_PREFERENCE
}

void Set_Current_Control(String setValue)
{
  //--------------------------------------
  // Input: Radion Button
  //--------------------------------------
  //name="CurrentControl" id="CurrentControl1" value="1"
  //name="CurrentControl" id="CurrentControl2" value="2"
  //name="CurrentControl" id="CurrentControl3" value="3"
  //name="CurrentControl" id="CurrentControl4" value="4"
  //name="CurrentControl" id="CurrentControl5" value="6"
  //name="CurrentControl" id="CurrentControl6" value="-1"
  
  //--------------------------------------
  // Output: CurrentControl
  //--------------------------------------
  //#define CurrentControl_1p0_kW           (0)
  //#define CurrentControl_2p0_kW           (1)
  //#define CurrentControl_3p0_kW           (2)
  //#define CurrentControl_4p0_kW           (3)
  //#define CurrentControl_6p0_kW           (4)
  //#define CurrentControl_Unrestricted_kW  (5)

  //Ex:
  // If OTA "CurrentControl" option is "1.0 KW"
  // Set_Current_Control(CurrentControl_1p0_kW);

  // Open the preferences
  prefs.begin("ESP32", false);
 /* 
  if(setValue == "1")
  {  
    Current_Control = CurrentControl_1p0_kW;
  }
  else if(setValue == "2")
  {  
    Current_Control = CurrentControl_2p0_kW;
  }
  else if(setValue == "3")
  {  
    Current_Control = CurrentControl_3p0_kW;
  }
  else if(setValue == "4")
  {  
    Current_Control = CurrentControl_4p0_kW;
  }
  else if(setValue == "6")
  {  
    Current_Control = CurrentControl_6p0_kW;
  }
  else if(setValue == "-1")
  {  
    Current_Control = CurrentControl_Unrestricted_kW;
  }
  else
  {    
    Current_Control = CurrentControl_1p0_kW;    
  }

  //write to flash 
  prefs.putUInt("CurrCont", Current_Control);

  // Close the Preferences
  prefs.end();
*/
  //debug NVM value
  #ifdef DEBUG_NVM_PREFERENCE
  Serial.println("\n(NVM) Current_Control = ");
  Serial.print(Current_Control);
  #endif //#ifdef DEBUG_NVM_PREFERENCE
}

//——————————————————————————————————————————————————————————————————————————————
// Web Request Parsing Routine, configuration values evaluation and storage to NVM
//——————————————————————————————————————————————————————————————————————————————
void WebRequestProcessing(const String data)
{
  int len1 = data.length();
  int len2;
  int y;
  char ch;
  String temp1;
  String temp2[14];  
  boolean prevAlphaNum = false;

  #ifdef DEBUG_WEB_PROCESSING
  Serial.printf("Parse Length: %d\n", len1);   
  Serial.println("String to parse: ");
  Serial.println(data);
  Serial.println("\n");
  #endif //#ifdef DEBUG_WEB_PROCESSING

  //First formatting
  //Read all alphanumeric, colon(:), hyphen (-) from the web request
  for(int x=0; x < len1; x++)
  {
    //Check each character
    ch = data.charAt(x);
    if(isAlphaNumeric(ch) || isDigit(ch) || ch == '-'  || ch == ':' || isWhitespace(ch))
    {
      //store is not space; replace space with ',' and store to temp1 buffer
      if(!isWhitespace(ch))
      {
        temp1 += ch;
      }
      else
      {
        temp1 += ',';
      }
    }
  }

  #ifdef DEBUG_WEB_PROCESSING
  Serial.println("\nParsed String Temp1: ");
  Serial.print(temp1);
  #endif //#ifdef DEBUG_WEB_PROCESSING
  
  //Second formatting
  //From temp1 buffer, read the names and values and store to temp2 buffer in an array organization
  len2 = temp1.length();
  y = 0; //this is used to jump to next array
  for(int x=0; x < len2; x++)
  {
    ch = temp1.charAt(x);
    if(isAlphaNumeric(ch) || isDigit(ch) || ch == '-' || ch == ':' || ch == ',')
    {
      if(isAlphaNumeric(ch) || isDigit(ch) || ch == '-')
      {
        temp2[y] += ch;
        prevAlphaNum = true;        
      }
      else if(ch == ':' || ch == ',')
      {
        if(prevAlphaNum == true)
        {
          prevAlphaNum = false;
          y++;
        }
      }
      else
      {;}
    }
  }

  #ifdef DEBUG_WEB_PROCESSING
  Serial.println("\n");
  Serial.println("\n(NEW WEB CONFIG)" + temp2[0] + " = " + temp2[1] + "\n");
  Serial.println("\n(NEW WEB CONFIG)" + temp2[2] + " = " + temp2[3] + "\n");
  Serial.println("\n(NEW WEB CONFIG)" + temp2[4] + " = " + temp2[5] + "\n");
  Serial.println("\n(NEW WEB CONFIG)" + temp2[6] + " = " + temp2[7] + "\n");
  Serial.println("\n(NEW WEB CONFIG)" + temp2[8] + " = " + temp2[9] + "\n");
  Serial.println("\n(NEW WEB CONFIG)" + temp2[10] + " = " + temp2[11] + "\n");
  Serial.println("\n(NEW WEB CONFIG)" + temp2[12] + " = " + temp2[13] + "\n");
  #endif //#ifdef DEBUG_WEB_PROCESSING

  //Copy final parsed values to global data and store to NVM
  Set_Vehicle_Selection(temp2[1]);
  Set_Inverter_Upgrade_110Kw_160Kw(temp2[3]);
  Set_Battery_Selection(temp2[5]);
  Set_Battery_Saver(temp2[7]);
  Set_Glide_In_Drive(temp2[9]);
  Set_Current_Control(temp2[13]);

  //End of Web request processing
}

//——————————————————————————————————————————————————————————————————————————————
// Websocket Implementation
//——————————————————————————————————————————————————————————————————————————————
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  
  switch (type) {
    case WS_EVT_CONNECT:
      #ifdef DEBUG_WEB_SOCKET
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      #endif

	    notifyClients("Vehicle");
	    notifyClients("Inverter");
      notifyClients("Battery");
      notifyClients("BatterySaver");  	  
      notifyClients("Glide");
      notifyClients("Capacity");
      notifyClients("CurrentControl");
      
      break;
    case WS_EVT_DISCONNECT:
      #ifdef DEBUG_WEB_SOCKET
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      #endif
      break;
	  
    case WS_EVT_DATA:
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void notifyClients(String type) { 
  
  #ifdef DEBUG_WEB_SOCKET
  Serial.println("\n notifyClients for type -> ");
  Serial.print(type);
  #endif  
  
  ws.textAll( String(GetConfigValue(type)) );
  
}

String GetConfigValue(String type)
{
	String retValue;
	
	#ifdef DEBUG_WEB_SOCKET
	Serial.println("GetConfigValue");
	#endif 
	
	//Read NVM stored values  

    //Vehicle Selection
    //#define NISSAN_LEAF_2010_to_2019()    (Vehicle_Selection == Vehicle_Selection_Nissan_LEAF_2010_2019)
    //#define NISSAN_ENV200()               (Vehicle_Selection == Vehicle_Selection_Nissan_ENV200)     
    //#define NISSAN_ZE0_2011_2012()        (Vehicle_Selection == Vehicle_Selection_ZE0_2011_2012)/
    //#define NISSAN_AZE0_2013_2017()       (Vehicle_Selection == Vehicle_Selection_AZE0_2013_2017) 
    //#define NISSAN_ZE1_2018_2022()        (Vehicle_Selection == Vehicle_Selection_ZE1_2018_2022) 
      
	if(type == "Vehicle")
	{
		if(NISSAN_LEAF_2010_to_2019()) {
			retValue = "Vehicle1";
		}
		else if(NISSAN_ENV200()){
			retValue = "Vehicle2";
		}
		else if(NISSAN_ZE0_2011_2012()){
			retValue = "Vehicle3";
		}
		else if(NISSAN_AZE0_2013_2017()){
			retValue = "Vehicle4";
		}
		else if(NISSAN_ZE1_2018_2022()){
			retValue = "Vehicle5";
		}
		else{
			retValue = "Vehicle1";
		}
	}
    //Inverter Upgrade 110Kw/160Kw
    //#define INVERTER_UPGRADE_EM57_MOTOR_WITH_110KW()    (Inverter_Upgrade_110Kw_160Kw == Inverter_Upgrade_EM57_Motor_with_110Kw_inverter)
    //#define INVERTER_UPGRADE_EM57_MOTOR_WITH_160KW()    (Inverter_Upgrade_110Kw_160Kw == Inverter_Upgrade_EM57_Motor_with_160Kw_Inverter)
    //#define INVERTER_UPGRADE_DISABLED()                 (Inverter_Upgrade_110Kw_160Kw == Inverter_Upgrade_Disabled)
	else if(type == "Inverter")
	{	
		if(INVERTER_UPGRADE_EM57_MOTOR_WITH_110KW()){
			retValue = "Inverter1";
		}
		else if(INVERTER_UPGRADE_EM57_MOTOR_WITH_160KW()){
			retValue = "Inverter2";
		}
		else if(INVERTER_UPGRADE_DISABLED()){
			retValue = "Inverter3";
		}
		else{
			retValue = "Inverter1";
		}
    }  
    //Battery Selection
    //#define BATTERY_24KWH()       (Battery_Selection == Battery_Selection_24Kwh)
    //#define BATTERY_30KWH()       (Battery_Selection == Battery_Selection_30Kw)
    //#define BATTERY_40KWH()       (Battery_Selection == Battery_Selection_40Kwh)
    //#define BATTERY_62KWH()       (Battery_Selection == Battery_Selection_62Kwh)
    //#define BATTERY_BRUTE_FORCE() (Battery_Selection == Battery_Selection_BruteForce_30KWh_24KWH_LBC) // Is this referring to BATTERY_SWAP???
 /*   else if(type == "Battery")
	{
		if(BATTERY_24KWH()){
			retValue = "Battery1";
		}
		  else if(BATTERY_30KWH()){
			retValue = "Battery2";
		}
		else if(BATTERY_40KWH()){
			retValue = "Battery3";
		}
		else if(BATTERY_62KWH()){
			retValue = "Battery4";
		}
		else if(BATTERY_BRUTE_FORCE()){
			retValue = "Battery5";
		}
		else{
			retValue = "Battery1";
		}
	}      
    //BatterySaver  
    //#define BATTERY_SAVER_50_PERCENT()  (Battery_Saver == BatterySaver_50percent)
    //#define BATTERY_SAVER_60_PERCENT()  (Battery_Saver == BatterySaver_60percent)
    //#define BATTERY_SAVER_80_PERCENT()  (Battery_Saver == BatterySaver_80percent)
    //#define BATTERY_SAVER_DISABLED()    (Battery_Saver == BatterySaver_Disabled)
    else if(type == "BatterySaver")
	{		
		if(BATTERY_SAVER_50_PERCENT()){
			retValue = "BatterySaver1";
		}
		else if(BATTERY_SAVER_60_PERCENT()){
			retValue = "BatterySaver2";
		}
		else if(BATTERY_SAVER_80_PERCENT()){
			retValue = "BatterySaver3";
		}
		else if(BATTERY_SAVER_DISABLED()){
			retValue = "BatterySaver4";
		}
		else {
			retValue = "BatterySaver1";
		}
	}	      
    //Glide in Drive
    //#define GLIDE_IN_DRIVE_ENABLED()    (Glide_In_Drive == Glide_In_Drive_Enabled)
    //#define GLIDE_IN_DRIVE_DISABLED()   (Glide_In_Drive == Glide_In_Drive_Disabled)
	else if(type == "Glide")
	{	
		if(GLIDE_IN_DRIVE_ENABLED()){
			retValue = "Glide1";
		}
		else if(GLIDE_IN_DRIVE_DISABLED()){
			retValue = "Glide2";
		}
		else{
			retValue = "Glide1";
		}
    }  
    
    }  
    //CurrentControl
    //#define CURRENT_CONTROL_1p0_KW()            (Current_Control == CurrentControl_1p0_kW)
    //#define CURRENT_CONTROL_2p0_KW()            (Current_Control == CurrentControl_2p0_kW)
    //#define CURRENT_CONTROL_3p0_KW()            (Current_Control == CurrentControl_3p0_kW)
    //#define CURRENT_CONTROL_4p0_KW()            (Current_Control == CurrentControl_4p0_kW)
    //#define CURRENT_CONTROL_6p0_KW()            (Current_Control == CurrentControl_6p0_kW)
    //#define CURRENT_CONTROL_UNRESTRICTED_KW()   (Current_Control == CurrentControl_Unrestricted_kW)
    else if(type == "CurrentControl")
	{
		if(CURRENT_CONTROL_1p0_KW()){
			retValue = "CurrentControl1";
		}
		else if(CURRENT_CONTROL_2p0_KW()){
			retValue = "CurrentControl2";
		}
		else if(CURRENT_CONTROL_3p0_KW()){
			retValue = "CurrentControl3";
		}
		else if(CURRENT_CONTROL_4p0_KW()){
			retValue = "CurrentControl4";
		}
		else if(CURRENT_CONTROL_6p0_KW()){
			retValue = "CurrentControl5";
		}
		else if(CURRENT_CONTROL_UNRESTRICTED_KW()){
			retValue = "CurrentControl6";
		}
		else{
			retValue = "CurrentControl1";
		}
	}	
	else 
	{
		//Nothing for this unknown type
	}	
*/	
	return retValue;
}
//——————————————————————————————————————————————————————————————————————————————
// End of Websocket Implementation
//——————————————————————————————————————————————————————————————————————————————
