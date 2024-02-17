  #include "htmlFiles.h"

/* Struct that stores system settings*/
#define NW_SETTINGS_CONSTANT    65
struct NW_SETTINGS 
{
  uint8_t isAlreadyInit;

  char dev_mode[50];                // device mode - NOT_CONFIGURED, AP, STATION
  char ip_type[50];
  
  //Static IP related variables
  char s_ip[50];
  char gateway[50];
  char subnet[50];
  char primaryDNS[50];
  char secondaryDNS[50];
  
  // router settings
  char ssid[50];
  char pwd[50];
  
  // config page password
  char config_pswd[50];
  char user_pswd[50];
  uint8_t logging_frequency;
  uint8_t ap_channel;
} nw_settings;

/* This function decodes URL special character changes */
String decode_post_data(String user_input)
{
    user_input.replace("+"," ");
    user_input.replace("%21","!");
    user_input.replace("%23","#");
    user_input.replace("%24","$");
    user_input.replace("%26","&");
    user_input.replace("%27","'");
    user_input.replace("%28","(");
    user_input.replace("%29",")");
    user_input.replace("%2A","*");
    user_input.replace("%2B","+");
    user_input.replace("%2C",",");
    user_input.replace("%2F","/");
    user_input.replace("%3A",":");
    user_input.replace("%3B",";");
    user_input.replace("%3D","=");
    user_input.replace("%3F","?");
    user_input.replace("%40","@");
    user_input.replace("%5B","[");
    user_input.replace("%5D","]");
    
    return user_input;
}

void startConfigPortal(void) 
{
  const byte DNS_PORT = 53;
  IPAddress apIP(192,168,4,1);
  DNSServer dnsServer;
  WiFiServer server(80);

  WiFi.mode(WIFI_AP);
  WiFi.softAP("Leaf");
  WiFi.softAPConfig(apIP, apIP, IPAddress(255,255,255,0));
  delay(100);

  dnsServer.start(DNS_PORT, "*", apIP);
  server.begin();

  while(1) 
  {
    dnsServer.processNextRequest();
    WiFiClient client = server.available();
    uint8_t flagFirstLine = 0;
    String req_method = "";

    if(client) 
    {
      String line = "";
      while(line = client.readStringUntil('\n')) 
      {
        // Serial.println(line);
        if(line.length() < 1) { break; }
        
        if(flagFirstLine == 0) 
        {
          req_method = line.substring(0, line.indexOf(" ", 0));
          Serial.println(line);
          flagFirstLine = 1;
          if(req_method == "POST") 
          {
            flagFirstLine = 5;
          }
          else 
          {
            if(line.indexOf("favicon") > -1)
            {
              flagFirstLine = 2;
            }
          }
        }
    
        if(flagFirstLine == 5) 
        {
          // String line = dev_mode=AP&ip_type=STATIC&s_ip=45.45.45.45&gateway=&subnet=&primaryDNS=&secondaryDNS=&ssid=&pwd=
          if(line.indexOf("dev_mode") > -1) 
          {
            uint8_t index = 0;

            index = line.indexOf("dev_mode") + 9;
            strcpy(nw_settings.dev_mode, decode_post_data(line.substring(index, line.indexOf("&", index))).c_str());
            index = line.indexOf("ip_type") + 8;
            strcpy(nw_settings.ip_type, decode_post_data(line.substring(index, line.indexOf("&", index))).c_str());
            index = line.indexOf("s_ip") + 5;
            strcpy(nw_settings.s_ip, decode_post_data(line.substring(index, line.indexOf("&", index))).c_str());
            index = line.indexOf("gateway") + 8;
            strcpy(nw_settings.gateway, decode_post_data(line.substring(index, line.indexOf("&", index))).c_str());
            index = line.indexOf("subnet") + 7;
            strcpy(nw_settings.subnet, decode_post_data(line.substring(index, line.indexOf("&", index))).c_str());
            index = line.indexOf("primaryDNS") + 11;
            strcpy(nw_settings.primaryDNS, decode_post_data(line.substring(index, line.indexOf("&", index))).c_str());
            index = line.indexOf("secondaryDNS") + 13;
            strcpy(nw_settings.secondaryDNS, decode_post_data(line.substring(index, line.indexOf("&", index))).c_str());
            index = line.indexOf("ssid") + 5;
            strcpy(nw_settings.ssid, decode_post_data(line.substring(index, line.indexOf("&", index))).c_str());
            index = line.indexOf("pwd") + 4;
            strcpy(nw_settings.pwd, decode_post_data(line.substring(index, line.indexOf("&", index))).c_str());
            flagFirstLine = 6;
          }
        }
      }
    
      if((req_method == "GET") && (flagFirstLine != 2)) 
      {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/html");
        client.println();
        client.println(captivePortalMainPage);
      }
      else if (req_method == "POST") 
      {
        if(flagFirstLine == 6) 
        {
          Serial.print("dev_mode : ");     Serial.println(nw_settings.dev_mode);
          Serial.print("ip_type : ");      Serial.println(nw_settings.ip_type);
          Serial.print("s_ip : ");         Serial.println(nw_settings.s_ip);
          Serial.print("gateway : ");      Serial.println(nw_settings.gateway);
          Serial.print("subnet : ");       Serial.println(nw_settings.subnet);
          Serial.print("primaryDNS : ");   Serial.println(nw_settings.primaryDNS);
          Serial.print("secondaryDNS : "); Serial.println(nw_settings.secondaryDNS);
          Serial.print("ssid : ");         Serial.println(nw_settings.ssid);
          Serial.print("pwd : ");          Serial.println(nw_settings.pwd);
          
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.println(resetPage);
          client.stop();

          EEPROM.put(512, nw_settings);
          EEPROM.commit();

          Serial.print("\nPlease reset device ");
          delay(5000);
          ESP.restart();
          while (1)
          {
          delay(1000);
          Serial.print(".");
          }
        }
        else 
        {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.println(errorPage);
        }
      }
    
      client.stop();
    }
    
    //if(millis() > (ledTimer + 1000)) 
    //{
      //ledTimer = millis();
      //if(flagLedState) 
      //{
        //flagLedState = 0;
        //digitalWrite(2, HIGH);
      //}
      //else 
      //{
        //flagLedState = 1;
        //digitalWrite(2, LOW);
      //}
    //}
  }
}
