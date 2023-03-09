/************************************************************
 * ESP32 Hello World
 ************************************************************
 * (c) by dario carluccio davidsgateway [at] carluccio.de
 ************************************************************
 * Hardware:
 * - ESP 32
 * - ESP RFM69 @ 868MHz Board
 ************************************************************
 * Functionality:
 * - Receive Data from Davis Vanage ISS Weather Station
 * - Decode Raw Data to Weather Measurements
 * - This Measurments will be decoded:
 *   - WindSpeed [km/h]
 *   - WindDirection [0-359°]
 *   - BattWarning [0|1]
 *   - Additional Measurments: 
 *     - GustSpeed [km/h]
 *     - OutsideTemperature [°C]
 *     - OutsideHumidity [%rel]
 *     - Rainrate [mm/h]
 *     - SolarRadiation 
 *     - GoldcapVoltage [V]
 *   - Publish Weather Measurements after each Packet received 
 *     (every 2.5s) as Json Data e.g:
 *     {"WindSpeed": 3.22,  
 *     "WindDirection": 257,
 *     "BattWarning": 0, 
 *     "Payload": "80:02:37:1b:ab:0d:4e:dc", 
 *     "Channel":1, 
 *     "RSSI":-74,
 *     "msgID":8,
 *     "OutsideTemperature":6.67}
 * - Each Packet from the Davis ISS contains only one additional 
 *   Measurment.  
 ************************************************************
 * Basic Core Functionality:
 * - Wifi Connect (Provide SSID+Pass in platformio.ini)
 * - MQTT Connect (Provide TOPIC in platformio.ini)
 * - OTA Update (needs a UDP connection from ESP to IDE-PC)
 * - Monitor Wfi & MQTT and reconnect on error
 * - Send different States every 10s, 30s or 60s
 * - Accept and Parse commands over MQTT
 * - Automatic increment Version 
 *   - incrementafter upload to Production target
 *   - copy binary to release Folder
 ***********************************************************/

/************************************************************
 * Includes
 ************************************************************/ 
// External Libraries
#include <WiFi.h>                // Wifi
#include <WiFiUdp.h>             // Wifi / OTA
#include <WiFiClient.h>          // Wifi 
#include <PubSubClient.h>        // MQTT  
#include <ESPmDNS.h>             // for OTA-Update
#include <ArduinoOTA.h>          // for OTA-Update
#include <CommandParser.h>       // To Parse MQTT Commands
#include <SimpleTime.h>          // Time Conversions 
// Own Project Files
#include <prototypes.h>          // Prototypes 
#include <myHWconfig.h>          // Hardware Wireing
#include <Version.h>             // Automatic Version Incrementing (triggered by Upload to Production)
#include <debugOptions.h>        // Debugging [my be improved]
// Project Libraries
#include <SPI.h>
#include <DavisRFM69.h>   // C:\Users\vandusen\Documents\VSCode\ESP32-Davis-Gateway\include\DavisRFM69.h


/************************************************************
 * Compile Target
 ************************************************************/ 
#ifndef TARGET
  #define TARGET "UNKNOWN"
#endif

/************************************************************
 * WIFI Settings
 ************************************************************/ 
// WiFi-SSID 
// should be defined in plattformio.ini e.g.: build_flags = '-DWIFI_SSID="myhomerouter"'
#ifndef WIFI_SSID
  #define WIFI_SSID "myssid"
#endif
// WiFi-Pass 
// should be defined in plattformio.ini e.g.: build_flags = '-DWIFI_PSK="mysecretpassword"'
#ifndef WIFI_PSK
  #define WIFI_PSK  "mypassword"
#endif

/************************************************************
 * MQTT-Settings
 ************************************************************/ 
// Prefix for MQTT Topics should be defined in plattformio.ini
// e.g.: build_flags = '-DMQTT_PREFIX="homectrl/tisch"'
#ifndef  MQTT_SERVER    
  #define MQTT_SERVER "mqtt.example.de"
#endif
#ifndef  MQTT_PORT
  #define MQTT_PORT 1883
#endif
#ifndef  MQTT_USER
  #define MQTT_USER ""
#endif
#ifndef  MQTT_PASS
  #define MQTT_PASS ""
#endif
#ifndef MQTT_PREFIX
  #define MQTT_PREFIX "esp32/default"
#endif

// MQTT-Connection Settings
#define MQTT_BUFSIZE   2048                       // MQTT-Buffersize (may be augmented, when Scan returns many BLE-Devices
// Topic used to subscribe, MQTT_PREFIX will be added
#define T_CMD          "cmd"                      // Topic for Commands (subscribe) (MQTT_PREFIX will be added)
// Topics used to publish, MQTT_PREFIX will be added
#define T_ISS          "ISS"                      // Topic for ISS Data
#define T_RFMSTATS     "rfmstats"                 // Topic for RFM69 Statistics
#define T_CPU          "cpu"                      // Topic for CPU Status
#define T_LOG          "log"                      // Topic for Logging
#define T_NETWORK      "network"                  // Topic for Network Status 
#define T_RESULT       "result"                   // Topic for Commands Responses
#define T_SKETCH       "sketch"                   // Topic for Sketch Status 
#define T_STATUS       "status"                   // Topic for Online-Status 'ONLINE/OFFLINE' (published at birth and lastwill) (MQTT_PREFIX will be added)
#define STATUS_MSG_ON  "ONLINE"                   // Online Message
#define STATUS_MSG_OFF "OFFLINE"                  // Last Will Message

/************************************************************
 * Debug LED
 ************************************************************/ 
#define DBG_LED 2

/************************************************************
 * Timings
 ************************************************************/ 
#define T_HEARTBEAT_1S         1000  // cron every 1 second
#define T_HEARTBEAT_10S       10000  // cron every 10 seconds
#define T_HEARTBEAT_30S       30000  // cron every 30 seconds
#define T_HEARTBEAT_60S       60000  // cron every 60 seconds
#define T_STATE_LONG          60000  // Print detailed Status every 1 minute
#define T_STATE_SHORT          1000  // Print Status every 1 second
#define T_MQTT_RECONNECT       5000  // How often check MQTT: 5 seconds
#define T_NET_MONITORING      10000  // How often check Wifi: 10 seconds 
#define T_WIFI_MAX_TRIES         10  // Howoften retry to reconnect Wifi: 10 (repeated later)
#define T_REBOOT_TIMEOUT       5000  // ms until Reboot is triggered when g_rebootActive = true

/************************************************************
 * RFM Params
 ************************************************************/ 
#define PACKET_INTERVAL 2500   // After 2,5s a Packet should be recived, if not: Hop anyway
#define PACKET_OFFSET   500    // Hop after (N * PACKET_INTERVAL) + PACKET_OFFSET  [N = number off Packets missed so far]
#define PACKET_LONGHOP  20000  // Hop every PACKET_LONGHOP, if more than 20 Packes in a steak have been missed


/************************************************************
 * Objects
 ************************************************************/ 
// WIFI Client
WiFiClient myWiFiClient;

// MQTT Client
PubSubClient mqtt(MQTT_SERVER, MQTT_PORT, myWiFiClient);

// IRQ Handling
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// CommandParser
#define PARSER_NUM_COMMANDS   4   // limit number of commands 
#define PARSER_NUM_ARGS       2   // limit number of arguments
#define PARSER_CMD_LENGTH     10  // limit length of command names [characters]
#define PARSER_ARG_SIZE       32  // limit size of all arguments [bytes]
#define PARSER_RESPONSE_SIZE  64  // limit size of response strings [bytes]
typedef CommandParser<PARSER_NUM_COMMANDS, PARSER_NUM_ARGS, PARSER_CMD_LENGTH, PARSER_ARG_SIZE, PARSER_RESPONSE_SIZE> MyCommandParser;
MyCommandParser parser;
// Command Handler Prototypes
void cmd_hello (MyCommandParser::Argument *args, char *response);      // "hello", ""
void cmd_newday(MyCommandParser::Argument *args, char *response);      // "newDay", ""
void cmd_setrc (MyCommandParser::Argument *args, char *response);      // "setRC", "u"
void cmd_reset (MyCommandParser::Argument *args, char *response);      // "reset", ""

// DavisRFM69 radio;            
DavisRFM69  radio(RFM_CS, RFM_IRQ, RFM_HW);


/************************************************************
 * Global Vars
 ************************************************************/ 
boolean       g_Firstrun; 
// Monitoring
uint32_t      g_LastHeartbeat_1s; 
uint32_t      g_LastHeartbeat_10s;
uint32_t      g_LastHeartbeat_30s;
uint32_t      g_LastHeartbeat_60s;
uint32_t      g_LastMqttReconnectAttempt;
uint32_t      g_LastStateLong;
uint32_t      g_LastStateShort;
uint32_t      g_LastNetMonitoring;
uint32_t      g_LastRollerMonitoring;
uint8_t       g_LedState;
// MQTT
uint32_t      g_MqttReconnectCount;
// Wifi
boolean       g_wificonnected;
const char*   g_wifissid = WIFI_SSID;
const char*   g_wifipass = WIFI_PSK;
const char*   g_otahash = OTA_HASH;
// IRQ
volatile boolean g_IrqFlag;
boolean       g_LastIRQ;
// Reboot Timer
boolean       g_rebootActive;                // if true trigger reeboot 5s after g_reboot_triggered
uint32_t      g_rebootTriggered;             // millis() when reboot was started
boolean       g_lastDebug;
// RFM69
byte          g_hopCount;
uint32_t      g_lastRxTime;
uint32_t      g_lastTimeout;
uint16_t      g_packetsReceived;    // Number of packets with correct CRC
uint16_t      g_autoHops;           // How often did we HOP because of missing packets
uint16_t      g_numResyncs;         // How often did we have to resync
uint16_t      g_receivedStreak;     // Number of uninterruptedly receiverd correct packages
uint16_t      g_receivedStreakMax;  // Maximum Number of uninterruptedly receiverd correct packages
uint16_t      g_crcErrors;          // Number of packets with CRC ERROR
// ISS Weather Values
float         g_windSpeed;
uint16_t      g_windDirection;
boolean       g_transmitterBatteryStatus;
float         g_goldcapChargeStatus;
float         g_rainRate;
float         g_solarRadiation;
float         g_outsideTemperature;
float         g_gustSpeed;
float         g_outsideHumidity;
uint16_t      g_rainClicks;
uint16_t      g_rainClicksLast;
uint16_t      g_rainClicksDay;
unsigned long g_rainClicksSum;


/************************************************************
 * IRQ Handler
 * - called whenever GPIO goes low
 ************************************************************/ 
void IRAM_ATTR irqHandler(void){
  portENTER_CRITICAL(&mux);
  g_IrqFlag = true;
  portEXIT_CRITICAL(&mux);    
}

/************************************************************
 * Command "hello"
 * - Return: `world` 
 ************************************************************/ 
void cmd_hello(MyCommandParser::Argument *args, char *response) {  
  String msgStr;  
  msgStr = "world";  
  msgStr.toCharArray(response, MyCommandParser::MAX_RESPONSE_SIZE);
}

/************************************************************
 * Command "newday"
 * - Return: `world` 
 ************************************************************/ 
void cmd_newday(MyCommandParser::Argument *args, char *response) {  
  String msgStr;    
  g_rainClicksDay = 0;
  msgStr = "Daily Rain-Click counter set to 0";    
  msgStr.toCharArray(response, MyCommandParser::MAX_RESPONSE_SIZE);
}

/************************************************************
 * Command "setrc NEWVAL"
 * - Return: `[STRING]` 
 ************************************************************/ 
void cmd_setrc(MyCommandParser::Argument *args, char *response) {      
  String msgStr;  
  g_rainClicksSum = args[0].asUInt64;
  msgStr = "Raincounter set to ";  
  msgStr.concat(g_rainClicksSum);
  msgStr.toCharArray(response, MyCommandParser::MAX_RESPONSE_SIZE);  
}


/************************************************************
 * Command "reset"
 * - Reboot ESP32
 ************************************************************/ 
void cmd_reset(MyCommandParser::Argument *args, char *response) {
  String msgStr;  
  msgStr = "Rebooting in 5 seconds ... [please standby]: ";
  g_rebootActive = true;
  g_rebootTriggered = millis();
  msgStr.toCharArray(response, MyCommandParser::MAX_RESPONSE_SIZE);    
}


/************************************************************
 * Compose ClientID
 * - clientId = "esp32_"+ MAC 
 ************************************************************/ 
String composeClientID(void) {
  String myClientId;
  uint8_t myMac[6];
  WiFi.macAddress(myMac);  
  myClientId = ("esp32_");  
  for (int i=3; i<6; ++i) {
    myClientId.concat(String(myMac[i], 16));    
    if (i < 5)
      myClientId.concat('-');
  }  
  return myClientId;
}


/************************************************************
 * Debug Print String
 * - to Serial Console 
 * - MQTT-Message to TOPIC_LOG
 * @param[in] mes Message to be send
 ************************************************************/ 
void dbgout(String msg){  
  mqttPub (T_LOG, msg, false);
}


/************************************************************
 * Monitor Connections
 * - Check Wifi (and reconnect)
 * - Check MQTT (and reconnect)
 * TODO: reconnect on wifi error without rebooting!
 ************************************************************/ 
void monitorConnections(void) {  
  // Monitor WIFI- and MQTT-Connection   
  if (millis() - g_LastNetMonitoring > T_NET_MONITORING) {    
    // Monitor WIFI-Connection   
    g_LastNetMonitoring = millis();
    DBG_MONITOR.print("!!! WiFi localIP: ");
    DBG_MONITOR.println(WiFi.localIP());    
    if ((WiFi.status() != WL_CONNECTED) || (WiFi.localIP()[0] == 0)) {      
      DBG_ERROR.println("WiFi CONNECTION LOST");
      DBG_ERROR.println("reconnecting ...");
      WiFi.disconnect();
      WiFi.reconnect();
      if (WiFi.status() != WL_CONNECTED) {      
        DBG_ERROR.println("WiFi RECONNECTION FAILED, TRYING AGAIN LATER");  
        DBG_MONITOR.println("Not Monitoring MQTT because WiFi OFFLINE");     
        g_wificonnected = false;
      } else {        
        DBG_ERROR.println("WiFi CONNECTION RESTORED");
        g_wificonnected = true;
      }
    } else {        
        DBG_MONITOR.println("Monitoring WiFi... ONLINE");
        g_wificonnected = true;
    }
    // Monitor MQTT-Connection
    if (g_wificonnected){
      DBG_MONITOR.print("!!! MQTT: ");      
      if (!mqtt.connected()) {        
        if (millis() - g_LastMqttReconnectAttempt > T_MQTT_RECONNECT) {
          g_LastMqttReconnectAttempt = millis();
          g_MqttReconnectCount++;
          DBG_ERROR.print("MQTT Connection lost! - Error:");
          DBG_ERROR.println("mqtt.state()");
          DBG_ERROR.print(" - trying to reconnect [");
          DBG_ERROR.print(g_MqttReconnectCount);
          DBG_ERROR.println("]... ");      
          // Attempt to reconnect
          String myClientID;
          myClientID = composeClientID();
          if (mqtt.connect(myClientID.c_str(), MQTT_USER, MQTT_PASS, MQTT_PREFIX "/" T_STATUS, 1, true, STATUS_MSG_OFF, true))  { 
            // connected: publish Status ONLINE
            mqtt.publish(MQTT_PREFIX "/" T_STATUS, STATUS_MSG_ON, true);
            // resubscribe
            mqtt.subscribe(MQTT_PREFIX "/" T_CMD);          
            g_LastMqttReconnectAttempt = 0;
            g_MqttReconnectCount = 0;
            DBG_ERROR.println("MQTT SUCCESSFULLY RECONNECTED");
          } else {
            DBG_ERROR.println("MQTT RECONNECTION FAILED");
          } 
        } 
      } else {
        DBG_MONITOR.println("... ONLINE");             
      }
    }
  } 
}


/************************************************************
 * MQTT Message Received
 * - Callback function started when MQTT Message received
 * - convert Payload to lower case
 * - 
 * @param[in] topic Topic received
 * @param[in] topic Message received
 * @param[in] length Length of the Message received
 ************************************************************/ 
void mqttCallback(char* topic, byte* payload, unsigned int length) {  
  String msg;  
  char* myBuf = (char*)malloc(length + 1);    
  char response[MyCommandParser::MAX_RESPONSE_SIZE];
  // copy Buffer to String
  //   payload[length] = '\0';  // ensure that buffer is null-terminated
  //   msg = String((char*)payload);
  // Copy payload to new buffer and add '0x00' to the end
  for (int i=0; i<length; i++) {
    myBuf [i] = payload [i] ;
  }
  myBuf[length] = '\0';
  // Convert Buffer to String
  msg = String((char*)myBuf);    
  // convert String to Lower-Case
  // msg.toLowerCase();
  // Echo String
  dbgout("received MQTT-Message: \"" + msg + "\"");
  // Parse Command    
  msg.toCharArray(myBuf, msg.length() + 1);    
  parser.processCommand(myBuf, response);            
  // Publish Result;
  mqttPub (T_RESULT, String(response), false);
  // free(msgBuf);
  free(myBuf);
}


/************************************************************
 * Publish & Print Message
 * - to Serial Console
 *   - if mqttOnly is false
 * - Publish MQTT-Message to topic MQTT_PREFIX/TOPIC_LOG
 *   - MQTT_PREFIX is added by this function
 * @param[in] topic MQTT-SubTopic (MQTT_PREFIX will be added)
 * @param[in] msg Message to be send
 * @param[in] mqttOnly if false, then also Serial Output is generated
 ************************************************************/ 
void mqttPub(String subtopic, String msg, boolean mqttOnly){  
  String myTopic;
  // Serial
  if (!mqttOnly) {
    DBG.println(msg);    
  }  
  // MQTT Topic
  myTopic = MQTT_PREFIX;
  myTopic.concat("/" + subtopic);
  char* topicBuf = (char*)malloc(myTopic.length() + 1);  // allocate memory
  myTopic.toCharArray(topicBuf, myTopic.length() + 1);  
  // MQTT Message
  char* msgBuf = (char*)malloc(msg.length() + 1);  // allocate memory
  msg.toCharArray(msgBuf, msg.length() + 1);  
  if (mqtt.connected()) {
    mqtt.publish(topicBuf, msgBuf);
  } else {
    DBG_ERROR.println("ERROR: MQTT-Connection lost");
  }
  free(msgBuf);
  free(topicBuf);
}


/************************************************************
 * cronjob
 * - execute things periodicaly
 ************************************************************/ 
void cronjob(void) {
  // once on Startup
  if (g_Firstrun) {
      g_LastHeartbeat_1s = millis();          
      g_LastHeartbeat_10s = g_LastHeartbeat_1s;    
      g_LastHeartbeat_30s = g_LastHeartbeat_1s;    
      g_LastHeartbeat_60s = g_LastHeartbeat_1s;    
      oncePerSecond();        
      oncePerTenSeconds();
      oncePerThirtySeconds();
      oncePerMinute();          
  } else {
    // once a second  
    if ((millis() - g_LastHeartbeat_1s) > T_HEARTBEAT_1S) {
      g_LastHeartbeat_1s = millis();    
      oncePerSecond();        
      if ((millis() - g_LastHeartbeat_10s) > T_HEARTBEAT_10S) {
        g_LastHeartbeat_10s = g_LastHeartbeat_1s;    
        oncePerTenSeconds();
        if ((millis() - g_LastHeartbeat_30s) > T_HEARTBEAT_30S) {
          g_LastHeartbeat_30s = g_LastHeartbeat_1s;    
          oncePerThirtySeconds();
          if ((millis() - g_LastHeartbeat_60s) > T_HEARTBEAT_60S) {
            g_LastHeartbeat_60s = g_LastHeartbeat_1s;    
            oncePerMinute();
          }
        }
      }
    }
  }
}


/************************************************************
 * Once per Second
 * - execute things once every second
 ************************************************************/ 
void oncePerSecond(void) {
  // Insert here Actions, which should occure every Second
}


/************************************************************
 * Once per 10 Seconds
 * - execute things once every 10 seconds
 ************************************************************/ 
void oncePerTenSeconds(void) {
  // Insert here Actions, which should occure every 10 Seconds
  sendCPUState(true);    
  // DBG.print("radio._mode:   "); DBG.println(radio._mode);
  // DBG.print("radio.CHANNEL: "); DBG.println(radio.CHANNEL);
  // DBG.print("radio.RSSI:    "); DBG.println(radio.RSSI);
}

/************************************************************
 * Once per 30 Seconds
 * - execute things once every 30 seconds
 ************************************************************/ 
void oncePerThirtySeconds(void) {
  // Insert here Actions, which should occure every 10 Seconds
  sendNetworkState(true);
}

/************************************************************
 * Once per 60 Seconds
 * - execute things once every minute
 ************************************************************/ 
void oncePerMinute(void) {
  static uint8_t fidx = 4;
  byte t;
  // Insert here Actions, which should occure every 10 Seconds
  sendSketchState(true);  
}


/************************************************************
 * Process the received RFM Data Packet
 * - Parse Databytes and store to g_ Variables
 * - 
 ************************************************************/ 
void parseIssData() {
  uint16_t rawrr;
  float cph; 
  byte msgID;
  uint16_t rainDiff;
  
  // *********************
  // wind speed (all packets)          
  g_windSpeed = (float) radio.DATA[1] * 1.60934;  
  DBG_ISS.print("WindSpeed");
  DBG_ISS.println(g_windSpeed);  
  // *********************
  // wind direction (all packets)
  // There is a dead zone on the wind vane. No values are reported between 8
  // and 352 degrees inclusive. These values correspond to received byte
  // values of 1 and 255 respectively
  // See http://www.wxforum.net/index.php?topic=21967.50    
  // 0 = South    
  g_windDirection = (uint16_t)(radio.DATA[2] * 360.0f / 255.0f);
  // convert to 180° = South
  if (g_windDirection >= 180) {
      g_windDirection -= 180;
  } else {
      g_windDirection += 180;
  }  
  DBG_ISS.print("WindDirection: ");
  DBG_ISS.println(g_windDirection);      
  // *********************
  // battery status (all packets)    
  g_transmitterBatteryStatus = (boolean)(radio.DATA[0] & 0x8) == 0x8;
  DBG_ISS.print(F("Battery status: "));
  if (g_transmitterBatteryStatus) {
      DBG_ISS.print(F("ALARM "));
  } else {
      DBG_ISS.print(F("OK    "));
  }  
  // Now look at each individual packet. Mask off the four low order bits. 
  // The highest order bit of these four bits is set high when the ISS battery is low. 
  // The other three bits are the MessageID.  
  msgID = (radio.DATA[0] & 0xf0) >>4 ;
  switch (msgID) {
    case 0x2:  // goldcap charge status (MSG-ID 2) 
      g_goldcapChargeStatus = (float)((radio.DATA[3] << 2) + ((radio.DATA[4] & 0xC0) >> 6)) / 100;     
      DBG_ISS.print("Goldcap Charge Status: ");
      DBG_ISS.print(g_goldcapChargeStatus);
      DBG_ISS.println(" [V]");      
      break;
    case 0x3:  // MSG ID 3: unknown - not used
      DBG_ISS.println("Message-ID 3: unknown");
      break;
    case 0x5:  // rain rate (MSG-ID 5) as number of rain clicks per hour
               // ISS will transmit difference between last two clicks in seconds      
      DBG_ISS.print("Rain Rate ");
      if ( radio.DATA[3] == 255 ){
          // no rain
          g_rainRate = 0;
          DBG_ISS.print("(NO rain): ");
      } else {
        rawrr = radio.DATA[3] + ((radio.DATA[4] & 0x30) * 16);
        if ( (radio.DATA[4] & 0x40) == 0 ) {
          // HiGH rain rate 
          // Clicks per hour = 3600 / (VALUE/16)
          cph = 57600 / (float) (rawrr);
          DBG_ISS.print("(HIGH rain rate): ");
        } else {
          // LOW rain rate
          // Clicks per hour = 3600 / VALUE
          cph = 3600 / (float) (rawrr);
          DBG_ISS.print("(LOW rain rate): ");
        }
        // Rainrate [mm/h] = [Clicks/hour] * [Cupsize]
        g_rainRate = cph * 0.2;
      }        
      DBG_ISS.print(g_rainRate);              
      DBG_ISS.println(" [mm/h]");
      break;
    case 0x7:  // solarRadiation (MSG-ID 7)
      g_solarRadiation = (float)((radio.DATA[3] * 4) + ((radio.DATA[4] & 0xC0) >> 6));
      DBG_ISS.print("Solar Radiation: ");
      DBG_ISS.println(g_solarRadiation);      
      break;
    case 0x8:  // outside temperature (MSG-ID 8)
      g_outsideTemperature = (float) (((radio.DATA[3] * 256 + radio.DATA[4]) / 160) -32) * 5 / 9;  
      DBG_ISS.print("Outside Temp: ");
      DBG_ISS.print(g_outsideTemperature);
      DBG_ISS.println(" [C]");      
      break;
    case 0x9:  // gust speed (MSG-ID 9), maximum wind speed in last 10 minutes - not used
      g_gustSpeed = (float) radio.DATA[3] * 1.60934;
      DBG_ISS.print("Gust Speed: ");
      DBG_ISS.print(g_gustSpeed);
      DBG_ISS.println(" [km/h]");
      break;
    case 0xa:  // outside humidity (MSG-ID A)      
      g_outsideHumidity = (float)(word((radio.DATA[4] >> 4), radio.DATA[3])) / 10.0;   
      DBG_ISS.print("Outside Humdity: ");
      DBG_ISS.print(g_outsideHumidity);
      DBG_ISS.println(" [%relH]");
      break;
    case 0xe:  // rain counter (MSG-ID E)      
      g_rainClicks = (radio.DATA[3] & 0x7F);              
      rainDiff = 0;      
      // First run
      if (g_rainClicksLast == 255) {
        g_rainClicksLast = g_rainClicks;
      }      
      if (g_rainClicks > g_rainClicksLast) {               
        // Rainclicks higher than last time 
        rainDiff = g_rainClicks - g_rainClicksLast;        
      } else if (g_rainClicksLast > g_rainClicks) {        
        // Rainclicks lower than last time (overflow) 
        rainDiff = g_rainClicks + 128 - g_rainClicksLast;
      } 
      g_rainClicksLast = g_rainClicks;
      g_rainClicksDay += rainDiff;
      g_rainClicksSum += rainDiff;
      DBG_ISS.print("Rain Counter: ");
      DBG_ISS.print(g_rainClicks);
      DBG_ISS.println(" [clicks]");        
      DBG_ISS.print("Rain Counter Diff: ");
      DBG_ISS.print(rainDiff);
      DBG_ISS.println(" [clicks]");        
      DBG_ISS.print("Dayly Rain Clicks: ");
      DBG_ISS.print(g_rainClicksDay);
      DBG_ISS.println(" [clicks]");        
      DBG_ISS.print("Overall Rain Clicks: ");
      DBG_ISS.print(g_rainClicksSum);
      DBG_ISS.println(" [clicks]");              
      break;      
  }  
  DBG_ISS.println("*** Finished Parsing ISS Data *** ");
}


/************************************************************
 * Poll Radio
 * - Check for received Packet
 * - Hop
 *   - After correct Packet has been received
 *   - every 2.5s for 25 times after last correct Packet
 *   - every 20s if no correct Packet has been received for a long time 
 ************************************************************/ 
void pollRadio(void) {
  String msgStr;
  // *************************
  // * RF-Packet received
  // * - check CRC
  // * - process values if CRC OK
  uint16_t crc; // CRC16 XModem Value
  if (radio.receiveDone()) {         
    DBG_RFM.println("Packet received: ");    
    msgStr = "{";    
    // millis
    msgStr.concat("\"Millis\":" + String(millis()));
    // Channel
    DBG_RFM.print("Channel: ");
    DBG_RFM.println(radio.CHANNEL);
    msgStr.concat(", \"Channel\":" + String(radio.CHANNEL));
    // 8 Data-Byte
    msgStr.concat(", \"Data\":\"");
    DBG_RFM.print("Data: ");
    for (byte i = 0; i < DAVIS_PACKET_LEN; i++) {          
      if (radio.DATA[i] < 10) {
        msgStr.concat("0");
        DBG_RFM.print("0");
      }
      msgStr.concat(String(radio.DATA[i], 16));    
      DBG_RFM.print(radio.DATA[i], HEX);          
      if (i < DAVIS_PACKET_LEN-1) {
        msgStr.concat(":");
        DBG_RFM.print(":");
      }          
    }
    msgStr.concat("\"");        
    DBG_RFM.println(" ");
    // RSSI
    DBG_RFM.print("RSSI: ");
    DBG_RFM.println(radio.RSSI);
    msgStr.concat(", \"RSSI\":" + String(radio.RSSI) + ",");
    // Compute CRC
    crc = radio.crc16_ccitt(radio.DATA, 6);   // crc = crc & 0xffff;
    DBG_RFM.print("CRC: ");
    DBG_RFM.println(crc, HEX);                
    msgStr.concat(", \"CRC\":\"");
    msgStr.concat(String(crc, 16));
    msgStr.concat("\",");
    // verify CRC    
    if ((crc == (word(radio.DATA[6], radio.DATA[7]))) && (crc != 0)) {
      g_lastRxTime = millis();
      g_packetsReceived++;
      // Hop to next Channel if CRC was correct
      DBG_RFM.println("CRC OK");
      DBG_RFM.print("Hop! - New Channel: ");
      msgStr.concat(", \"CRC\":\"OK\"");
      radio.hop();    
      DBG_RFM.println(radio.CHANNEL);
      g_hopCount = 1;
      g_receivedStreak++;
      if (g_receivedStreak > g_receivedStreakMax) {
        g_receivedStreakMax = g_receivedStreak;
      }
      // Parse the RFM Data
      parseIssData();
      sendIssMqtt();
    } else {            
      // don`t try  again on same channel      
      radio._packetReceived = false;
      DBG_RFM.println("Wrong CRC");        
      msgStr.concat(", \"CRC\":\"ERROR\"");    
      g_crcErrors++;
      g_receivedStreak = 0;
    }
    // Finish MQTT Message
    msgStr.concat("}");
  }
  // *************************
  // Hop if packet was not received in expected time.
  // Auto-Hopping machanism:
  // - If a packet as been correctly received (g_hopCount == 1)
  //   - hop every 2,5s 
  //   - for maximum of 25 packets missing
  // Timing:
  // - Hop after (N * PACKET_INTERVAL) + PACKET_OFFSET  [N = number off Packets missed so far]
  //   - 1st Hop after 3,0 s
  //   - 2nd Hop after 5,5 s
  //   - 3rd Hop after 8,0 s  
  if ((g_hopCount > 0) && ((millis() - g_lastRxTime) > (unsigned long)(g_hopCount * PACKET_INTERVAL + PACKET_OFFSET))) {    
    g_receivedStreak = 0;
    // 1st missed Packet: Increment Resync-Counter
    if (g_hopCount == 1) {
      g_numResyncs++;
    }
    // after 25 missed Packets, no automatic HOP every 2,5s
    if (++g_hopCount > 25) {
      g_hopCount = 0;
    }
    g_autoHops++;
    radio.hop();
    DBG_RFM.print("HOP: ");
    DBG_RFM.print(g_hopCount - 1);
    DBG_RFM.println(" PACKET(S) MISSED");    
    // MQTT Message
    msgStr = "HOP: ";
    msgStr.concat(String(g_hopCount - 1));
    msgStr.concat(" Packets(s) missed, hopping anyway to Channel:");
    msgStr.concat(String(radio.CHANNEL));    
  }
  // *************************
  // Hop if NO packet was not received for a LONG time.
  // Auto-Hopping machanism:
  // Hop every PACKET_LONGINTERVAL 
  // - if no Packet was received at all
  // - OR if more than 25 Packets were missing
  if ( (g_hopCount == 0) && ( (millis() - g_lastTimeout) > PACKET_LONGHOP) ) {
    g_lastTimeout = millis();    
    radio.hop();    
    DBG_RFM.println(F("HOP: RESYNC (20s)"));
    msgStr = "HOP: RESYNC, new Channel:";
    msgStr.concat(String(radio.CHANNEL));
  }
  // Publish MQTT Message    
  #if DEBUG_RFM
  mqttPub(T_LOG, msgStr, true);  
  #endif
}


/************************************************************
 * Reset Handler
 * - Reboot ESP32 if 
 *   - g_rebootActive 
 *   - g_rebootTriggered was T_REBOOT_TIMEOUT ms ago (default 5000)
 ************************************************************/ 
void resetHandler() {  
  if (g_rebootActive) {
    if (millis() - g_rebootTriggered > T_REBOOT_TIMEOUT) {
      g_rebootActive = false;       
      delay(1000);      
      ESP.restart();    
    }
  }
}


/************************************************************
 * Send CPU State
 * this will send Status of CPU as JSON Message:
 ************************************************************
 * {"Heap Size":349264,"FreeHeap":260632,"Minimum Free Heap":253140,
 *  "Max Free Heap":113792,"Chip Model":"ESP32-D0WDQ5",
 *  "Chip Revision":1,"Millis":5220121,"Cycle Count":3019255534
 * }
 ************************************************************
 * @param[in] mqttOnly if false, then also Serial Output is generated
 ************************************************************/ 
void sendCPUState(boolean mqttOnly) {    
  String msgStr;
  msgStr = "{";    
  msgStr.concat("\"Heap Size\":" + String(ESP.getHeapSize()) + ",");
  msgStr.concat("\"FreeHeap\":" + String(ESP.getFreeHeap()) + ",");
  msgStr.concat("\"Minimum Free Heap\":" + String(ESP.getMinFreeHeap()) + ",");
  msgStr.concat("\"Max Free Heap\":" + String(ESP.getMaxAllocHeap()) + ",");
  msgStr.concat("\"Chip Model\":\"" + String(ESP.getChipModel()) + "\",");
  msgStr.concat("\"Chip Revision\":" + String(ESP.getChipRevision()) + ",");
  msgStr.concat("\"Millis\":" + String(millis()) + ",");
  msgStr.concat("\"Cycle Count\":" + String(ESP.getCycleCount()) + "");
  msgStr.concat("}");
  mqttPub(T_CPU, msgStr, mqttOnly);  
}


 
/* ###################################################
 * # Send Received Packet to MQTT over Software Serial
 * ###################################################
 * sends Data to ESP via Softwareserial
 * Format Template:
 * - {"WindSpeed": 31.415,
 *    "WindDirection" : 314,
 *    "BattWarning": 1,              
 *    "Payload" : 80;00;B2;30;A9;00;AA;DA;    
 *    "Channel": 4,
 *    "RSSI" : -58,
 *    "GoldcapVoltage" : 3.1415,     // when msgID = 0x2
 *    "Rainrate" : 31.415,           // when msgID = 0x5
 *    "SolarRadiation" : 3141,       // when msgID = 0x7
 *    "OutsideTemperature":31.4,     // when msgID = 0x8
 *    "GustSpeed" : 314.15,          // when msgID = 0x9
 *    "OutsideHumidity" : 31,        // when msgID = 0xA
 *    "RainClicks": 314,             // when msgID = 0xE
 *    "packetsReceived":169,
 *    "autoHops":152",
 *    "numResyncs":42",   
 *    "receivedStreak":23,     
 *    "receivedStreakMax":2423,
 *    "crcerrors":12" 
 **************************************************************************/
void sendIssMqtt(void) {    
    String msgStr;
    uint8_t msgID;       
    // {"WindSpeed": 31.415,
    msgStr = "{\"WindSpeed\": ";
    msgStr.concat(g_windSpeed);
    // "WindDirection" : 314,
    msgStr.concat(", \"WindDirection\": ");
    msgStr.concat(g_windDirection);
    // "BattWarning": 1,              
    msgStr.concat(", \"BattWarning\": ");
    if (g_transmitterBatteryStatus){
      msgStr.concat("1");
    } else {
      msgStr.concat("0");  
    }    
    // "Payload" : 80;00;B2;30;A9;00;AA;DA;    
    msgStr.concat(", \"Payload\": \"");
    for (byte i = 0; i < DAVIS_PACKET_LEN; i++) {
        if (radio.DATA[i] < 0x10) {
            msgStr.concat(F("0"));
        }
        msgStr.concat(String(radio.DATA[i], HEX));
        if (i < DAVIS_PACKET_LEN -1 ) {
          msgStr.concat(":");
        } else {
          msgStr.concat("\"");
        }        
    }
    // "Channel": 4,
    msgStr.concat(", \"Channel\":");
    msgStr.concat(radio.CHANNEL);            
    // "RSSI" : -58}
    msgStr.concat(", \"RSSI\":");
    msgStr.concat(radio.RSSI);   
    // get msgID
    msgID = (radio.DATA[0] & 0xf0) >>4 ;                
    // "msgID" : 1}
    msgStr.concat(", \"msgID\":");
    msgStr.concat(msgID);
    msgStr.concat(", ");    
    switch (msgID) {
        case 0x2: // "GoldcapVoltage" : 3.1415,     
            msgStr.concat("\"GoldcapVoltage\":");
            msgStr.concat(g_goldcapChargeStatus);
            break;
        case 0x3: // Unknown value, insert Pressure instead
            #if HAS_BMP085 
                myString.concat(F("\"Pressure\" : "));
                myString.concat(g_pressPa);             
                myString.concat(F(", \"PressureAtSealevel\" : "));
                myString.concat(g_pressPaSea);
                myString.concat(F(", \"InsideTemperature\" : "));
                myString.concat(g_insideTemperature);
            #else
                msgStr.concat("\"Unknown msgID\":3");
            #endif // HAS_BMP085 
            break;
        case 0x5: // "Rainrate" : 31.415,           
            msgStr.concat("\"Rainrate\":");
            msgStr.concat(g_rainRate);
            break;
        case 0x7: // "SolarRadiation" : 3141,       
            msgStr.concat("\"SolarRadiation\":");
            msgStr.concat(g_solarRadiation);
            break;
        case 0x8: // "OutsideTemperature":31.4,     
            msgStr.concat("\"OutsideTemperature\":");
            msgStr.concat(g_outsideTemperature);
            break;
        case 0x9: // "GustSpeed" : 314.15,          
            msgStr.concat("\"GustSpeed\":");
            msgStr.concat(g_gustSpeed);
            break;
        case 0xa: // "OutsideHumidity" : 31,        
            msgStr.concat("\"OutsideHumidity\":");
            msgStr.concat(g_outsideHumidity);
            break;
        case 0xe: // "RainClicks": 314,             
            msgStr.concat("\"RainClicks\":");
            msgStr.concat(g_rainClicks);
            msgStr.concat(", \"RainClicksDay\":");
            msgStr.concat(g_rainClicksDay);
            msgStr.concat(", \"RainClicksSum\":");                        
            msgStr.concat(g_rainClicksSum);
            break;
    }         
    // Statistics
    msgStr.concat(",");
    msgStr.concat("\"millis\":" + String(millis()) + ",");
    msgStr.concat("\"lastpacketreceived\":" + String(g_lastRxTime) + ",");
    msgStr.concat("\"packetsReceived\":" + String(g_packetsReceived) + ",");
    msgStr.concat("\"autoHops\":" + String(g_autoHops) + ",");     
    msgStr.concat("\"numResyncs\":" + String(g_numResyncs) + ",");     
    msgStr.concat("\"receivedStreak\":" + String(g_receivedStreak) + ",");
    msgStr.concat("\"receivedStreakMax\":" + String(g_receivedStreakMax) + ",");       
    msgStr.concat("\"crcerrors\":" + String(g_crcErrors));     
    msgStr.concat("}");    
    // Publish MQTT
    mqttPub(T_ISS, msgStr, true);      
}


/************************************************************
 * Send Network State
 * this will send State of Network  as JSON Message:
 ************************************************************
 * {"IP-Address":"192.168.1.42",
 *  "MQTT-ClientID":"esp32_00_00_00"  
 * }
 ************************************************************
 * @param[in] mqttOnly if false, then also Serial Output is generated
 ************************************************************/ 
void sendNetworkState(boolean mqttOnly) {    
  String msgStr;    
  // Publish MQTT
  msgStr = '{';  
  msgStr.concat("\"IP-Address\":\"" + WiFi.localIP().toString() + "\",");
  msgStr.concat("\"MQTT-ClientID\":\"" + composeClientID() + "\"");     
  msgStr.concat("}");    
  mqttPub(T_NETWORK, msgStr, mqttOnly);
}


/************************************************************
 * Send Sketch State
 * this will send Status of Sketch as JSON Message:
 ************************************************************
 * {"Compiletime":"1.11.2022, 20:9:4",
 *  "Sdk Version":"v3.3.5-1-g85c43024c","CpuFreq":240,
 *  "SketchSize":790608,"Free SketchSpace":1310720,
 *  "Sketch MD5":"fc84355a94fd722e55310621cf3645da",
 *  "Flash ChipSize":4194304,"Flash Chip Speed":40000000
 * }
 ************************************************************
 * @param[in] mqttOnly if false, then also Serial Output is generated
 ************************************************************/ 
void sendSketchState(boolean mqttOnly) {    
  String msgStr;    
  msgStr = '{';
  msgStr.concat("\"Project version\":\"" + String(VERSION) + "\",");
  msgStr.concat("\"Target\":\"" + String(TARGET) + "\",");
  msgStr.concat("\"Build timestamp\":\"" + String(BUILD_TIMESTAMP)+ "\",");  
  msgStr.concat("\"Sdk Version\":\"" + String(ESP.getSdkVersion()) + "\",");     
  msgStr.concat("\"CpuFreq\":" + String(ESP.getCpuFreqMHz()) + ",");
  msgStr.concat("\"SketchSize\":" + String(ESP.getSketchSize()) + ",");
  msgStr.concat("\"Free SketchSpace\":" + String(ESP.getFreeSketchSpace()) + ",");
  msgStr.concat("\"Sketch MD5\":\"" + String(ESP.getSketchMD5()) + "\",");
  msgStr.concat("\"Flash ChipSize\":" + String(ESP.getFlashChipSize()) + ",");
  msgStr.concat("\"Flash Chip Speed\":" + String(ESP.getFlashChipSpeed()));  
  msgStr.concat("}");  
  mqttPub(T_SKETCH, msgStr, true);  
}
                  

/************************************************************
 * Init Global Vars
 ************************************************************/ 
void setupGlobalVars(void){
  DBG_SETUP.print("- Global Vars ... ");    
  g_Firstrun = true;
  g_LastHeartbeat_1s = millis();       
  g_LastHeartbeat_10s = millis();       
  g_LastHeartbeat_30s = millis();       
  g_LastHeartbeat_60s = millis();       
  g_LastMqttReconnectAttempt = millis();     
  g_LastNetMonitoring = millis();          // Timer for Monitoring Network 
  g_LedState = 0;    
  g_MqttReconnectCount = 0;  
  g_wificonnected = false;
  g_IrqFlag = false;
  g_LastIRQ = true;  
  g_rebootActive = false;                  // if true trigger reeboot 5s after g_reboot_triggered
  g_rebootTriggered = millis();            // millis() when reboot was started  
  // RFM69
  g_hopCount = 0;
  g_lastRxTime = 0;    
  g_lastTimeout = 0;  
  g_packetsReceived = 0;
  g_autoHops = 0;
  g_numResyncs = 0;
  g_receivedStreak = 0;
  g_receivedStreakMax = 0;
  g_crcErrors = 0;  
  // ISS Weather Data
  g_windSpeed = -1;
  g_windDirection = 999;
  g_transmitterBatteryStatus = true;
  g_goldcapChargeStatus = -1;
  g_rainRate = -1;
  g_solarRadiation = -1;
  g_outsideTemperature = 999;
  g_gustSpeed = -1;
  g_outsideHumidity = -1;
  g_rainClicks = 0;
  g_rainClicksLast = 255;
  g_rainClicksDay = 0;
  g_rainClicksSum = 0;
  DBG_SETUP.println("done.");
  delay(DEBUG_SETUP_DELAY);  
}


/************************************************************
 * Init GPIO-Ports
 ************************************************************/ 
void setupGPIO(void) {  
  DBG_SETUP.print("- Init GPIO-Port... ");  
  pinMode(DBG_LED, OUTPUT);    
  pinMode(RFM_IRQ, INPUT);    
  DBG_SETUP.println("done.");
  delay(DEBUG_SETUP_DELAY);  
}


/************************************************************
 * MQTT Init
 * - Set LastWill to 
 *   - topic:   TOPIC_STATUS 
 *   - message: STATUS_MSG_OFF
 *   - retain:  yes
 * - subscribe to MQTT_PREFIX "/" T_CMD
 * - publish 
 *   - topic:   TOPIC_STATUS 
 *   - message: STATUS_MSG_ON
 *   - retain:  yes
 ************************************************************/ 
void setupMQTT(void) {    
  String myClientID;
  myClientID = composeClientID();
  DBG_SETUP.println("Connecting to MQTT-Server ... ");  
  DBG_SETUP.print("  - ClientID: ");
  DBG_SETUP.println(myClientID);  
  if (mqtt.connect(myClientID.c_str(), MQTT_USER, MQTT_PASS, MQTT_PREFIX "/" T_STATUS, 1, true, STATUS_MSG_OFF, true))  { 
    DBG_SETUP.println("  - Register Callback");
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(MQTT_BUFSIZE);
    DBG_SETUP.println("  - Publish State ONLINE");
    mqtt.publish(MQTT_PREFIX "/" T_STATUS, STATUS_MSG_ON, true);
    DBG_SETUP.print("  - Subscribe to ");
    DBG_SETUP.println(MQTT_PREFIX "/" T_CMD);
    mqtt.subscribe(MQTT_PREFIX "/" T_CMD);
    DBG_SETUP.println("  connected.");    
  } else {      
      DBG_SETUP.println("Connection failed - trying later...");  
  }   
  mqtt.loop();
}


/************************************************************
 * Init Over-The-Air Update Handler
 * - set OTA-Password with ArduinoOTA.setPasswordHash("[MD5(Pass)]");
 * MD5 ("mysecretOtaPassword") = 5386fe58bd9627e6a22aee5f1726c868 
 * ATTENTION: setPasswordHash MUST NOT CONTAIN CAPS Letters A-F
 *   NO:  5386FE58BD9627E6A22AEE5F1726C868
 *   YES: 5386fe58bd9627e6a22aee5f1726c868
 ************************************************************/ 
void setupOTA(void) {  
  DBG_SETUP.print("- Init OTA... ");
  // Set Port 3232
  ArduinoOTA.setPort(3232);
  
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("homectrl32");
  
  // authenticate using password      
  // ArduinoOTA.setPasswordHash("45159b2115f99bf96aa00fa2b9da0cb9");
  ArduinoOTA.setPasswordHash(g_otahash);

  // OTA Callback: onStart
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    // DBG_SETUP.println("Start updating " + type);
    dbgout("Update Started: " + type);
  });  

  // OTA Callback: onEnd
  ArduinoOTA.onEnd([]() {
    dbgout("Update finished");
  });  

  // OTA Callback: onProgress
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // Progress output disabled
    // DBG_SETUP.printf("Progress: %u%%\r", (progress / (total / 100));
  });  

  // OTA Callback: onError
  ArduinoOTA.onError([](ota_error_t error) {
    DBG_SETUP.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      DBG_SETUP.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      DBG_SETUP.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      DBG_SETUP.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      DBG_SETUP.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      DBG_SETUP.println("End Failed");
    }
  });  

  // OTA Init
  ArduinoOTA.begin();

  DBG_SETUP.println("done.");
  delay(DEBUG_SETUP_DELAY);  
}


/************************************************************
 * Setup Radio
 * - Init RFM69 to receive
 * - Set Channel 0
 ************************************************************/ 
void setupRadio(void) {
  DBG.print(F("init radio..."));
  radio.initialize();
  radio.setChannel(0);              // Select Channel 0 
  DBG.println(F("done"));
}


/************************************************************
 * Init Wifi 
 * - SSID: WIFI_SSID 
 * - PSK:  WIFI_PSK
 ************************************************************/ 
void setupWIFI(void) {      
  int cnt = 0;  
  DBG_SETUP.println("- Init WiFi... ");
  DBG_SETUP.print("  - connecting to '");    
  DBG_SETUP.print(g_wifissid);    
  DBG_SETUP.println("'");      
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_wifissid, g_wifipass);
  delay(5000);
  while ((WiFi.status() != WL_CONNECTED) && (cnt < T_WIFI_MAX_TRIES)){
    cnt++;
    DBG_SETUP.print("  - Connection failed! - Retrying [");      
    DBG_SETUP.print(cnt);      
    DBG_SETUP.println("]...");    
    delay(2000);
  }
  if (WiFi.status() != WL_CONNECTED) {
    g_wificonnected = false;
    DBG_SETUP.println("  - Connection failed! - trying later...");
  } else {
    g_wificonnected = true;
    DBG_SETUP.println("  - Successfully connected");    
    DBG_SETUP.print("  IP address: ");
    DBG_SETUP.println(WiFi.localIP());
  }  
  delay(DEBUG_SETUP_DELAY);
}


/************************************************************
 * Main Setup
 * - setupGPIO
 * - setupWIFI
 * - setupOTA
 * - setupGlobalVars 
 ************************************************************/ 
void setup(void) {  
  // Serial Port
  Serial.begin(115200);  
  DBG.println("");
  DBG.println("################################");
  DBG.println("### Darios ESP32 Hello-World ###");
  DBG.println("################################");  
  DBG.println("Version: " + String(VERSION));
  DBG.println("Target: " + String(TARGET));
  DBG.println("Build timestamp: " + String(BUILD_TIMESTAMP));

  DBG_SETUP.println("\nInit ...");
  delay(DEBUG_SETUP_DELAY);  

  // Global Vars
  setupGlobalVars();  

  // GPIO-Ports
  setupGPIO(); 

  // WiFi
  setupWIFI();

  // OTA-Update-Handler  
  setupOTA();    

  // MQTT
  setupMQTT();
   
  // MQTT Command Parser
  // "command", Params, Callback-Function 
  // s: String, d:Double, u:Unsigned Int , i:Signed Integer  
  parser.registerCommand("hello",  "",  &cmd_hello);                  // hello  - Ping  
  parser.registerCommand("newday", "",  &cmd_newday);                 // newday - Reset Daily Raincounter
  parser.registerCommand("setRC",  "s", &cmd_setrc);                  // setRC  - Set Raincounter
  parser.registerCommand("reset",  "",  &cmd_reset);                  // reset  - Reboot
  
  // RFM-Radio
  setupRadio();

  // Setup finished  
  dbgout("Init complete, starting Main-Loop");  
  DBG_SETUP.println("##########################################");
  delay(DEBUG_SETUP_DELAY);
}


/************************************************************
 * Main Loop
 * - OTA handler
 * - HeartBeat handler
 ************************************************************/ 
void loop(void) {
  // Main Handler
  resetHandler();                  // reset ESP if triggered (see: g_rebootActive and g_rebootTriggered)
  monitorConnections();            // Monitor (and restore) Wifi & MQTT Connection
  mqtt.loop();                     // handle MQTT Messaging  
  ArduinoOTA.handle();             // handle OTA  
  cronjob();                       // Cronjob-Handler  
  // APP Handler
  
  // First Loop completed
  g_Firstrun = false;              
  //   setupRadio(void);
  pollRadio();
}




 