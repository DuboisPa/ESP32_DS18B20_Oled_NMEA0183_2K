/**************************************

   target :    ESP32 Dev Module
               (HiLetgo ESP32S pinout) 
               display 4 lines 128 * 32 SSD1306
               button
               transducer BME280
               transducer DS18B20
               resistor 4.7 kOhm
               voltage regulator LM2596D
               connectors

               Partition Scheme : Minimal SPIFFS(1.9MB APP with OTA/190KB SPIFFS)
               don't forget to mount SPIFFS and/or SD
               
   Can be started in WIFI_MODE_STA        WiFi station mode
                  or WIFI_MODE_AP         WiFi soft-AP mode
                  or WIFI_MODE_APSTA      WiFi station + soft-AP mode
                  if WIFI_MODE_STA cannot be started because of false credentials
                     WIFI_MODE_AP is enabled
                     
   TODO        Nmea2000 Output
         
   version     2026-03-15
   
   look at https://github.com/FrancisSailor/NMEA2000-DISPLAY-on-ESP32S3
 
**************************************/

// Import required libraries
// if the library is global   use <>
// if the library is local    use ""

#include <algorithm>          
#include <deque>
#include <time.h>
#include <sys/time.h>

#include <esp_mac.h>
// #include "extern_var.h"

#include <Adafruit_BME280.h>  
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>      // https://arduinojson.org/
#include <AsyncTCP.h>
//#include <BluetoothSerial.h>
#include <DallasTemperature.h>
#include <ElegantOTA.h>       // warning if library is updated https://docs.elegantota.pro/getting-started/async-mode
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <OneWire.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <Wire.h>

// #define NMEA2K_CODE
#if defined NMEA2K_CODE
   // must be placed before #include <NMEA2000_esp32.h>
   #define ESP32_CAN_TX_PIN GPIO_NUM_16
   #define ESP32_CAN_RX_PIN GPIO_NUM_17
   #include <NMEA2000_CAN.h>
   #include <N2kMessages.h>
#endif

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// Hitlego Esp32S : SCL on GPIO22, SDA on GPIO21
#define SCL_GPIO     22          // BME280 SCK  pin 33
#define SDA_GPIO     21          // BME280 SDI  pin 36
#define SCL0_GPIO    22
#define SDA0_GPIO    21
#define SCL1_GPIO    33
#define SDA1_GPIO    32

// #define buttonPin 0     // (pin 25)
#define buttonPin    17          // (pin 28)
#define BuiltInLed    2          // GPIO led allumée par erreur dans la boucle Wifi not connected ?
#define tempo      1000          // 1000

#define SCREEN_WIDTH 128         // OLED display width, in pixels
#define SCREEN_HEIGHT 32         // OLED display height, in pixels
#define OLED_RESET    -1         // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C      // Look at datasheet for Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// HardwareSerial on passe les GPIO !
#define rx0GPIO  3               // pin 34 
#define tx0GPIO  1               // pin 35 
#define rx1GPIO 14               // pin 12 mapped, as native rx1GPIO  9 is used to flash
#define tx1GPIO 12               // pin 13 mapped, as native tx1GPIO 10 is used to flash
#define rx2GPIO 16               // pin 27 
#define tx2GPIO 17               // pin 28 

#define baudrateSerial  38400    // vitesse communication Arduino - PC

//#define SerialBT_SPPName "Nmea0183_2K_SPP"
//BluetoothSerial SerialBT;
HardwareSerial SerialPort1(1);

#define MAX_MEASURES                      336                           // 7 days : 2 per hour * 24 * 3 
//#define MAX_MEASURES                      672                           // 14 days : 2 per hour * 24 * 3 

struct oneRecord { 
  uint32_t seconds;
  float water_temperature;
  float air_temperature;
  float humidity;
  float pressure;
}; 

std::deque<oneRecord> weatherHistory;

#define wSettings_FILE "weatherSettings.json"    // SPIFFS files are case-sensitive
#define CONFIGRESET     0
#define CONFIGREAD      1
#define CONFIGWRITE     2
JsonDocument jConfig;
String jsonReplyString;

bool bDisplay = false;
volatile bool bButtonPressed = false;

uint32_t newMillis, oldMillis, oldWeatherMillis;

bool bSerial0Port, bSerial1Port, bBTSerial, bNMEA2K;
bool bWWeather, bAWeather, bWIXDR, bIIMDA;
uint32_t baudrateSerial0, baudrateSerial1;
uint32_t tempoWeather;   
float fWTemp, fATemp, fAHumi, fAP;

struct tm timeinfo, startTimeInfo;

String APSTAHostName;
uint8_t nbAPclients;
bool bAPSTAmode, bSTAmode, bAPmode, bUDP, bDHCP, bStatic;
IPAddress STAlocalIP, STAsubnetMaskIP, STAgatewayIP, STAbroadcastIP,
         APlocalIP, APsubnetMaskIP, APgatewayIP, APbroadcastIP, APdhcp_startIP,
         localIP, subnetMaskIP, gatewayIP;
uint16_t UDPPort;      
String Global_SSID_NAME, Global_PASSWORD;

//The udp library class
WiFiUDP STAudp, APudp;            // WiFiUDP est un typedef de NetworkUDP

// --- Configuration pour la sonde de température ---
#define ONE_WIRE_BUS 4                       // GPIO 4 pin 26                 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Initialize BME280 sensor
#define SEALEVELPRESSURE_HPA 1013.25f
Adafruit_BME280 bme;  // I2C

// Initialize HTTP server
#define Web_Server_Port       80
AsyncWebServer server(Web_Server_Port);

// function prototype declarations
void AddChecksum(char* msg);
void Display_Init(void);
void DisplayIMessage(String sMessage, bool bClear, bool bCRLN);
void AirData(); 
void MDAWeatherData();
void WaterData(); 
void WiFi_Init(void);
void WSettingsRW(JsonDocument& jnewConfig, byte configValue);

#if defined NMEA2K_CODE
   void initNMEA2K();
   void SendN2kTemperature();
   //void NMEA2000.ParseMessages();
   // Liste des messages PGN que cet appareil va transmettre sur le bus NMEA2000
   const unsigned long TransmitMessages[] PROGMEM={130310L, 130312L, 130313L, 130314L, 130315L, 130316L, 0}; // 0 = fin de liste

   tN2kSyncScheduler TemperatureScheduler(false,2000,500);
   //tN2kSyncScheduler EnvironmentalScheduler(false,500,510);
   tN2kSyncScheduler OutsideEnvironmentalScheduler(false,500,520);

#endif   
// end declarations

//#define DEBUG

void IRAM_ATTR onButtonEvent() {
   bButtonPressed = true;
}

void convertFromJson(JsonVariantConst source, IPAddress& dest) {
  dest.fromString(source.as<const char*>());
}

void setup() {
   btStop();
   Serial.begin(baudrateSerial);   // Serial0 & USB, toujours démarré, arrété ou modifié selon config

   pinMode(buttonPin, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(buttonPin), onButtonEvent, RISING);
   pinMode(BuiltInLed, OUTPUT);     
   digitalWrite(BuiltInLed, LOW);         // ON after Wifi connection so set it OFF
     
   Display_Init();
   Storage_Init();
   WSettingsRW(jConfig, CONFIGREAD);
   WiFi_Init();
    
   if (bWWeather) { 
      // Init DS18B20 water temp
      sensors.begin();                       // OneWire
   }
   if (bAWeather) {
      // Init BME280 weather data
      // Wire.begin(SDA_GPIO, SCL_GPIO);  
      if (! bme.begin(BME280_ADDRESS, &Wire)) {          // 0x77 BlueDot / WaveShare  BME_ADRESS_ are defined in Adafruit_BME280.h
         bme.begin(BME280_ADDRESS_ALTERNATE, &Wire);     // 0x76 Gybmep BME-BMP680
      }
   }

   if (bSerial0Port) { Serial.begin(baudrateSerial0, SERIAL_8N1, rx0GPIO, tx0GPIO); } 
   if (bSerial1Port) { SerialPort1.begin(baudrateSerial1, SERIAL_8N1, rx1GPIO, tx1GPIO); } 
   //if (bBTSerial) { SerialBT.begin(SerialBT_SPPName); }   

#if defined NMEA2K_CODE   
   if (bNMEA2K) { initNMEA2K();}
#endif

   // home page with weather values 
   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/index.html", "text/html");
   });
   // read weather values and send them to index.html
   server.on("/loadValues", HTTP_GET, [](AsyncWebServerRequest *request) {
      String jsonString;
      JsonDocument jConfig;
      jConfig["water_temperature"] = String(fWTemp);
      jConfig["air_temperature"] = String(fATemp); 
      jConfig["humidity"] = String(fAHumi);
      jConfig["pressure"] = String(fAP);
      serializeJson(jConfig, jsonString);  // Convertit JSON en string
      request->send(200, "application/json", jsonString);  // Envoi JSON
   });

   // configuration page
   server.on("/config.html", HTTP_GET, [](AsyncWebServerRequest *request){ 
      request->send(SPIFFS, "/config.html", "text/html");
   });
   server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (request->url() == "/saveConfig") {  
         JsonDocument jnewConfig;
         DeserializationError error = deserializeJson(jnewConfig, data, len);
         WSettingsRW(jnewConfig, CONFIGWRITE);
         //request->send(200, "text/plain", "Config reçue");
      }
   });   
   server.on("/loadConfig", HTTP_GET, [](AsyncWebServerRequest *request) {
      String jsonString;
      serializeJson(jConfig, jsonString);  // Convertit JSON en string
      //Serial.println(jsonString);
      request->send(200, "application/json", jsonString);  // Envoi JSON
   }); 
    
   ElegantOTA.begin(&server);    // Start ElegantOTA  warning if library is updated https://docs.elegantota.pro/getting-started/async-mode
   // Start HTTP server
   server.begin();
   
   struct timeval now = { .tv_sec = 0, .tv_usec = 0 };                  // 1970-01-01 00:00:00
   settimeofday(&now, NULL);
   getLocalTime(&startTimeInfo);
   
   oldMillis = oldWeatherMillis = millis();
}

void loop() {
   if (bButtonPressed) { WSettingsRW(jConfig, CONFIGRESET);}
   
   uint32_t newMillis = millis();
   if (newMillis > oldWeatherMillis + tempoWeather) {
      oldWeatherMillis = newMillis;
      if (bWIXDR && bWWeather) { WaterData();}
      if (bWIXDR && bAWeather) { AirData();}
      if (bIIMDA && (bWWeather || bAWeather)){ MDAWeatherData();}

      #if defined NMEA2K_CODE   
      SendN2kTemperature();
      NMEA2000.ParseMessages();
      #endif
   }
   ElegantOTA.loop();
}

void AddChecksum(char* msg) {
  unsigned int i=1;        // First character not included in checksum
  uint8_t tmp, chkSum = 0;
  char ascChkSum[5];       // 5 instead of 4

  while (msg[i] != '\0') {
    chkSum ^= msg[i++];
  }
  ascChkSum[0] = '*';
  ascChkSum[3] = '\r';     // added, else problem in qtVlm
  ascChkSum[4] = '\0';
  tmp = chkSum / 16;
  ascChkSum[1] = tmp > 9 ? 'A' + tmp-10 : '0' + tmp;
  tmp = chkSum % 16;
  ascChkSum[2] = tmp > 9 ? 'A' + tmp-10 : '0' + tmp;
  strcat(msg, ascChkSum);
}

/**************************************
   temperature pressure humidity 
       
   $II                                       // Integrated Instrumentation
   $WI                                       // Weather Instrument
   $..MTW                                    // Mean Temperature of Water
   $..XDR                                    // Transducer Values
   $..MDA                                    // Meteorological Composite, obsolete as of 2009
                                             // https://gpsd.gitlab.io/gpsd/NMEA.html#_mda_meteorological_composite

   $WIMTW,19.52,C*checksum                   // Mean Temperature of Water
   $WIXDR,C,19.52,C,TempAir*checksum         // air temperature
   $WIXDR,C,19.52,C,ENV_OUTAIR_T*checksum    // air temperature
   $WIXDR,C,19.52,C,ENV_WATER_T*checksum     // water temperature
   $WIXDR,P,1.02481,B,Barometer*checksum     // pressure
   $WIXDR,H,66.66,P,ENV_OUTAIR_T*checksum    // humididity

   $WIXDR,C,%.2f,C,ENV_OUTAIR_T,H,%.2f,P,ENV_OUTAIR_T,P,%.4f,B,Barometer*checksum  // 3 values
   
**************************************/

void WaterData() {
   char nmea0183string[48];

   bool bOK = sensors.requestTemperatures();
   fWTemp = sensors.getTempCByIndex(0);
   snprintf(nmea0183string, sizeof(nmea0183string), "$WIXDR,C,%.2f,C,ENV_WATER_T", fWTemp);
   AddChecksum(nmea0183string);
   if (bSerial0Port) { Serial.println(nmea0183string); }
   if (bSerial1Port) { Serial1.println(nmea0183string); }
   //if (bSerial2Port) { Serial2.println(nmea0183string); }
   if (bUDP && bSTAmode) {
      STAudp.beginPacket(STAbroadcastIP, UDPPort);  // broadcast_IP
      STAudp.print(nmea0183string);
      STAudp.endPacket();      
   }
   if (bUDP && bAPmode) {
      APudp.beginPacket(APbroadcastIP, UDPPort);  // broadcast_IP
      APudp.print(nmea0183string);
      APudp.endPacket();      
   }
   snprintf(nmea0183string, sizeof(nmea0183string), "Water Temper.%7.2f%c", fWTemp, byte(223));
   DisplayIMessage(nmea0183string, true, true);
}

void AirData() {
   char nmea0183string[128]; 
    
   fATemp = bme.readTemperature();
   fAHumi = bme.readHumidity();
   fAP    = bme.readPressure() / 100.0F;
   float fAPressure = fAP / 1000.0F;
   snprintf(nmea0183string, sizeof(nmea0183string), "$WIXDR,C,%.2f,C,ENV_OUTAIR_T,H,%.2f,P,ENV_OUTAIR_T,P,%.4f,B,Barometer", fATemp, fAHumi, fAPressure);
   AddChecksum(nmea0183string);
   if (bSerial0Port) { Serial.println(nmea0183string); }
   if (bSerial1Port) { Serial1.println(nmea0183string); }
   //if (bSerial2Port) { Serial2.println(nmea0183string); }
   if (bUDP && bSTAmode) {
      STAudp.beginPacket(STAbroadcastIP, UDPPort);  // broadcast_IP
      STAudp.print(nmea0183string);
      STAudp.endPacket();      
   }
   if (bUDP && bAPmode) {
      APudp.beginPacket(APbroadcastIP, UDPPort);  // broadcast_IP
      APudp.print(nmea0183string);
      APudp.endPacket();      
   }
   snprintf(nmea0183string, sizeof(nmea0183string), "Air Temper.    %5.2f%c", fATemp, byte(223));
   DisplayIMessage(nmea0183string, !bWWeather, true);                    // if no DS18B20B for Water Temp., clear Display
   snprintf(nmea0183string, sizeof(nmea0183string), "Humidity       %5.2f%c", fAHumi, byte(37));
   DisplayIMessage(nmea0183string, false, true);
   snprintf(nmea0183string, sizeof(nmea0183string), "Pressure   %7.2fhPa", fAP);
   DisplayIMessage(nmea0183string, false, true);
}

void MDAWeatherData() {
   char nmea0183string[128];
   float fWTemp = 0.0, fATemp = 0.0, fAHumi = 0.0, fAP = 0.0, fAPressure = 1013.25;

   if (bWWeather && !bAWeather) {           // Water temperature
      sensors.requestTemperatures();
      fWTemp = sensors.getTempCByIndex(0);
      snprintf(nmea0183string, sizeof(nmea0183string), "$IIMDA,,I,,B,,C,%.1f,C,,,,C,,T,,M,,N,,M", fWTemp);
      //snprintf(nmea0183string, sizeof(nmea0183string), "$IIMTW,%.1f,C", fWTemp);
   }
   else if (!bWWeather && bAWeather) {      // Air: temperature, humidity, pressure
      fATemp = bme.readTemperature();
      fAHumi = bme.readHumidity();
      fAP    = bme.readPressure() / 100.0F;
      fAPressure = fAP / 1000.0F;
      snprintf(nmea0183string, sizeof(nmea0183string), "$IIMDA,,I,%.3f,B,%.1f,C,,C,%.1f,,,C,,T,,M,,N,,M", fAPressure, fATemp, fAHumi); 
   }
   else if (bWWeather && bAWeather) {       // Water temperature and Air: temperature, humidity, pressure
      sensors.requestTemperatures();
      fWTemp = sensors.getTempCByIndex(0);
      fATemp = bme.readTemperature();
      fAHumi = bme.readHumidity();
      fAP    = bme.readPressure() / 100.0F;
      fAPressure = fAP / 1000.0F;
      snprintf(nmea0183string, sizeof(nmea0183string), "$IIMDA,,I,%.3f,B,%.1f,C,%.1f,C,%.1f,,,C,,T,,M,,N,,M", fAPressure, fATemp, fWTemp, fAHumi); 
   }
   AddChecksum(nmea0183string);
   if (bSerial0Port) { Serial.println(nmea0183string); }
   if (bSerial1Port) { Serial1.println(nmea0183string); }
   //if (bSerial2Port) { Serial2.println(nmea0183string); }
   if (bUDP && bSTAmode) {
      STAudp.beginPacket(STAbroadcastIP, UDPPort);  // broadcast_IP
      STAudp.print(nmea0183string);
      STAudp.endPacket();      
   }
   if (bUDP && bAPmode) {
      APudp.beginPacket(APbroadcastIP, UDPPort);  // broadcast_IP
      APudp.print(nmea0183string);
      APudp.endPacket();      
   }
   if (bWWeather) {
      snprintf(nmea0183string, sizeof(nmea0183string), "Water Temper.  %.2f%c", fWTemp, byte(223));
      DisplayIMessage(nmea0183string, true, true);
   }
   if (bAWeather) {
      snprintf(nmea0183string, sizeof(nmea0183string), "Air Temper.    %5.2f%c", fATemp, byte(223));
      DisplayIMessage(nmea0183string, !bWWeather, true);                    // if no DS18B20B for Water Temp., clear Display
      snprintf(nmea0183string, sizeof(nmea0183string), "Humidity       %5.2f%c", fAHumi, byte(37));
      DisplayIMessage(nmea0183string, false, true);
      snprintf(nmea0183string, sizeof(nmea0183string), "Pressure   %7.2fhPa", fAP);
      DisplayIMessage(nmea0183string, false, true);
   }
}

void WSettingsRW(JsonDocument& jnewConfig, byte configValue) {      
   
   if (!SPIFFS.exists("/" wSettings_FILE)) { 
      // Serial.println(String(wSettings_FILE) + " doesn't exist");
      configValue = CONFIGRESET;
   }
   
   switch (configValue) {
      case CONFIGRESET : {             // 0
         File file = SPIFFS.open("/" wSettings_FILE, FILE_WRITE);
         jnewConfig["APSTAHostName"] = "ESP32_NMEA0183_2K";
         jnewConfig["APmode"] = true; 
         jnewConfig["APClients"] = 4; 
         jnewConfig["APPassword"] = "12345678"; 
         jnewConfig["APlocalIP"] = "192.168.4.1"; 
         jnewConfig["APsubnetMaskIP"] = "255.255.255.0"; 
         jnewConfig["APgatewayIP"] = "192.168.4.99"; 
         jnewConfig["APdhcp_startIP"] = "192.168.4.11";
         jnewConfig["STAmode"] = false; 
         jnewConfig["DHCP"] = true; 
         jnewConfig["Static"] = false; 
         jnewConfig["STASSID"] = "FreeOrangeSFRBouygues"; 
         jnewConfig["STAPassword"] = "12345678";
         jnewConfig["STAlocalIP"] = "192.168.1.64"; 
         jnewConfig["STAsubnetMaskIP"] = "255.255.255.0"; 
         jnewConfig["STAgatewayIP"] = "192.168.1.254"; 
         jnewConfig["WaterSensor"] = true; 
         jnewConfig["AirSensor"] = true; 
         jnewConfig["WIXDR"] = true; 
         jnewConfig["IIMDA"] = false;
         jnewConfig["DelayTime"] = 60;
         jnewConfig["UDPPort"] = 2010; 
         jnewConfig["USBPort"] = 38400; 
         jnewConfig["Serial1"] = 0; 
         jnewConfig["BTSerial"] = 0;
         jnewConfig["NMEA2K"] = false;

         // jnewConfig.shrinkToFit();  // optional
         serializeJsonPretty(jnewConfig, file);       // indent 2 spaces
         file.close();
         ESP.restart();
         break;
      }
      case CONFIGREAD : {              // 1
         File file = SPIFFS.open("/" wSettings_FILE, FILE_READ);
         DeserializationError error = deserializeJson(jnewConfig, file);
         /* if (error) {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
            //return;
         } 
         else { 
            Serial.println(jnewConfig.as<String>());
         } */
         APSTAHostName = jConfig["APSTAHostName"].as<String>(); // "ESP32_NMEA0183_2K"
         bAPmode = jConfig["APmode"];
         nbAPclients = jConfig["APClients"];
         bSTAmode = jConfig["STAmode"]; 
         bAPSTAmode = bAPmode && bSTAmode;
         bDHCP = jConfig["DHCP"]; 
         bStatic = jConfig["Static"]; 
      
         convertFromJson(jConfig["APlocalIP"], APlocalIP);
         convertFromJson(jConfig["APsubnetMaskIP"], APsubnetMaskIP);
         convertFromJson(jConfig["APgatewayIP"], APgatewayIP);
         convertFromJson(jConfig["APdhcp_startIP"], APdhcp_startIP);
         convertFromJson(jConfig["STAlocalIP"], STAlocalIP);
         convertFromJson(jConfig["STAsubnetMaskIP"], STAsubnetMaskIP);
         convertFromJson(jConfig["STAgatewayIP"], STAgatewayIP);

         bWWeather = jConfig["WaterSensor"]; 
         bAWeather = jConfig["AirSensor"]; 
         bWIXDR = jConfig["WIXDR"]; 
         bIIMDA = jConfig["IIMDA"]; 
         tempoWeather = jConfig["DelayTime"];
         tempoWeather *= 1000; // milliseconds 

         UDPPort = jConfig["UDPPort"];
         bUDP = (UDPPort > 0);
         baudrateSerial0 = jConfig["USBPort"];
         baudrateSerial1 = jConfig["Serial1"];
         //baudrateSerial2 = jConfig["Serial2"];
         bSerial0Port = (baudrateSerial0 > 0);
         bSerial1Port = (baudrateSerial1 > 0);
         bBTSerial = jConfig["BTSerial"];
         bNMEA2K = jConfig["NMEA2K"];  

         #if defined DEBUG
            Serial.println("\nUDPPort " + String(UDPPort));     
            Serial.println("bUPD " + String(bUDP));
            Serial.println("USB baudrateSerial0 " + String(baudrateSerial0));     
            Serial.println("USB bSerial0Port " + String(bSerial0Port));
            Serial.println("bWWeather " + String(bWWeather));     
            Serial.println("bAWeather " + String(bAWeather));     
            Serial.println("bWIXDR " + String(bWIXDR));  
            Serial.println("bIIMDA " + String(bIIMDA));  
            Serial.println("tempoWeather " + String(tempoWeather));     
         #endif
         
         file.close();
         break;
      }
      case CONFIGWRITE : {             // 2
         File file = SPIFFS.open("/" wSettings_FILE, FILE_WRITE);
         // Serial.println(jnewConfig.as<String>());
         file.print(jnewConfig.as<String>()); 
         file.close();
         ESP.restart();
         break;
      }
   }
}

void WiFi_Init() {
   String strUDP;
   
   if (bAPSTAmode) { WiFi.mode(WIFI_AP_STA);}
   else if (bSTAmode && !bAPmode) { WiFi.mode(WIFI_STA);}
   else if (bAPmode && !bSTAmode ) { WiFi.mode(WIFI_AP);}
   WiFi.setHostname(APSTAHostName.c_str());
   
   if (bSTAmode) {
      uint32_t oldMillis = millis();
      if (bStatic) {
         WiFi.config(STAlocalIP, STAgatewayIP, STAsubnetMaskIP); // primaryDNS, secondaryDNS))
      }
      Global_SSID_NAME = jConfig["STASSID"].as<String>(); 
      Global_PASSWORD = jConfig["STAPassword"].as<String>();
      WiFi.begin(Global_SSID_NAME, Global_PASSWORD);
      DisplayIMessage("Connecting " + String(Global_SSID_NAME), true, true);
      do {
         delay(500);
         Serial.print(".");
      }
      while ((WiFi.status() != WL_CONNECTED) && (millis() < oldMillis + 15000));     // 15 seconds
      jConfig["STAlocalIP"] = STAlocalIP = WiFi.localIP();
      jConfig["STAsubnetMaskIP"] = STAsubnetMaskIP = WiFi.subnetMask();
      jConfig["STAgatewayIP"] = STAgatewayIP = WiFi.gatewayIP();
      STAbroadcastIP = WiFi.broadcastIP();
      if (WiFi.status() != WL_CONNECTED) {
         DisplayIMessage("No Station connection", true, true);
         bAPmode = true;
         bSTAmode = false;
         WiFi.mode(WIFI_AP);
      }
      else {
         if (bUDP) {
            STAudp.begin(UDPPort);
            strUDP = String(UDPPort);
         }
         else strUDP = "none";
         DisplayIMessage("IP    " + STAlocalIP.toString(), true, true);
         DisplayIMessage("Mask  " + STAsubnetMaskIP.toString(), false, true);
         //DisplayIMessage("GW    " + STAgatewayIP.toString(), false, true);
         DisplayIMessage("Broad." + STAbroadcastIP.toString(), false, true);
         DisplayIMessage("Port  " + strUDP, false, true);
      }
      delay(2000);
   }

   if (bAPmode) {
      // bool softAPConfig(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dhcp_lease_start = (uint32_t) 0);
      // bool softAP(const char* ssid, const char* passphrase = NULL, int channel = 1, int ssid_hidden = 0, int max_connection = 4, bool ftm_responder = false);
      Global_SSID_NAME = APSTAHostName.c_str();
      //Global_PASSWORD = jCredentials["AP"][0][1].as<String>();  
      Global_PASSWORD = jConfig["APPassword"].as<String>();
      DisplayIMessage("Starting Autonomous", true, true);
      delay(1000);
      WiFi.softAPConfig(APlocalIP, APgatewayIP, APsubnetMaskIP, APdhcp_startIP);
      WiFi.softAP(Global_SSID_NAME, Global_PASSWORD, 1, 0, nbAPclients, false); 
      APlocalIP = WiFi.softAPIP();
      APsubnetMaskIP = WiFi.softAPSubnetMask();
      APbroadcastIP = WiFi.softAPBroadcastIP();
      if (bUDP) {
         APudp.begin(UDPPort);
         strUDP = String(UDPPort);
      }
      else strUDP = "none";
      DisplayIMessage("IP    " + APlocalIP.toString(), true, true);
      DisplayIMessage("Mask  " + APsubnetMaskIP.toString(), false, true);
      //DisplayIMessage("GW    " + APgatewayIP.toString(), false, true);
      DisplayIMessage("Broad." + APbroadcastIP.toString(), false, true);
      DisplayIMessage("Port  " + strUDP, false, true);
      delay(2000);
   }
   MDNS.begin(APSTAHostName.c_str());
}

void Storage_Init() {
   if(!SPIFFS.begin(true))
      DisplayIMessage("Storage Mount Failed", true, true);
   else {
      DisplayIMessage("Storage Mount OK", true, true);
   }
   delay(1500);
}

void Display_Init() {
   // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
   if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { // Address 0x3C for 128x32
      DisplayIMessage("SSD1306 allocation failed", false, true);
      // for (;;); // Don't proceed, loop forever
   }   
   else {   
      bDisplay = true;
      display.clearDisplay();
      display.setTextSize(1);         // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);        // Draw white text
      display.cp437(true);            // Use full 256 char 'Code Page 437' font 
      display.setCursor(0, 0); 
   }
}

void DisplayIMessage(String sMessage, bool bClear, bool bCRLN) {
   if (bDisplay) {
      if (bClear) {
         display.clearDisplay();
         display.setCursor(0, 0); 
      }
      if (bCRLN) 
         display.println(sMessage);
      else 
         display.print(sMessage); 
      display.display();
   }
   else {
      Serial.println(sMessage);
   }
}

#if defined NMEA2K_CODE

void initNMEA2K() {
   uint32_t NMEA2K_DeviceNumber = 0;
   uint8_t chipid[6];
   // Generate unique number from chip id
   esp_efuse_mac_get_default(chipid);
   for (uint8_t i = 0; i < 6; i++) NMEA2K_DeviceNumber += (chipid[i] << (7 * i));
   
   // NMEA2000 initialisation section
   NMEA2000.SetN2kCANMsgBufSize(8);
   NMEA2000.SetN2kCANReceiveFrameBufSize(150);
   NMEA2000.SetN2kCANSendFrameBufSize(150);

   // Set product information
   NMEA2000.SetProductInformation("P-0103-D", // Manufacturer's Model serial code
                                 103, // Manufacturer's product code (101 GNSS, 102 AIS, 103 Sensors)
                                 "Sensor Communication Interface",   // Manufacturer's Model ID
                                 "SW-Vers:  1.0 (2025-03-17)",       // Manufacturer's Software version code
                                 "Mod-Vers: 1.0 (2025-03-17)"        // Manufacturer's Model version
                                 );
   // Set device information
   NMEA2000.SetDeviceInformation(NMEA2K_DeviceNumber, // Unique number. Use e.g. Serial number.
                              130, // Devices that measure/report temperature
                              75,  // Sensor Communication Interface
                              2025 // Just choosen free from code list on 
                              );
   /*
   NMEA2000.SetDeviceInformation(NMEA2K_DeviceNumber+1, // Unique number. Use e.g. Serial number.
                              130, // Devices that measure/report temperature
                              75,  // Sensor Communication Interface
                              2025 // Just choosen free from code list on 
                              );
   NMEA2000.SetDeviceInformation(NMEA2K_DeviceNumber+2, // Unique number. Use e.g. Serial number.
                              140, // Devices that measure/report pressure
                              75,  // Sensor Communication Interface
                              2025 // Just choosen free from code list on 
                              );
   NMEA2000.SetDeviceInformation(NMEA2K_DeviceNumber+3, // Unique number. Use e.g. Serial number.
                              170, // Devices that measure/report humidity
                              75,  // Sensor Communication Interface
                              2025 // Just choosen free from code list on 
                              );                                 
   */                       
   // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
   //NodeAddress = 0;           // to check
  
   NMEA2000.SetForwardStream(&Serial);
   NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);     // Show in clear text. Leave uncommented for default Actisense format.
   NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 25);
   NMEA2000.ExtendTransmitMessages(TransmitMessages);                              
   
   TemperatureScheduler.UpdateNextTime();
   //EnvironmentalScheduler.UpdateNextTime();
   OutsideEnvironmentalScheduler.UpdateNextTime();
   
   NMEA2000.Open();

}

// *****************************************************************************
void SendN2kTemperature() {
   tN2kMsg N2kMsg;

   if ( TemperatureScheduler.IsTime() ) {
      TemperatureScheduler.UpdateNextTime();
      SetN2kTemperature(N2kMsg, 1, 1, N2kts_MainCabinTemperature, fATemp);
      NMEA2000.SendMsg(N2kMsg);
   }
  
   /*if ( EnvironmentalScheduler.IsTime() ) {
      EnvironmentalScheduler.UpdateNextTime();
      SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_MainCabinTemperature, ReadCabinTemp());
      NMEA2000.SendMsg(N2kMsg);
   }*/
  
   if ( OutsideEnvironmentalScheduler.IsTime() ) {
      OutsideEnvironmentalScheduler.UpdateNextTime();
      SetN2kOutsideEnvironmentalParameters(N2kMsg, 1, fWTemp);
      NMEA2000.SendMsg(N2kMsg);
   }
   Serial.print(millis()); Serial.println(", Temperature send ready");
}

#endif