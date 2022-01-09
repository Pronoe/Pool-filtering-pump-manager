/**********************************************************************
   Definitions for pump management software
  /**********************************************************************/
//
// definitions for network access
#define AUTHOR "Pronoe"
#define WIFI_DEVICE_NAME "PumpCtrl"
#define GATEWAY_IP "192.168.0.254"
#define DNS_IP "8.8.8.8"
#define MICROCONTROLLER_IP "192.168.0.118"
#define MQTT_SERVER_IP "192.168.0.117"
#define MQTT_SERVER_PORT "1883"
#define WIFI_SSID "XXX"

// Import required libraries
#include <parameterPrint.h>
#include <WiFi.h>
#include "esp_wps.h"
// libraries required for OTA uploading
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// library required for temp. sensor
#include <OneWire.h>
#include <DallasTemperature.h>
// library for SHA encoding
#include <mbedtls/md.h> 
#include "MyConfiguration.h"
#include "Secrets.h"
/****************** BOOTSTRAP MANAGER ******************/
// MQTT, FS and OTA management
#include <BootstrapManager.h>
#include <PingESP.h>
BootstrapManager bootstrapManager;
Helpers helper;
PingESP pingESP;

#ifdef __cplusplus
extern "C"
{
#endif

uint8_t temprature_sens_read(); // STM32 internal temperature sensor

#ifdef __cplusplus
}
#endif

uint8_t temprature_sens_read();
//
// parameters for WPS connection
//
#define ESP_WPS_MODE WPS_TYPE_PBC
esp_wps_config_t config = WPS_CONFIG_INIT_DEFAULT(ESP_WPS_MODE);
String wpspin2string(uint8_t a[])
{
  //...
}
//

// retrieve network credentials
const char *ssid1 = SECRET_SSID1;
const char *password1 = SECRET_PASS1;
const char *ssid2 = SECRET_SSID2;
const char *password2 = SECRET_PASS1;
String hostname = "ESP32_pump_ctrl";

long rssi, rssi1, rssi2; // variable for WiFi signal strengh reading and printing
int timeoutCount;        // counter for connection attemps count
#define maxAttemp 20     // max number of attemps for WiFi connection

// definitions for temp. sensor
const int resDisplay = 1; // number of digits after . in temperature display
// temp. sensor configuration
const int SensorDataPin = 14;
// definition for relay board
const int relayIn1Pin = 12;

OneWire oneWire(SensorDataPin);      // set GPIO pin used for communication with DS18B20 temp. sensor through one wire bus
DallasTemperature sensors(&oneWire); // create an instance sensors attched to onewire bus
DeviceAddress deviceAddress;         // variable for sensor address
// int8_t tempResolution = 12;                     // sensor resolution 12 bits - 0.0625°C - 750 ms
int8_t tempResolution = 11; // 11 bits - 0.125°C - 375 ms
// int8_t tempResolution = 10;                     // 10 bits - 0.25°C - 187.5 ms
// int8_t tempResolution = 9;                      // 9 bits - 0.5°C - 93.75 ms

float temperature_Celsius;
float temperature_Fahrenheit;
// end of definitions for temp. sensor

const int LedPin = 2; // pin connected to on board Led

String sliderValue = "0"; // init slider value = 0, i.e. Led off

// setting PWM properties for Led brightness
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
boolean stateLed;

// definitions for time management
#include "time.h"
#include <TimeLib.h>
unsigned int localPort = 2390; // local port to listen for UDP packets
IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
// NTP server to request epoch time
const char *ntpServer = "pool.ntp.org";  // "129.6.15.28" ="time.nist.gov" or "pool.ntp.org"
uint32_t Time1970 = 0;    // variable for storage of time delivered by the NTP server with the format seconds from 01/01/1970
uint32_t Time2020 = 0;    // variable for storage of time delivered by the NTP server with the format seconds from 01/01/2020
uint32_t myTime;          // for general time management
uint32_t epoch;           // used for UTC time computations
uint32_t lastDSSave;      // time of last DS save
uint32_t lastShortTime; // time of last short action such as blinking led
uint32_t lastRefreshTime; // time of last refresh of information to MQTT server
uint32_t lastSaveTime;    // time of last saving of context in flash storage
const uint32_t shortPeriod = 1000;
const uint32_t refreshPeriod = 10000;  // sensor information refresh rate in ms - current value = 10s
const uint32_t savePeriod = 600000;    // context saving rate in ms - current value = 10 mn

// end of definitions for time management

// definitions for pump management
enum pumpModes
{
  winter,
  midSeason,
  summer
};
enum pumpModes currentPumpMode;        // contain current pump mode
float temperatureThresholdWinter = 12; // temperature threshold to enter winter mode, in °C
float temperatureThresholdSummer = 17; // temperature threshold to enter Summer mode, in °C
struct pumpEvent
{
  boolean pumpActivated;        // true if pump is ON, false if pump is OFF
  uint32_t pumpEventEpoch; // epoch of the beginning of the current pump state
};
pumpEvent pumpEventList[28];     // array for saving lats 28 pump events
uint32_t weekPumpTime;      // cumulative pump ON time over the last 7 days
uint32_t cumulativePumpTime; // cumulative pump ON time over the total software life
uint32_t meanDayPumpTime;   // mean pump activation duration per day over the last 7 seven days
// management of activity time of the controller device
uint32_t cumulativeDeviceActivity; // cumulative power up time of the controller device in s
uint32_t cumulativeDeviceActivityAtPowerOn; // cumulative power up time of the controller device in s, reference value at last power up
uint32_t currentDeviceActivity; // device power up time since last power up, in s
uint32_t deviceActivity_1; // device last power up time before current one, in s
uint32_t deviceActivity_2; // device  power up time before the last one, in s
unsigned int devicePowerUp; // current number of power up
String datePowerUp; // date and time of last power up
const uint32_t shortPowerOn = 10; // threshold in s for identification of short power On

float currentWaterTemp;       // water temperature in °C
float hourWaterTempDrift;     // water temperature change over the last hour in °C
float dayWaterTempDrift;      // water temperature change over the last day in °C
boolean currentPumpState;     // ON of OFF corresponding to true and false
uint32_t nextEventEpoch; // schedule epoch for the next pump event
boolean nextEventType;        // defines next pump event ON of OFF
String lastPumpControlerEvent; // used to publish major event to MQTT
String lastSavedEvent;
// enum for file access error management
enum fileErrorCode
{
  NO_ERROR = 0,
  OPENING_ERROR,
  WRITTING_ERROR,
  INTEGRITY_ERROR
};
//
//**************************** MQTT TOPICS ****************************
const char *WATER_TEMP = "WATER_TEMP";
const char *PUMP_STATE = "PUMP_STATE";
const char *PUMP_MODE = "PUMP_MODE";
const char *PUMP_RUNNING_TIME = "PUMP_RUNNING_TIME";
const char *PUMP_CONF = "PUMP_CONF";
const char *CONTROLER_RUNNING_TIME = "CONTROLER_RUNNING_TIME";
const char *PUMP_CONTROLER_EVENT = "PUMP_CONTROLER_EVENT";

const char* CHANGE_ME_TOPIC = "tele/changeme/CHANGEME";
const char* CHANGE_ME_JSON_TOPIC = "tele/changeme/CHANGEME_JSON";
//
//**************************** definitions for context save and retrieve status
enum contextRetrieveStatus
{
  emptyContext,
  regularContext,
  minorErrorContext,
  criticalErrorContext
};
/********************************** FUNCTION DECLARATION (NEEDED BY PLATFORMIO WHILE COMPILING CPP FILES) *****************************************/
// Bootstrap functions
void callback(char *topic, byte *payload, unsigned int length);
void manageDisconnections();
void manageQueueSubscription();
void manageHardwareButton();
// WiFi functions
boolean connectWPS();
boolean connectWithCredentials();
long connectSSID(const char *ssid, const char *password);
boolean reconnectSSID();
void c();
void pumpON();
void pumpOFF();
