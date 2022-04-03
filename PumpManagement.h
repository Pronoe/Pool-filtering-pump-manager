/**********************************************************************
   Definitions for pump management software

   Note: if you are using ARDUINO IDE for compiling, these defintions
   do not override those defined in the header file of the Bootstarpper
   library (SSID, IP addresses, ...)
   You have then to modify these defintion in the library it-self.
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
#include "MySecrets.h"
// library and defintion for tasl watch dog timer
#include <esp_task_wdt.h>
#ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
#if CONFIG_IDF_TARGET_ESP32  // ESP32/PICO-D4
#include <rom/rtc.h>
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/rtc.h"
#else
#error Target CONFIG_IDF_TARGET is not supported
#endif
#else // ESP32 Before IDF 4.0
#include "rom/rtc.h"
#endif
// 6 seconds WDT
#define WDT_TIMEOUT 60 // must be enough to allow OTA uploading within the watch dog time out
/****************** BOOTSTRAP MANAGER ******************/
// MQTT, FS and OTA management
#include <BootstrapManager.h>
#include <PingESP.h>
BootstrapManager bootstrapManager;
Helpers helper;
PingClass pingESP; // instead of PingESP pingESP => use class provided by ESP32ping
// library ezTime
#include <ezTime.h>
Timezone myTZ;

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

uint32_t myTime;                       // for general time management
uint32_t loopTime;                     // beginning time of main loop
uint32_t lastLoopTime;                 // saved loop time for rollover detection
uint32_t lastDSSave;                   // time of last DS save
uint32_t lastShortTime;                // time of last short action such as blinking led
uint32_t lastRefreshTime;              // time of last refresh of information to MQTT server
uint32_t lastSaveTime;                 // time of last saving of context in flash storage
uint32_t lastHourTime;                 // time of last hourly data assessment
uint32_t lastPumpActivationTime;       // time of last power ON of the pump
const uint32_t shortPeriod = 1000;     // rate of fast actions such as led blinking
const uint32_t refreshPeriod = 60000;  // sensor information refresh rate in ms - current value = 60s
const int timeSlotNb = 10;             // slow refresh rate information are published in a single time slot, 10 slots are allocated, one slot per minute
int slotCounter;                       // current slot number
const uint32_t savePeriod = 600000;    // context saving rate in ms - current value = 10 mn
const uint32_t hourlyPeriod = 3600000; // one hour in ms
const uint32_t minutePeriod = 60000;   // one minute in ms
const uint32_t secondsPerMinute = 60;  //
const uint32_t secondsPerHour = 3600;  //
int indexDSFile;                       // index for construction of the DS file name (multiple DS files for redundancy)
const int nbIndexDS = 3;               // number of DS redundant DS files

// end of definitions for time management

// definitions for event message stack
#define stackSize 10
String eventStack[stackSize] = {""}; // stack for saving event messages before publishing
int indexEventStackR = 0;
int indexEventStackW = 0;
unsigned int eventCounter = 0;
unsigned int wifiDisconnectCount = 0;

// definitions for pump management
enum pumpModes
{
  winter,
  midSeason,
  summer
};
enum pumpModes currentPumpMode;                                     // contain current pump mode
String pumpModeVerbose[4] = {"Hiver", "Mi saison", "Eté", "error"}; // pump mode for display
float temperatureThresholdWinter = 12.;                             // temperature threshold to enter winter mode, in °C
float temperatureThresholdSummer = 17.;                             // temperature threshold to enter Summer mode, in °C
float temperatureThresholdFrozen = 4.;                              // temperature threshold for continuous pump activation
uint32_t winterPumpScheduledTime = 90;                              // activation time in winter in each of the activation window - in minutes
boolean triggerTS;                                                  // flag for summer mode init
boolean triggerTW;                                                  // flag for winter mode init
boolean triggerTMS;                                                 // flag for mid season mode init
int TScounter, TWcounter, TMScounter;                               // counter to secure mode change decision
const int counterThreshold = 8;
// pump activation windows definition
uint8_t morningWindowStartHour = 5;
uint8_t morningWindowStartMinute = 0;
uint8_t eveningWindowStartHour = 18;
uint8_t eveningWindowStartMinute = 0;
uint32_t dayPumpTime[7] = {0}; // table for each day of one week - in ms
uint8_t indexPumpTime = 0;     // index for the table dayPumpTime
// new definition
uint32_t pumpDayRunningTime;                                                                                                                           // current pump running time within the the current day, in s
uint32_t cumulativePumpTime;                                                                                                                           // cumulative pump ON time over the total software life, in seconds (capacity 136 years)
uint32_t pumpRunningTimeManualOff;                                                                                                                     // time counter for OFF manual state within a regular ON activation window - in seconds
uint32_t pumpRunningTimeManualOn;                                                                                                                      // time counter for ON manual state outside a reglar ON activation window - in seconds
uint32_t scheduledPumpRunningTime;                                                                                                                     // scheduled pump running time within the current wactivation window, in seconds
boolean pumpEveningIsRunning;                                                                                                                          // true during evening activation window
boolean pumpMorningIsRunning;                                                                                                                          // true during morning activation window
#define pumpWindowIsRunning (pumpMorningIsRunning || pumpEveningIsRunning)                                                                             // true during activation window (morning and evening)
boolean pumpManualOn;                                                                                                                                  // true when pump has been switched ON mannually outside a regular activation window
boolean pumpManualOff;                                                                                                                                 // true when pump has been switched OFF mannually within a regular activation window
boolean pumpContinuousOn = false;                                                                                                                      // true when pump shall be maintained continuously ON, for intensive cleaning or shock chlorination for example
boolean pumpContinuousOff = false;                                                                                                                     // true when pump shall be maintained continuously OFF, for maintenance
// pumpShouldBeOn true whenever the pump should be switched ON
#define pumpShouldBeOn (!pumpContinuousOff) && ((((pumpWindowIsRunning) || (triggerPAC) || (pumpContinuousOn) || (triggerRefMeasurement)) && (!pumpManualOff)) || (pumpManualOn))
boolean stateChanged = true;                                                                                                                           // trigger flag that enables controler status publishing when true
uint8_t currentDay;                                                                                                                                    // day number of the current day
uint8_t currentMonth;                                                                                                                                  // month number of the current day
uint8_t currentYear;                                                                                                                                   // year number of the current day
uint8_t readDay;                                                                                                                                       // used for reading the day with ezTime library
uint8_t readHour;                                                                                                                                      // used for reading the Hour with ezTime library
time_t readTime;                                                                                                                                       // read time in second since Jan 1st 1970
time_t pumpWindowT1b;                                                                                                                                  // timedate of morning window begin, in second since Jan 1st 1970
time_t pumpWindowT1e;                                                                                                                                  // timedate of morning window end, in second since Jan 1st 1970
time_t pumpWindowT2b;                                                                                                                                  // timedate of evening window begin, in second since Jan 1st 1970
time_t pumpWindowT2e;                                                                                                                                  // timedate of evening window end, in second since Jan 1st 1970
time_t lastUpdateTime;                                                                                                                                 // time when the last pump running time was updated (nominally each minute)
boolean pumpIsON;                                                                                                                                      // ON of OFF corresponding to true and false
uint32_t maxManualTime = 3600;                                                                                                                         // manual mode is disabled automatically after this delay in seconds, i.e. 1 hour
uint32_t meanDayPumpTime;                                                                                                                              // mean pump activation duration per day over the last 7 seven days - ins hour
// management of activity time of the controller device
uint32_t cumulativeDeviceActivity;          // cumulative power up time of the controller device in s
uint32_t cumulativeDeviceActivityAtPowerOn; // cumulative power up time of the controller device in s, reference value at last power up
uint32_t currentDeviceActivity;             // device power up time since last power up, in s
uint32_t deviceActivity_1;                  // device last power up time before current one, in s
uint32_t deviceActivity_2;                  // device  power up time before the last one, in s
unsigned int devicePowerUp;                 // current number of power up
String datePowerUp;                         // date and time of last power up
const uint32_t shortPowerOn = 10;           // threshold in s for identification of short power On
// heat pump automation (note: heat pump translates into Pompe A Chaleur = PAC in french)
float PACTempTarget = 28.7; // water temperature threshold that will need filtering pump activation to trigger PAC heating (based on water flow detection and temperetaure < 29°C)
#define minPACtempTarget 25.0
#define maxPACtempTarget 30.0
#define PACTempStartOffset 0.5 // in heat pump automation mode, filtering pump will be activated when water temp < PACTempTarget - PACTempStartOffset
#define PACTempStopOffset 1.0  // filtering pump will be stopped when water temp > PACTempTarget + PACTempStopOffset (excepetd if running is vactivated manually or for filtering)
boolean PACAutomation = false; // must be set to true to enable automation
boolean triggerPAC;            // set to true if water flow needed for PAC heating activation computed dynamically)
// water temperature parameters
float currentWaterTemp;                 // water temperature in °C
float filteredWaterTemp;                // filtered water temp with digital filter
float referenceTemp;                    // contains test temperature or filteredWaterTemp depending on ctrlTest state, used for pump mode decision and pump running time scheduling
float PACreferenceTemp;                 // contains test temperature or refWaterTemp depending on ctrlTest state, used for PAC automation
float hourWaterTempDrift;               // water temperature change over the last hour in °C
float dayWaterTempDrift;                // water temperature change over the last day in °C
float waterTempTest = 28.0;             // dummy water temp for tests
boolean triggerRefMeasurement = false;  // activated each hour to manage water temperature measurement with water flow activaed
boolean requestRefMeasurement = false;  // flag to trigger a reference measurement
float refWaterTemp;                     // used for reference water temperature measurement
float lastRefWaterTemp;                 // reference water temp saved nominally each hour
uint32_t lastRefTime;                   // time in ms of last reference measurement
uint32_t triggerTime;                   // records time of measurement trigger activation
int refMeasurementCounter;              //
#define maxRefMeasumentCount 20         // number of temperature measurements required for setting a reference measurement based on average value
const uint32_t waterFlowDelay = 300000; // preliminary duration for pump activation before reference measurement - in ms =>

boolean ctrlTest = false; // enable/disable test mode of the pump controller

uint32_t nextEventEpoch;       // schedule epoch for the next pump event
boolean nextEventType;         // defines next pump event ON of OFF
String lastPumpControlerEvent; // used to publish major event to MQTT
String lastSavedEvent;         // used to retrieve at startup the last event saved in DS file
boolean justStarted;           // allows specific actions at startup in the main loop
// definitions for digital filter of water temperature
// IIR digital filter order 1 - definition: s = z^(-1) x s + filterCoef * (x - z^(-1) * s)
// filterCoef = 1 / (filterTimeConstant * samplingFrequency)
#define filterTimeConstant 900. // time constant in s
const float filterCoef = float(refreshPeriod) / (filterTimeConstant * 1000.);
//
// defintions for myPrint function
String myPrintStream = "";
// enum for file access error management
enum fileErrorCode
{
  NO_FILE_ERROR = 0,
  OPENING_ERROR,
  WRITTING_ERROR,
  INTEGRITY_ERROR,
  JSON_ERROR
};
//
// const String definitions
const String ACTIVE = "Active";
const String INACTIVE = "Inactive";
//**************************** MQTT TOPICS ****************************
const char *WATER_TEMP = "SW/WATER/TEMP";
const char *WATER_TEMP_DRIFT = "SW/WATER/TEMP_DRIFT";
const char *WATER_TEMP_REF = "SW/WATER/TEMP_REF";
const char *CONTROLER_STATE = "SW/CONTROLER/STATE";
const char *PUMP_RUNNING_TIME = "SW/PUMP/RUNNING_TIME";
const char *MEAN_PUMP_RUNNING_TIME = "SW/PUMP/MEAN_RUNNING_TIME";
const char *CONTROLER_RUNNING_TIME = "SW/CONTROLER/RUNNING_TIME";
const char *PUMP_CONTROLER_EVENT = "SW/CONTROLER/EVENT";
const char *WIFI_INFO = "SW/CONTROLER/WIFI_INFO";     // used to publish WiFi link informations
const char *PUMP_INFO = "SW/PUMP/INFO";               // used to publish pump running time and status informations
const char *TECH_INFO = "SW/CONTROLER/TECH_INFO";     // used to publish context data saved in flash memory (for debug purposes)
const char *SERIAL_MQTT = "SW/CONTROLER/SERIAL_MQTT"; // used to publish a copy of data sent on the serial line (for debug)
// command keywords for CMD commands
const String ignore_CMD = "ignore";
const String RESET_CMD = "reset";     // used to reset pump running time informations
const String TEST = "TEST";           // used to enable test mode with a dummy water temperature
const String PAC_AUTO = "PAC_AUTO";   // used to enable/disable heat pump automatic mode
const String SEND = "SEND";           // used to send specific parameter on request
const String CONT_RUN = "CONT_RUN";   // used to enable/disable continuous run of the pump
const String CONT_STOP = "CONT_STOP"; // used to enable/disable continuous run of the pump
const String STATE = "STATE";         // used to set state of the controller
// command keywords for SET commands
const String WINDOW = "WINDOW";       // used to set windows begining times
const String TEMP_TEST = "TEMP_TEST"; // used to set a dummy water temmperature for testing purposes
const String TEMP_PAC = "TEMP_PAC";   // used to set temperature threshold for heat pump activation
// commands Topics
const char *PUMP_CMD = "SW/PUMP/CMD";      // used to send pump ON/OFF commands
const char *CTRL_SET = "SW/CONTROLER/SET"; // used to set controller parameters (windows times, PAC temperature ctrl, test water temp)
const char *CTRL_CMD = "SW/CONTROLER/CMD"; // used to send commands to controller (TEST, PAC_AUTO, SEND <parameter>, FORMAT FS, )
// keywords for SEND command
const String MICRO = "MICRO"; // used to send micro conroller information

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
void myPrint(const char *value);
void myPrint(String value);
void myPrint(int value);
void myPrint(long int value);
void myPrint(long unsigned int value);
void myPrint(float value);
void myPrint(boolean value);
void myPrint(uint32_t value);
void myPrint(double value);
void myPrint();

void myPrintln(const char *value);
void myPrintln(String value);
void myPrintln(int value);
void myPrintln(long int value);
void myPrintln(long unsigned int value);
void myPrintln(float value);
void myPrintln(boolean value);
void myPrintln(uint32_t value);
void myPrintln(double value);
// print String on Serial or Serial + MQTT channel
void myPrint(const char *value)
{
  if (serial2MQTT)
    myPrintStream += String(value);
  Serial.print(String(value));
}
void myPrintln(const char *value)
{
  Serial.println(String(value));
  if (serial2MQTT)
  {
    myPrintStream += String(value);
    bootstrapManager.publish(SERIAL_MQTT, helper.string2char(myPrintStream), true);
    myPrintStream = "";
  }
}

void myPrint(String value)
{
  if (serial2MQTT)
    myPrintStream += String(value);
  Serial.print(String(value));
}
void myPrintln(String value)
{

  Serial.println(String(value));
  if (serial2MQTT)
  {
    myPrintStream += String(value);
    bootstrapManager.publish(SERIAL_MQTT, helper.string2char(myPrintStream), true);
    myPrintStream = "";
  }
}
// print int on Serial or Serial + MQTT channel
void myPrint(int value)
{
  if (serial2MQTT)
    myPrintStream += String(value);
  Serial.print(value);
}
void myPrintln(int value)
{
  Serial.println(value);
  if (serial2MQTT)
  {
    myPrintStream += value;
    bootstrapManager.publish(SERIAL_MQTT, helper.string2char(myPrintStream), true);
    myPrintStream = "";
  }
}
// print long int on Serial or Serial + MQTT channel
void myPrint(long int value)
{
  if (serial2MQTT)
    myPrintStream += String(value);
  Serial.print(value);
}
void myPrintln(long int value)
{
  Serial.println(value);
  if (serial2MQTT)
  {
    myPrintStream += value;
    bootstrapManager.publish(SERIAL_MQTT, helper.string2char(myPrintStream), true);
    myPrintStream = "";
  }
}
// print long unsigned int on Serial or Serial + MQTT channel
void myPrint(long unsigned int value)
{
  if (serial2MQTT)
    myPrintStream += String(value);
  Serial.print(value);
}
void myPrintln(long unsigned int value)
{
  Serial.println(value);
  if (serial2MQTT)
  {
    myPrintStream += value;
    bootstrapManager.publish(SERIAL_MQTT, helper.string2char(myPrintStream), true);
    myPrintStream = "";
  }
}
// print float on Serial or Serial + MQTT channel
void myPrint(float value)
{
  if (serial2MQTT)
    myPrintStream += String(value, 2);
  Serial.print(value);
}
void myPrintln(float value)
{
  Serial.println(value);
  if (serial2MQTT)
  {
    myPrintStream += String(value, 2);
    bootstrapManager.publish(SERIAL_MQTT, helper.string2char(myPrintStream), true);
    myPrintStream = "";
  }
}
// print boolean on Serial or Serial + MQTT channel
void myPrint(boolean value)
{
  if (serial2MQTT)
    myPrintStream += String(value);
  Serial.print(value);
}
void myPrintln(boolean value)
{
  Serial.println(value);
  if (serial2MQTT)
  {
    myPrintStream += value;
    bootstrapManager.publish(SERIAL_MQTT, helper.string2char(myPrintStream), true);
    myPrintStream = "";
  }
}
// print uint32_t on Serial or Serial + MQTT channel
void myPrint(uint32_t value)
{
  if (serial2MQTT)
    myPrintStream += String(value);
  Serial.print(value);
}
void myPrintln(uint32_t value)
{
  Serial.println(value);
  if (serial2MQTT)
  {
    myPrintStream += value;
    bootstrapManager.publish(SERIAL_MQTT, helper.string2char(myPrintStream), true);
    myPrintStream = "";
  }
}
/*
  // print unsigned int on Serial or Serial + MQTT channel
  void myPrint( unsigned int value)
  {
  if (serial2MQTT) myPrintStream += String(value);
  Serial.print(value);
  }
  void myPrintln( unsigned int value)
  {
  Serial.println(value);
  if (serial2MQTT)
  {
    myPrintStream += value;
    bootstrapManager.publish(SERIAL_MQTT, helper.string2char(myPrintStream), true);
  myPrintStream = "";
  }
  }
*/
// print double on Serial or Serial + MQTT channel
void myPrint(double value)
{
  if (serial2MQTT)
    myPrintStream += String(value, 2);
  Serial.print(value);
}
void myPrintln(double value)
{
  Serial.println(value);
  if (serial2MQTT)
  {
    myPrintStream += String(value, 2);
    bootstrapManager.publish(SERIAL_MQTT, helper.string2char(myPrintStream), true);
    myPrintStream = "";
  }
}
// print null on Serial or Serial + MQTT channel
void myPrintln()
{
  Serial.println();
  if (serial2MQTT)
  {
    bootstrapManager.publish(SERIAL_MQTT, helper.string2char(myPrintStream), true);
    myPrintStream = "";
  }
}
