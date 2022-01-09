/*********
   Patrick Souty
   Pool filter pump management application

  See README.txt and changelog.txt for additional information

  *********/

#define debug false

// Import required libraries
#include <FS.h>             // library file management - this needs to be first, or it all crashes and burns...
#include "PumpManagement.h" // all other definitions can be found here

boolean readDS(boolean initValue = false);

// functions for WiFi monitoring
void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("WiFi monitoring event");
  Serial.println("Successfully connected to Access Point");
  Serial.print("Channel: ");
  Serial.println(info.connected.channel);
  Serial.print("Athh mode: ");
  Serial.println(info.connected.authmode);
  lastPumpControlerEvent = "Pump controler connected to WiFi AP";
  bootstrapManager.publish(PUMP_CONTROLER_EVENT, helper.string2char(lastPumpControlerEvent), true);
}

void Get_IPAddress(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("WiFi monitoring event");
  Serial.println("WIFI is connected!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Channel: ");
  Serial.println(info.connected.channel);
  Serial.print("Athh mode: ");
  Serial.println(info.connected.authmode);
  lastPumpControlerEvent = "Pump controler IP configured";
  bootstrapManager.publish(PUMP_CONTROLER_EVENT, helper.string2char(lastPumpControlerEvent), true);
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("WiFi monitoring event");
  Serial.println("Disconnected from WIFI access point");

  lastPumpControlerEvent = "WiFi lost connection. Reason: ";
  lastPumpControlerEvent += info.disconnected.reason;
  Serial.println(lastPumpControlerEvent);
}

// Function that gets current epoch time
uint32_t myGetCurrentTime()
{
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    if (debug) Serial.println("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}

// function that returns local date and time
String dateAndTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    if (debug) Serial.println("Failed to obtain time");
    return ("date & time N/A");
  }
  char timeStringBuff[50]; //50 chars should be enough
  strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  //Optional: Construct String object
  return String(timeStringBuff);
}

void initTimeManagement()
// routine to be called after WiFi com init
{
  configTime(0, 0, ntpServer); // retrieve time from ntp server and synchronize internal clock
  if (debug)
  {
    Serial.println(dateAndTime());
  }
}

/********************************** MANAGE WIFI AND MQTT DISCONNECTION *****************************************/
void manageDisconnections()
{

  // specific task if wifi disconnects (such as shut down ...)
}
/********************************** MQTT SUBSCRIPTIONS *****************************************/
void manageQueueSubscription()
{

  bootstrapManager.subscribe(PUMP_CONF);
}
/********************************** MANAGE HARDWARE BUTTON *****************************************/
void manageHardwareButton()
{
  // no HW button to manage
}
/********************************** START CALLBACK *****************************************/
void callback(char *topic, byte *payload, unsigned int length)
{

  // Transform all messages in a JSON format
  StaticJsonDocument<BUFFER_SIZE> json = bootstrapManager.parseQueueMsg(topic, payload, length);

  if (strcmp(topic, CHANGE_ME_TOPIC) == 0)
  {
    String simpleMsg = json[VALUE];
    // Serial.println(simpleMsg);
  }
  else if (strcmp(topic, CHANGE_ME_JSON_TOPIC) == 0)
  {
    String simpleMsg = json["ROOT_EXAMPLE"];
    // Serial.println(simpleMsg);
  }
}
void setup()
{
  // retrieve secret and other network information

  // Serial port for debugging purposes
  Serial.begin(115200);
  Serial.println("-----------------------------------------------------------------------");
  Serial.println("Pump management programm started !");
  Serial.println("----------------------------------");
  Serial.println();
  if (debug)
  {
    Serial.println(AUTHOR);
    Serial.print("SSID: ");
    Serial.println(SSID);
  }
  // initialize digital pin relayIn1Pin as an output.
  pinMode(relayIn1Pin, OUTPUT);
  pumpOFF();
  datePowerUp = "unknown";
  // Datastore management
  // try to read a datastore file
  if (readDS(true) != NO_ERROR)
  {
    // file does not exists or integrity error
    // Then init datastore information
    devicePowerUp = 1;
    currentDeviceActivity = max(millis() / 1000, 1UL);
    deviceActivity_1 = 0;
    deviceActivity_2 = 0;
    cumulativeDeviceActivity =  millis() / 1000;
    cumulativeDeviceActivityAtPowerOn = 0;
    saveDS(); // save datastore
  }
  else
  {
    // detection of double short power ON sequence
    bootstrapManager.forceWebServer = ((deviceActivity_1 < shortPowerOn) && (deviceActivity_1 > 0) && (deviceActivity_2 < shortPowerOn) && (deviceActivity_2 > 0));
    // update power on information
    if (bootstrapManager.forceWebServer)
    {
      currentDeviceActivity = 120; // takes into account the webserver activity because as the ESP will be restarted DS will not be updated - typical duration 120 s
    }
    else
    {
      currentDeviceActivity = max(millis() / 1000, 1UL);  // takes into account current elapsed time
    }
    cumulativeDeviceActivityAtPowerOn = cumulativeDeviceActivity;
    cumulativeDeviceActivity +=  currentDeviceActivity;
    // save immediatly datastore for the case of a short power ON
    saveDS(); // save datastore
  }
  if (debug)
  {
    // print DS information
    parameterPrint("device power up:", devicePowerUp, " ", true);
    parameterPrint("device activity -2:", deviceActivity_2, "s", true);
    parameterPrint("device activity -1:", deviceActivity_1, "s", true);
    parameterPrint("device cumulative activity", cumulativeDeviceActivity, "s", false);
  }
  // WiFi monitoring functions init
  WiFi.onEvent(Wifi_connected, SYSTEM_EVENT_STA_CONNECTED);
  WiFi.onEvent(Get_IPAddress, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(Wifi_disconnected, SYSTEM_EVENT_STA_DISCONNECTED);
  // bootstrap manager setup
  bootstrapManager.bootstrapSetup(manageDisconnections, manageHardwareButton, callback);
  if (debug) readSetup();   // for debug print of setup file afetr init
  printWiFiStatus();
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(LedPin, ledChannel);
  ledcWrite(ledChannel, 255);
  stateLed = true;
  if (debug)
  {
    Serial.println(mqttIP); // debug
    Serial.println(mqttPort);
    Serial.println(mqttuser);
    Serial.println(mqttpass); // debug end
  }

  // DS18B20 temp sensor init
  initDS18B20();
  getDS18B20Readings();
  // init time management
  initTimeManagement();
  datePowerUp = dateAndTime();
  saveDS(); // save datastore

  // context init
  switch (retrieveLastContext()) {
    case emptyContext:
      // init default values
      break;
    case regularContext:
      //
      break;
    case minorErrorContext:
      //
      break;
    case criticalErrorContext:
      //
      break;
  }

  currentPumpState = false;
  currentPumpMode = winter;
  lastRefreshTime = millis(); // save current time at startup
  lastSaveTime = millis();
  printWiFiStatus();
  if (debug) Serial.println("End of setup phase");
  lastPumpControlerEvent = "Pump controler started";
  bootstrapManager.publish(PUMP_CONTROLER_EVENT, helper.string2char(lastPumpControlerEvent), true);
  bootstrapManager.publish(PUMP_CONTROLER_EVENT, helper.string2char(lastSavedEvent), true);
  //
} // end of setup code

void loop()
{

  // boostarpManagere call for OTA and MQTT management
  bootstrapManager.bootstrapLoop(manageDisconnections, manageQueueSubscription, manageHardwareButton);
  uint32_t loopTime = millis();
  // short action loop
  if ((loopTime - lastShortTime) >= shortPeriod)
  {
    lastShortTime = loopTime;
    if (stateLed) {
      stateLed = false;
      ledcWrite(ledChannel, 0);
    }
    else
    {
      stateLed = true;
      ledcWrite(ledChannel, 255);
    }
  }
  // information refresh management
  if ((loopTime - lastRefreshTime) >= refreshPeriod)
  {
    // its time to refresh information
    lastRefreshTime = loopTime; // save current time
    Serial.print("Processor Temperature: ");
    // Convert raw processor temperature in F to Celsius degrees
    Serial.print((temprature_sens_read() - 32) / 1.8);
    Serial.println(" C");
    //
    getDS18B20Readings();
    Serial.printf("DS18B20 sensor temperature = %.2f ºC \n", temperature_Celsius);
    if (debug)
    {
      Serial.print("millis: ");
      Serial.println(millis());
    }
    currentWaterTemp = temperature_Celsius;
    bootstrapManager.publish(WATER_TEMP, helper.string2char(String(currentWaterTemp)), true);
    bootstrapManager.publish(PUMP_STATE, (currentPumpState) ? helper.string2char(ON_CMD) : helper.string2char(OFF_CMD), true);

  }
  // context saving management
  if ((loopTime - lastSaveTime) >= savePeriod)
  {
    // its time to save context
    saveCurrentContext();
    lastSaveTime = loopTime; // save current time
    sendTimeInfo();
    bootstrapManager.publish(PUMP_CONTROLER_EVENT, helper.string2char(lastPumpControlerEvent), true);
  }
}
//------------------------------------------------------------------------------------------------
enum contextRetrieveStatus retrieveLastContext()
// retrieved last saved context in flash memory
{
  // read ID in memory
  // if(ID != refID) {return contextEmpty}
  // currentContextRetrieveStatus = regularContext
  // retrieve index of last saved context
  // load context in indexed zone
  // check hash code
  // if (hash code == KO) {
  //    currentContextRetrieveStatus = minorErrorContext;
  //    decrements index
  //    load context in indexed zone
  //    check hash code
  //    if (hash code == KO) {currentContextRetrieveStatus = minorErrorContext;}
  //  }
  //  return currentContextRetrieveStatus;

  // actual code to be put here
}
//------------------------------------------------------------------------------------------------
void saveCurrentContext()
{
  // retrieve last context index in flash
  // increments index
  // save context in new indexed zone
  // save hash code of data for integrity check
  // save current index as last parameters, this insures that in case of power down during saving, a valid context will be restored
  // read ID in memory
  // if(ID != refID) {write refID}
  // actual code to be put here

  // no index management for this preliminary implementation
  myTime = millis();
  uint32_t elapsedTime = myTime - lastSaveTime;
  if (elapsedTime < 0)
  {
    // millis function has overflown
    elapsedTime += 3600000;   // add an offset of 1 hour to provide a positive integer
    cumulativeDeviceActivity += 4291367 + (elapsedTime / 1000); // add roll out value = 2^32 / 1000 s - 3600 s
  }
  else
  {
    cumulativeDeviceActivity += elapsedTime / 1000;
  }
  currentDeviceActivity = cumulativeDeviceActivity - cumulativeDeviceActivityAtPowerOn;  // refresh device activity for the current power up period
  saveDS(); // save context information
  if (debug)
  {
    Serial.print("Check DS reading: ");
    Serial.println(readDS());
  }
}
//------------------------------------------------------------------------------------------------
void eraseContextMemory()
{
  // used on MQTT command for context management testing purpose
  // write 0 in context ID in memory

  // actual code to be put here
}
//----------------------------------------------------------------------------------
boolean connectWPS()
// test of WiFi WPS connection
{
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_STA);

  Serial.println("Starting WPS");

  esp_wifi_wps_enable(&config);
  esp_wifi_wps_start(0);

  // attempt to connect to Wifi network:
  timeoutCount = 0;                                                 // reset attemp counter
  while ((WiFi.status() != WL_CONNECTED) && (timeoutCount++ < 430)) // max WPS timeout is 5 mn
  {
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    delay(700);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED)
  {
    printWiFiStatus();
    return true;
  }
  else
  {
    Serial.println("WiFi connection with WPS failed !");
    return false;
  }
} // end of connectWPS routine
//------------------------------------------------------
boolean connectWithCredentials()
// try to connect to WiFi link with 2 avalaible SSID, choose the best one based on RSSI and return connection status
// if no connection available, default RSSI = -100
{
  Serial.println("Try to connect with credentials ...");
  WiFi.mode(WIFI_STA);
  // define IP adress and host name
  IPAddress ip(192, 168, 0, 39);
  IPAddress dns(192, 168, 0, 254);
  IPAddress gateway(192, 168, 0, 254);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(ip, dns, gateway, subnet);
  //  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  if (!WiFi.setHostname(hostname.c_str())) // define hostname
  {
    Serial.println("set host named failed !");
  }
  // Connect to Wi-Fi
  // try to connect with 1st SSID and save 1st RSSI
  Serial.println("Try 1st SSID");
  rssi1 = connectSSID(ssid1, password1);
  // try to connect with 2nd SSID and save 2nd RSSI
  Serial.println("Try 2nd SSID");
  rssi2 = connectSSID(ssid2, password2);
  // check best rssi
  if (rssi2 > rssi1)
  {
    // the 2nd SSID is the best one and can be kept
    // the connection is necessarily OK, then do nothing else
  }
  else
  {
    // connect again to 1st SSID and save RSSI in rssi2 that is the common variable for final RSSI
    rssi2 = connectSSID(ssid1, password1);
  }

  if (rssi2 > -100)
  { // connection success
    printWiFiStatus();
    return true;
  }
  else
  {
    Serial.println("WiFi connection with credentials failed !");
    return false;
  }
} // end of connectWithCredentials routine
//
// ----------------------------------------------------------------------------------
long connectSSID(const char *ssid, const char *password)
// connect to WiFi ssid and return RSSI of the link
// in case of failed connection, RSSI defaul value is -100
{
  WiFi.disconnect(); // force disconnect first
  delay(100);
  timeoutCount = 0; // reset attemp counter
  WiFi.begin(ssid, password);
  while ((WiFi.status() != WL_CONNECTED) && (timeoutCount++ < maxAttemp))
  {
    delay(1000);
    Serial.print(timeoutCount);
    Serial.print(" Connecting to WiFi with SSID ");
    Serial.println(ssid);
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    rssi = WiFi.RSSI(); // save current rssi value
  }
  else
  {
    rssi = -100; // force rssi to a very low level
  }
  // print the received signal strength:
  Serial.print("Current signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
  return rssi;
} // end of connectSSID routine
//
//----------------------------------------------------------------------------------
boolean reconnectSSID()
// try to connect with last successful link credentials
{
  if (!WiFi.setHostname(hostname.c_str())) // define hostname
  {
    Serial.println("set host named failed !");
  }
  timeoutCount = 0; // reset attemp counter
  WiFi.begin();     // with no credentials, uses last known by defualt
  while ((WiFi.status() != WL_CONNECTED) && (timeoutCount++ < maxAttemp))
  {
    delay(1000);
    Serial.print(timeoutCount);
    Serial.println(" Connecting to WiFi using last credentials ");
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    printWiFiStatus();
    return true;
  }
  else
  {
    Serial.println("WiFi connection with last credentials failed !");
    return false;
  }
} // end of reconnectSSID routine
//
//----------------------------------------------------------------------------------
void printWiFiStatus()
{
  Serial.println("WiFi status");
  Serial.print("WiFi connected to SSID: ");
  Serial.println(WiFi.SSID());
  rssi = WiFi.RSSI(); // save current rssi value
  // print the received signal strength:
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
//----------------------------------------------------------------------------------
void getDS18B20Readings()
{
  sensors.requestTemperatures();
  temperature_Celsius = sensors.getTempCByIndex(0);
  temperature_Fahrenheit = sensors.getTempFByIndex(0);
}
//----------------------------------------------------------------------------------
String processor(const String &var)
{
  getDS18B20Readings();
  // Serial.println(var);
  if (var == "TEMPERATURE_C")
  {
    return String(temperature_Celsius, resDisplay);
  }
  else if (var == "TEMPERATURE_F")
  {
    return String(temperature_Fahrenheit, resDisplay);
  }
}
//----------------------------------------------------------------------------------
void initDS18B20()
{
  // init temp sensor configuration
  sensors.begin();
  // Get ID of first sensor (at index 0)
  sensors.getAddress(deviceAddress, 0);

  // By default configuration and alarm/userdata registers are also saved to EEPROM
  // when they're changed. Sensors recall these values automatically when powered up.

  // Turn OFF automatic saving of configuration and alarm/userdata registers to EEPROM
  sensors.setAutoSaveScratchPad(false);

  // Change configuration and alarm/userdata registers on the scratchpad

  sensors.setResolution(deviceAddress, resolution);

  // Save configuration and alarm/userdata registers to EEPROM
  sensors.saveScratchPad(deviceAddress);
}

/*
  Since PumpStatus is READ_WRITE variable, onPumpStatusChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onPumpStatusChange()
{
  // Add your code here to act upon PumpStatus change
  Serial.println("Pump status changed");
  Serial.println(currentPumpState);
  if (currentPumpState)
  {
    pumpON();
  }
  else
  {
    pumpOFF();
  }
}
//------------------------------------------------------------------------------------------------

void WiFiEvent(WiFiEvent_t event, system_event_info_t info)
{
  switch (event)
  {
    case SYSTEM_EVENT_STA_START:
      Serial.println("Station Mode Started");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("Connected to :" + String(WiFi.SSID()));
      Serial.print("Got IP: ");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from station, attempting reconnection");
      WiFi.reconnect();
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      Serial.println("WPS Successfull, stopping WPS and connecting to: " + String(WiFi.SSID()));
      esp_wifi_wps_disable();
      delay(10);
      WiFi.begin();
      break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      Serial.println("WPS Failed, retrying");
      esp_wifi_wps_disable();
      esp_wifi_wps_enable(&config);
      esp_wifi_wps_start(0);
      break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      Serial.println("WPS Timedout, retrying");
      esp_wifi_wps_disable();
      esp_wifi_wps_enable(&config);
      esp_wifi_wps_start(0);
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      Serial.println("WPS_PIN = " + wpspin2string(info.sta_er_pin.pin_code));
      break;
    default:
      break;
  }
}
//-----------------------------------------------------------------------------------------------------------
void pumpON()
// filtration pump activation
{
  digitalWrite(relayIn1Pin, LOW); // switch ON relay 1 (In1 relay input active low)
  digitalWrite(LedPin, HIGH);     // switch ON internal Led
}
void pumpOFF()
// filtration pump stop
{
  digitalWrite(relayIn1Pin, HIGH); // switch OFF relay 1 (HIGH is the voltage level)
  digitalWrite(LedPin, LOW);       // switch OFF internal Led
}

//------------------------------------------------------------------------------------------------------------
boolean readDS(boolean initValue)
// try to read datastore file and return true if successfull
{
  enum fileErrorCode fileError = NO_ERROR;
  // retrieve DS file informations
  DynamicJsonDocument DSdoc = bootstrapManager.readSPIFFS("DS.json");
  const char* header = DSdoc["header"];
  if (header)
  {
    // file already exists and contains a header, then retrieve number of power up and cumulative up time
    if (initValue)
    {
      // init global parameter when readDS at power ON
      Serial.println("---- Init time parameters from DS file ----");
      cumulativeDeviceActivity = helper.getValue(DSdoc["cumulativeDeviceActivity"]).toInt() + millis() / 1000;
      deviceActivity_1 = helper.getValue(DSdoc["lastDeviceActivity_1"]).toInt();
      deviceActivity_2 = helper.getValue(DSdoc["lastDeviceActivity_2"]).toInt();
      devicePowerUp = 1 + helper.getValue(DSdoc["devicePowerUp"]).toInt();
      lastSavedEvent = "Last saved event : " + helper.getValue(DSdoc["lastEvent"]);
    }
    // check data integrity
    String readHashDS = helper.getValue(DSdoc["HC"]);
    Serial.println("Read hash code: " + readHashDS);
    DSdoc["HC"] = "null";  // hash code is calculated on the data including the CS, then this one must be set to a fix value before
    String hashDS = hashPayload(DSdoc);// computes the hash code
    if (hashDS == readHashDS)
    {
      Serial.println("file hash code OK");
    }
    else
    {
      lastPumpControlerEvent = "Integrity error in data store";
      Serial.println(lastPumpControlerEvent);
      fileError = INTEGRITY_ERROR;
    }
  }
  else
  {
    // files does not exist or does not contain a header
    fileError = OPENING_ERROR;
    lastPumpControlerEvent = "DS file opening error";
    Serial.println(lastPumpControlerEvent);

  }
  return fileError;
}
//------------------------------------------------------------------------------------------------------------
void saveDS()
// save current context in datastore file
{
  DynamicJsonDocument DSdoc(1024);
  DSdoc["header"] = "Header";
  DSdoc["cumulativeDeviceActivity"] = String(cumulativeDeviceActivity);
  DSdoc["lastDeviceActivity_1"] = String(currentDeviceActivity);
  DSdoc["lastDeviceActivity_2"] = String(deviceActivity_1);
  DSdoc["devicePowerUp"] = String(devicePowerUp);
  DSdoc["lastEvent"] = lastPumpControlerEvent;
  // computes and stores hash code for data integrity monitoring
  DSdoc["HC"] = "null";  // hash code is calculated on the data including the CS, then this one must be set to a fix value before
  String hashDS = hashPayload(DSdoc);// computes the hash code
  DSdoc["HC"] = String(hashDS);
  // save the file
  bootstrapManager.writeToSPIFFS(DSdoc, "DS.json");
}

//------------------------------------------------------------------------------------------------------------
void readSetup()
// read setup file
{
  DynamicJsonDocument doc = bootstrapManager.readSPIFFS("setup.json");
}
//------------------------------------------------------------------------------------------------------------
void sendTimeInfo()
// send time information on MQTT
{
  JsonObject root = bootstrapManager.getJsonObject();

  root["Time"] = serialized(jsonString(dateAndTime()));
  root["Last_power_ON_date_and_time"] = serialized(jsonString(datePowerUp));
  JsonObject Power_ON = root.createNestedObject("Power_ON");

  Power_ON["Cumulative_ON_time"] = cumulativeDeviceActivity;
  Power_ON["Current_ON_time"] = currentDeviceActivity;
  Power_ON["ON_time_previous_sequence"] = deviceActivity_1;
  Power_ON["ON_time_before_previous_sequence"] = deviceActivity_2;
  Power_ON["Number_of_power_ON_or_restart"] = devicePowerUp;

  bootstrapManager.publish(CONTROLER_RUNNING_TIME, root, true);

}
//------------------------------------------------------------------------------------------------------------
String jsonString(String input)
// add " arround the input string
{
  return "\"" + input + "\"";

}
// time functions
//--------------------------------------------------------------------------
// routines com serveur NTP and time management

// send an NTP request to the time server at the given address
uint32_t sendNTPpacket(IPAddress &address)
{
  // Serial.println("1");
  //  set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  // Serial.println("2");
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  // Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  // Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  // Serial.println("5");
  Udp.endPacket();
  // Serial.println("6");
}
//--------------------------------------------------------------------------
String Date(uint32_t epoch, boolean Reverse = false)
{ // convertit une date au format Unix secondes depuis 01/01/1970 en jour/mois/année

  String sDate;
  if (Reverse)
  {
    sDate = String(year(epoch)) + "/" + String(month(epoch)) + "/" + String(day(epoch));
  }
  else
  {
    sDate = String(day(epoch)) + "/" + String(month(epoch)) + "/" + String(year(epoch));
  }
  return sDate;
}
//--------------------------------------------------------------------------
uint32_t UnixTime(uint32_t secsSince1900)
{
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  const uint32_t seventyYears = 2208988800;
  // subtract seventy years:
  uint32_t epoch = secsSince1900 - seventyYears;
  return epoch;
}
//--------------------------------------------------------------------------
uint32_t PandemicTime(uint32_t secsSince1900)
{
  // Pandemic time starts on Jan 01 2020. In seconds that's 3786825600
  const uint32_t ManyYears = 3786825600UL;
  // subtract many years:
  uint32_t epoch = secsSince1900 - ManyYears;
  return epoch;
}
//--------------------------------------------------------------------------
String UTCTime(uint32_t epoch)
{
  // provides UTC time from time in seconds frome 01/01/1970 (time at Greenwich Meridian (GMT))
  String Hour = String((epoch % 86400L) / 3600); // calculate the hour (86400 equals secs per day)
  // Serial.print(':');
  String Minute = String((epoch % 3600) / 60); // calculate the minute (3600 equals secs per minute)
  if (((epoch % 3600) / 60) < 10)
  {
    // In the first 10 minutes of each hour, we'll want a leading '0'
    Minute = "0" + Minute;
  }
  String Seconds = String(epoch % 60); // calculate the second
  if ((epoch % 60) < 10)
  {
    // In the first 10 seconds of each minute, we'll want a leading '0'
    Seconds = "0" + Seconds;
  }
  return Hour + ":" + Minute + ":" + Seconds;
}
//--------------------------------------------------------------------------
String hashPayload(DynamicJsonDocument jsonDocument)
// provides a SHA-256 hash of the input json document, the output is formatted into a string of hexadecimal values
{
  String output = "";

  // converts json document into a string buffer
  char *payload[1024];
  const size_t payloadLength = serializeJson(jsonDocument, payload, 1024);

  // hash the buffer string
  byte shaResult[32];

  mbedtls_md_context_t ctx;
  mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;

  //  const size_t payloadLength = strlen(payload);

  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
  mbedtls_md_starts(&ctx);
  mbedtls_md_update(&ctx, (const unsigned char *) payload, payloadLength);
  mbedtls_md_finish(&ctx, shaResult);
  mbedtls_md_free(&ctx);

  for (int i = 0; i < sizeof(shaResult); i++) {
    char str[3];

    sprintf(str, "%02x", (int)shaResult[i]);
    output += str;
  }
  Serial.println("Computed hash code: " + output);
  return output;
}
