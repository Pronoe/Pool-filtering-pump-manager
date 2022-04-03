/*********
   Patrick Souty
   Pool filter pump management application

  See README.txt and changelog.txt for additional information

  *********/

#define debug false
#define debugTime false
#define ignoreHash false  // set true for inhibition of file hash code checking when file structure is changed
#define serial2MQTT true // if true, maps all serial print and println to MQTT (when called by myPrint or myPrintln)
#define codeTest false   // enable dedicated functionnal test

// Import required libraries
#include <FS.h>             // library file management - this needs to be first, or it all crashes and burns...
#include "PumpManagement.h" // all other definitions can be found here

boolean readDS(boolean initValue = false);

void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  myPrintln("WiFi monitoring event");
  myPrintln("Successfully connected to Access Point");
  myPrint("Channel: ");
  myPrintln(info.connected.channel);
  myPrint("Athh mode: ");
  myPrintln(info.connected.authmode);
  lastPumpControlerEvent = "Pump controler connected to WiFi AP";
  pushEvent(lastPumpControlerEvent);
  //  publishLastEvents();
}

void Get_IPAddress(WiFiEvent_t event, WiFiEventInfo_t info)
{
  myPrintln("WiFi monitoring event");
  myPrintln("WIFI is connected!");
  myPrintln("IP address: ");
  myPrintln(WiFi.localIP());
  myPrint("Channel: ");
  myPrintln(info.connected.channel);
  myPrint("Athh mode: ");
  myPrintln(info.connected.authmode);
  lastPumpControlerEvent = "Pump controler IP configured";
  pushEvent(lastPumpControlerEvent);
  //  publishLastEvents();
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  myPrintln("WiFi monitoring event");
  myPrintln("Disconnected from WIFI access point");
  wifiDisconnectCount++;
  lastPumpControlerEvent = "WiFi lost connection. Reason: ";
  lastPumpControlerEvent += verbose_wifi_reason(info.disconnected.reason);
  lastPumpControlerEvent += " time slot ";
  lastPumpControlerEvent += String(slotCounter);
  pushEvent(lastPumpControlerEvent);
  //  publishLastEvents();
  myPrintln(lastPumpControlerEvent);
}

// function that returns local date and timesn
String dateAndTime()
{
  return myTZ.dateTime("d/m/y - H:i:s");
}

void initTimeManagement()
// routine to be called after WiFi com init
{
  // initialize ezTime
  setDebug(INFO);
  setInterval(1980); // refresh NTP time every 33 mn
  waitForSync(60);   // try to synchronize time with NTP server, with a timeout of 60 seconds
  // Provide official timezone names
  // https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
  myTZ.setLocation("Europe/Paris");
  // check if time synchro is OK
  if (timeStatus() != timeSet)
  {
    lastPumpControlerEvent = "NTP time synchronization not achieved within 60 s";
    pushEvent(lastPumpControlerEvent);
    Serial.println(lastPumpControlerEvent);
    // set time by default
    setTime(makeTime(12, 0, 0, 1, 1, 2022)); // default date & time 01/01/2022 @ 12:00:00
  }

  if (debugTime)
    myPrintln(myTZ.dateTime("d/m/y - H:i:s"));
}

/********************************** MANAGE WIFI AND MQTT DISCONNECTION *****************************************/
void manageDisconnections()
{
  // specific task if wifi disconnects permanently (such as shut down ...)
  esp_task_wdt_reset(); // resets watch dog
  /*
    if (wifiReconnectAttemp > MAX_RECONNECT)
    {
    // cold restart of WiFi connect
    Serial.println("Max WiFi reconnect attemps reached ! try to reset WiFi config");
    bootstrapManager.initWifi(manageDisconnections, manageHardwareButton);
    }
  */
}
/********************************** MQTT SUBSCRIPTIONS *****************************************/
void manageQueueSubscription()
{

  bootstrapManager.subscribe(PUMP_CMD);
  bootstrapManager.subscribe(CTRL_SET);
  bootstrapManager.subscribe(CTRL_CMD);
}
/********************************** MANAGE HARDWARE BUTTON *****************************************/
void manageHardwareButton()
{
  // no HW button to manage
}
/********************************** START CALLBACK *****************************************/
void callback(char *topic, byte *payload, unsigned int length)
{
  String streamString;
  unsigned int topiclength;
  topiclength = strlen(topic);
  char topicValue[topiclength + 1];
  for (unsigned int i = 0; i < topiclength; i++)
  {
    topicValue[i] = (char)topic[i];
  }
  topicValue[topiclength] = '\0';
  char message[length + 1];
  for (unsigned int i = 0; i < length; i++)
  {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  streamString = "Topic: " + String(topicValue) + "  Payload: " + String(message);

  StaticJsonDocument<BUFFER_SIZE> json = bootstrapManager.parseQueueMsg(topic, payload, length);

  if (!justStarted)
  {
    if (strcmp(topic, PUMP_CMD) == 0)
    {
      processPumpOnOff(json);
    }
    else if (strcmp(topic, CTRL_SET) == 0)
    {
      processCtrlSet(json);
    }
    else if (strcmp(topic, CTRL_CMD) == 0)
    {
      processCtrlCmd(json);
    }
    else
    {
      // ignore incomming message at startup because old retained messages could be sent agin by the broker
    }
    // print for debug
    // printing is at the end because myPrintln uses MQTT publish thus json object is being corrupted
    myPrintln(streamString);
    streamString = "";
    serializeJson(json, streamString);
    streamString = "Decoded json: " + streamString;
    myPrintln(streamString);
  }
}
void setup()
{
  // retrieve secret and other network information

  // Serial port for debugging purposes
  Serial.begin(115200);
  myPrintln("-----------------------------------------------------------------------");
  myPrintln("Pump management programm started !");
  myPrintln("----------------------------------");
  myPrintln();
  if (debug)
  {
    myPrintln(AUTHOR);
    myPrint("SSID: ");
    myPrintln(SSID);
  }
  // initialize digital pin relayIn1Pin as an output.
  pinMode(relayIn1Pin, OUTPUT);
  pumpOFF();
  datePowerUp = "unknown";
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(LedPin, ledChannel);
  ledcWrite(ledChannel, 255); // switch ON LED to show startup of the controller
  stateLed = true;
  // stacks reset cause and last event
  pushEvent(verbose_reset_reason());
  //
  // Datastore management
  // try to read a datastore file
  indexDSFile = 0; // first DS file saving will be with file index = 0
  if (readDS(true) != NO_FILE_ERROR)
  {
    // file does not exists or integrity error
    // Then init datastore information
    devicePowerUp = 1;
    currentDeviceActivity = max(millis() / 1000, 1UL);
    deviceActivity_1 = 0;
    deviceActivity_2 = 0;
    cumulativeDeviceActivity = millis() / 1000;
    cumulativeDeviceActivityAtPowerOn = 0;
    resetPumpRunningTimeInfo();
    saveDS(); // save datastore
    lastPumpControlerEvent = "DS file not found or integrity error";
    Serial.println(lastPumpControlerEvent);
    pushEvent(lastPumpControlerEvent);
  }
  else
  {
    // assess if it is a double short power ON sequence
    forceWebServer = ((deviceActivity_1 < shortPowerOn) && (deviceActivity_1 > 0) && (deviceActivity_2 < shortPowerOn) && (deviceActivity_2 > 0));
    // update power on information
    if (forceWebServer)
    {
      Serial.println("Double short power off detected, enter WiFi configuration by Web server");
      currentDeviceActivity = 120; // takes into account the webserver activity because as the ESP will be restarted DS will not be updated - typical duration 120 s
    }
    else
    {
      currentDeviceActivity = max(millis() / 1000, 1UL); // takes into account current elapsed time
    }
    cumulativeDeviceActivityAtPowerOn = cumulativeDeviceActivity;
    cumulativeDeviceActivity += currentDeviceActivity;
    pushEvent(lastSavedEvent);
    lastPumpControlerEvent = "nominal startup - previous state retrieved";
    // save immediatly datastore for the case of a short power ON
    saveDS(); // save datastore
    Serial.println(lastPumpControlerEvent);
    pushEvent(lastPumpControlerEvent);
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
  // switch ON or OFF the pump accordingly with last known power state (or default state)
  if (pumpIsON)
  {
    delay(2000); // delay to avoid very fast ON/OFF/ON sequence for relay and pump protection
    pumpON();
  }
  else
  {
    // no delay required as the pump has already been switched OFF at software init
    pumpOFF();
  }
  //
  // bootstrap manager setup
  fastDisconnectionManagement = true; // used to enable watch dog management event in case of WiFi connection issue
  bootstrapManager.bootstrapSetup(manageDisconnections, manageHardwareButton, callback);
  wifiDisconnectCount = 0; // initialize a counter for WiFi disconnection events
  if (debug)
    readSetup(); // for debug print of setup file afetr initi
  printWiFiStatus();

  ledcWrite(ledChannel, 0); // swith OFF LED to show end of init phase
  stateLed = false;
  if (debug)
  {
    myPrintln(mqttIP); // debug
    myPrintln(mqttPort);
    myPrintln(mqttuser);
    myPrintln(mqttpass); // debug end
  }

  // DS18B20 temp sensor init
  initDS18B20();
  getDS18B20Readings();
  //
  // init water temperature reading
  getDS18B20Readings();
  Serial.printf("DS18B20 sensor initial temperature = %.2f ºC \n", temperature_Celsius);
  currentWaterTemp = temperature_Celsius;
  filteredWaterTemp = temperature_Celsius;
  refWaterTemp = temperature_Celsius;   // used for heat automation decision
  referenceTemp = temperature_Celsius;  // used for activation window duration definition and pump mode decision 
  // init time management
  initTimeManagement();
  datePowerUp = dateAndTime();
  lastUpdateTime = myTZ.now(); // init tag time corresponding to pump state initial setting
  lastPumpControlerEvent = "Pump controler started";
  saveDS(); // save datastore
  pushEvent(lastPumpControlerEvent);

  printWiFiStatus();
  justStarted = true; // allows specific actions at startup in the main loop
  myPrintln("End of setup phase");

  //  init watch dog
  esp_task_wdt_init(WDT_TIMEOUT, true); // set watch dog time out and enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);               // add current thread to WDT watch
  Serial.println("Watch dog enabled");
  // init other variables
  slotCounter = 0;
  lastLoopTime = millis();
  initSavedTime(lastLoopTime);
  scheduledPumpRunningTime = expectedPumpRunningTime();
  currentMonth = myTZ.month();
  currentYear = myTZ.year();
  //
} // end of setup code

void loop()
{

  // boostarpManagere call for OTA and MQTT management
  bootstrapManager.bootstrapLoop(manageDisconnections, manageQueueSubscription, manageHardwareButton);
  if (stateChanged)
  {
    sendStatusInfo();
    stateChanged = false;
  }
  loopTime = millis();
  if (loopTime < lastLoopTime)
  {
    // a rollover of millis() function has been detected
    // this has actually no impact on computations such as (loopTime - lastShortTime) that still deliver a correct result
    // however this event will be reported for troubleshooting in required
    lastPumpControlerEvent = "millis timer rollover after" + String(lastLoopTime);
    pushEvent(lastPumpControlerEvent);
  }
  // save loop time for further rollover detection
  lastLoopTime = loopTime;
  // short action block
  if (((loopTime - lastShortTime) >= shortPeriod) || justStarted)
  {
    lastShortTime = loopTime;
    blinkLed();
    // processes water temperature reference measurements when triggered
    if (triggerRefMeasurement && ((loopTime - triggerTime) > waterFlowDelay))
    {
      getDS18B20Readings();
      refWaterTemp += temperature_Celsius;
      refMeasurementCounter++;
      if (refMeasurementCounter >= maxRefMeasumentCount)
      {
        refWaterTemp /= maxRefMeasumentCount;
        triggerRefMeasurement = false;
        if (!pumpShouldBeOn)
          pumpOFF();    // switches OFF immediatly the pump if it should not be ON for other reason to avoid 1 mn in excess count of the running time
        // computes water temp drift in °C/hour and publish information through MQTT
        hourWaterTempDrift = (refWaterTemp - lastRefWaterTemp) / float(loopTime - lastRefTime) * float(hourlyPeriod);
        bootstrapManager.publish(WATER_TEMP_DRIFT, helper.string2char(String(hourWaterTempDrift, 2)), true);
        bootstrapManager.publish(WATER_TEMP_REF, helper.string2char(String(refWaterTemp, 1)), true);
        lastRefWaterTemp = refWaterTemp;
        lastRefTime = loopTime;
      }
    }
    esp_task_wdt_reset(); // resets watch dog
  }
  // minute ation loop
  // information refresh management and pump management
  if (((loopTime - lastRefreshTime) >= refreshPeriod) || justStarted)
  {
    // its time to refresh information
    lastRefreshTime = loopTime; // save current time
    myPrint("Processor Temperature: ");
    // Convert raw processor temperature in F to Celsius degrees
    myPrint((temprature_sens_read() - 32) / 1.8);
    myPrintln(" C");
    //
    getDS18B20Readings();
    if (debug)
    {
      Serial.printf("DS18B20 sensor temperature = %.2f ºC \n", temperature_Celsius);
      myPrint("millis: ");
      myPrintln(millis());
    }
    currentWaterTemp = temperature_Celsius;
    // computes filtered water temperature to avoid mode changes on spurious temperature values
    if (justStarted)
    {
      filteredWaterTemp = currentWaterTemp;
      lastRefWaterTemp = currentWaterTemp;
      lastRefTime = loopTime;
      computeMeanDayPumpTime(); // this information is not stored in a file and must be computed at power on, after that it is computed once a day
    }
    else
    {
      filteredWaterTemp += filterCoef * (currentWaterTemp - filteredWaterTemp);
    }
    // defines referenceTemp used for pump mode decision, PAC automation and pump running time scheduling
    if (ctrlTest)
    {
      referenceTemp = waterTempTest;
      PACreferenceTemp = waterTempTest;
    }
    else
    {
      referenceTemp = filteredWaterTemp;
      PACreferenceTemp = refWaterTemp;
    }
    // manages reference water temperature measurment request
    if (requestRefMeasurement)
      if (!pumpManualOff && !pumpContinuousOff)
      {
        // processes request only if the pump is not halted for maintenance
        // otherwised this will be delayed until this permanent or momentary stop ends
        triggerRefMeasurement = true;
        requestRefMeasurement = false;
        triggerTime = loopTime;    // saves the current time
        refWaterTemp = 0;          // initialize an accumulator for water temperature measurement
        refMeasurementCounter = 0; // resets measurement counter
        pumpON();
      }
    // Ping gateway to add presence on the routing table,
    // command is synchrounous and adds a bit of lag to the loop
    if (!pingESP.ping(WiFi.gatewayIP()))
    {
      // no response to ping
      myPrintln("No response to ping");
    }
    // checks pump relay state and ajusts it if not correct
    if (pumpIsON)
    {
      if (!digitalRead(relayIn1Pin))
        pumpON();
    }
    else
    {
      if (digitalRead(relayIn1Pin))
        pumpOFF();
    }
    // manage pump mode change decision based on filtered water temperature measured continuously reflecting temperature near the surface
    decidePumpMode();
    bootstrapManager.publish(WATER_TEMP, helper.string2char(String(currentWaterTemp)), true);
    //    bootstrapManager.publish(PUMP_STATE, (pumpIsON) ? helper.string2char(ON_CMD) : helper.string2char(OFF_CMD), true);

    // pump management block
    // # 1 - identify any day change, even in the case of an unnatended power OFF at midnight
    readDay = myTZ.day();   // extract current day number with ezTime library
    readHour = myTZ.hour(); // extract curent hour with ezTime library
    // check day number change and take it into account after 3 hours AM
    if ((readDay != currentDay) && (readHour >= 3))
    {
      // a new day has been detected (@ 3 PM or later in case of shut down)
      //
      lastPumpControlerEvent = "Nouveau jour de filtration détecté (index : " + String(indexPumpTime) + ")";
      pushEvent(lastPumpControlerEvent);
      // records pump time for the ending day
      recordPumpTime();
      // computes once a day the filtering time needed, function of water temperature and pump mode
      scheduledPumpRunningTime = expectedPumpRunningTime();
      // resets pump running time counters
      pumpDayRunningTime = 0;
      pumpRunningTimeManualOff = 0;
      pumpRunningTimeManualOn = 0;
      pumpManualOn = false;
      pumpManualOff = false;
      // then save current day, month and year number
      currentDay = readDay;
      currentMonth = myTZ.month();
      currentYear = myTZ.year();
      myPrint("current day, month, year: ");
      myPrint(currentDay);
      myPrint(",");
      myPrint(currentMonth);
      myPrint(",");
      myPrintln(currentYear);
    }
    // manage pump activation
    // computes start and stop instants of morning and evening windows in seconds since Jan 1st 1970
    computePumpWindowTime();
    // retrieve current time in seconds since Jan 1st 1970
    readTime = myTZ.now();
    //
    // sets activation state
    // morning window state is defined only by begin and end instants
    pumpMorningIsRunning = ((readTime >= pumpWindowT1b) && (readTime < pumpWindowT1e));
    // evening window is defined by begin instant for the start and total day running time for the end
    pumpEveningIsRunning = ((readTime >= pumpWindowT2b) && (pumpDayRunningTime < scheduledPumpRunningTime * 2));
    if (pumpWindowIsRunning)
      pumpManualOn = false; // reset any manual ON state when a regular activation window is running
    if (!pumpWindowIsRunning && !pumpContinuousOn)
      pumpManualOff = false; // reset any manual OFF state when a regular activation window is not running and continuous running is not activated
    //
    // manage heat pump automation
    if (PACAutomation && (PACreferenceTemp < (PACTempTarget - PACTempStartOffset)))
      triggerPAC = true; // pump activation will be triggered, the trigger will remain true until the temperature rises above the stop limit or if PACAutomation is reseted
    if (PACreferenceTemp > (PACTempTarget + PACTempStopOffset))
      triggerPAC = false;
    if (!PACAutomation)
      triggerPAC = false;
    // note: at the end of the summer season, when the heat pump is stopped, PACAutomation should be set to false to avoid unnecessary pump filtering activation
    // triggerPAC is used in the macro pumpShouldBeOn

    // increments pump running time
    uint32_t timeIncrement = readTime - lastUpdateTime; // elapsed time in seconds since last update
    if (timeIncrement > 100)
    {
      myPrint("Warning: timeIncrement > 100s : ");
      myPrintln(timeIncrement);
      myPrint("readTime: ");
      myPrintln(readTime);
      myPrint("lastUpdateTime: ");
      myPrintln(lastUpdateTime);
      timeIncrement = refreshPeriod / 1000; // use nominal value as default value
    }
    if (pumpIsON)
    {
      pumpDayRunningTime += timeIncrement;
      cumulativePumpTime += timeIncrement; // cumulative time in s
    }
    if (pumpManualOn)
      pumpRunningTimeManualOn += timeIncrement;
    if (pumpManualOff)
      pumpRunningTimeManualOff += timeIncrement;
    lastUpdateTime = readTime;
    //
    // check in a pump manual mode is running over the time limit and resets if true
    if (pumpRunningTimeManualOn > maxManualTime)
    {
      pumpManualOn = false;
      pumpRunningTimeManualOn = 0;
      lastPumpControlerEvent = "reset auto de la marche forcée pompe";
      pushEvent(lastPumpControlerEvent);
    }
    if (pumpRunningTimeManualOff > maxManualTime)
    {
      pumpManualOff = false;
      pumpRunningTimeManualOff = 0;
      lastPumpControlerEvent = "reset auto de l'arrêt forcé pompe";
      pushEvent(lastPumpControlerEvent);
    }
    //
    // switch ON or OFF the pump as required
    if (pumpShouldBeOn)
    {
      if (!pumpIsON)
      {
        pumpON(); // switch ON pump if it was not ON
        lastPumpControlerEvent = "mise en marche pompe";
        pushEvent(lastPumpControlerEvent);
      }
    }
    else
    {
      if (pumpIsON)
      {
        pumpOFF();
        lastPumpControlerEvent = "arrêt pompe";
        pushEvent(lastPumpControlerEvent);
      }
    }
    // end of pump management block
    //
    // publish event from event stack
    publishLastEvents();
    // publish slow refrash rate informtion, allocated per 1 minute time slot to limit MQTT data flow
    switch (slotCounter)
    {
    case 0:
      sendTimeInfo(); // publish controller running time information
      break;
    case 1:
      if (codeTest)
      {
        Serial.println(functionnalTest());
      }
      break;
    case 2:
      // context saving management
      saveCurrentContext();
      lastSaveTime = loopTime; // save current loop time
      events();                // update local time with NTP server if required
      break;
    case 4:
      sendPumpInfo(); // publish pump running time information
      break;
    case 6:
      sendWifiInfo(); // publish WiFi link information
      break;
    case 8:
      sendStatusInfo(); // publish state information every 10 minutes, event if state not changed
      break;
    default:
      // do nothing
      break;
    }
    slotCounter = ++slotCounter % timeSlotNb; // increments and clips slot counter

  } // end of refresh actions block

  if ((loopTime - lastHourTime) >= hourlyPeriod)
  {
    // refresh MEAN_PUMP_RUNNING_TIME information because this daily value seems to be lost by the broker after several hours
    bootstrapManager.publish(MEAN_PUMP_RUNNING_TIME, helper.string2char(String(meanDayPumpTime)), true);
    lastHourTime = loopTime;
    // manages water temperature reference measurement with water flow
    requestRefMeasurement = true;
  }
  //
  justStarted = false;
}
//------------------------------------------------------------------------------------------------
// retrieved last saved context in flash memory
// enum contextRetrieveStatus retrieveLastContext()
//{
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
//}
//------------------------------------------------------------------------------------------------
// save the current context in a file in flash memory
void saveCurrentContext()
{

  myTime = millis();
  uint32_t elapsedTime = myTime - lastSaveTime;
  if (elapsedTime < 0)
  {
    // millis function has overflown
    elapsedTime += 3600000;                                     // add an offset of 1 hour to provide a positive integer
    cumulativeDeviceActivity += 4291367 + (elapsedTime / 1000); // add roll out value = 2^32 / 1000 s - 3600 s
  }
  else
  {
    cumulativeDeviceActivity += elapsedTime / 1000;
  }
  currentDeviceActivity = cumulativeDeviceActivity - cumulativeDeviceActivityAtPowerOn; // refresh device activity for the current power up period
  saveDS();                                                                             // save context information
  if (debug)
  {
    myPrint("Check DS reading: ");
    myPrintln(readDS());
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
void printWiFiStatus()
{
  myPrintln("WiFi status");
  myPrint("WiFi connected to SSID: ");
  myPrintln(WiFi.SSID());
  rssi = WiFi.RSSI(); // save current rssi value
  // print the received signal strength:
  myPrint("signal strength (RSSI):");
  myPrint(rssi);
  myPrintln(" dBm");
  myPrint("IP address: ");
  myPrintln(WiFi.localIP());
}
//----------------------------------------------------------------------------------
void getDS18B20Readings()
{
  sensors.requestTemperatures();
  temperature_Celsius = sensors.getTempCByIndex(0);
  temperature_Fahrenheit = sensors.getTempFByIndex(0);
}
//----------------------------------------------------------------------------------
// init temp sensor configuration
void initDS18B20()
{
  sensors.begin();
  // Get ID of first sensor (at index 0)
  sensors.getAddress(deviceAddress, 0);

  // By default configuration and alarm/userdata registers are also saved to EEPROM
  // when they're changed. Sensors recall these values automatically when powered up.

  // Turn OFF automatic saving of configuration and alarm/userdata registers to EEPROM
  sensors.setAutoSaveScratchPad(false);

  // Change configuration and alarm/userdata registers on the scratchpad

  sensors.setResolution(deviceAddress, tempResolution);

  // Save configuration and alarm/userdata registers to EEPROM
  sensors.saveScratchPad(deviceAddress);
}
//-----------------------------------------------------------------------------------------------------------
// filtering pump activation
void pumpON()
{
  digitalWrite(relayIn1Pin, LOW); // switch ON relay 1 (In1 relay input active low)
  digitalWrite(LedPin, HIGH);     // switch ON internal Led
  pumpIsON = true;
  stateChanged = true;
}
void pumpOFF()
// filtration pump stop
{
  digitalWrite(relayIn1Pin, HIGH); // switch OFF relay 1 (HIGH is the voltage level)
  digitalWrite(LedPin, LOW);       // switch OFF internal Led
  pumpIsON = false;
  stateChanged = true;
}
//------------------------------------------------------------------------------------------------------------
// try to read datastore file and return true if successfull
boolean readDS(boolean initValue)
// retrieve all indexed files, check their integrity with hash code
// use the most recent file with hash code correct
// the most recent file is defined by the highest value of cumulative running time
{
  enum fileErrorCode fileError = OPENING_ERROR;
  int foundIndex = -1;
  int indexFile;
  uint32_t cumulativeTime = 0;
  uint32_t retrievedTime;
  String DSfileName;
  String selectedDSfileName;
  for (indexFile = 0; indexFile < nbIndexDS; indexFile++)
  // search the more recent DS file with no integrity issue
  {
    // retrieve DS file informations

    if (indexFile == 0)
    {
      DSfileName = "DS.json";
    }
    else
    {
      DSfileName = "DS" + String(indexFile) + ".json";
    }
    myPrintln("Reading file: " + DSfileName);
    DynamicJsonDocument DSdoc = bootstrapManager.readSPIFFS(DSfileName);
    //    const char *header = DSdoc["header"];
    if (DSdoc.containsKey("header"))
    //    if (header)
    {
      // file exists and contains a header, then check json validity and integrity hash code
      //
      // check for any json deserialization error
      // in case of deserialization error, readSPIFFS returns a default json document {"value":"ERROR"}
      const char *jsonError = DSdoc[VALUE];
      if (jsonError)
      {
        lastPumpControlerEvent = "Json deserialization error in data store";
        Serial.println(lastPumpControlerEvent);
        pushEvent(lastPumpControlerEvent);
        //      publishLastEvents();
        myPrintln(lastPumpControlerEvent);
        fileError = JSON_ERROR;
      }
      else
      {
        // no json deserialization error
        // check data integrity with hash code
        String readHashDS = helper.getValue(DSdoc["HC"]);
        myPrintln("Read hash code: " + readHashDS);
        DSdoc["HC"] = "null";               // hash code is calculated on the data including the CS, then this one must be set to a fix value before
        String hashDS = hashPayload(DSdoc); // computes the hash code
        if ((hashDS == readHashDS) || ignoreHash)
        {
          myPrintln("file hash code OK");
          retrievedTime = helper.getValue(DSdoc["CDA"]).toInt();
          if (retrievedTime > cumulativeTime)
          {
            cumulativeTime = retrievedTime;  // save max value found
            foundIndex = indexFile;          // save current index value
            selectedDSfileName = DSfileName; // save current file name
          }
        }
        else
        {
          lastPumpControlerEvent = "Integrity error in data store";
          Serial.println(lastPumpControlerEvent);
          pushEvent(lastPumpControlerEvent);
          //      publishLastEvents();
          myPrintln(lastPumpControlerEvent);
          fileError = INTEGRITY_ERROR;
        }
      } // end of json OK block
    }   // end of header found block
  }     // end of multiple DS files read
  if (foundIndex >= 0)
  // a valid DS file has been found
  {
    fileError = NO_FILE_ERROR;
    if (initValue)
    {
      // init global parameter when readDS at power ON
      // so retrieve number of power up and running time information from the most recent file

      //
      myPrintln("Reading selected file: " + selectedDSfileName);
      DynamicJsonDocument DSdoc = bootstrapManager.readSPIFFS(selectedDSfileName);
      setInternalState(DSdoc); // Init time parameters and state variables from DS file
    }
  }
  else
  {
    // no valid DS file found
    // then fileError contains INTEGRITY_ERROR or OPENING_ERROR
    lastPumpControlerEvent = "No valid DS file found";
    pushEvent(lastPumpControlerEvent);
    //    publishLastEvents();
    myPrintln(lastPumpControlerEvent);
  }
  return fileError;
}
//------------------------------------------------------------------------------------------------------------
// save current context in datastore file
void saveDS()
// retrieve last context index
// increments index
// save context into a json document
// computes hash code of the document and add it to the document
// save document in new indexed file
// in case of power off during file saving, the next reading will use the most recent file with correct hash code
{
  DynamicJsonDocument DSdoc(1024);
  JsonObject root = DSdoc.to<JsonObject>(); // clears DSdoc and convert it to a Json object
  DSdoc["header"] = "Header";
  // controller running time info
  DSdoc["CDA"] = cumulativeDeviceActivity;
  DSdoc["LDA_1"] = currentDeviceActivity;
  DSdoc["LDA_2"] = deviceActivity_1;
  DSdoc["DPU"] = devicePowerUp;
  DSdoc["DLE"] = lastPumpControlerEvent + " @" + myTZ.dateTime("d/m/y - H:i:s");
  // pump running time info
  JsonObject PRTinfo = DSdoc.createNestedObject("PRT"); // pump running time information
  PRTinfo["CPM"] = currentPumpMode;
  PRTinfo["PMON"] = pumpManualOn;
  PRTinfo["PMOFF"] = pumpManualOff;
  PRTinfo["PCON"] = pumpContinuousOn;
  PRTinfo["PCOFF"] = pumpContinuousOff;
  PRTinfo["MOFFT"] = pumpRunningTimeManualOff;
  PRTinfo["MONT"] = pumpRunningTimeManualOn;
  PRTinfo["DPT"] = pumpDayRunningTime;
  PRTinfo["CDN"] = currentDay;
  PRTinfo["TTS"] = triggerTS;
  PRTinfo["TTW"] = triggerTW;
  PRTinfo["TMS"] = triggerTMS;
  PRTinfo["TSC"] = TScounter;
  PRTinfo["TMC"] = TWcounter;
  PRTinfo["TMSC"] = TMScounter;
  PRTinfo["CPT"] = cumulativePumpTime;
  PRTinfo["PION"] = pumpIsON;
  PRTinfo["IPT"] = indexPumpTime;
  JsonArray dayPumpTimeArray = PRTinfo.createNestedArray("DPTA");
  for (int index = 0; index < 7; index++)
  {
    dayPumpTimeArray.add(dayPumpTime[index]);
  }
  PRTinfo["PACT"] = PACTempTarget;
  PRTinfo["PACA"] = PACAutomation;
  // computes and stores hash code for data integrity monitoring
  DSdoc["HC"] = "null";               // hash code is calculated on the data including the CS, then this one must be set to a fix value before
  String hashDS = hashPayload(DSdoc); // computes the hash code
  DSdoc["HC"] = String(hashDS);

  // save the file
  String DSfileName;
  if (indexDSFile == 0)
  {
    DSfileName = "DS.json"; // keep file name with no index for legacy test
  }
  else
  {
    DSfileName = "DS" + String(indexDSFile) + ".json";
  }
  myPrintln("Saving file: " + DSfileName);
  bootstrapManager.writeToSPIFFS(DSdoc, DSfileName);
  indexDSFile = (indexDSFile + 1) % nbIndexDS; // increments index for DS file name construction
  // publish DSdoc on MQTT
  bootstrapManager.publish(TECH_INFO, root, true);
}

//------------------------------------------------------------------------------------------------------------
// read setup file
void readSetup()
{
  DynamicJsonDocument doc = bootstrapManager.readSPIFFS("setup.json");
}
//------------------------------------------------------------------------------------------------------------
// send pump controler running time information on MQTT
void sendTimeInfo()
{
  JsonObject root = bootstrapManager.getJsonObject();

  root["Time"] = serialized(jsonString(dateAndTime()));
  root["Last_power_ON_date_and_time"] = serialized(jsonString(datePowerUp));
  JsonObject Power_ON = root.createNestedObject("Power_ON");

  Power_ON["Cumulative"] = cumulativeDeviceActivity;
  Power_ON["Current"] = currentDeviceActivity;
  Power_ON["Previous"] = deviceActivity_1;
  Power_ON["Before_previous"] = deviceActivity_2;
  Power_ON["Number"] = devicePowerUp;

  bootstrapManager.publish(CONTROLER_RUNNING_TIME, root, true);
}
//------------------------------------------------------------------------------------------------------------
// send micro controler technical information on MQTT
void sendMicroInfo()
{
  JsonObject root = bootstrapManager.getJsonObject();
  root["Free_heap_KB"] = ESP.getFreeHeap() / 1024;
  root["Free_flash_KB"] = ESP.getFreeSketchSpace() / 1024;
  root["CPU_freq_MHz"] = ESP.getCpuFreqMHz();
  root["Flash_size_KB"] = ESP.getFlashChipSize() / 1024;
  root["MAC_addr"] = serialized(jsonString(WiFi.macAddress()));
  root["SDK_version"] = serialized(jsonString(ESP.getSdkVersion()));
  bootstrapManager.publish(TECH_INFO, root, true);
}
//------------------------------------------------------------------------------------------------------------
// send pump running time information on MQTT
void sendPumpInfo()
{
  JsonObject root = bootstrapManager.getJsonObject();

  root["Time"] = serialized(jsonString(dateAndTime()));
  root["Filtered_WT"] = myRound(filteredWaterTemp, 2);
  root["PAC_WT"] = myRound(PACTempTarget, 1);
  root["TEST_WT"] = myRound(waterTempTest, 1);
  JsonObject PRT = root.createNestedObject("PRT"); // PTR stands for 'Pump Running Time'
  // computes and records values in minutes, from values in seconds
  PRT["Day"] = pumpDayRunningTime / secondsPerMinute;                           // current pump running time in minutes
  PRT["PreviousDay"] = dayPumpTime[(indexPumpTime + 6) % 7] / secondsPerMinute; // pump running time of previous day, in minutes
  PRT["Scheduled"] = scheduledPumpRunningTime / secondsPerMinute;               // scheduled pump running time in minutes
  // computes and records other values
  PRT["Cumulative"] = cumulativePumpTime / secondsPerHour; // cumulative running time in hours
  PRT["Mean_Day"] = meanDayPumpTime;                       // mean pump running time over 7 days, in minutes

  bootstrapManager.publish(PUMP_INFO, root, true);
}
//------------------------------------------------------------------------------------------------------------
// send pump controler status information on MQTT
void sendStatusInfo()
{
  JsonObject root = bootstrapManager.getJsonObject();

  root["Time"] = serialized(jsonString(dateAndTime()));
  // records pump status
  JsonObject Status = root.createNestedObject("STATUS"); //
  Status["Pump"] = (pumpIsON) ? ON_CMD : OFF_CMD;
  Status["Mode"] = serialized(jsonString(pumpModeVerbose[currentPumpMode]));
  Status["Window"] = (pumpWindowIsRunning) ? ACTIVE : INACTIVE;
  Status["ON_man"] = (pumpManualOn) ? ACTIVE : INACTIVE;
  Status["OFF_man"] = (pumpManualOff) ? ACTIVE : INACTIVE;
  Status["Test_mode"] = (ctrlTest) ? ACTIVE : INACTIVE;
  Status["PAC_auto"] = (PACAutomation) ? ACTIVE : INACTIVE;
  Status["Cont_run"] = (pumpContinuousOn) ? ACTIVE : INACTIVE;
  Status["Cont_stop"] = (pumpContinuousOff) ? ACTIVE : INACTIVE;
  Status["Ref_mes"] = (triggerRefMeasurement) ? ACTIVE : INACTIVE;
  bootstrapManager.publish(CONTROLER_STATE, root, true);
}
//------------------------------------------------------------------------------------------------------------
// send WiFi information on MQTT
void sendWifiInfo()
{
  JsonObject root = bootstrapManager.getJsonObject();

  root["Time"] = serialized(jsonString(dateAndTime()));

  JsonObject WifiInfo = root.createNestedObject("Wifi_info");

  WifiInfo["Wifi_disconnect"] = wifiDisconnectCount;
  WifiInfo["RSSI"] = WiFi.RSSI();
  WifiInfo["SSID"] = serialized(jsonString(WiFi.SSID()));
  WifiInfo["Channel"] = WiFi.channel();

  bootstrapManager.publish(WIFI_INFO, root, true);
}
//------------------------------------------------------------------------------------------------------------
// add "" arround the input string
String jsonString(String input)
{
  return "\"" + input + "\"";
}

//--------------------------------------------------------------------------
// provides a SHA-256 hash of the input json document, the output is formatted into a string of hexadecimal values
String hashPayload(DynamicJsonDocument jsonDocument)
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
  mbedtls_md_update(&ctx, (const unsigned char *)payload, payloadLength);
  mbedtls_md_finish(&ctx, shaResult);
  mbedtls_md_free(&ctx);

  for (int i = 0; i < sizeof(shaResult); i++)
  {
    char str[3];

    sprintf(str, "%02x", (int)shaResult[i]);
    output += str;
  }
  myPrintln("Computed hash code: " + output);
  return output;
}
//--------------------------------------------------------------------------------
// push and event message in the event stack and set stack index
void pushEvent(String event)
{
  // add a time tag to the event message - only time, not date because Home assistant add a time tag
  String tagEvent = "#" + String(++eventCounter) + " " + event + " @" + myTZ.dateTime("H:i:s");
  // stores event message in the stack and then increment the write index
  eventStack[indexEventStackW] = tagEvent;
  indexEventStackW = (++indexEventStackW) % stackSize;
  // old messages are overwritten and if the write pointer reaches the read pointer, this one is incremented
  if (indexEventStackW == indexEventStackR)
  {
    indexEventStackR = (++indexEventStackR) % stackSize;
  }
  // note: at the end of the function, the write index is pointing on the next saving position (empty or oldest message to be read)
  // at startup, both index = 0 and the 1st message is writen at index 0
}

String pullEvent()
// pull the oldest message in the event stack
{
  String tagEvent = "";
  // if the read pointer is equal to the write pointer, no unread message is available
  if (indexEventStackR == indexEventStackW)
  {
    return tagEvent;
  }
  else
  {
    tagEvent = eventStack[indexEventStackR];
    indexEventStackR = (++indexEventStackR) % stackSize;
    return tagEvent;
  }
  // note: at the end of the function, the read index is pointing on the next message position
  // at startup both index = 0 and no message can be read. This is coherent with the stack being empty
}

boolean stackIsEmpty()
// return true if the event stack is empty
{
  return (indexEventStackR == indexEventStackW);
}

// print stack content and index values
void printStack()
{
  myPrintln("Stack information - Index R: " + String(indexEventStackR) + " / Index W: " + String(indexEventStackW));
  for (int i = 0; i < stackSize; i++)
  {
    myPrintln(String(i) + " " + eventStack[i]);
  }
}

//--------------------------------------------------------------------------------
/*
  if WiFi is connected, retrieve the oldest available events in the stack and publish it
*/
void publishLastEvents()
{
  if (bootstrapManager.getWifiQuality() > 30)
  // WiFi not disconnected and RSSI index > 30%
  {
    if (debug)
      printStack();
    if (!stackIsEmpty())
    {
      String tagEvent = pullEvent();
      if (debug)
        myPrintln("Publishing: " + tagEvent);
      if (tagEvent.length() > 0)
      {
        // a message is available then publish it
        // tagEvent = "> " + tagEvent;
        bootstrapManager.publish(PUMP_CONTROLER_EVENT, helper.string2char(tagEvent), true);
      }
    }
  }
}
//-------------------------------------------------------------------------------------------
/* process pump ON/OFF messages transmitted by Home Assistant
 */
void processPumpOnOff(StaticJsonDocument<BUFFER_SIZE> json)
{
  myPrintln("Pump CMD received");

  String pumpCmd = json[VALUE];
  if (pumpCmd == ON_CMD)
  {
    if (pumpContinuousOff)
    {
      // for security reasons, any ON command is ignored if the pump is in permanent OFF mode
      lastPumpControlerEvent = "commande marche pompe ignorée en mode \"arrêt permanent\"";
      pumpManualOn = false;
    }
    else
    {
      if (!pumpIsON)
      {
        // the pump was not running, so a manual ON command makes sense
        if (pumpWindowIsRunning)
        {
          // the ON state is the nominal state, then does not save a manual ON
          lastPumpControlerEvent = "commande marche pompe";
          pumpManualOn = false;
        }
        else
        {
          // this is a pump activation outside a nominal activation window
          pumpManualOn = true; // save the information that a manual activation is requested
          lastPumpControlerEvent = "commande marche forcée pompe";
        }
        pumpManualOff = false;
        pumpON();
      }
      else
      {
        //  the pump was already ON, then the command is ignored
        lastPumpControlerEvent = "commande marche forcée pompe sans objet";
      }
    }
  } // end of ON_CMD block
  else if (pumpCmd == OFF_CMD)
  {
    if (pumpIsON)
    {
      // the pump is running, so an OFF command makes sense
      if (triggerRefMeasurement || pumpContinuousOn)
      {
        // command is ignored if a reference measurement is running or a permanent pump activation is set
        lastPumpControlerEvent = "commande arrêt pompe ignorée (mesure en cours ou activation continue)";
      }
      else
      {
        // the OFF command is accepted
        if (pumpWindowIsRunning)
        {
          pumpManualOff = true; // save the information that a manual stop is requested
          lastPumpControlerEvent = "commande arrêt forcé pompe";
        }
        else
        {
          // the OFF state is the nominal state when an activation window is not running, then does not save a manual OFF
          pumpManualOff = false;
          lastPumpControlerEvent = "commande arrêt pompe";
        }
        pumpManualOn = false;
        pumpOFF();
      }
    }
    else
    {
      // the pump was already OFF, so the OFF command is ignored
      lastPumpControlerEvent = "commande arrêt forcé pompe sans objet";
    }
  } // end of OFF_cmd block
  else if (pumpCmd == ignore_CMD)
  {
    // do nothing
    lastPumpControlerEvent = "comand 'ignored'";
  }
  else if (pumpCmd == RESET_CMD)
  {
    // reset pump running time data
    resetPumpRunningTimeInfo();
    lastPumpControlerEvent = "commande Reset info timing pompe";
  }
  else
  {
    lastPumpControlerEvent = "commande pompe inconnue : " + pumpCmd;
  }
  myPrintln(lastPumpControlerEvent);
  pushEvent(lastPumpControlerEvent);
} // end of processPumpOnOff routine
//-------------------------------------------------------------------------------------------
/* process controller SET commands messages transmitted by Home Assistant
 */
void processCtrlSet(StaticJsonDocument<BUFFER_SIZE> json)
{
  myPrint("Controller setting command received: ");
  boolean validSetCmd = false;

  // test setting keywords

  // process any SET WINDOW command
  if (json.containsKey(WINDOW))
  {
    myPrintln(WINDOW); // print the command keyword
    // extract windows parameters
    JsonArray readWindow = json[WINDOW];
    if (readWindow.size() != 4)
    {
      lastPumpControlerEvent = "Invalid window parameter list: " + String(readWindow.size()) + " parameters";
      pushEvent(lastPumpControlerEvent);
      myPrintln(lastPumpControlerEvent);
    }
    else
    {
      uint8_t H1 = (int)readWindow[0];
      uint8_t M1 = (int)readWindow[1];
      uint8_t H2 = (int)readWindow[2];
      uint8_t M2 = (int)readWindow[3];
      // check values and set corresponding  parameters if correct
      boolean valueOK = true;
      if (H1 <= 12)
        morningWindowStartHour = H1;
      else
        valueOK = false;
      if (M1 < 60)
        morningWindowStartMinute = M1;
      else
        valueOK = false;
      if (H2 <= 22)
        eveningWindowStartHour = H2;
      else
        valueOK = false;
      if (M2 < 60)
        eveningWindowStartMinute = M2;
      else
        valueOK = false;

      lastPumpControlerEvent = "Set Window = " + String(H1) + ":" + String(M1) + " / " + String(H2) + ":" + String(M2);
      if (!valueOK)
        lastPumpControlerEvent += " - errors detected in some input values, they have been ignored";
      pushEvent(lastPumpControlerEvent);
      myPrintln(lastPumpControlerEvent);
      validSetCmd = true;
    }
  }
  // proceses any SET TEMP_TEST command
  if (json.containsKey(TEMP_TEST))
  {
    myPrintln(TEMP_TEST); // print the command keyword
    waterTempTest = (float)json[TEMP_TEST];
    lastPumpControlerEvent = "Set test temperature = " + String(waterTempTest) + " °C";
    pushEvent(lastPumpControlerEvent);
    validSetCmd = true;
  }
  // proceses any SET TEMP_PAC command
  if (json.containsKey(TEMP_PAC))
  {
    myPrintln(TEMP_PAC); // print the command keyword
    PACTempTarget = (float)json[TEMP_PAC];
    lastPumpControlerEvent = "Set heat pump threshold temperature = " + String(PACTempTarget) + " °C";
    pushEvent(lastPumpControlerEvent);
    validSetCmd = true;
  }
  if (!validSetCmd)
  {
    lastPumpControlerEvent = "Unvalid SET command received";
    myPrintln(lastPumpControlerEvent);
    pushEvent(lastPumpControlerEvent);
  }
} // end of processCtrlSet routine
//-------------------------------------------------------------------------------------------
/* process controller CMD commands messages transmitted by Home Assistant
 */
void processCtrlCmd(StaticJsonDocument<BUFFER_SIZE> json)
{
  myPrint("Controller activation command received: ");
  boolean validCmd = false;
  // myPrintln("PAC automation CMD received");
  // test setting keywords

  // process any CMD TEST command
  if (json.containsKey(TEST))
  {
    myPrintln(TEST);
    String ctrlCmd = json[TEST];
    if (ctrlCmd == ON_CMD)
    {
      ctrlTest = true;
      lastPumpControlerEvent = "Controller test mode enabled";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
    else if (ctrlCmd == OFF_CMD)
    {
      ctrlTest = false;
      lastPumpControlerEvent = "Controller test mode disabled";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
    else
    {
      lastPumpControlerEvent = "Controller test mode invalid command";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
  }
  // process any CMD PAC_AUTO command
  if (json.containsKey(PAC_AUTO))
  {
    myPrintln(PAC_AUTO);
    String ctrlCmd = json[PAC_AUTO];
    if (ctrlCmd == ON_CMD)
    {
      PACAutomation = true;
      lastPumpControlerEvent = "Heat pump automatic control enabled";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
    else if (ctrlCmd == OFF_CMD)
    {
      PACAutomation = false;
      lastPumpControlerEvent = "Heat pump automatic control disabled";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
    else
    {
      lastPumpControlerEvent = "Heat pump automatic control invalid command";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
  }
  // process any CMD SEND command
  if (json.containsKey(SEND))
  {
    myPrintln(SEND);
    String ctrlCmd = json[SEND];
    if (ctrlCmd == MICRO)
    {
      lastPumpControlerEvent = "Controller CMD SEND microcontroller information on 'TECH_INFO'";
      sendMicroInfo();
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
  }
  // process any CONT_RUN command
  if (json.containsKey(CONT_RUN))
  {
    myPrintln(TEST);
    String ctrlCmd = json[CONT_RUN];
    if (ctrlCmd == ON_CMD)
    {
      pumpContinuousOn = true;
      lastPumpControlerEvent = "Continuous filtering enabled";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
    else if (ctrlCmd == OFF_CMD)
    {
      pumpContinuousOn = false;
      lastPumpControlerEvent = "Continuous filtering disabled";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
    else
    {
      lastPumpControlerEvent = "Continuous filtering invalid command";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
  }
  // process any CONT_STOP command
  if (json.containsKey(CONT_STOP))
  {
    myPrintln(TEST);
    String ctrlCmd = json[CONT_STOP];
    if (ctrlCmd == ON_CMD)
    {
      pumpContinuousOff = true;
      lastPumpControlerEvent = "Pump permanent OFF enabled";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
    else if (ctrlCmd == OFF_CMD)
    {
      pumpContinuousOff = false;
      lastPumpControlerEvent = "Pump permanent OFF disabled";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
    else
    {
      lastPumpControlerEvent = "Pump permanent OFF invalid command";
      myPrintln(lastPumpControlerEvent);
      pushEvent(lastPumpControlerEvent);
      validCmd = true;
    }
  }
  // process any CMD STATE command
  if (json.containsKey(STATE))
  {
    myPrintln(STATE);
    lastPumpControlerEvent = "Controller CMD STATE command not yet implemented";
    myPrintln(lastPumpControlerEvent);
    pushEvent(lastPumpControlerEvent);
    validCmd = true;
  }
  if (!validCmd)
  {
    lastPumpControlerEvent = "Unvalid controller command received";
    myPrintln(lastPumpControlerEvent);
    pushEvent(lastPumpControlerEvent);
  }
  else
    stateChanged = true; // will be used to publish new state without delay
} // end of processCtrlCmd routine
//------------------------------------------------------------------------------------
// this routine initiaze state parameters from a json document sent by MQTT channel
void processSetStateFromMQTT(StaticJsonDocument<BUFFER_SIZE> json)
{
  char buffer[measureJson(json) + 1];
  serializeJson(json, buffer, sizeof(buffer));
  DynamicJsonDocument jsonDoc(1024);
  DeserializationError error = deserializeJson(jsonDoc, buffer);
  String receivedString;
  if (!error)
  {
    serializeJsonPretty(jsonDoc, receivedString);
    lastPumpControlerEvent = "commande SET/STATE processed" + receivedString;
    pushEvent(lastPumpControlerEvent);
  }
  else
  {
    lastPumpControlerEvent = "commande SET/STATE: error while deserialization";
    pushEvent(lastPumpControlerEvent);
  }
} // end of processSetStateFromMQTT routine
// ------------------------------------------------------------------------------------
// this routine selects the pump mode based on filtered water temp
void decidePumpMode()
{
  enum pumpModes previousPumpMode = currentPumpMode; // save current pump mode

  if (referenceTemp > temperatureThresholdSummer)
  {
    if (triggerTS)
    {
      TScounter = min(++TScounter, counterThreshold); // incrments and clip TScounter
    }
    triggerTS = true;
    triggerTW = false;
    triggerTMS = false;
    TWcounter = 0;
    TMScounter = 0;
  }
  else if (referenceTemp < temperatureThresholdWinter)
  {
    if (triggerTW)
    {
      TWcounter = min(++TWcounter, counterThreshold); // incrments and clip TWcounter
    }
    triggerTW = true;
    triggerTS = false;
    triggerTMS = false;
    TScounter = 0;
    TMScounter = 0;
  }
  else
  {
    if (triggerTMS)
    {
      TMScounter = min(++TMScounter, counterThreshold); // incrments and clip TMScounter
    }
    triggerTMS = true;
    triggerTW = false;
    triggerTS = false;
    TWcounter = 0;
    TScounter = 0;
  }
  // decide mode change if one of the counter reaches the limit
  if (TScounter >= counterThreshold)
  {
    currentPumpMode = summer;
  }
  if (TWcounter >= counterThreshold)
  {
    currentPumpMode = winter;
  }
  if (TMScounter >= counterThreshold)
  {
    currentPumpMode = midSeason;
  }
  if (currentPumpMode != previousPumpMode)
  {
    stateChanged = true; // flag the change so that the new state will be published
  }
}
//---------------------------------------------------------------------------------------------------------
// computes scheduled pump running time based on pump mode and filtered water temperature - value in seconds
uint32_t expectedPumpRunningTime()
{
  if (referenceTemp < temperatureThresholdFrozen)
  {
    return 24 * secondsPerHour; // continuous pump activation if risk of water frozen
  }
  switch (currentPumpMode)
  {
  case summer:
    return uint32_t(referenceTemp / 4. * secondsPerHour); // time in seconds for each of the 2 daily activation windows
    break;
  case winter:
    return uint32_t(winterPumpScheduledTime * secondsPerMinute); // time in seconds for each of the 2 daily activation windows
    break;
  case midSeason:
    return uint32_t(referenceTemp / 6. * secondsPerHour); // time in seconds for each of the 2 daily activation windows
    break;
  default:
    return uint32_t(referenceTemp / 4. * secondsPerHour); // time in seconds for each of the 2 daily activation windows
    break;
  }
}
// init saved time values
void initSavedTime(uint32_t resetValue)
{
  lastShortTime = resetValue;
  lastRefreshTime = resetValue;
  lastSaveTime = resetValue;
  lastHourTime = resetValue;
}
// functions that return a string containing the last reset reason
String verbose_reset_reason()
{
  String resetReason = "";
  for (int i = 0; i <= 1; i++)
  {
    resetReason += " Reset CPU" + String(i) + ": ";
    switch (rtc_get_reset_reason(i))
    {
    case 1:
      resetReason += "Vbat power on reset";
      break;
    case 3:
      resetReason += "Software reset digital core";
      break;
    case 4:
      resetReason += "Legacy watch dog reset digital core";
      break;
    case 5:
      resetReason += "Deep Sleep reset digital core";
      break;
    case 6:
      resetReason += "Reset by SLC module, reset digital core";
      break;
    case 7:
      resetReason += "Timer Group0 Watch dog reset digital core";
      break;
    case 8:
      resetReason += "Timer Group1 Watch dog reset digital core";
      break;
    case 9:
      resetReason += "RTC Watch dog Reset digital core";
      break;
    case 10:
      resetReason += "Instrusion tested to reset CPU";
      break;
    case 11:
      resetReason += "Time Group reset CPU";
      break;
    case 12:
      resetReason += "Software reset CPU";
      break;
    case 13:
      resetReason += "RTC Watch dog Reset CPU";
      break;
    case 14:
      resetReason += "for APP CPU, reseted by PRO CPU";
      break;
    case 15:
      resetReason += "Reset when the vdd voltage is not stable";
      break;
    case 16:
      resetReason += "RTC Watch dog reset digital core and rtc module";
      break;
    default:
      resetReason += "NO_MEAN";
      break;
    }
  }
  return resetReason;
}

// functions that return a string containing the last WiFi lost reason
String verbose_wifi_reason(uint8_t wifiReason)
{
  String wifiLostReason;
  switch (wifiReason)
  {
  case 1:
    wifiLostReason = "UNSPECIFIED";
    break;
  case 2:
    wifiLostReason = "AUTH_EXPIRE";
    break;
  case 3:
    wifiLostReason = "AUTH_LEAVE";
    break;
  case 4:
    wifiLostReason = "ASSOC_EXPIRE";
    break;
  case 5:
    wifiLostReason = "ASSOC_TOOMANY";
    break;
  case 6:
    wifiLostReason = "NOT_AUTHED";
    break;
  case 7:
    wifiLostReason = "NOT_ASSOCED";
    break;
  case 8:
    wifiLostReason = "ASSOC_LEAVE";
    break;
  case 9:
    wifiLostReason = "ASSOC_NOT_AUTHED";
    break;
  case 10:
    wifiLostReason = "DISASSOC_PWRCAP_BAD";
    break;
  case 11:
    wifiLostReason = "DISASSOC_SUPCHAN_BAD";
    break;
  case 13:
    wifiLostReason = "IE_INVALID";
    break;
  case 14:
    wifiLostReason = "MIC_FAILURE";
    break;
  case 15:
    wifiLostReason = "4WAY_HANDSHAKE_TIMEOUT";
    break;
  case 16:
    wifiLostReason = "GROUP_KEY_UPDATE_TIMEOUT";
    break;
  case 17:
    wifiLostReason = "IE_IN_4WAY_DIFFERS";
    break;
  case 18:
    wifiLostReason = "GROUP_CIPHER_INVALID";
    break;
  case 19:
    wifiLostReason = "PAIRWISE_CIPHER_INVALID";
    break;
  case 20:
    wifiLostReason = "AKMP_INVALID";
    break;
  case 21:
    wifiLostReason = "UNSUPP_RSN_IE_VERSION";
    break;
  case 22:
    wifiLostReason = "INVALID_RSN_IE_CAP";
    break;
  case 23:
    wifiLostReason = "802_1X_AUTH_FAILED";
    break;
  case 24:
    wifiLostReason = "CIPHER_SUITE_REJECTED";
    break;
  case 200:
    wifiLostReason = "BEACON_TIMEOUT";
    break;
  case 201:
    wifiLostReason = "NO_AP_FOUND";
    break;
  case 202:
    wifiLostReason = "AUTH_FAIL";
    break;
  case 203:
    wifiLostReason = "ASSOC_FAIL";
    break;
  case 204:
    wifiLostReason = "HANDSHAKE_TIMEOUT";
    break;
  case 205:
    wifiLostReason = "CONNECTION_FAIL";
    break;
  default:
    wifiLostReason = "NOT_RECOGNIZED";
    break;
  }
  return wifiLostReason;
}
// function that manages recording of pump running time
void recordPumpTime()
{
  // save previous day information
  dayPumpTime[indexPumpTime] = pumpDayRunningTime; // time in seconds
  // computes mean running time over the last 7 days
  computeMeanDayPumpTime();
  indexPumpTime = (++indexPumpTime) % 7;
}
//---------------------------------------------------------------
// computes mean running time over the last 7 days and publish information
void computeMeanDayPumpTime()
{
  // computes mean running time over the last 7 days, discarding days with 0 information
  int j = 0;
  meanDayPumpTime = 0;
  for (int i = 0; i < 7; i++)
  {
    if (dayPumpTime[i] > 0)
    {
      // this table record is not empty
      j++;
      meanDayPumpTime += dayPumpTime[i];
    }
  }
  if (j == 0)
  {
    meanDayPumpTime = pumpDayRunningTime / secondsPerMinute;
  }
  else
  {
    meanDayPumpTime = meanDayPumpTime / (j * secondsPerMinute); // mean time in minutes
  }
  // publish information once a day
  bootstrapManager.publish(MEAN_PUMP_RUNNING_TIME, helper.string2char(String(meanDayPumpTime)), true);
  bootstrapManager.publish(PUMP_RUNNING_TIME, helper.string2char(String(pumpDayRunningTime / secondsPerMinute)), true);
} // end of computeMeanDayPumpTime
//---------------------------------------------------------------------
void blinkLed()
{
  if (stateLed)
  {
    stateLed = false;
    ledcWrite(ledChannel, 0);
  }
  else
  {
    stateLed = true;
    ledcWrite(ledChannel, 255);
  }
}
// rounds a number to n decimal places
// example: round(3.14159, 2) -> 3.14
float myRound(float value, int digit)
{
  switch (digit)
  {
  case 0:
    return (int32_t)(value + 0.5);
    break;
  case 1:
    return (int32_t)(value * 10 + 0.5) / 10.0;
    break;
  case 2:
    return (int32_t)(value * 100 + 0.5) / 100.0;
    break;
  case 3:
    return (int32_t)(value * 1000 + 0.5) / 1000.0;
    break;
  case 4:
    return (int32_t)(value * 10000 + 0.5) / 10000.0;
    break;
  default:
    return (int32_t)(value + 0.5);
    break;
  }
}
void computePumpWindowTime()
{
  // use ezTime library makeTime function
  pumpWindowT1b = makeTime(morningWindowStartHour, morningWindowStartMinute, 0, currentDay, currentMonth, currentYear);
  pumpWindowT1e = pumpWindowT1b + scheduledPumpRunningTime;
  pumpWindowT2b = makeTime(eveningWindowStartHour, eveningWindowStartMinute, 0, currentDay, currentMonth, currentYear);
  pumpWindowT2e = pumpWindowT1b + scheduledPumpRunningTime;
  // note evening window end time may occur on the next day but before 3 AM - with water temp = 30°C max, the evening window max allowed start is 19:30
  // morning window may overlap the evening window but this will never occur in nominal conditions
}
//----------------------------------------------------------------------
// reset with default value pump running time information when no history data are available
void resetPumpRunningTimeInfo()
{
  currentPumpMode = summer; // will be quickly updated after initial running (8 mn)
  pumpManualOn = false;
  pumpManualOff = false;
  pumpContinuousOn = false;
  pumpContinuousOff = false;
  pumpRunningTimeManualOff = 0;
  pumpRunningTimeManualOn = 0;
  pumpDayRunningTime = 0;
  currentDay = 0; // as real day number are from 1 to 31, will force a new day init later on
  triggerTS = false;
  triggerTW = false;
  triggerTMS = false;
  TScounter = 0;
  TWcounter = 0;
  TMScounter = 0;
  cumulativePumpTime = 0;
  pumpIsON = false;
  indexPumpTime = 0;
  for (int index = 0; index < 7; index++)
  {
    dayPumpTime[index] = 0;
  }
} // end of resetPumpRunningTimeInfo routine
/*
  initialize pump controller state variables from a json document
  called after data file reading in flash memory or following a restore command trhough MQTT channel
*/
void setInternalState(DynamicJsonDocument DSdoc)
{
  // Json parsing code from https://arduinojson.org/v5/assistant/
  myPrintln("---- Init time parameters from DS file ----");
  cumulativeDeviceActivity = helper.getValue(DSdoc["CDA"]).toInt() + millis() / 1000;
  deviceActivity_1 = helper.getValue(DSdoc["LDA_1"]).toInt();
  deviceActivity_2 = helper.getValue(DSdoc["LDA_2"]).toInt();
  devicePowerUp = 1 + helper.getValue(DSdoc["DPU"]).toInt();
  lastSavedEvent = "Last saved event : " + helper.getValue(DSdoc["DLE"]);
  lastPumpControlerEvent = lastSavedEvent;
  // retrieves pump running time info
  JsonObject PRTinfo = DSdoc["PRT"]; // Pump Running Time inormation
  currentPumpMode = PRTinfo["CPM"];
  pumpManualOn = PRTinfo["PMON"];
  pumpManualOff = PRTinfo["PMOFF"];
  pumpContinuousOn = PRTinfo["PCON"];
  pumpContinuousOff = PRTinfo["PCOFF"];
  pumpRunningTimeManualOff = PRTinfo["MOFFT"];
  pumpRunningTimeManualOn = PRTinfo["MONT"];
  pumpDayRunningTime = PRTinfo["DPT"];
  currentDay = PRTinfo["CDN"];
  triggerTS = PRTinfo["TTS"];
  triggerTW = PRTinfo["TTW"];
  triggerTMS = PRTinfo["TMS"];
  TScounter = PRTinfo["TSC"];
  TWcounter = PRTinfo["TMC"];
  TMScounter = PRTinfo["TMSC"];
  cumulativePumpTime = PRTinfo["CPT"];
  pumpIsON = PRTinfo["PION"];
  indexPumpTime = PRTinfo["IPT"];
  JsonArray PRT_DPT = PRTinfo["DPTA"];
  for (int index = 0; index < 7; index++)
  {
    dayPumpTime[index] = PRT_DPT[index];
  }
  float readPACTempTarget = PRTinfo["PACT"];
  if ((readPACTempTarget >= minPACtempTarget) && (readPACTempTarget <= maxPACtempTarget))
  {
    // a valid temperature value has been read and can be used
    PACTempTarget = readPACTempTarget;
  }
  else
  {
    lastPumpControlerEvent = "température activation PAC invalide" + String(readPACTempTarget, 2);
    pushEvent(lastPumpControlerEvent);
  }
  PACAutomation = PRTinfo["PACA"];
} // end of setInternalState routine
//-------------------------------------------------------------------------
/* functionnal test
   used for testing of modifications before pull request
   code is subject to modifications
*/
boolean functionnalTest()
{
  // tests BootstrapManager::readValueFromFile(String filename, String paramName)
  // creates a json file and reads parameters
  boolean testResult = true;
  const int intValue = 50;
  const float floatValue = 200.3;
  const String stringValue = "test readValueFromFile";
  String filename = "testFile.json";
  Serial.println("Start of functionnal test");
  DynamicJsonDocument DSdoc(1024);
  JsonObject root = DSdoc.to<JsonObject>(); // clears DSdoc and convert it to a Json object
  DSdoc["STRING_VALUE"] = stringValue;
  DSdoc["INT_VALUE"] = intValue;
  DSdoc["FLOAT_VALUE"] = floatValue;
  bootstrapManager.writeToSPIFFS(DSdoc, filename);
  Serial.println();
  // reads value and check
  // check int value
  String readValue = bootstrapManager.readValueFromFile(filename, "INT_VALUE");
  Serial.println();
  Serial.print("intValue: ");
  Serial.print(intValue);
  Serial.print(" read value: ");
  Serial.print(readValue);
  if (readValue.toInt() != intValue)
  {
    testResult = false;
    Serial.println(" KO");
  }
  else
    Serial.println(" OK");
  // check String value
  readValue = bootstrapManager.readValueFromFile(filename, "STRING_VALUE");
  Serial.println();
  Serial.print("stringValue: ");
  Serial.print(stringValue);
  Serial.print(" read value: ");
  Serial.print(readValue);
  if (readValue != stringValue)
  {
    testResult = false;
    Serial.println(" KO");
  }
  else
    Serial.println(" OK");
  // check float value
  readValue = bootstrapManager.readValueFromFile(filename, "FLOAT_VALUE");
  Serial.println();
  Serial.print("floatValue: ");
  Serial.print(floatValue);
  Serial.print(" read value: ");
  Serial.print(readValue);
  if (readValue.toFloat() != floatValue)
  {
    testResult = false;
    Serial.println(" KO");
  }
  else
    Serial.println("OK");
  Serial.println("end of functionnal test");
  return testResult;
}
