Author : Patrick Souty

-----------------------------------------------------------------------------------------------------------------------
    10/10/2021 : version 1.0 - slider control WEb server demo + multiple strategy for WiFi connection
    11/10/2021 : version 1.1 - adds basic OTA capabilities (uploading through WiFi) and clone project into platformIO
    01/11/2021 : version 1.1 - adds basic code for temp. sensor management
    18/12/2021 : version 1.3 - final hardware test - use of Arduino IoT library for communication - basic temperature display and relay command
    
    21/12/2021 : version 2.0 - backup of 1.3 version and start of version based on Bootstrapper library
    29/12/2021 : version 2.1 - preliminary operationnal version coupled with Home-asssistant through Eclipse-Mosquitto MQTT broker
    06/01/2022 : version 2.2 - added last event message, WiFi event monitoring, fix an issue in bootstrapper library with reading json file
                             - added file integrity control with SHA-256 hash code
    30/01/2022 : version 2.3 - adds redundant context saving files
                             - modify LED use at startup for better monitoring of startup phase
                             - adds basic pump ON/OFF control through MQTT messages
                             - adds monitoring of WiFi disconnections
    06/02/2022 : version 3.0 - adds initial pump management algorithms
                             - implement the use of the ESP32 hardware watchdog with reboot after a timeout of 5s
                             - publish pump running time infomration on MQTT
                             - publish reason of last reboot
    17/02/2022 : version 3.1 - slow rate messages @ 1/10 minutes are sent separately in specific 1 mn time slot within 10 mns to reduce message load burst (leading to mosquitto issue)
                             - implements secured periodic storage in flash memory of pump data information (needed in case of power OFF)
                             - pump running time information are published in minutes and cumulative time in hours
    25/02/2022 : version 3.2 - pump management algorithms made simpler and more robust by using true datetime provided by ezTime library
                             - adds routing of Serial.print over MQTT SERIAL_MQTT topic
                             - send json context file over TECH_INFO MQTT topic (when saving file each 10 minutes)
                             - adds periodic ping to try to avoid WiFi disconnects
                             - adds a reset command of pump timing data (through MQTT CMD/PUMP topic)
                             - remove unnecessary delays in Arduino Bootstrapper
    02/03/2022 : version 3.3 - adds heat pump automation (activates filtering pump when temperture below a threshold level to authorize heat pump activity)
                             - adds configuration commands through MQTT: heat pump temperature threshold, pump activation time, heat pump automation ON/OFF
                             - modify bootstrapper:parseQueueMsg to solve an issue with simple numeric values
                             - corrects bugs in new code for pump timing management
                             - migrates to bootstrapper library release 12.1.6 and corrects some remaining issues (such as DNS incorrect configuration with ESP32)
                             - watchdog timeout set to 180s to allow OTA uplaod
    11/03/2022 : version 3.4 - migrates to bootstrapper library release 12.1.7 and corrects bugs
    17/03/2022 : version 3.5 - MQTT topics redefined to be conform to best practices
                             - add state commands and set commands trough MQTT (including Home Assistant buttons, switches and binary sensors configuration)
                             - add ocntinuous run mode of the filtering pump
                             - implements PAC automation mode
                             - publish state information as soon as a change is detected
                             - corrects bugs
    26/03/2022 : version 3.6 - add permanent pump stop mode (for maintenance purpose)
                                note: the hardware has been installed in the pump cabinet on 29/03/2022 and successfully tested (high power pump contrôl, WiFi link, pump management)
    31/03/2022 : version 3.7 - add WiFi channel publishing (following an issue with channel switching of the gateway wich gave "beacon lost" detceted by the ESP32 )
    
    Done steps:
    - test temperature sensor acqusition and libraries - done
    - test relay board connection and activation - done
    - implements specific data structure - done
    - implement Mosquitto MQTT server on RPI3 and Home-Assistant application on RPI3 - done (IP 192.168.0.117)
    - implement MQTT management with bootstarp library (validated by coupling with home-assistant on 29/12/2021)
    - basic time information saving in flash memory
    - implement conversion from raw data in s to time in week, day, hour, mn, s through Home-assistant template in Configuration.yaml
    - test OTA update with Bootstrapper library (note: the OTA password entered in flash memory is asked before uploading)
    - implements ezTime library and uses it for local time management instead of UTC time
    - implements a stack of event with time tag and event number
    - saves in the event stack early event occuring during setup
    - remove all unused code from initial version
    - implements pump forced ON or OFF through Home-assistant buttons
    - implements redondant context information saving and selection of the latest available context at startup
    - implements secured parameters saving in flash memory (integrity monitoring achieved)
    - implements pump running time computing
    - implements pump automation routine
    - implement saving of pump information in flash memory
    - check temperature filter
    - implements new version of Arduino bootstrapper (v1.12.6 and v1.12.7)
    - resolve re-connect issue (add periodic ping + watchdog)
    - controler board installed within the pump cabinet and successfully tested with the hardware

    Next steps:
    - long duration test in the final configuration
    - implements parameters modification through Home-Assistant (started with version 3.3)
    - code cleaning, removal of unnecessary debug code
    Nice to have:
    - multiple SSID selection at startup