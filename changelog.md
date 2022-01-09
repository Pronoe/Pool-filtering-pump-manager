Author : Patrick Souty

-----------------------------------------------------------------------------------------------------------------------
    10/10/2021 : version 1.0 - slider control WEb server demo + multiple strategy for WiFi connection
    11/10/2021 : version 1.1 - add basic OTA capabilities (uploading through WiFi) and clone project into platformIO
    01/11/2021 : version 1.1 - add basic code for temp. sensor management
    18/12/2021 : version 1.3 - final hardware test - use of Arduino IoT library for communication - basic temperature display and relay command
    
    21/12/2021 : version 2.0 - backup of 1.3 version and start of version based on Bootstrapper library
    29/12/2021 : version 2.1 - preliminary operationnal version coupled with Home-asssistant through Eclipse-Mosquitto MQTT broker
    06/01/2022 : version 2.2 - added last event message, WiFi event monitoring, fix an issue in bootstrapper library with reading json file
                                added file integrity control with SHA-256 hash code

    Done steps:
    - test temperature sensor acqusition and libraries - done
    - test relay board connection and activation - done
    - implements specific data structure - done
    - implement Mosquitto MQTT server on RPI3 and Home-Assistant application on RPI3 - done (IP 192.168.0.117)
    - implement MQTT management with bootstarp library (validated by coupling with home-assistant on 29/12/2021)
    - basic time information saving in flash memory
    - implement conversion from raw data in s to time in week, day, hour, mn, s through Home-assistant template in Configuration.yaml
    - test OTA update with Bootstrapper library (note: the OTA password entered in flash memory is asked before uploading)
    Next steps:
    - implements ezTime library and uses it for local time management instead of UTC time
    - implements a stack of event with time tag
    - force imediate time info publishing at startup
    - implements pump forced ON or OFF through Home-assistant buttons
    - implements redondant context information saving and selection of the latest available context at startup
    - implements scheduler
    - implements pump automation routine
    - add connection monitoring and reconnect
    - implements secured parameters saving in flash memory (integrity monitoring achieved)
    - implements parameters modification through Home-Assistant