Author: Patrick Souty
  
Pool filter pump management application
  
Based on following hardware
  
- ESP32DOIT-DEVT-KIT-V1 including module ESP-WROOM32
- waterproof temperature sensor DS18B20 with one wire data interface
- 220V relay board with digitals inputs

Software : 
- pump activation based on water temperature and time profile
- message exchange through Eclipse-Mosquitto MQTT broker, for system status monitoring and parameters setting
- global management with Home-assistant app running on Raspberry PI3

Part of the sketch generated by the Arduino IoT Cloud Thing "Untitled" (initial version up to 1.3)
  https://create.arduino.cc/cloud/things/b9d17064-5bec-4f3e-8e8c-2d238bb0c55d

Development since 2.0 version, based on Boostrapper library (https://github.com/sblantipodi/arduino_bootstrapper) and ezTime library
