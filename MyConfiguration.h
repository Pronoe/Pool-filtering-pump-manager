/*
  MyConfiguration.h - Config header
  
  Copyright (C) - 2021  Patrick Souty
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of 
  this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
  copies of the Software, and to permit persons to whom the Software is 
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in 
  all copies or substantial portions of the Software.
  
  You should have received a copy of the MIT License along with this program.  
  If not, see <https://opensource.org/licenses/MIT/>.
*/

#define AUTHOR "Pronoe"
#define SERIAL_RATE 115200
#define DEBUG_QUEUE_MSG false
// Specify if you want to use a display or only Serial
#define DISPLAY_ENABLED false
// SENSORNAME will be used as device network name if configuration through Web server is activated
#define WIFI_DEVICE_NAME "ArduinoBootstrapper"
// Port for the OTA firmware uplaod
#define MICROCONTROLLER_OTA_PORT 8199
// Set wifi power in dbm range 0/0.25, set to 0 to reduce PIR false positive due to wifi power, 0 low, 20.5 max.
#define WIFI_SIGNAL_STRENGTH 0
// GATEWAY IP
#define GATEWAY_IP "192.168.0.254"
// DNS IP
#define DNS_IP "8.8.8.8"  // Goggle DNS server
// STATIC IP FOR THE MICROCONTROLLER OF SENSOR
#define MICROCONTROLLER_IP "192.168.0.118"
// MQTT server IP
#define MQTT_SERVER_IP "192.168.0.117"
// MQTT server port
#define MQTT_SERVER_PORT "1883"
// Maximum number of reconnection (WiFi/MQTT) attemp before powering off peripherals
#define MAX_RECONNECT 50
// Maximum JSON Object Size
#define MAX_JSON_OBJECT_SIZE 50
// Maximum JSON Object Size
#define SMALL_JSON_OBJECT_SIZE 50
// Maximum MQTT packet Size
#define MQTT_MAX_PACKET_SIZE 1024
// MQTT Keep Alive
#define MQTT_KEEP_ALIVE 60
// Additional param that can be used for general purpose use
#define ADDITIONAL_PARAM_TEXT "ADDITIONAL PARAM"
// Additional param that can be used for general purpose use
#define ADDITIONAL_PARAM "none"
