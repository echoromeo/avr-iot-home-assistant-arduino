/*
  AVR-IoT Home Assistant MQTT Client

  Using MegaCoreX by MCUdude for ATmega4808 support
  Libraries:
  * WiFi101 by Arduino
  * home-assistant-integration by David Chyrzynski
  * Adafruit MCP9808 Library by Adafruit

 */
#include <Wire.h>
#include <SPI.h>
#include <WiFi101.h>
#include <ArduinoHA.h>
#include "Adafruit_MCP9808.h"
#include "avr-iot.h"
#include "arduino_secrets.h" 

// Turn on/off SerialCOM for debugging/deployment
#define DEBUG_SERIAL 1   // 1: send terminal messages; 0: quiet for deployment

#if DEBUG_SERIAL
  #define DBG_BEGIN(x)      SerialCOM.begin(x)
  #define DBG_PRINT(...)    SerialCOM.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  SerialCOM.println(__VA_ARGS__)
#else
  #define DBG_BEGIN(x)
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
#endif

// Wifi client stuff for the winc1510
WiFiClient client;
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password
int status = WL_IDLE_STATUS;

// MQTT device stuff for Home Assistant
byte mac[] = SECRET_MAC;      // can we get a mac directly from the winc?
char ha_user[] = SECRET_HA_USER;    // the device homeassistant (mqtt) username
char ha_pass[] = SECRET_HA_PASS;    // the device homeassistant (mqtt) password
HADevice device(mac, sizeof(mac));
HAMqtt mqtt(client, device);

// Home Assistant entities stuff
// "iotLightSensor" and "iotTempSensor" are unique IDs of the sensors
HASensorNumber brightnessSensor("iotLightSensor", HASensorNumber::PrecisionP0);
HASensorNumber temperatureSensor("iotTempSensor", HASensorNumber::PrecisionP1);
Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();
unsigned long lastUpdateAt = 0;

// You can also specify the precision of the sensor by providing the second argument to the constructor as follows:
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP1);
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP2);
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP3);

void setup()
{
  // Configure LEDs off
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  pinMode(LED_YELLOW, OUTPUT);
  digitalWrite(LED_YELLOW, HIGH);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);

   // Initialize serial communication for debugging
  DBG_BEGIN(115200);
  
  // Set WiFi module pins
  WiFi.setPins(
    PIN_WIFI_CS,
    PIN_WIFI_IRQ,
    PIN_WIFI_RST,
    PIN_WIFI_EN
  );
  
  // Initialize MCP9808 sensor
  if (mcp9808.begin(ADDRESS_I2C_MCP9808))
  {
    DBG_PRINT("MCP9808 online");
  }
  else
  {
    DBG_PRINT("Couldn't find MCP9808!");
    digitalWrite(LED_ERROR, LOW);
  }

  //while (!SerialCOM) {
  //  ; // wait for serial port to connect. Must be commented out if not connected to PC
  //}

  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED)
  {
    DBG_PRINT("Attempting to connect WiFi: ");
    DBG_PRINTLN(ssid);
    status = WiFi.begin(ssid, pass);

    if (status == WL_CONNECTED)
    {
      DBG_PRINTLN("WINC1510 online");
      printWiFiStatus();
      digitalWrite(LED_WIFI, LOW);
    }
    else
    {
      // wait 10 seconds for connection:
      delay(10000);
    }
  }

  // Set Home Assistant device details
  device.setName("AVR-IoT");
  device.setSoftwareVersion("1.0.0");

  // Configure Home Assistant sensors
  brightnessSensor.setIcon("mdi:brightness-percent");
  brightnessSensor.setName("Brightness");
  brightnessSensor.setUnitOfMeasurement("%");
  temperatureSensor.setIcon("mdi:thermometer");
  temperatureSensor.setName("Temperature");
  temperatureSensor.setUnitOfMeasurement("°C");

  // Connect to Home Assistant MQTT broker  
  mqtt.begin(SECRET_BROKER, ha_user, ha_pass);
}

void loop() {

  // Check if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) //TODO: No need for similar to Ethernet.maintain()?
  {
	  digitalWrite(LED_WIFI, LOW);
    mqtt.loop(); // This maintains the mqtt connection and reconnects (and sends data)
    
    // Check if MQTT is connected
    if (mqtt.isConnected())
    {
      digitalWrite(LED_CONN, LOW);
      
      // Update sensor data every 10 seconds
      if ((millis() - lastUpdateAt) > 10000) {
          digitalWrite(LED_DATA, LOW);
       
          brightnessSensor.setValue(readLightPct());
          temperatureSensor.setValue(mcp9808.readTempC());
          lastUpdateAt = millis();
    
          // you can reset the sensors as follows:
          // brightnessSensor.setValue(nullptr);
          // temperatureSensor.setValue(nullptr);
          
          digitalWrite(LED_DATA, HIGH);
      }
    }
    else // !mqtt.isConnected()
    {
        digitalWrite(LED_CONN, HIGH);
    }
  }
  else // !WiFi.status()
  {
    digitalWrite(LED_WIFI, HIGH);
    digitalWrite(LED_CONN, HIGH);
    digitalWrite(LED_ERROR, LOW); // Indicate error if WiFi has been disconnected
  }
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  DBG_PRINT("SSID: ");
  DBG_PRINTLN(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  DBG_PRINT("IP Address: ");
  DBG_PRINTLN(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  DBG_PRINT("signal strength (RSSI):");
  DBG_PRINT(rssi);
  DBG_PRINTLN(" dBm");
}
