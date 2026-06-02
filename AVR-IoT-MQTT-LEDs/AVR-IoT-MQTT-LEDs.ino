/*
  AVR-IoT subscribes MQTT topics and drives an LED meter bar

  Using MegaCoreX by MCUdude for ATmega4808 support
  Libraries:
  * WiFi101 by Arduino
  * home-assistant-integration by David Chyrzynski
  * Adafruit MCP9808 Library by Adafruit

 */
//#define SERIAL_RX_BUFFER_SIZE 256 // increase from default 64, so Serial1 will cope better with long lists from the HAN port

#include <Wire.h>
#include <SPI.h>
#include <WiFi101.h>
#include <ArduinoHA.h>
#include "Adafruit_MCP9808.h"
#include "avr-iot.h"
#include "arduino_secrets.h" 
#include <Adafruit_NeoPixel.h>

// Wifi client stuff for the winc1510
WiFiClient client;
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password
int status = WL_IDLE_STATUS;

// MQTT device stuff 
byte mac[6];                        // to be filled with actual MAC address
char ha_user[] = SECRET_HA_USER;    // the device homeassistant (mqtt) username
char ha_pass[] = SECRET_HA_PASS;    // the device homeassistant (mqtt) password
HADevice device;                    // use in setup()
HAMqtt mqtt(client, device);        // use in setup()

// Home Assistant entities stuff
// "iotLightSensor" and "iotTempSensor" are unique IDs of the sensors
HASensorNumber brightnessSensor("iotLightSensor", HASensorNumber::PrecisionP0);
HASensorNumber temperatureSensor("iotTempSensor", HASensorNumber::PrecisionP1);
HANumber powerPlus("iotHANSensorPowerPlus");
HANumber powerMinus("iotHANSensorPowerMinus");

Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();
unsigned long lastUpdateAt = 0;

// You can also specify the precision of the sensor by providing the second argument to the constructor as follows:
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP1);
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP2);
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP3);

// The payload, to be read from the MQTT Server 
struct topicValues {
    uint16_t activePlus;
    uint16_t activeMinus;
//    uint32_t energyImport;
//    uint32_t energyExport;
};
volatile topicValues tv;

//LED Indicator bar definition
Adafruit_NeoPixel strip(
  LED_COUNT,
  LED_PIN,
  NEO_GRB + NEO_KHZ800      //this may need adoption to your actual strip of LEDs
);


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
  attemptWifiConnection();

  // Set Home Assistant device details
  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));
  device.setName("AVR-IoT LED Power Bar");
  device.setSoftwareVersion("1.0.0");
  
  // Configure Home Assistant sensors
  brightnessSensor.setIcon("mdi:brightness-percent");
  brightnessSensor.setName("Brightness");
  brightnessSensor.setUnitOfMeasurement("%");
  
  temperatureSensor.setIcon("mdi:thermometer");
  temperatureSensor.setName("Temperature");
  temperatureSensor.setUnitOfMeasurement("°C");
  
  // Connect to Home Assistant MQTT broker  
  mqtt.setDiscoveryPrefix("homeassistant");
  mqtt.setDataPrefix("Haus");
  mqtt.begin(SECRET_BROKER, ha_user, ha_pass);

  //Initialize with non-zero values, just to make sure the data comes through  
  tv.activePlus = 1;    // importing
  tv.activeMinus = 1;   // exporting/selling

  
  powerPlus.onCommand(onPowerPlusChanged); 
  powerMinus.onCommand(onPowerMinusChanged); 

  //LED stuff
 
  strip.begin();           
  strip.setBrightness(80); // 0–255
  strip.show();            // Turn all LEDs off

}

void loop() {

//const char topic ="Haus/cce7aa05f0f8/iotHANSensorPowerPlus";
//uint16_t payload = 111;
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
          //send MQTT packages
          brightnessSensor.setValue(readLightPct());
          temperatureSensor.setValue(mcp9808.readTempC());

          //write LEDs from topic data
          energyStatusBar(tv.activePlus, tv.activeMinus);

      
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
    attemptWifiConnection();
  }
}   // end loop()


bool attemptWifiConnection()
{
    int status = WiFi.status();

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
            return true;   // success
        }
        else
        {
            // wait 10 seconds before retrying
            delay(10000);
        }
    }

    return true; // already connected
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  DBG_PRINT("SSID: ");
  DBG_PRINTLN(WiFi.SSID());

  // print your WiFi shield's MAC address:
  WiFi.macAddress(mac);
  DBG_PRINT("MAC: ");       //note the bytes are "backwards"
  DBG_PRINT(mac[5],HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[4],HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[3],HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[2],HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[1],HEX);
  DBG_PRINT(":");
  DBG_PRINTLN(mac[0],HEX);

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



void onPowerPlusChanged(HANumeric val){
    DBG_PRINT("PowerPlus changed to: ");
    DBG_PRINTLN(val.toInt16());
    tv.activePlus = val.toInt16();
}

void onPowerMinusChanged(HANumeric val){
    DBG_PRINT("PowerMinus changed to: ");
    DBG_PRINTLN(val.toInt16());
    tv.activeMinus = val.toInt16();
}


void energyStatusBar(int16_t importPower, int16_t exportPower){
   strip.clear();

  uint32_t barColor = 0;
  int16_t value = 0;
  int16_t maxValue = 1;

  // -------- MODE SELECTION --------
  if (importPower > 0) {
    // Clamp
    importPower = constrain(importPower, 0, IMPORT_MAX);
    value = importPower;
    maxValue = IMPORT_MAX;

    // Color selection
    if (importPower < IMPORT_YELLOW) {
      barColor = strip.Color(0, 120, 0);       // Green
    }
    else if (importPower < IMPORT_ORANGE) {
      barColor = strip.Color(120, 120, 0);     // Yellow
    }
    else if (importPower < IMPORT_RED) {
      barColor = strip.Color(120, 80, 0);     // Orange
    }
    else {
      barColor = strip.Color(120, 0, 0);       // Red
    }
  }
  else if (exportPower > 0) {
    // EXPORT MODE
    exportPower = constrain(exportPower, 0, EXPORT_MAX);
    value = exportPower;
    maxValue = EXPORT_MAX;
    barColor = strip.Color(0, 0, 120);         // Blue
  }
  else {
    // Idle
    strip.show();
    return;
  }

  // -------- DRAW FULL BAR --------
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, barColor);
  }

  // -------- NEEDLE POSITION --------
  uint8_t needle = map(value, 0, maxValue, 0, LED_COUNT - 1);

  // Draw needle (white)
  strip.setPixelColor(needle, strip.Color(155, 155, 155));

  strip.show();
}