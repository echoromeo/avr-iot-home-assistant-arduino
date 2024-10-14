/*
  AVR-IoT Home Assistant MQTT Client

  Using MegaCoreX for ATmega4808 support
 */
#include <Wire.h>
#include <SPI.h>
#include <WiFi101.h> // Need library WiFi101 by Arduino
#include <ArduinoHA.h> // Need library home-assistant-integration by David Chyrzynski
#include "Adafruit_MCP9808.h" // Need library Adafruit MCP9808 Library by Adafruit
#include "avr-iot.h"
#include "arduino_secrets.h" 


// Initialize the Wifi client library for the winc
WiFiClient client;
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password
int status = WL_IDLE_STATUS;

// Initialize the mqtt device 
byte mac[] = SECRET_MAC;      // can we get a mac directly from the winc?
char ha_user[] = SECRET_HA_USER;    // the device homeassistant (mqtt) username
char ha_pass[] = SECRET_HA_PASS;    // the device homeassistant (mqtt) password

HADevice device(mac, sizeof(mac));
HAMqtt mqtt(client, device);
unsigned long lastUpdateAt = 0;

// Initialize the HA device(s)
// "iotLightSensor" is unique ID of the sensor
HASensorNumber brightnessSensor("iotLightSensor", HASensorNumber::PrecisionP0);
HASensorNumber temperatureSensor("iotTempsSensor", HASensorNumber::PrecisionP1);

// You can also specify the precision of the sensor by providing the second argument to the constructor as follows:
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP1);
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP2);
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP3);

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();

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

  SerialCOM.begin(115200);
  
  WiFi.setPins(
    PIN_WIFI_CS,
    PIN_WIFI_IRQ,
    PIN_WIFI_RST,
    PIN_WIFI_EN
  );
  
  if (!mcp9808.begin(ADDRESS_I2C_MCP9808))
  {
    SerialCOM.print("Couldn't find MCP9808!");
    digitalWrite(LED_ERROR, LOW);
  }

  //while (!SerialCOM) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}

  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    SerialCOM.print("Attempting to connect to SSID: ");
    SerialCOM.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  SerialCOM.println("Connected to wifi");
  printWiFiStatus();
  digitalWrite(LED_WIFI, LOW);

  // Set device's details
  device.setName("AVR-IoT");
  device.setSoftwareVersion("1.0.0");

  // Configure sensors
  brightnessSensor.setIcon("mdi:brightness-percent");
  brightnessSensor.setName("Brightness");
  brightnessSensor.setUnitOfMeasurement("%");
  temperatureSensor.setIcon("mdi:thermometer");
  temperatureSensor.setName("Temperature");
  temperatureSensor.setUnitOfMeasurement("°C");
  
  mqtt.begin(SECRET_BROKER, ha_user, ha_pass);
}

void loop() {

  if (WiFi.status() == WL_CONNECTED) //TODO: No need for similar to Ethernet.maintain()?
  {
    mqtt.loop();
    if (mqtt.isConnected())
    {
      digitalWrite(LED_CONN, LOW);
      
      if ((millis() - lastUpdateAt) > 1000) { // 1000ms debounce time
          digitalWrite(LED_DATA, LOW);
        
          uint16_t reading = analogRead(PIN_LIGHT_SENSOR);
          float voltage = reading * 100.f / 1023.f; //Get percent of maximum value (1023)
    
          brightnessSensor.setValue(voltage);
          temperatureSensor.setValue(mcp9808.readTempC());
          lastUpdateAt = millis();
    
          // you can reset the sensors as follows:
          // brightnessSensor.setValue(nullptr);
          // temperatureSensor.setValue(nullptr);
          
          digitalWrite(LED_DATA, HIGH);
      }
    }
    else
    {
        digitalWrite(LED_CONN, HIGH);
    }
  }
  else
  {
    // For now, never turn off the error once enabled
    digitalWrite(LED_ERROR, LOW);
  }
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  SerialCOM.print("SSID: ");
  SerialCOM.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  SerialCOM.print("IP Address: ");
  SerialCOM.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  SerialCOM.print("signal strength (RSSI):");
  SerialCOM.print(rssi);
  SerialCOM.println(" dBm");
}
