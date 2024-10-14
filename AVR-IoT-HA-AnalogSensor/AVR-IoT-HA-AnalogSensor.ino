/*
  AVR-IoT Home Assistant MQTT Client

  Using MegaCoreX for ATmega4808 support
 */

#include <SPI.h>
#include <WiFi101.h> // Need to library WiFi101 by Arduino
#include <ArduinoHA.h> // Need to library home-assistant-integration by David Chyrzynski
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
HASensorNumber analogSensor("iotLightSensor", HASensorNumber::PrecisionP0);

// You can also specify the precision of the sensor by providing the second argument to the constructor as follows:
// HASensorNumber analogSensor("myAnalogInput", HASensorNumber::PrecisionP1);
// HASensorNumber analogSensor("myAnalogInput", HASensorNumber::PrecisionP2);
// HASensorNumber analogSensor("myAnalogInput", HASensorNumber::PrecisionP3);

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

  WiFi.setPins(
    PIN_WIFI_CS,
    PIN_WIFI_IRQ,
    PIN_WIFI_RST,
    PIN_WIFI_EN
  );

  //Initialize serial and wait for port to open:
  SerialCOM.begin(115200);
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

  // Configure sensor
  analogSensor.setIcon("mdi:brightness-percent");
  analogSensor.setName("Brightness");
  analogSensor.setUnitOfMeasurement("%");

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
    
          analogSensor.setValue(voltage);
          lastUpdateAt = millis();
    
          // you can reset the sensor as follows:
          // analogSensor.setValue(nullptr);
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
