/*
  AVR-IoT Home Assistant MQTT Client
 */

#include <SPI.h>
#include <WiFi101.h>
#include "avr-iot.h"
#include "arduino_secrets.h" 
#include <ArduinoHA.h>

// Initialize the Wifi client library for the winc
WiFiClient client;
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password
int status = WL_IDLE_STATUS;

// Initialize the mqtt device 
byte mac[] = SECRET_MAC;      // can we get a mac from the winc?
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
  // Configure pins
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  WiFi.setPins(
    PIN_WIFI_CS,
    PIN_WIFI_IRQ,
    PIN_WIFI_RST,
    PIN_WIFI_EN
  );

  //Initialize serial and wait for port to open:
  Serial2.begin(115200);
  while (!Serial2) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Check for the presence of the shield:
  // TODO: Skip this for the on-board winc?
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial2.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial2.print("Attempting to connect to SSID: ");
    Serial2.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial2.println("Connected to wifi");
  printWiFiStatus();

  // Set device's details (optional)
  device.setName("Arduino");
  device.setSoftwareVersion("1.0.0");

  // Configure sensor (optional)
  analogSensor.setIcon("mdi:home");
  analogSensor.setName("Brightness");
  analogSensor.setUnitOfMeasurement("%");

  mqtt.begin(SECRET_BROKER);
}

void loop() {

  if (client.available()) //TODO: Does this replace Ethernet.maintain()?
  {
    mqtt.loop();

    if ((millis() - lastUpdateAt) > 1000) { // 1000ms debounce time
        uint16_t reading = analogRead(PIN_LIGHT_SENSOR);
        float voltage = reading * 100.f / 1023.f; //Get percent of maximum value (1023)

        analogSensor.setValue(voltage);
        lastUpdateAt = millis();

        // you can reset the sensor as follows:
        // analogSensor.setValue(nullptr);
    }
  }
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial2.print("SSID: ");
  Serial2.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial2.print("IP Address: ");
  Serial2.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial2.print("signal strength (RSSI):");
  Serial2.print(rssi);
  Serial2.println(" dBm");
}





