/*
  AVR-IoT Home Assistant MQTT Client with NeoPixel

  Using MegaCoreX by MCUdude for ATmega4808 support
  Libraries:
  * WiFi101 by Arduino
  * home-assistant-integration by David Chyrzynski
  * waveshare e-Paper repo: https://github.com/waveshareteam/e-Paper/tree/master/Arduino

 */
#include <Wire.h>
#include <SPI.h>
#include <WiFi101.h>
#include <ArduinoHA.h>
#include "avr-iot.h"
#include "arduino_secrets.h" 

// Wifi client stuff for the winc1510
WiFiClient client;
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password
int status = WL_IDLE_STATUS;

// MQTT device stuff for Home Assistant
byte mac[6];                        // we get the mac from the winc
char ha_user[] = SECRET_HA_USER;    // the device homeassistant (mqtt) username
char ha_pass[] = SECRET_HA_PASS;    // the device homeassistant (mqtt) password
HADevice device(mac, sizeof(mac));
HAMqtt mqtt(client, device);

// Home Assistant entities stuff
// "iotNumberOne" and "iotNumberTwo" are unique IDs of the sensors
HASwitch onOff("iotSwitchOnOff");
HANumber adventDay("iotNeopixels", HANumber::PrecisionP0);

void onNumberCommand(HANumeric number, HANumber* sender)
{
    sender->setState(number); // report the selected option back to the HA panel
}

void onSwitchCommand(HASwitch sw, HANumber* sender)
{
    sender->setState(sw); // report the selected option back to the HA panel
}

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

  // Configure SW1
  pinMode(PIN_SW1, INPUT_PULLUP);

  // Serial communication for debugging removed due to flash size constraints
  
  // Set WiFi module pins
  WiFi.setPins(
    PIN_WIFI_CS,
    PIN_WIFI_IRQ,
    PIN_WIFI_RST,
    PIN_WIFI_EN
  );

  // Init the NeoPixels here

  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED)
  {
    status = WiFi.begin(ssid, pass);

    if (status == WL_CONNECTED)
    {
      digitalWrite(LED_WIFI, LOW);
    }
    else
    {
      // wait 2 seconds for connection:
      delay(2000);
    }
  }

  // Set Home Assistant device details
  device.setName("AVR-IoT Advent");
  device.setSoftwareVersion("1.0.0");
  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));

  // Configure Home Assistant switch on/off
  onOff.onCommand(onSwitchCommand);
  onOff.setIcon("mdi:candle");
  onOff.setName("Switch");
  //onOff.setMin(-50); // can be float if precision is set via the constructor
  //onOff.setStep(0.1f); // minimum step: 0.001f
  //onOff.setMode(HANumber::ModeBox);
  onOff.setRetain(true);

  // Configure Home Assistant sensor Day of Advent
  adventDay.onCommand(onNumberCommand);
  adventDay.setIcon("mdi:pine-tree");
  adventDay.setName("Day of Advent");
  adventDay.setMax(24); // can be float if precision is set via the constructor
  adventDay.setStep(1.0f); // minimum step: 0.001f
  adventDay.setMode(HANumber::ModeBox);
  adventDay.setRetain(true);

  // Connect to Home Assistant MQTT broker  
  mqtt.begin(SECRET_BROKER, ha_user, ha_pass);
}

void loop()
{
  // Check if WiFi is connected
  while (status != WL_CONNECTED)
  {
    // WiFi has been disconnected
    digitalWrite(LED_ERROR, LOW); 
    digitalWrite(LED_WIFI, HIGH);
    digitalWrite(LED_CONN, HIGH);

    // Attempt to reconnect
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection
    delay(2000);
  }

  // Wifi connected
	digitalWrite(LED_WIFI, LOW);

  digitalWrite(LED_DATA, LOW);
  mqtt.loop(); // This maintains the mqtt connection and transmits data
  digitalWrite(LED_DATA, HIGH);
  
  // Check if MQTT is connected
  if (mqtt.isConnected())
  {
    digitalWrite(LED_CONN, LOW);

    if (!digitalRead(PIN_SW1))
    {
      // toggle onOff

      update = true;
    }

    // if onOff update neopixels

    // else turn off neopixels

  }
  else // !mqtt.isConnected()
  {
      digitalWrite(LED_CONN, HIGH);
  }
}

