/*
  AVR-IoT Home Assistant MQTT Client with e-Paper

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
#include "epd4in2.h"
#include "imagedata.h"
#include "epdpaint.h"

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
// "iotNumberOne" and "iotNumberTwo" are unique IDs of the sensors
HANumber tempIn("iotNumberOne", HANumber::PrecisionP1);
HANumber tempOut("iotNumberTwo", HANumber::PrecisionP1);
unsigned long lastUpdateAt = -30*60000+10000;

void onNumberCommand(HANumeric number, HANumber* sender)
{
    sender->setState(number); // report the selected option back to the HA panel
}

// ePaper stuff
Epd epd;
/**
* Due to RAM not enough in Arduino UNO, a frame buffer is not allowed.
* In this case, a smaller image buffer is allocated and you have to 
* update a partial display several times.
* 1 byte = 8 pixels, therefore you have to set 8*N pixels at a time.
*/
unsigned char image[1500];
Paint paint(image, 400, 28);    //width should be the multiple of 8 
#define COLORED     0
#define UNCOLORED   1

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
  //SerialCOM.begin(115200);
  
  // Set WiFi module pins
  WiFi.setPins(
    PIN_WIFI_CS,
    PIN_WIFI_IRQ,
    PIN_WIFI_RST,
    PIN_WIFI_EN
  );

  //while (!SerialCOM) {
  //  ; // wait for serial port to connect. Must be commented out if not connected to PC
  //}

  if (epd.Init() != 0) {
    //SerialCOM.print("e-Paper init failed");
    digitalWrite(LED_ERROR, LOW);
  }
  else
  {
  	//SerialCOM.print("e-Paper online");
    updateEpd();
  }

  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED)
  {
    //SerialCOM.print("Attempting to connect WiFi: ");
    //SerialCOM.println(ssid);
    status = WiFi.begin(ssid, pass);

    if (status == WL_CONNECTED)
    {
      //SerialCOM.println("WINC1510 online");
      //printWiFiStatus();
      digitalWrite(LED_WIFI, LOW);
    }
    else
    {
      // wait 10 seconds for connection:
      delay(10000);
    }
  }

  // Set Home Assistant device details
  device.setName("AVR-IoT eInk");
  device.setSoftwareVersion("1.0.0");

  // Configure Home Assistant sensor
  tempIn.onCommand(onNumberCommand);

  // Optional configuration
  tempIn.setIcon("mdi:thermometer");
  tempIn.setName("Temperature Indoor");
  tempIn.setMin(-40); // can be float if precision is set via the constructor
  tempIn.setMax(40); // can be float if precision is set via the constructor
  tempIn.setStep(0.1f); // minimum step: 0.001f
  tempIn.setMode(HANumber::ModeBox);
  // tempIn.setMode(HANumber::ModeSlider);

  // You can set retain flag for the HA commands
  tempIn.setRetain(true);

  // You can also enable optimistic mode for the HASelect.
  // In this mode you won't need to report state back to the HA when commands are executed.
  //tempIn.setOptimistic(true);

  // Configure Home Assistant sensor
  tempOut.onCommand(onNumberCommand);

  // Optional configuration
  tempOut.setIcon("mdi:thermometer");
  tempOut.setName("Temperature Outdoor");
  tempOut.setMin(-40); // can be float if precision is set via the constructor
  tempOut.setMax(40); // can be float if precision is set via the constructor
  tempOut.setStep(0.1f); // minimum step: 0.001f
  tempOut.setMode(HANumber::ModeBox);
  // tempIn.setMode(HANumber::ModeSlider);

  // You can set retain flag for the HA commands
  tempOut.setRetain(true);

  // You can also enable optimistic mode for the HASelect.
  // In this mode you won't need to report state back to the HA when commands are executed.
  //tempIn.setOptimistic(true);

  // Connect to Home Assistant MQTT broker  
  mqtt.begin(SECRET_BROKER, ha_user, ha_pass);
}

void loop() {

  // Check if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) //TODO: No need for similar to Ethernet.maintain()?
  {
	  digitalWrite(LED_WIFI, LOW);

    digitalWrite(LED_DATA, LOW);
    mqtt.loop(); // This maintains the mqtt connection and reconnects (and sends data)
    digitalWrite(LED_DATA, HIGH);
    
    // Check if MQTT is connected
    if (mqtt.isConnected())
    {
      digitalWrite(LED_CONN, LOW);

      // Update sensor data every 30 minutes
      if ((millis() - lastUpdateAt) > 30*60000) {

          SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
          updateEpd();
          SPI.endTransaction();
          lastUpdateAt = millis();
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

void updateEpd() {

  // Convert float to string
  // dtostrf(float, width, precision, buffer)
  char stringBuf[10]; // Buffer to hold the resulting string

  epd.Reset();
  epd.Init();
  epd.ClearFrame();

  paint.Clear(UNCOLORED);
  paint.DrawStringAt(0, 0, "Temperatur inne:     C", &Font24, COLORED);
  dtostrf(tempIn.getCurrentState().toFloat(), 3, 1, stringBuf);
  paint.DrawStringAt(290, 0, stringBuf, &Font24, COLORED);
  epd.SetPartialWindow(paint.GetImage(), 0, 40, paint.GetWidth(), paint.GetHeight());

  paint.Clear(UNCOLORED);
  paint.DrawStringAt(0, 0, "Temperatur ute:      C", &Font24, COLORED);
  dtostrf(tempOut.getCurrentState().toFloat(), 3, 1, stringBuf);
  paint.DrawStringAt(290, 0, stringBuf, &Font24, COLORED);
  epd.SetPartialWindow(paint.GetImage(), 0, 64, paint.GetWidth(), paint.GetHeight());

  /* This displays the data from the SRAM in e-Paper module */
  epd.DisplayFrame();
  epd.Sleep();
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
