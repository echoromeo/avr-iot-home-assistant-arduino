/*
  AVR-IoT Home Assistant MQTT Client

  Using MegaCoreX by MCUdude for ATmega4808 support
  Libraries:
  * WiFi101 by Arduino
  * home-assistant-integration by David Chyrzynski
  * Adafruit SSD1306 Library by Adafruit

 */
#include <Wire.h>
#include <SPI.h>
#include <time.h>
#include <WiFi101.h>
#include <ArduinoHA.h>
#include "avr-iot.h"
#include "arduino_secrets.h" 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Wifi client stuff for the winc1510
WiFiClient client;
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password
int status = WL_IDLE_STATUS;

// MQTT device stuff for Home Assistant
byte mac[6];                        // the MAC address to be used as unique ID
char ha_user[] = SECRET_HA_USER;    // the device homeassistant (mqtt) username
char ha_pass[] = SECRET_HA_PASS;    // the device homeassistant (mqtt) password
HADevice device(mac, sizeof(mac));
HAMqtt mqtt(client, device);

// Home Assistant entities stuff
HASwitch clockOn("iotClockOn");
bool currentClockState = false;

// OLED display stuff
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     PIN_PD7 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN1_ADDRESS 0x3C
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//#define SCREEN2_ADDRESS 0x3D
//Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Time handling stuff
time_t starttime = 0;  // for WiFi.getTime()
time_t lastUpdateAt = 0;

void InitRTC(void) // Register magic to initialize the 1 second ISR(RTC_PIT_vect)
{
  RTC.PITINTCTRL = RTC_PI_bm;
  RTC.PITCTRLA = RTC_PITEN_bm | RTC_PERIOD_CYC32768_gc;
}

void onSwitchCommand(bool state, HASwitch* sender)
{
    sender->setState(state); // report state back to the Home Assistant
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

  // Initialize the RTC for the system tick
  InitRTC();

  // Initialize serial communication for debugging
  SerialCOM.begin(115200);
  
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

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display1.begin(SSD1306_SWITCHCAPVCC, SCREEN1_ADDRESS)) {
    digitalWrite(LED_ERROR, LOW);
    for(;;); // Don't proceed, loop forever
  }

  // if(!display2.begin(SSD1306_SWITCHCAPVCC, SCREEN2_ADDRESS)) {
  //   SerialCOM.println(F("SSD1306 allocation 2 failed"));
  //   for(;;); // Don't proceed, loop forever
  // }


  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED)
  {
    SerialCOM.print("Attempting to connect WiFi: ");
    SerialCOM.println(ssid);
    status = WiFi.begin(ssid, pass);

    if (status == WL_CONNECTED)
    {
      SerialCOM.println("WINC1510 online");
      printWiFiStatus();
      digitalWrite(LED_WIFI, LOW);
    }
    else
    {
      // wait 10 seconds for connection:
      delay(10000);
    }
  }

  // Get time
  while (starttime == 0) {
    Serial.println("Attempting to get network time");
    starttime = WiFi.getTime();
    if (starttime)
    {
      set_system_time(starttime);
    }
    else
    {
      // wait 2 seconds for connection:
      delay(2000);
    }
  }

  // Set Home Assistant device details
  device.setName("AVR-IoT OLED");
  device.setSoftwareVersion("1.0.0");
  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));

  // Configure Home Assistant sensors
  clockOn.onCommand(onSwitchCommand);
  clockOn.setIcon("mdi:clock-digital");
  clockOn.setName("Display Clock");

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
      
      bool newClockState = clockOn.getCurrentState();
      if (currentClockState != newClockState) {
          digitalWrite(LED_DATA, LOW);

          if (newClockState)
          {
            //TODO: Do some OLED enable here

            //TODO: set_system_time(WiFi.getTime());?
          }
          else
          {
            //TODO: Do some OLED disable/clear here
          }

          currentClockState = newClockState;
         
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

  if (currentClockState)
  {
    time_t thisTime;
    if(time(&thisTime) - lastUpdateAt >= 60)
    {
      tm clock;
      gmtime_r(&thisTime, &clock);
      lastUpdateAt = thisTime-clock.tm_sec;

      // Create time string "HH:MM"
      char displayTime[6];
      strftime(displayTime, sizeof(displayTime), "%H:%M", &clock);
      SerialCOM.print(displayTime);

      //TODO: OLED!
    }
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

ISR(RTC_PIT_vect)
{
  system_tick();
  RTC.PITINTFLAGS = RTC_PI_bm;
}