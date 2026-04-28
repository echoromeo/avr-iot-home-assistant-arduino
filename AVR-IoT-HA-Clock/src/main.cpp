/*
  AVR-IoT Home Assistant MQTT Client

  Using PlatformIO for all the stuff under the hood
  Libraries:
  * WiFi101 by Arduino
  * home-assistant-integration by David Chyrzynski
    https://github.com/dawidchyrzynski/arduino-home-assistant/blob/main/examples/light/light.ino
  * FastLED by 

 */
#include <Arduino.h>
#include <time.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi101.h>
#include <ArduinoHA.h>
#include <Adafruit_GFX.h>
#include <FastLED_Neomatrix.h>
#include <FastLED.h>
#include "../include/avr-iot.h"
#include "../include/arduino_secrets.h" 

// FastLED stuff for the Neopixels
#define NUM_LEDS_W    6
#define NUM_LEDS_H    9
#define NUM_LEDS_A    5
#define NUM_LEDS      (NUM_LEDS_W*NUM_LEDS_H*NUM_LEDS_A)
#define LED_TYPE   WS2812B
#define COLOR_ORDER   GRB
#define DATA_PIN        20 // according to _FL_DEFPIN(20, 4, D) in FastLED\src\platforms\avr\fastpin_avr_atmega4809.h
#define VOLTS          3.3
#define MAX_MA       500

CRGB leds[NUM_LEDS];

FastLED_NeoMatrix *matrix = new FastLED_NeoMatrix(leds, NUM_LEDS_W, NUM_LEDS_H, NUM_LEDS_A, 1, 
                                                  NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
                                                  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE);

// Font stuff - https://tchapi.github.io/Adafruit-GFX-Font-Customiser/
#define NOT_COLON ';'
const uint8_t Font6x9_clockBitmaps[] PROGMEM = {
  0xFF, 0x3C, 0xF3, 0xCF, 0x3C, 0xF3, 0xFC, 0xFF, 0xFF, 0xC0, 0xFC, 0x30, 
  0xC3, 0xFF, 0x0C, 0x30, 0xFC, 0xF8, 0xC6, 0x3F, 0x8C, 0x63, 0xF8, 0x00, 
  0xCF, 0x3C, 0xF3, 0xFC, 0x30, 0xC3, 0x0C, 0xFF, 0x0C, 0x30, 0xFC, 0x30, 
  0xC3, 0xFC, 0xFF, 0x0C, 0x30, 0xFF, 0x3C, 0xF3, 0xFC, 0xFC, 0x30, 0xC6, 
  0x18, 0xC3, 0x0C, 0x30, 0x00, 0xFF, 0x3C, 0xF3, 0xFF, 0x3C, 0xF3, 0xFC, 
  0xFF, 0x3C, 0xF3, 0xFC, 0x30, 0xC3, 0xFC, 0x00, 0x0F, 0x0F, 0x00, 0x00, 
  0x00, 0x00
};

const GFXglyph Font6x9_clockGlyphs[] PROGMEM = {
  {     0,   6,   9,   7,    0,   -9 },   // 0x30 '0'
  {     7,   2,   9,   7,    2,   -9 },   // 0x31 '1'
  {    10,   6,   9,   7,    0,   -9 },   // 0x32 '2'
  {    17,   5,   9,   7,    1,   -9 },   // 0x33 '3'
  {    24,   6,   9,   7,    0,   -9 },   // 0x34 '4'
  {    31,   6,   9,   7,    0,   -9 },   // 0x35 '5'
  {    38,   6,   9,   7,    0,   -9 },   // 0x36 '6'
  {    45,   6,   9,   7,    0,   -9 },   // 0x37 '7'
  {    53,   6,   9,   7,    0,   -9 },   // 0x38 '8'
  {    60,   6,   9,   7,    0,   -9 },   // 0x39 '9'
  {    68,   2,   9,   3,    0,   -9 },   // 0x3A ':'
  {    71,   2,   9,   3,    0,   -9 }    // 0x3B ';'
};

const GFXfont Font6x9_clock PROGMEM = {
  (uint8_t  *)Font6x9_clockBitmaps, 
  (GFXglyph *)Font6x9_clockGlyphs, 0x30, 0x3B,  9
};

// Clock stuff
bool display = true, blink = true;
unsigned long millis_tick = 0;
time_t lastTime = 0, lastWifiTime = 0;

// Wifi client stuff for the winc1510
WiFiClient client;
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password
int status = WL_IDLE_STATUS;

// MQTT device stuff for Home Assistant
byte mac[6];
char ha_user[] = SECRET_HA_USER;    // the device homeassistant (mqtt) username
char ha_pass[] = SECRET_HA_PASS;    // the device homeassistant (mqtt) password
HADevice device;                    // use in setup()
HAMqtt mqtt(client, device);        // use in setup()

// Home Assistant entities stuff
// "iotClockDisplay" are unique IDs
HALight displaySettings("iotClockDisplay", HALight::RGBFeature + HALight::BrightnessFeature);
HASwitch displayBlink("iotClockBlink");
unsigned long lastUpdateAt = 0;

bool WiFiConnect();
void onDisplayBlinkCommand(bool state, HASwitch* sender);
void onDisplayStateCommand(bool state, HALight* sender);
void onBrightnessCommand(uint8_t brightness, HALight* sender);
void onColorTemperatureCommand(uint16_t temperature, HALight* sender);
void onRGBColorCommand(HALight::RGBColor color, HALight* sender);

void wifiClockUpdate();

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

  //FastLED.setMaxPowerInVoltsAndMilliamps( VOLTS, MAX_MA);
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS)
    .setCorrection(TypicalLEDStrip);

  matrix->begin();
  matrix->setFont(&Font6x9_clock); // Set the font
  matrix->setTextWrap(false);
  matrix->setBrightness(30);
  matrix->setTextColor(matrix->Color(100, 0, 0),0); // Set initial color
  matrix->show();

  // Set WiFi module pins
  WiFi.setPins(
    PIN_WIFI_CS,
    PIN_WIFI_IRQ,
    PIN_WIFI_RST,
    PIN_WIFI_EN
  );

#if DEBUG
   // Initialize serial communication for debugging
  SerialCOM_BEGIN(115200);
  
  while (!SerialCOM) {
   ; // wait for serial port to connect. Must be commented out if not connected to PC
  }
#endif

  while (!WiFiConnect())
  {
    // wait 10 seconds for connection
    delay(10000);
  }

  // Set Home Assistant device details
  byte mac_reverse[6];   
  WiFi.macAddress(mac_reverse);
  for (uint8_t i = 0; i < 6; i++)
  {
    mac[i] = mac_reverse[5-i];
  }
  device.setUniqueId(mac, sizeof(mac));
  device.setName("AVR-IoT Clock");
  device.setSoftwareVersion("1.0.0");
  device.setManufacturer("Microchip");
  device.enableExtendedUniqueIds();

  // Configure Home Assistant sensors
  displaySettings.setName("Display");
  displaySettings.setRetain(true);

  // Maximum brightness level can be changed as follows:
  displaySettings.setBrightnessScale(30);

  // Optionally you can enable optimistic mode for the HALight.
  // In this mode you won't need to report state back to the HA when commands are executed.
  //displaySettings.setOptimistic(true);

  // Color temperature range (optional)
  // displaySettings.setMinMireds(50);
  // displaySettings.setMaxMireds(200);

  // handle light states
  displaySettings.onStateCommand(onDisplayStateCommand);
  displaySettings.onBrightnessCommand(onBrightnessCommand); // optional
  //displaySettings.onColorTemperatureCommand(onColorTemperatureCommand); // optional
  displaySettings.onRGBColorCommand(onRGBColorCommand); // optional

  displayBlink.setName("Blink");
  displayBlink.setRetain(true);
  displayBlink.onCommand(onDisplayBlinkCommand);

  // Connect to Home Assistant MQTT broker  
  while (!mqtt.begin(SECRET_BROKER, ha_user, ha_pass))
  {
    digitalWrite(LED_ERROR, LOW); // Indicate error
    delay(10000);
  }
  digitalWrite(LED_ERROR, HIGH); // No error
}

void loop() {

  // Check if WiFi is connected
  status = WiFi.status();
  if (status == WL_CONNECTED) //TODO: No need for similar to Ethernet.maintain()?
  {
	  digitalWrite(LED_WIFI, LOW);
    mqtt.loop(); // This maintains the mqtt connection and reconnects (and sends data)
    
    // Check if MQTT is connected
    if (mqtt.isConnected())
    {
      digitalWrite(LED_CONN, LOW);

      // Not sure if the clock needs to do anything here?
    }
    else // !mqtt.isConnected()
    {
      digitalWrite(LED_CONN, HIGH);

      // Not sure if the clock needs to do anything here?
    }

    if (difftime(lastTime,lastWifiTime) > ONE_HOUR)
    {
      wifiClockUpdate();
      lastWifiTime = lastTime;
    }
  }
  else // !WiFi.status()
  {
    digitalWrite(LED_WIFI, HIGH);
    digitalWrite(LED_CONN, HIGH);
    digitalWrite(LED_ERROR, LOW); // Indicate error if WiFi has been disconnected

    while (!WiFiConnect())
    {
      // wait 10 seconds for reconnection
      delay(10000);
    }
  }

  // TODO: Add display on/off on PIN_SW1

  // TODO: Make sure the time is ticking properly
  if (!time(NULL))
  {
      wifiClockUpdate();
  }
  else if(millis() >= millis_tick)
  {
    millis_tick += 1000;
    system_tick();
  }

  time_t currentTime = time(NULL);
  if (difftime(currentTime, lastTime)) // TODO: Merge with if (millis() % 1000)?
  {
    lastTime = currentTime;

    // Update display every second
    if (display)
    {
      struct tm *formattedTime = localtime(&currentTime);
      char clockString[6];
      snprintf(clockString, sizeof(clockString), "%02d:%02d", formattedTime->tm_hour, formattedTime->tm_min);

      // Blink the colon every second if enabled
      if (blink && (formattedTime->tm_sec & 0x01))
      {
          clockString[2] = NOT_COLON;
      }

      matrix->clear();
      matrix->setCursor(0, 9);
      matrix->print(clockString);
    }
    else
    {
      matrix->clear();
    }

    matrix->show();
  }
}

bool WiFiConnect() {
  SerialCOM_PRINT("Attempting to connect WiFi: ");
  SerialCOM_PRINTLN(ssid);
  status = WiFi.begin(ssid, pass);

  if (status == WL_CONNECTED)
  {
    SerialCOM_PRINTLN("WINC1510 online");
    digitalWrite(LED_WIFI, LOW);
  }

  return status == WL_CONNECTED;
}

void onDisplayBlinkCommand(bool state, HASwitch* sender)
{
    lastUpdateAt = millis();
    blink = state;

    SerialCOM_PRINT("Blink State: ");
    SerialCOM_PRINTLN(state);

    sender->setState(state); // report state back to the Home Assistant
}

void onDisplayStateCommand(bool state, HALight* sender) {
    lastUpdateAt = millis();
    display = state;

    SerialCOM_PRINT("Light State: ");
    SerialCOM_PRINTLN(state);

    sender->setState(state); // report state back to the Home Assistant
}

void onBrightnessCommand(uint8_t brightness, HALight* sender) {
    lastUpdateAt = millis();
    matrix->setBrightness(brightness);

    SerialCOM_PRINT("Brightness: ");
    SerialCOM_PRINTLN(brightness);

    sender->setBrightness(brightness); // report brightness back to the Home Assistant
}

void onRGBColorCommand(HALight::RGBColor color, HALight* sender) {
    lastUpdateAt = millis();
    matrix->setTextColor(matrix->Color(color.red, color.green, color.blue), 0);

    SerialCOM_PRINT("Red: ");
    SerialCOM_PRINTLN(color.red);
    SerialCOM_PRINT("Green: ");
    SerialCOM_PRINTLN(color.green);
    SerialCOM_PRINT("Blue: ");
    SerialCOM_PRINTLN(color.blue);

    sender->setRGBColor(color); // report color back to the Home Assistant
}

void wifiClockUpdate() {
  millis_tick = millis() + 1000;
  unsigned long epoch = WiFi.getTime(); // get wifi time
  // Todo: compensate for time zone, dst ++
  epoch += 2*ONE_HOUR;
  set_system_time( (time_t)epoch );     // set the time library
  lastTime = time(NULL);
}
