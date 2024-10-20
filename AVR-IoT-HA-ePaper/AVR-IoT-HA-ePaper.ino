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
HANumber tempFuture("iotNumberSeven", HANumber::PrecisionP1);
HANumber tempOut("iotNumberTwo", HANumber::PrecisionP1);
HANumber tempUp("iotNumberOne", HANumber::PrecisionP1);
HANumber tempDown("iotNumberSix", HANumber::PrecisionP1);
HANumber co2In("iotNumberThree", HANumber::PrecisionP0);
HANumber humidUp("iotNumberFour", HANumber::PrecisionP0);
HANumber humidDown("iotNumberFive", HANumber::PrecisionP0);
bool update = true;

void onNumberCommand(HANumeric number, HANumber* sender)
{
    sender->setState(number); // report the selected option back to the HA panel
    update = true;
}

// ePaper stuff
Epd epd;
/**
* Due to RAM not enough in Arduino UNO, a frame buffer is not allowed.
* In this case, a smaller image buffer is allocated and you have to 
* update a partial display several times.
* 1 byte = 8 pixels, therefore you have to set 8*N pixels at a time.
*/
#define EPD_BUFFER_HEIGHT (EPD_HEIGHT / 11) // int(300 / 11) * 11 = 297
unsigned char image[(EPD_WIDTH*EPD_BUFFER_HEIGHT)/8];
Paint paint(image, EPD_WIDTH, EPD_BUFFER_HEIGHT);    //width should be the multiple of 8 

#define COLORED     0
#define UNCOLORED   1

#define NEXT_UPDATE_TIME (60*1000ul*10) // 10 Minutes
unsigned long lastUpdateAt = 0;

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

  if (epd.Init() != 0) {
    digitalWrite(LED_ERROR, LOW);
  }

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
      // wait 10 seconds for connection:
      delay(10000);
    }
  }

  // Set Home Assistant device details
  device.setName("AVR-IoT eInk");
  device.setSoftwareVersion("1.0.0");

  // Configure Home Assistant sensor Temperature Report
  tempFuture.onCommand(onNumberCommand);
  tempFuture.setIcon("mdi:thermometer");
  tempFuture.setName("Temperature Report");
  tempFuture.setMin(-50); // can be float if precision is set via the constructor
  tempFuture.setStep(0.1f); // minimum step: 0.001f
  tempFuture.setMode(HANumber::ModeBox);
  tempFuture.setRetain(true);

  // Configure Home Assistant sensor Temperature Outdoor
  tempOut.onCommand(onNumberCommand);
  tempOut.setIcon("mdi:thermometer");
  tempOut.setName("Temperature Outdoor");
  tempOut.setMin(-50); // can be float if precision is set via the constructor
  tempOut.setStep(0.1f); // minimum step: 0.001f
  tempOut.setMode(HANumber::ModeBox);
  tempOut.setRetain(true);

  // Configure Home Assistant sensor Temperature Upstairs
  tempUp.onCommand(onNumberCommand);
  tempUp.setIcon("mdi:thermometer");
  tempUp.setName("Temperature Upstairs");
  tempUp.setStep(0.1f); // minimum step: 0.001f
  tempUp.setMode(HANumber::ModeBox);
  tempUp.setRetain(true);

  // Configure Home Assistant sensor Temperature Downstairs
  tempDown.onCommand(onNumberCommand);
  tempDown.setIcon("mdi:thermometer");
  tempDown.setName("Temperature Downstairs");
  tempDown.setStep(0.1f); // minimum step: 0.001f
  tempDown.setMode(HANumber::ModeBox);
  tempDown.setRetain(true);

  // Configure Home Assistant sensor CO2 Indoor
  co2In.onCommand(onNumberCommand);
  co2In.setIcon("mdi:molecule-co2");
  co2In.setName("CO2 Indoor");
  co2In.setMax(6000); // can be float if precision is set via the constructor
  co2In.setStep(1.0f); // minimum step: 0.001f
  co2In.setMode(HANumber::ModeBox);
  co2In.setRetain(true);

  // Configure Home Assistant sensor Mositure Upstairs
  humidUp.onCommand(onNumberCommand);
  humidUp.setIcon("mdi:water-percent");
  humidUp.setName("Humidity Upstairs");
  humidUp.setStep(1.0f); // minimum step: 0.001f
  humidUp.setMode(HANumber::ModeBox);
  humidUp.setRetain(true);

  // Configure Home Assistant sensor Mositure Downstairs
  humidDown.onCommand(onNumberCommand);
  humidDown.setIcon("mdi:water-percent");
  humidDown.setName("Humidity Downstairs");
  humidDown.setStep(1.0f); // minimum step: 0.001f
  humidDown.setMode(HANumber::ModeBox);
  humidDown.setRetain(true);

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
    delay(10000);
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
      lastUpdateAt = millis()-(NEXT_UPDATE_TIME+1);
      update = true;
    }

    // Update sensor data every NEXT_UPDATE_TIME
    if ((millis() - lastUpdateAt) > NEXT_UPDATE_TIME) {

        SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
        updateEpd();
        SPI.endTransaction();
    }
  }
  else // !mqtt.isConnected()
  {
      digitalWrite(LED_CONN, HIGH);
  }
}

uint8_t countDigits(uint16_t num)
{
  uint8_t count = 1;  // everything has at least one digit, right?
  uint32_t comparison = 10;
  while (comparison <= num) {
    comparison *= 10;
    count++;
  }
  return count;
}

void updateEpd()
{
  char stringBuf[24]; // dtostrf(float, width, precision, stringBuf)
  float floatTemp;
  int16_t int16temp;
  static uint16_t y = 0;

  if (!update)
  {
    lastUpdateAt = millis();
    return;    
  }

  paint.Clear(UNCOLORED);

  switch (y) // split into FSM to not hog the SPI forever
  {
    case 0:
      epd.Reset();
      if (epd.Init() != 0) {
        // e-Paper init failed
        digitalWrite(LED_ERROR, LOW);
        return;
      }
      epd.ClearFrame();
      y += EPD_BUFFER_HEIGHT;
      return; //break;

    case (EPD_BUFFER_HEIGHT*1):  
      paint.DrawStringAt(10, 0, "Veirmelding:        C", &Font24, COLORED);
      floatTemp = tempFuture.getCurrentState().toFloat();
      dtostrf(floatTemp, 3, 1, stringBuf);
      if (floatTemp < 0.0)
      {
        int16temp = (countDigits(-floatTemp)+3);
      }
      else
      {
        int16temp = (countDigits(floatTemp)+2);
      }
      paint.DrawStringAt(334-MAX_WIDTH_FONT*int16temp, 0, stringBuf, &Font24, COLORED);
      paint.DrawCircle(347, 3, 3, COLORED);    
      break;

    case (EPD_BUFFER_HEIGHT*2):  
      paint.DrawStringAt(10, 0, "Grader  ute:        C", &Font24, COLORED);
      floatTemp = tempOut.getCurrentState().toFloat();
      dtostrf(floatTemp, 3, 1, stringBuf);
      if (floatTemp < 0.0)
      {
        int16temp = (countDigits(-floatTemp)+3);
      }
      else
      {
        int16temp = (countDigits(floatTemp)+2);
      }
      paint.DrawStringAt(334-MAX_WIDTH_FONT*int16temp, 0, stringBuf, &Font24, COLORED);
      paint.DrawCircle(347, 3, 3, COLORED);    
      break;

    case (EPD_BUFFER_HEIGHT*3):
      paint.DrawStringAt(10, 0, "Grader oppe:        C", &Font24, COLORED);
      floatTemp = tempUp.getCurrentState().toFloat();
      dtostrf(floatTemp, 3, 1, stringBuf);
      int16temp = (countDigits(floatTemp)+2);
      paint.DrawStringAt(334-MAX_WIDTH_FONT*int16temp, 0, stringBuf, &Font24, COLORED);    
      paint.DrawCircle(347, 3, 3, COLORED);    
      break;

    case (EPD_BUFFER_HEIGHT*4):
      paint.DrawStringAt(10, 0, "Grader nede:        C", &Font24, COLORED);
      floatTemp = tempDown.getCurrentState().toFloat();
      dtostrf(floatTemp, 3, 1, stringBuf);
      int16temp = (countDigits(floatTemp)+2);
      paint.DrawStringAt(334-MAX_WIDTH_FONT*int16temp, 0, stringBuf, &Font24, COLORED);    
      paint.DrawCircle(347, 3, 3, COLORED);    
      break;

    case (EPD_BUFFER_HEIGHT*5):
      paint.DrawStringAt(10, 0, "CO2    oppe:        ppm", &Font24, COLORED);
      int16temp = co2In.getCurrentState().toInt16();
      itoa(int16temp, stringBuf, 10);
      int16temp = countDigits(int16temp);
      paint.DrawStringAt(334-MAX_WIDTH_FONT*int16temp, 0, stringBuf, &Font24, COLORED);    
      break;

    case (EPD_BUFFER_HEIGHT*6):
      paint.DrawStringAt(10, 0, "Fukt   oppe:        %", &Font24, COLORED);
      int16temp = humidUp.getCurrentState().toInt16();
      itoa(int16temp, stringBuf, 10);
      int16temp = countDigits(int16temp);
      paint.DrawStringAt(334-MAX_WIDTH_FONT*int16temp, 0, stringBuf, &Font24, COLORED);    
      break;

    case (EPD_BUFFER_HEIGHT*7):
      paint.DrawStringAt(10, 0, "Fukt   nede:        %", &Font24, COLORED);
      int16temp = humidDown.getCurrentState().toInt16();
      itoa(int16temp, stringBuf, 10);
      int16temp = countDigits(int16temp);
      paint.DrawStringAt(334-MAX_WIDTH_FONT*int16temp, 0, stringBuf, &Font24, COLORED);    
      break;

    case (EPD_BUFFER_HEIGHT*8): 
      y = EPD_HEIGHT-EPD_BUFFER_HEIGHT; // Uptime, at the bottom
      paint.DrawStringAt(10, 0, "    oppetid:        min", &Font24, COLORED);
      int16temp = millis()/60000ul;
      itoa(int16temp, stringBuf, 10);
      int16temp = countDigits(int16temp);
      paint.DrawStringAt(334-MAX_WIDTH_FONT*int16temp, 0, stringBuf, &Font24, COLORED);    
      break;

    default:
      update = false;
      y = 0;
      lastUpdateAt = millis();

      /* This displays the data from the SRAM in e-Paper module */
      epd.DisplayFrame();
      epd.Sleep();

      return; //break;
  }

  epd.SetPartialWindow(image, 0, y+3, EPD_WIDTH, EPD_BUFFER_HEIGHT);
  y += EPD_BUFFER_HEIGHT;

}
