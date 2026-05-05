/*
  AVR-IoT Home Assistant MQTT Client

  Using MegaCoreX by MCUdude for ATmega4808 support
  Libraries:
  * WiFi101 by Arduino
  * home-assistant-integration by David Chyrzynski
  * Adafruit MCP9808 Library by Adafruit

 */
#define SERIAL_RX_BUFFER_SIZE 256  // increase from default 64, so Serial1 will cope better with long lists from the HAN port

#include <Wire.h>
#include <SPI.h>
#include <WiFi101.h>
#include <ArduinoHA.h>
#include "Adafruit_MCP9808.h"
#include "avr-iot.h"
#include "arduino_secrets.h"

// Turn on/off SerialCOM for debugging/deployment
#define DEBUG_SERIAL 0  // 1: send terminal messages; 0: quiet for deployment

#if DEBUG_SERIAL
#define DBG_BEGIN(x) SerialCOM.begin(x)
#define DBG_PRINT(...) SerialCOM.print(__VA_ARGS__)
#define DBG_PRINTLN(...) SerialCOM.println(__VA_ARGS__)
#else
#define DBG_BEGIN(x)
#define DBG_PRINT(...)
#define DBG_PRINTLN(...)
#endif

// Wifi client stuff for the winc1510
WiFiClient client;
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password
int status = WL_IDLE_STATUS;

// MQTT device stuff for Home Assistant
byte mac[6];                      // to be filled with actual MAC address
char ha_user[] = SECRET_HA_USER;  // the device homeassistant (mqtt) username
char ha_pass[] = SECRET_HA_PASS;  // the device homeassistant (mqtt) password
HADevice device;                  // use in setup()
HAMqtt* mqtt;                     // use in setup(), needs HADevice from previous line

// Home Assistant entities stuff
// "iotLightSensor" and "iotTempSensor" are unique IDs of the sensors
HASensorNumber brightnessSensor("iotLightSensor", HASensorNumber::PrecisionP0);
HASensorNumber temperatureSensor("iotTempSensor", HASensorNumber::PrecisionP1);
HASensorNumber activePowerPlusSensor("iotHANSensorPowerPlus", HASensorNumber::PrecisionP0);
HASensorNumber activePowerMinusSensor("iotHANSensorPowerMinus", HASensorNumber::PrecisionP0);
HASensorNumber activeEnergyInSensor("iotHANSensorEnergyIn", HASensorNumber::PrecisionP0);
HASensorNumber activeEnergyOutSensor("iotHANSensorEnergyOut", HASensorNumber::PrecisionP0);

Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();
unsigned long lastUpdateAt = 0;

// You can also specify the precision of the sensor by providing the second argument to the constructor as follows:
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP1);
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP2);
// HASensorNumber brightnessSensor("myAnalogInput", HASensorNumber::PrecisionP3);

// The payload, to be read from the utility meter
struct FrameValues {
  uint16_t activePlus;
  uint16_t activeMinus;
  uint32_t energyImport;
  uint32_t energyExport;
};
FrameValues fv;

void setup() {
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

  // Initialize serial communication with mikroBUS (USART1)
  Serial1.begin(2400, SERIAL_8E1);

  // Set WiFi module pins
  WiFi.setPins(
    PIN_WIFI_CS,
    PIN_WIFI_IRQ,
    PIN_WIFI_RST,
    PIN_WIFI_EN);

  // Initialize MCP9808 sensor
  if (mcp9808.begin(ADDRESS_I2C_MCP9808)) {
    DBG_PRINT("MCP9808 online");
  } else {
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
  device.setName("AVR-IoT HAN Sensor");
  device.setSoftwareVersion("1.0.0");

  // Configure Home Assistant sensors
  brightnessSensor.setIcon("mdi:brightness-percent");
  brightnessSensor.setName("Brightness");
  brightnessSensor.setUnitOfMeasurement("%");

  temperatureSensor.setIcon("mdi:thermometer");
  temperatureSensor.setName("Temperature");
  temperatureSensor.setUnitOfMeasurement("°C");

  activePowerPlusSensor.setIcon("mdi:home-lightning-bolt-outline");
  activePowerPlusSensor.setName("HAN Power Active +");
  activePowerPlusSensor.setUnitOfMeasurement("W");

  activePowerMinusSensor.setIcon("mdi:home-lightning-bolt");
  activePowerMinusSensor.setName("HAN Power Active -");
  activePowerMinusSensor.setUnitOfMeasurement("W");

  activeEnergyInSensor.setIcon("mdi:home-import-outline");
  activeEnergyInSensor.setName("HAN Energy Import");
  activeEnergyInSensor.setUnitOfMeasurement("Wh");


  activeEnergyOutSensor.setIcon("mdi:home-export-outline");
  activeEnergyOutSensor.setName("HAN Energy Export");
  activeEnergyOutSensor.setUnitOfMeasurement("Wh");


  // Connect to Home Assistant MQTT broker
  mqtt = new HAMqtt(client, device);
  mqtt->setDiscoveryPrefix("homeassistant");
  mqtt->begin(SECRET_BROKER, ha_user, ha_pass);

  //Initialize with non-zero values, just to make sure the data comes through
  fv.activePlus = 1;    // every two seconds
  fv.activeMinus = 1;   // every ten seconds
  fv.energyImport = 1;  // only every 3600 seconds
  fv.energyExport = 1;  // together with energyImport
}

void loop() {

  // Check if WiFi is connected
  if (WiFi.status() == WL_CONNECTED)  //TODO: No need for similar to Ethernet.maintain()?
  {
    digitalWrite(LED_WIFI, LOW);
    mqtt->loop();  // This maintains the mqtt connection and reconnects (and sends data)

    // Check if MQTT is connected
    if (mqtt->isConnected()) {
      digitalWrite(LED_CONN, LOW);

      // readFrameValue() is asynchronous (takes up to 0.5s every 2s), timing is better this way
      readFrameValue(fv);  //update data, leave the previous untouched if list type too short


      // Update sensor data every 10 seconds
      if ((millis() - lastUpdateAt) > 10000) {
        digitalWrite(LED_DATA, LOW);

        brightnessSensor.setValue(readLightPct());
        temperatureSensor.setValue(mcp9808.readTempC());
        activePowerPlusSensor.setValue(fv.activePlus);
        activePowerMinusSensor.setValue(fv.activeMinus);
        activeEnergyInSensor.setValue(fv.energyImport);
        activeEnergyOutSensor.setValue(fv.energyExport);

        DBG_PRINT("Inside the 10s updating loop, last power value is: ");
        DBG_PRINT(lastPowerValue);
        DBG_PRINTLN(" W");

        lastUpdateAt = millis();

        // you can reset the sensors as follows:
        // brightnessSensor.setValue(nullptr);
        // temperatureSensor.setValue(nullptr);

        digitalWrite(LED_DATA, HIGH);
      }
    } else  // !mqtt->isConnected()
    {
      digitalWrite(LED_CONN, HIGH);
    }
  } else  // !WiFi.status()
  {
    digitalWrite(LED_WIFI, HIGH);
    digitalWrite(LED_CONN, HIGH);
    digitalWrite(LED_ERROR, LOW);  // Indicate error if WiFi has been disconnected
    attemptWifiConnection();
  }
}  // end loop()


bool attemptWifiConnection() {
  int status = WiFi.status();

  while (status != WL_CONNECTED) {
    DBG_PRINT("Attempting to connect WiFi: ");
    DBG_PRINTLN(ssid);

    status = WiFi.begin(ssid, pass);

    if (status == WL_CONNECTED) {
      DBG_PRINTLN("WINC1510 online");
      printWiFiStatus();
      digitalWrite(LED_WIFI, LOW);
      digitalWrite(LED_ERROR, HIGH);  //Turn off WiFi error indication if it was on
      return true;                    // success
    } else {
      // wait 10 seconds before retrying
      delay(10000);
    }
  }

  return true;  // already connected
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  DBG_PRINT("SSID: ");
  DBG_PRINTLN(WiFi.SSID());

  // print your WiFi shield's MAC address:
  WiFi.macAddress(mac);
  DBG_PRINT("MAC: ");  //note the bytes are "backwards"
  DBG_PRINT(mac[5], HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[4], HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[3], HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[2], HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[1], HEX);
  DBG_PRINT(":");
  DBG_PRINTLN(mac[0], HEX);

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

// USART Frame Parser
bool readFrameValue(FrameValues& out) {
  static uint8_t buffer[157];  // // Buffer for incoming data
  static uint8_t index = 0;

  enum {
    WAIT_7E,
    WAIT_A0,
    WAIT_LEN,
    READ_LIST1,
    READ_LIST2,
    READ_LIST3
  };
  static uint8_t state = WAIT_7E;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    switch (state) {

      case WAIT_7E:  //stay here until a 0x7E comes by
        if (b == 0x7E) {
          buffer[0] = b;
          index = 1;
          state = WAIT_A0;
        }
        break;

      case WAIT_A0:       //sent here from WAIT_7E only
        if (b == 0xA0) {  //verify list start 0x7E 0xA0
          buffer[index++] = b;
          state = WAIT_LEN;      //was indeed a list start, move over
        } else if (b == 0x7E) {  //something went wrong, maybe it's now a list start
          buffer[0] = 0x7E;
          index = 1;
        } else {
          state = WAIT_7E;
        }
        break;

      case WAIT_LEN:      // Sent here from WAIT_A0 only, get list length
        if (b == 0x27) {  // List Type 1
          buffer[index++] = b;
          state = READ_LIST1;    // We have a go
        } else if (b == 0x65) {  // List Type 2
          buffer[index++] = b;
          state = READ_LIST2;  // We have a go
        }

        else if (b == 0x87) {  // List Type 3
          buffer[index++] = b;
          state = READ_LIST3;  // We have a go
        } else {               // Discard, move to WAIT_7E
          state = WAIT_7E;
          index = 0;
        }
        break;

      case READ_LIST1:  // We'll get here only after list 1 sequence 0x7E 0xA0 0x27
        buffer[index++] = b;

        if (index == 41) {  // List type 1 is over, but didn't get list end
          state = WAIT_7E;  //  discard list and wait for next start

          if (buffer[40] == 0x7E) {          // Entire list read successsfully
            out.activePlus =                 // Read active power in W
              ((uint16_t)buffer[36] << 8) |  // (The list is using four bytes, but
              buffer[37];                    // 64KW ought to be enough for anybody)
            index = 0;
            return true;
          }

          // bad frame, discard
          index = 0;
        }
        break;


      case READ_LIST2:  // We'll get here only after list 2 sequence 0x7E 0xA0 0x65
        buffer[index++] = b;

        if (index == 103) {  // List type 2 is over, but didn't get list end. 0x65 is a lie!
          state = WAIT_7E;   //  discard list and wait for next start

          if (buffer[102] == 0x7E) {  // Entire list read successsfully
            out.activePlus =          // Read "active power plus" in W
              ((uint16_t)buffer[73] << 8) | buffer[74];
            out.activeMinus =  // Read "active power minus" in W
              ((uint16_t)buffer[78] << 8) | buffer[79];
            index = 0;
            return true;
          }

          // bad frame, discard
          index = 0;
        }
        break;


      case READ_LIST3:  // We'll get here only after list 1 sequence 0x7E 0xA0 0x9B
        buffer[index++] = b;

        if (index == 137) {  // List type 3 is over, but didn't get list end
          state = WAIT_7E;   //  discard list and wait for next start

          if (buffer[136] == 0x7E) {  // Entire list read successsfully
            out.activePlus =          // Read "active power plus" in W
              ((uint16_t)buffer[73] << 8) | buffer[74];

            out.activeMinus =  // Read "active power minus" in W
              ((uint16_t)buffer[78] << 8) | buffer[79];

            out.energyImport =  // // cumultative active import energy
              ((uint32_t)buffer[115] << 24) | ((uint32_t)buffer[116] << 16) | ((uint32_t)buffer[117] << 8) | buffer[118];

            out.energyExport =  // // cumultative active export energy
              ((uint32_t)buffer[120] << 24) | ((uint32_t)buffer[121] << 16) | ((uint32_t)buffer[122] << 8) | buffer[123];

            index = 0;
            return true;
          }

          // bad frame, discard
          index = 0;
        }
        break;
    }
  }
  return false;
}
