/*
  AVR-IoT subscribes MQTT topics and drives an LED meter bar

  Using MegaCoreX by MCUdude for ATmega4808 support
  Libraries:
  * WiFi101 by Arduino
  * PubSubClient by Nick O'Leary
  * Adafruit NeoPixel by Adafruit

 */

#include <WiFi101.h>
#include <PubSubClient.h>
#include "avr-iot.h"
#include "arduino_secrets.h"
#include <Adafruit_NeoPixel.h>

// Wifi client stuff for the winc1510
WiFiClient client;
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password
int status = WL_IDLE_STATUS;
byte mac[6];  // to be filled with actual MAC address


// MQTT device stuff
const char* topicPlus = "Haus/cce7aa05f0f8/iotHANSensorPowerPlus/stat_t";    //Topic with the import power
const char* topicMinus = "Haus/cce7aa05f0f8/iotHANSensorPowerMinus/stat_t";  //Topic with the export power
PubSubClient mqttClient(client);

struct topicValues {
  uint16_t activePlus;
  uint16_t activeMinus;
  //    uint32_t energyImport;
  //    uint32_t energyExport;
};
volatile topicValues tv;  // The payloads in numeric form

//LED Indicator Bar definition
Adafruit_NeoPixel strip(
  LED_COUNT,
  LED_PIN,
  NEO_GRB + NEO_KHZ800  //this may need adoption to your actual strip of LEDs
);

unsigned long lastUpdateAt = 0;  //For the refresh loop

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

  // Set WiFi module pins
  WiFi.setPins(
    PIN_WIFI_CS,
    PIN_WIFI_IRQ,
    PIN_WIFI_RST,
    PIN_WIFI_EN);

  // Attempt to connect to WiFi network:
  attemptWifiConnection();

  // Connect to MQTT broker
  mqttClient.setServer(SECRET_BROKER, 1883);
  mqttClient.setCallback(callback);
  //connectMqtt();

  //Initialize with non-zero values, just to make sure the data comes through
  //tv.activePlus = 0;   // importing
  //tv.activeMinus = 600;  // exporting/selling

  //LED stuff
  strip.begin();
   strip.clear();             // Turn all LEDs off
   strip.setBrightness(50);  // 0–255
   

}

void loop() {
  // Check if WiFi is connected
  if (WiFi.status() == WL_CONNECTED)  //TODO: No need for similar to Ethernet.maintain()?
  {
    digitalWrite(LED_WIFI, LOW);

    if (!mqttClient.connected()) {
      digitalWrite(LED_CONN, HIGH);
      connectMqtt();
    }

    if (mqttClient.connected()) {
      digitalWrite(LED_CONN, LOW);
      mqttClient.loop();  // push from broker-> callback() populates the topic values tv

      if ((millis() - lastUpdateAt) > 10000) {  // Update LED bar every 10 seconds
        digitalWrite(LED_DATA, LOW);
        energyStatusBar(tv.activePlus, tv.activeMinus);  //write LEDs
        DBG_PRINTLN("Yep, I'm fine");
        
        lastUpdateAt = millis();
        digitalWrite(LED_DATA, HIGH);
      }
    } else  // mqttClient.connected() false
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
      return true;  // success
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

//callback, storing directly into tv.activePlus and tv.activeMinus
void callback(char* topicBuf, byte* payload, unsigned int length) {
  bool isPlus = (strcmp(topicBuf, topicPlus) == 0);
  bool isMinus = (strcmp(topicBuf, topicMinus) == 0);
  if (!isPlus && !isMinus) return;
  // Parse ASCII payload -> uint16_t
  uint16_t val = 0;
  for (unsigned int i = 0; i < length; i++) {
    char c = payload[i];
    if (c < '0' || c > '9') break;
    val = val * 10 + (c - '0');
  }
  // Store atomically
  if (isPlus) {
    tv.activePlus = val;
  } else {
    tv.activeMinus = val;
  }
  // Debug
  DBG_PRINT("MQTT ");
  DBG_PRINT(isPlus ? "activePlus" : "activeMinus");
  DBG_PRINT(" = ");
  DBG_PRINTLN(val);
}


void connectMqtt() {
  while (!mqttClient.connected()) {
    DBG_PRINTLN("Connecting MQTT...");
    if (mqttClient.connect("arduinoClient", SECRET_HA_USER, SECRET_HA_PASS)) {
      DBG_PRINTLN("connected");
      mqttClient.subscribe(topicPlus);
      DBG_PRINT("Subscribed to: ");
      DBG_PRINTLN(topicPlus);
      mqttClient.subscribe(topicMinus);
      DBG_PRINT("Subscribed to: ");
      DBG_PRINTLN(topicMinus);
      DBG_PRINT("MQTT connected? ");
      DBG_PRINTLN(mqttClient.connected());  //print some bool
    } else {
      DBG_PRINT("failed, rc=");
      DBG_PRINTLN(mqttClient.state());
      delay(2000);
    }
  }
}


void energyStatusBar(int16_t importPower, int16_t exportPower){
  strip.clear();

  uint32_t barColor = 0;
  int16_t value = 0;
  int16_t maxValue = 1;

  // -------- MODE SELECTION --------
  if (importPower > 0 && exportPower == 0) {      //Importing power
    importPower = constrain(importPower, 0, IMPORT_MAX);  // Clamp
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
      barColor = strip.Color(180, 90, 0);     // Orange
    }
    else {
      barColor = strip.Color(210, 0, 0);       // Red
    }
  }
  else if (importPower == 0 && exportPower > 0) {  // Exporting
    exportPower = constrain(exportPower, 0, EXPORT_MAX);
    value = exportPower;
    maxValue = EXPORT_MAX;
    barColor = strip.Color(0, 0, 120);         // Blue
  }
  else {      //Inconclusive data (e.g. importPower and exportPower > 0 due to async. timing): just refresh
    strip.show();
    return;
  }

  //draw bar and set needle dot on top
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, barColor);
  }
  uint8_t needle = map(value, 0, maxValue, 0, LED_COUNT - 1);
  strip.setPixelColor(needle, strip.Color(222, 222, 222));
  strip.show();
}