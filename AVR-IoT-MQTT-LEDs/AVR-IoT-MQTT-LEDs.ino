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
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password
int status = WL_IDLE_STATUS;
byte mac[6];                  // to be filled with actual MAC address


// MQTT device stuff 
const char* topicPlus ="Haus/cce7aa05f0f8/iotHANSensorPowerPlus/stat_t";   //Topic with the import power
const char* topicMinus ="Haus/cce7aa05f0f8/iotHANSensorPowerMinus/stat_t";  //Topic with the export power
PubSubClient PSclient(client); 

struct topicValues {
    uint16_t activePlus;
    uint16_t activeMinus;
//    uint32_t energyImport;
//    uint32_t energyExport;
};
volatile topicValues tv;      // The payloads in numeric form

//LED Indicator Bar definition
Adafruit_NeoPixel strip(
  LED_COUNT,
  LED_PIN,
  NEO_GRB + NEO_KHZ800      //this may need adoption to your actual strip of LEDs
);

unsigned long lastUpdateAt = 0; //For the refresh loop

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
  DBG_BEGIN(115200);
  
  // Set WiFi module pins
  WiFi.setPins(
    PIN_WIFI_CS,
    PIN_WIFI_IRQ,
    PIN_WIFI_RST,
    PIN_WIFI_EN
  );
  

  // Attempt to connect to WiFi network:
  attemptWifiConnection();
    
  // Connect to MQTT broker  
  PSclient.setServer(SECRET_BROKER,1883);
  PSclient.setCallback(callback);
  connectMqtt();

  //Initialize with non-zero values, just to make sure the data comes through  
  tv.activePlus = 1;    // importing
  tv.activeMinus = 1;   // exporting/selling

  //LED stuff 
  strip.begin();           
  strip.setBrightness(80); // 0–255
  strip.show();            // Turn all LEDs off
  
  // Draw full purple bar, as POST
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(120, 120, 0));
  }
}


void loop() {

  // Check if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) //TODO: No need for similar to Ethernet.maintain()?
  {
	  digitalWrite(LED_WIFI, LOW);
    
    if (!PSclient.connected()){
       digitalWrite(LED_CONN, HIGH);
       connectMqtt();
    } 

    if(PSclient.connected())
    {
      digitalWrite(LED_CONN, LOW);
      PSclient.loop();      // A push from the broker will populate the topic values tv
            
      if ((millis() - lastUpdateAt) > 10000) { // Update LED bar every 10 seconds
          digitalWrite(LED_DATA, LOW);
          energyStatusBar(tv.activePlus, tv.activeMinus); //write LEDs
          lastUpdateAt = millis();
          digitalWrite(LED_DATA, HIGH);
      }
    }
    else // PSclient.connected() false
    {
        digitalWrite(LED_CONN, HIGH);
    }
  }
  else // !WiFi.status()
  {
    digitalWrite(LED_WIFI, HIGH);
    digitalWrite(LED_CONN, HIGH);
    digitalWrite(LED_ERROR, LOW); // Indicate error if WiFi has been disconnected
    attemptWifiConnection();
  }
}   // end loop()


bool attemptWifiConnection()
{
    int status = WiFi.status();

    while (status != WL_CONNECTED)
    {
        DBG_PRINT("Attempting to connect WiFi: ");
        DBG_PRINTLN(ssid);

        status = WiFi.begin(ssid, pass);

        if (status == WL_CONNECTED)
        {
            DBG_PRINTLN("WINC1510 online");
            printWiFiStatus();
            digitalWrite(LED_WIFI, LOW);
            return true;   // success
        }
        else
        {
            // wait 10 seconds before retrying
            delay(10000);
        }
    }

    return true; // already connected
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  DBG_PRINT("SSID: ");
  DBG_PRINTLN(WiFi.SSID());

  // print your WiFi shield's MAC address:
  WiFi.macAddress(mac);
  DBG_PRINT("MAC: ");       //note the bytes are "backwards"
  DBG_PRINT(mac[5],HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[4],HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[3],HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[2],HEX);
  DBG_PRINT(":");
  DBG_PRINT(mac[1],HEX);
  DBG_PRINT(":");
  DBG_PRINTLN(mac[0],HEX);

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
  bool isPlus  = (strcmp(topicBuf, topicPlus)  == 0);
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

/*
void callback(char* topicBuf, byte* payload, unsigned int length) {
  // Identify topic
  if (strcmp(topicBuf, topic1) != 0 && strcmp(topicBuf, topic2) != 0) return;

  // Limit buffer size to avoid overflow (adjust size if needed)
  const unsigned int BUF_SZ = 32;
  char buf[BUF_SZ];
  unsigned int n = length < (BUF_SZ - 1) ? length : (BUF_SZ - 1);
  memcpy(buf, payload, n);
  buf[n] = '\0'; // null-terminate

  // Print using DBG_PRINT macros (or Serial if you prefer)
  DBG_PRINT("Topic: "); DBG_PRINTLN(topicBuf);
  DBG_PRINT("Payload (ASCII): "); DBG_PRINTLN(buf);
}
*/

void connectMqtt() {
  while (!PSclient.connected()) {
    DBG_PRINTLN("Connecting MQTT...");
    if (PSclient.connect("arduinoClient", SECRET_HA_USER, SECRET_HA_PASS)) {
      DBG_PRINTLN("connected");
      PSclient.subscribe(topicPlus);
      DBG_PRINT("Subscribed to: ");
      DBG_PRINTLN(topicPlus);
      PSclient.subscribe(topicMinus);
      DBG_PRINT("Subscribed to: ");
      DBG_PRINTLN(topicMinus);
      DBG_PRINT("MQTT connected? ");
      DBG_PRINTLN(PSclient.connected());  //print some bool
    } else {
      DBG_PRINT("failed, rc=");
      DBG_PRINTLN(PSclient.state());
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
      barColor = strip.Color(120, 80, 0);     // Orange
    }
    else {
      barColor = strip.Color(120, 0, 0);       // Red
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

  // Draw full bar
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, barColor);
  }

  // Draw needle dot
  uint8_t needle = map(value, 0, maxValue, 0, LED_COUNT - 1);
  strip.setPixelColor(needle, strip.Color(155, 155, 155));

  strip.show();
}