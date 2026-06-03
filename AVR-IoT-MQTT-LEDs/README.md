# AVR-IoT reading MQTT Data and displaying it on a LED status bar

## Components
### Summary
An Arduino file that uses an AVR-IoT board, connected via wifi, is reading data from an MQTT broker and displays a meter/needle on an LED strip.

### Motivation
My electric power provider is charging me (ha!) per kWh _and_ for "bandwidth". If I stay above certain thresholds (2, 5, 10 kW) for a certain time (interesting ruleset behind that) my "energiledd" or "kapasitetstrinn" will change for the next month. This information is not indicated in a regular meter display, but can be read out from the meter's HAN port - the AVR-IoT-HANSensor project is reading the HAN port and conveying it to an MQTT broker.

This project reads the output of the AVR-IoT-HANSensor on the MQTT broker, and lights up an LED strip with according color schemes.

If this can help you to stay under the next threshold/price step for a couple of months a year, you'll be saving dozens of dollars.


### Parts used
* AVR IoT board (doesn't matter whether it's a WG or AWS, we're not using outside forces)
    * Wifi connectivity to access MQTT Broker
    * An interesting topic with integer payload (AVR-IoT-HANSensor can provide that)
    * PA4 aka. pin 2 of the package, aka. pin 6 in Arduino's world, is addressing the LED bar
    * The output is sent to an MQTT Broker, where it can be used by Homeassistant
* An LED strip of WS2812-type. The length isn't too important, I am using a strip with 25 (adjust `LED_COUNT` in the àvr-iot.h` to your system). 
* Some power solution for the AVR-IoT (can be via the USB port or through the LiPo battery connector), and/or the LED stripe: The 5V pin of the AVR-IoT board can be used to power the LED, but it is limited. You may want to run a dedicated supply line when using large stripes at high brightness.


### AVR-IoT-MQTT-LEDs.ino
Based on AVR-IoT-HA, this connects to an MQTT Broker, and translates the relevant payloads to an LED-strip as a meter display. In the original configuration, it's reading both import power (you're currently consumingbuying from the grid) and export power (e.g. your solar cells produce more than you currently need, and you're selling).

|Power range kW|Color|
|:------------:|-----|
|**Importing**| white dot range 0-12 kW|
|0-5|green|
|5-8.5|yellow|
|8.5-10|orange|
|>10|red|
|**Exporting**|white dot range 0-800 W |
|<0 |blue|

### avr-iot.h
This file configures the LED strip: you must adopt the length `LED_COUNT` to your actual strip. There are also a number of threshold values for the color scheme, and the range of the white dot "needle" position.

### arduino_secrets.h
Configure the wifi name and password here, and provide the coordinates for the MQTT broker, too.


## What's left?
### Known issues
* The MQTT discovery of AVR-IoT-HANSensor doesn't work for me in HA, but the MQTT payloads from do arrive at the broker, and can be read from there. So theAVR-IoT-MQTt-LEDS code is subscribing to an explicit topic path. 


### To do
* Fix known issues
