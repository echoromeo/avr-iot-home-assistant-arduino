# AVR-IoT reading MQTT Data and displaying it on a LED status bar

## Summary
An Arduino file that uses an AVR-IoT board, connected via wifi, is reading data from an MQTT broker and displays a meter/needle on an LED strip.

## Motivation
My electric power provider is charging me (ha!) per kWh _and_ for "bandwidth". If I stay above certain thresholds (2, 5, 10 kW) for a certain time (interesting ruleset behind that) my "energiledd" or "kapasitetstrinn" will change for the next month. This information is not indicated in a regular meter display, but can be read out from the meter's HAN port - the AVR-IoT-HANSensor project is reading the HAN port and conveying it to an MQTT broker.

This project subscribes to two MQTT topics containing the current power consumption (the output of the AVR-IoT-HANSensor), and lights up an LED strip with according color schemes.

If this can help you to stay under the next threshold/price step for a couple of months a year because it reminds you to spread out the energy consumption, you'll be saving dozens of dollars.

## Components
### Parts used
* AVR IoT board (doesn't matter whether it's a WG or AWS, we're not using outside forces)
    * Wifi connectivity
    * Subscribe to an MQTT Broker that's brokering relevant topics (Use f.ex. AVR-IoT-HANSensor to provide that)
    * PA4 aka. pin 2 of the package, aka. pin 6 in Arduino's world, is addressing the LED bar
* An LED strip of WS2812-type. The length isn't too important, I am using a strip with 25 (adjust `LED_COUNT` in the àvr-iot.h` to your system). 
* Some power solution:
    * The AVR-IoT can be via the USB port or through the LiPo battery connector. 
    * The LED stripe can be powered through the 5V-pin of the mikroBus (see that the bridge next to it is closed). That pin can provide only so much power though, you may want to consider a separate supply line (common GND though).


### AVR-IoT-MQTT-LEDs.ino
This connects to an MQTT Broker, and translates two relevant payloads to an LED-strip as a meter display. In the original configuration, it's reading both import power (`topicPlus`, you're currently buying from the grid) and export power (`topicMinus`, e.g. your solar cells produce more than you currently need, so you're selling). The LED is updated every 10sec -  my utility meter is updating the export data every 10 s, and the import data every 2 s.

|Power range kW|Color|
|:------------:|------------------------|
|**Importing**| white dot range 0-12 kW|
|0-5|green|
|5-8.5|yellow|
|8.5-10|orange|
|>10|red|
|**Exporting**|white dot range 0-800 W |
|<0 |blue|

Note that due to asynchronuous timing between the utility meter, the HAN Sensor, and the MQTT broker, it may occur that both stored import and export values are simultaneously non-zero. Then the display is not altered.


### avr-iot.h
This file configures the LED strip: you must adopt the length `LED_COUNT` to your actual strip. There are also a number of threshold values for the color scheme, and the range of the white dot "needle" position.

### arduino_secrets.h
Configure the wifi name and password here, and provide the coordinates for the MQTT broker, too.

## To do
Fix known issues.
