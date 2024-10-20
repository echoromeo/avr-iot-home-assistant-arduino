# AVR-IoT Home Assistant MQTT Client

This project is an AVR-IoT Home Assistant MQTT Client using the ATmega4808 microcontroller. It leverages the MegaCoreX by MCUdude for ATmega4808 support and integrates with Home Assistant via MQTT.

## Sketches
 - **AVR-IoT-HA**: The onboard AVR-IoT Light and Temperature sensors, this is the template sketch that should work out of the box and can be used to confirm WiFi and MQTT connectivity.
 - **AVR-IoT-HA-ePaper**: Waveshare 4.2" 400x300 pixels e-paper display set up with number entities to show temperature, CO2 and humidity numbers from Home Assistant.

## Libraries Used
- **[WiFi101 by Arduino](https://docs.arduino.cc/libraries/wifi101/)**: For WiFi connectivity using the WINC1510 module.
- **[home-assistant-integration by David Chyrzynski](https://github.com/dawidchyrzynski/arduino-home-assistant)**: For integrating with Home Assistant via MQTT.
- **[Adafruit MCP9808 Library by Adafruit](https://github.com/adafruit/Adafruit_MCP9808_Library)**: For interfacing with the MCP9808 temperature sensor.  

## Hardware Requirements
- AVR-IoT WG Development Board
- micro-USB Cable
- Optional Battery

## Software Requirements
- Arduino IDE with MegaCoreX installed
- The required libraries mentioned above

## Configuration
The following secrets need to be defined in the separate `arduino_secrets.h` file:
- `SECRET_SSID`: Your WiFi network SSID.
- `SECRET_PASS`: Your WiFi network password.
- `SECRET_MAC`: The MAC address of your device.
- `SECRET_HA_USER`: The Home Assistant (MQTT) username.
- `SECRET_HA_PASS`: The Home Assistant (MQTT) password.
- `SECRET_BROKER`: The MQTT broker address.
