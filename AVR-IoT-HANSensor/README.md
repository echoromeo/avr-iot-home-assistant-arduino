# AVR-IoT HAN Sensor Reading and sending the HAN port data from a Norwegian power meter

## Components
### Summary
An Arduino sketch that uses an AVR-IoT board and a [mikroE M-Bus Slave click board](https://www.mikroe.com/m-bus-slave-click) to read the HAN output of a meter and hand it to an MQTT Broker via wifi. 

### Motivation
My electric power provider is charging me (ha!) per kWh _and_ for "bandwidth". If I stay above certain thresholds (2, 5, 10 kW) for a certain time (interesting ruleset behind that) my "energiledd" or "kapasitetstrinn" will change for the next month. This information is not indicated in a regular meter display, but can be read out from the meter's HAN port. 

This project hands that power value to an MQTT Broker, where it can be used by Homeassistant, which in turn could adjust the color of a warning lamp or something. If this can help you to stay under the next threshold, you'd be saving 1-2 thousand Norwegian Kroner per year for avoiding the next step in "bandwidth".


### Parts used
* AVR IoT board (doesn't matter whether it's a WG or AWS, we're not using outside forces)
    * USART1 RX is reading the click board's output
    * The output is sent to an MQTT Broker, where it can be used by Homeassistant
* Mikroe M-Bus Slave Click - converts the 40V output from the power meter to USART TX level and format
* Half of an old Ethernet cable; Pins 0 and 1 of the RJ45 go into the terminals of the MBus click, the intact end into the HAN port
* Some power solution for the AVR-IoT
* The Kaifa MA105H2E, as installed at my house. The HAN port output had to be activated by the energy provider: there is now a little triangle blinking in the display, right above the "HAN" label. 

I'm sure other hardware combinations will work in principle, but that's what I got. The HAN port data format is normed (OBIS in Norway), so that should work with any conforming meter.


### AVR-IoT-HANSensor.ino
Based on AVR-IoT-HA, this reads the incoming stream, identifies the list type, reads the value for the active power, and sends an updated payload to the MQTT Broker

It is expecting normed settings for Norwegian utility meters: 2400 bit/s, 8 bit, parity even, 1 stop bit. The data is organized (OBIS) and comes in three types of "lists", as described for [Norwegian utility meters](https://www.nek.no/info-ams-han-utviklere/).


## The lists
The Norwegian standard requires the meter to send one list of type 1 every 2s.
Every 5th list is if type 2.
Once every 3600s (1h), a list of type 3 is sent. 

### List types

I have these from places around the interwebs, as visual aid what the lists look like:


```
char list1 [] =              //0x27 = 39 bytes between two 0x7E
    {
        0x7E, 0xA0, 0x27, 0x01, 0x02, 0x01, 0x10, 0x5A,  0x87, 0xE6, 0xE7, 0x00, 0x0F, 0x40, 0x00, 0x00,  0x00, 0x09, 0x0C, 0x07, 0xE1, 0x09, 0x0E, 0x04, 0x14, 0x00, 0x08, 0xFF, 0x80, 0x00, 0x00, 0x02,  0x01, 0x06, 0x00, 0x00, 0x03, 0xFD, 0x2B, 0x8E, 0x7E
    };

char list2 [] =              //0x79 = 121 bytes between two 0x7E
    {
        0x7E, 0xA0, 0x79, 0x01, 0x02, 0x01, 0x10, 0x80,  0x93, 0xE6, 0xE7, 0x00, 0x0F, 0x40, 0x00, 0x00, 0x00, 0x09, 0x0C, 0x07, 0xE1, 0x09, 0x0E, 0x04, 0x14, 0x00, 0x00, 0xFF, 0x80, 0x00, 0x00, 0x02,  0x0D, 0x09, 0x07, 0x4B, 0x46, 0x4D, 0x5F, 0x30,  0x30, 0x31, 0x09, 0x10, 0x36, 0x39, 0x37, 0x30, 0x36, 0x33, 0x31, 0x34, 0x30, 0x31, 0x37, 0x35,  0x33, 0x39, 0x38, 0x35, 0x09, 0x08, 0x4D, 0x41, 0x33, 0x30, 0x34, 0x48, 0x33, 0x45, 0x06, 0x00, 0x00, 0x03, 0xFC, 0x06, 0x00, 0x00, 0x00, 0x00,  0x06, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,  0x00, 0x41, 0x06, 0x00, 0x00, 0x07, 0x8D, 0x06, 0x00, 0x00, 0x0C, 0x98, 0x06, 0x00, 0x00, 0x0D,  0x5E, 0x06, 0x00, 0x00, 0x09, 0x41, 0x06, 0x00,  0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x09, 0x4C, 0xD3, 0x4F, 0x7E
    };

char list3 [] =              // 0x9B = 155 bytes between two 0x7E
    {
        0xA0, 0x9B, 0x01, 0x02, 0x01, 0x10, 0xEE,  0xAE, 0xE6, 0xE7, 0x00, 0x0F, 0x40, 0x00, 0x00,  0x00, 0x09, 0x0C, 0x07, 0xE1, 0x09, 0x0E, 0x04, 0x14, 0x00, 0x0A, 0xFF, 0x80, 0x00, 0x00, 0x02,  0x12, 0x09, 0x07, 0x4B, 0x46, 0x4D, 0x5F, 0x30,  0x30, 0x31, 0x09, 0x10, 0x36, 0x39, 0x37, 0x30, 0x36, 0x33, 0x31, 0x34, 0x30, 0x31, 0x37, 0x35,  0x33, 0x39, 0x38, 0x35, 0x09, 0x08, 0x4D, 0x41,  0x33, 0x30, 0x34, 0x48, 0x33, 0x45, 0x06, 0x00, 0x00, 0x03, 0xFE, 0x06, 0x00, 0x00, 0x00, 0x00,  0x06, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,  0x00, 0x40, 0x06, 0x00, 0x00, 0x07, 0x91, 0x06, 0x00, 0x00, 0x0C, 0x9D, 0x06, 0x00, 0x00, 0x0D,  0x66, 0x06, 0x00, 0x00, 0x09, 0x41, 0x06, 0x00,  0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x09, 0x4C, 0x09, 0x0C, 0x07, 0xE1, 0x09, 0x0E, 0x04, 0x14,  0x00, 0x0A, 0xFF, 0x80, 0x00, 0x00, 0x06, 0x00,  0x02, 0xBF, 0x69, 0x06, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0xF7, 0x06, 0x00, 0x00,  0x3F, 0xFC, 0x71, 0x71, 0x7E
    };

```
**Note**
The actual meter and/or MBus slave is sending a continuous stream “0x00” between lists , so the SoF is basically never on position [0]! 
---


### LIST1 Interpretation
```
7E					SOF
A0 27 01 02 01 10 5A 87 	// 8 byte Header stuff, 0x27 is frame length
E6 E7 00 0F 			    // 4 byte LSAP LLC stuff
40 00 00 00 			    // 4 byte unknown

09 0C 			09      // "now comes a string of length.." 0C = 12 B
07E1 09 0E 04 14 00 08 FF 80 00 00	// the 12 bytes string
// yyyy mm dd nn	hhmmss ?? ?? ?? ??	(the first eight are a timestamp)
// 2017 09 14 thu 20:00:08
 
02 01 			//  "now comes one byte:"  01, but what is it?
06 				//  "now come 4 bytes of a value"
00 00 03 FD 	// 0x3FD = 1021 "W active power +"         <<------THIS IS IT
2B 8E 			// unknown/checksum?
7E				// EOF
```

### LIST2 Interpretation
```
7E, A0 79 01 02 01 10 80 93 E6 E7 00 0F 40 00 00 00    // SOF, 16B stuff (0x79 is frame length)

09 0C: 07E1 09 0E 04 14 00 00 FF 80 00 00	// now: 12B Timestamp

02:0D 					// one byte: 0D
09 07: 4B 46 4D 5F 30 30 31		//now come 7 bytes that spell "KFM_001"

09 10: 36 39 37 30 36 33 31 34 30 31 37 35 33 39 38 35	// now 10 bytes meter ID (as written on your meter)

09 08: 4D 41 33 30 34 48 33 45			//now: 8B meter type

06: 00 00 03 FC 				// Active power +		1020W       <<---- THIS IS IT
06: 00 00 00 00 				// Active power -		0
06: 00 00 00 00 				// Reactive power +		0 kVAr	
06: 00 00 00 41 				// Reactive power -		64kVAr
06: 00 00 07 8D 				// Current phase IL1	1.933A
06: 00 00 0C 98 				// Current phase IL2	3.224A
06: 00 00 0D 5E 				// Current phase IL3	3.422A
06: 00 00 09 41 				// Phase/line voltage	236.9V
06: 00 00 00 00 				// Phase/line voltage	0V
06: 00 00 09 4C 				// Phase/line voltage	238.0V

D3 4F, 7E    					// checksum?,  EOF

```

### LIST3 Interpretation
```
7E, A0 9B 01 02 01 10 EE AE E6 E7 00 0F 40 00 00 00	 //SOF, 68 bytes as in List 2
09 0C: 07 E1 09 0E 04 14 00 0A FF 80 00 00 02: 12 	
09 07: 4B 46 4D 5F 30 30 31						
09 10: 36 39 37 30 36 33 31 34 30 31 37 35 33 39 38 35	
09 08: 4D 41 33 30 34 48 33 45 					

06: 00 00 03 FE  06: 00 00 00 00  06: 00 00 00 00  06: 00 00 00 40 // same info as LIST2 (10 values)
06: 00 00 07 91  06: 00 00 0C 9D  06: 00 00 0D 66  06: 00 00 09 41 
06: 00 00 00 00  06: 00 00 09 4C 		     

09 0C: 07 E1 09 0E 04 14 00 0A FF 80 00 00 // meter-internal time, to be identical to timestamp "09 0C:" above

06: 00 02 BF 69 		// cumultative active import energy	        180073kWh   <- as displayed on meter
06: 00 00 00 00 		// cum.act. export energy (into network)    0kWh
06: 00 00 00 F7 		// cum.reactive import 	                    247kVArh
06: 00 00 3F FC 		// cum.reactive expor t        		        16380kvArh

71 71, 7E 		        // checksum?, EOF
```

## What's left?
### Known issues
* Lists of types 2 and 3 are currently not properly detected. Effectively, the energy values and the power->out are never updated
* The MQTT discovery in HA doesn't work for me (but the MQTT payloads from AVR-IoT arrive at the broker, and can be read from there)

For now, this is what I add in my `configuration.yaml` (you'll need to change the mac address to the actual one of your AVR-IoT board):

```
mqtt:                                                                                                                                  
 sensor:                                                                                                                               
   - name: "HAN Power +"                                                                                                               
     state_topic: "aha/cce7aa05f0f8/iotHANSensorPowerPlus/stat_t"                                                                      
     unit_of_measurement: "W"                                                                                                          
     device_class: "power"                                                                                                             
     state_class: "measurement"                                                                                                        
     value_template: "{{ value | int }}"                                                                                               
   - name: "HAN Power -"                                                                                                              
     state_topic: "aha/cce7aa05f0f8/iotHANSensorPowerMinus/stat_t"                                                                     
     unit_of_measurement: "W"                                                                                                          
     device_class: "power"                                                                                                             
     state_class: "measurement"                                                                                                        
     value_template: "{{ value | int }}"                                                                                               
   - name: "HAN Energy Import"                                                                                                         
     state_topic: "aha/cce7aa05f0f8/iotHANSensorEnergyIn/stat_t"                                                                       
     unit_of_measurement: "W"                                                                                                          
     device_class: "energy"                                                                                                            
     state_class: "measurement"                                                                                                        
     value_template: "{{ value | int }}"                                                                                               
   - name: "HAN Energy Export"                                                                                                         
     state_topic: "aha/cce7aa05f0f8/iotHANSensorEnergyOut/stat_t"                                                                      
     unit_of_measurement: "W"                                                                                                          
     device_class: "energy"                                                                                                            
     state_class: "measurement"                                                                                                        
     value_template: "{{ value | int }}" 
```

### Features to do
* Fix known issues
* Add List type 3 and add a sensor "cumultative active import energy" - aka. "power meter reading" 
* Add list type 2 for gapless surveillance
