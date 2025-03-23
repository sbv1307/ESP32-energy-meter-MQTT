#  ESP32 energy meter MQTT interface

This is an Esp32 MQTT interface for up till eight energy meters, which has a pulse output like
Carlo Gavazzi Type EM23 DIN and EM111.

The interface will publish data to Home Assistant (HA) and Google sheets.

The interface will publish kWh (Totals), subtotals and a calculated power consumption to a 
MQTT broker for each energy meter.
The MQTT topic and payload will be compliant with the Home Assistant (HA) integration
MQTT sensor and number integration's.

The interface will publish kWh (Totals) and subtotals to Google Sheets at the time defined by SCHEDULE_HOUR and SCHEDULE_MINUTE in the privateConfig.h file.

Subtotals is a periodic count, resat every time kWh (Totals) and subtotals are published to Google Sheets or **true** is published to the topic:
````bash
energy/monitor_ESP32_48E72997D320/subtotal_reset
````
at the MQTT broker connected.


Upon reset of subtotals, the interface will publish kWh and subtotals to Google sheets through
a HTTPS GET request, using Google sheet’s doGet() function.

The Google sheet’s doGet() will add the values passed to the specific sheet.

Totals (kWh) can be preset through the MQTT interface following the JSON Document format.
Publishing Payload:
````bash
 {
   "Total" : pulse-counts
 }
````

to topic: 
````bash
energy/monitor_ESP32_48E72997D320/<Energy meter number***>/threshold
````

Then latest status message (e.g. the return message from the call to Google sheets) can be collectged
by subscribing to topic:
````bash
energy/monitor_ESP32_48E72997D320/status
````

To get information about the current running version, subscribe to topic:
````bash
energy/monitor_ESP32_48E72997D320/sketch_version
````

The Esp32 MQTT interface can be restarted (rebooed) by publishing **true** to the topic:

````bash
energy/monitor_ESP32_48E72997D320/restart
````


** **pulse-counts** in the number of pulses counted by the energy meter. It is calculated by the kWh,
shown on the energy meter multiplied be the number of pulses per kWh for the energy meter. 
 Pulses per kWh can be found in the documentation for the energy meter.

*** **Energy meter number** is a number 0..7 for the channel, on which the energy meter is connected.
The relation between energy meter and channel number is defined in the privateConfig.h file.

### SD Card failure.

In case the SD card fails to record energy meter counts, the message "SD-Error" will be added to the entries in Google sheet. A more detailed message will be published to:
````bash
energy/monitor_ESP32_48E72997D320/sketch_version
````

## Calculating Consumption
Consumption is calculated on every pulse registrated. 

The time between pulses (pulse-time) is calculated as current time minus the timestamp logged, at last occourence of a pulse. Present time is logged as timestamp for the calculating the next pulse-time.

In order to handle the situation, where consumption is reduced or even stopped, then pulse-time is continuously checked.

If the timesstamp logged plus two times the pulse-time is greater than the current time it indicate that consumption has dropped, and a new fictional Consumption is calculated. It will be presented as minus value to indicate, that it is fictional.

## Compiler options
Insæt følgende i **platform.ini**

````bash
; Larger buffer is needed for HomeAssistant discovery messages, which are quite large
build_flags = -D MQTT_MAX_PACKET_SIZE=1024
``````
## Project depended libraries.
##### Installed by: PlatformIO -> PIO Home -> Open -> Libraries -> Registry "Search Libraries"
**ArduinoJson** by Benoit Blanchon  - Version 7.0.1<br>
**PubSubClient** 0                  - Version 2.8

### Local kicad-library
The Local kicad-library located at [github](https://github.com/sbv1307/kicad-library) is required for the hardware. See the README.md for more information. 
