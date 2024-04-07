#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <privateConfig.h>

/*
 * This is an Esp32 MQTT interface for up till eight Carlo Gavazzi energy meters type
 * EM23 DIN and Type EM111.
 * 
 * The interface will publish data to Home Assistant (HA) and Google sheets.
 *
 * The interface will publish kWh (Totals), subtotals and a calculated power consumption to a 
 * MQTT broker for each energy meter.
 * The MQTT topic and payload will be compliant with the Home Assistant (HA) integration
 * MQTT sensor and number integration's.
 * 
 * Subtotals is a periodic count, resat every time ‘true’ is published to the topic: 
 * ‘energy/monitor_ESP32_48E72997D320/subtotal_reset’ at the MQTT broker connected.
 * 
 * Upon reset of subtotals, the interface will publish kWh and subtotals to Google sheets through
 * a HTTPS GET request, using Google sheet’s doGet() function.
 *
 * The Google sheet’s doGet() will add the values passed to the specific sheet.
 *
 * Totals (kWh) can be preset through the MQTT interface following the JSON Document format.
 * Publishing Payload:
 *  {
 *    "Total" : pulse-counts**
 *  }
 * to topic: energy/monitor_ESP32_48E72997D320/<Energy meter number***>/threshold
 * 
 * ** pulse-counts in the number of pulses counted by the energy meter. It is calculated by the kWh,
 * shown on the energy meter multiplied be the number of pulses per kWh for the energy meter. 
 *  Pulses per kWh can be found in the documentation for the energy meter.
 * *** ‘Energy meter number’ is a number 0..7 for the channel, on which the energy meter is connected.
 * The relation between energy meter and channel number is defined in the privateConfig.h file. 
 */ 

#define SKETCH_VERSION "PlatfomIO projext: Esp32 MQTT interface for Carlo Gavazzi energy meter - V1.0.0"

/* Version history:
 *  1.0.0   Initial production version.
 *
 *  Future versions:
 *  - Imlement ESP.restart() if required. Consider to implement EEPROM.write() https://randomnerdtutorials.com/esp32-flash-memory/
 *    during call to ESP.resart(). Writes for every update will kill flash memory. 
 *  - Implement read/write from/to SD memory card instead of using MQTT broker as a storage for configuration data and energy meter counts.
 *    Ideas: as writes to SD for each Energy meter count will be intensive, implement a method to write thise data to diffenret locations.    
 */

/*
 * ######################################################################################################################################
 * ######################################################################################################################################
 *                       V  A  R  I  A  B  L  E      D  E  F  I  N  A  I  T  O  N  S
 * ######################################################################################################################################
 * ######################################################################################################################################
 */
#define NO_OF_MQTT_ATTEMPTS 100000      // Define the number of times mqttClient.loop will be called in the initialation phase, where
                                        // stored values at the MQTT broker is read. 
                                        // Too less and not all entries will be read.
                                        // Too many the program stays in the loop too long, and energy meter pulses might be missed.

#define WIFI_CONNECT_POSTPONE 2000      // Number of millis between WiFi connect attempts, when WiFi.begin fails to connect. 
                                        // To prevent unessecary connetc attempts.
#define BLIP 250                        // Time in milliseconds the LED will blink
#define MAX_COMSUMPTION 2200            // Maximum of Watt comsumptino to be registrated. A software method to prevent fake pulses
#define MIN_CONSUMPTION 25              // Define the minimum powerconsumption published before publishing 0
                                        // See '>>>    Pulse time check  <<<' in the last part of the loop() for further details.
#define RETAINED true                   // Used in MQTT puplications. Can be changed during development and bugfixing.
#define UNRETAINED false
#define MAX_NO_OF_CHANNELS 8

/* Configurable MQTT difinitions
 * These definitions can be changed to suitable nanes.
 * Chagens might effect other inntegrations and configurations in HA!
 */

const String   MQTT_CLIENT = "Carlo-Gavazzi-Energy-Meters_";       // mqtt client_id prefix. Will be suffixed with Esp32 mac to make it unique

const String  MQTT_PREFIX                   = "energy/";              // include tailing '/' in prefix!
const String  MQTT_DEVICE_NAME              = "monitor_ESP32_";       // only alfanumeric and no '/'
const String  MQTT_DISCOVERY_PREFIX         = "homeassistant/";       // include tailing '/' in discovery prefix!
const String  MQTT_PREFIX_DEVICE            = "meter_";
const String  MQTT_ONLINE                   = "/online"; 
const String  MQTT_STATUS                   = "status";
const String  MQTT_SENSOR_ENERG_ENTITYNAME  = "Subtotal";  // name dislayed in HA device. No special chars, no spaces
const String  MQTT_SENSOR_POWER_ENTITYNAME  = "Forbrug";   // name dislayed in HA device. No special chars, no spaces
const String  MQTT_NUMBER_ENERG_ENTITYNAME  = "Total";     // name dislayed in HA device. No special chars, no spaces
const String  MQTT_PULSTIME_CORRECTION      = "pulscorr";
const String  MQTT_SKTECH_VERSION           = "/sketch_version";
const String  MQTT_SUFFIX_CONFIG            = "/config";
const String  MQTT_SUFFIX_VALUES            = "/values";
const String  MQTT_SUFFIX_TOTAL_TRESHOLD    = "/threshold";
const String  MQTT_SUFFIX_STATE             = "/state";
const String  MQTT_SUFFIX_SUBTOTAL_RESET    = "/subtotal_reset";
const String  MQTT_SUFFIX_CONSUMPTION       = "/watt_consumption";

/*  None configurable MQTT definitions
 *  These definitions are all defined in 'HomeAssistand -> MQTT' and cannot be changed.
*/
const String  MQTT_SENSOR_COMPONENT         = "sensor";
const String  MQTT_NUMBER_COMPONENT         = "number";
const String  MQTT_ENERGY_DEVICECLASS       = "energy";
const String  MQTT_POWER_DEVICECLASS        = "power";

String mqttDeviceNameWithMac;
String mqttClientWithMac;

int GlobalIRQ_PIN_index = 0;

// Define array of GPIO pin numbers used for IRQ.
const int channelPin[MAX_NO_OF_CHANNELS] = {private_Metr1_GPIO,private_Metr2_GPIO,private_Metr3_GPIO,private_Metr4_GPIO,
                                                    private_Metr5_GPIO,private_Metr6_GPIO,private_Metr7_GPIO,private_Metr8_GPIO};  

byte totalsSetForPin = 0;                       // Each bit in this byte represent a status for a specific energy meter.
                                                // A bit is set when an interrupt for the specific energy meter occours.

bool versionTopic_NOT_Published = true;               // Only publish SKETCH_VERSION once
bool totalsUpdatedFromMQTT = false;                   // Update totals from MQTT (Backup) only at startup.
bool configurationPublished[PRIVATE_NO_OF_CHANNELS];  // a flag for publishing the configuration to HA if required.
bool esp32Connected = false;                          // Is true, when connected to WiFi and MQTT Broker
bool LED_Toggled = false;                             
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Define structure for energy meter counters
struct data_t
  {
    long pulseTotal;                // For counting total number of pulses on each Channel
    long pulseSubTotal;             // For counting number of pulses within a period 
    unsigned long pulseTimeStamp;   // Stores timestamp, Usec to calculate millis bewteen pulses ==> Calsulate consupmtion.
    unsigned long pulseLength;      // Sorees time between pulses. Used to publish 0 to powerconsumption, when pulses stops arriving == poser comsumptino reduced
    int  pulse_per_kWh;             // Number of pulses as defined for each energy meter
  } meterData[PRIVATE_NO_OF_CHANNELS];

unsigned long pulseTimeCorrection = 0;  // Used to calibrate the calculated consumption.
unsigned long WiFiConnectAttempt = 0;   // 
unsigned long WiFiConnectPostpone = 0;
unsigned long LED_toggledAt = 0;
volatile unsigned long millsTimeStamp[PRIVATE_NO_OF_CHANNELS];  // Used by the ISR to store exactly when an interrupt occoured. 
                                                                // Used to calculate consuption.
volatile uint8_t  IRQ_PINs_stored = 0b00000000;   // Used by the ISR to register which energy meter caused an interrupt.

/*
 * ###################################################################################################
 * ###################################################################################################
 * ###################################################################################################
 *                       F  U  N  C  T  I  O  N      D  E  F  I  N  I  T  I  O  N  S
 * ###################################################################################################
 * ###################################################################################################
 * ###################################################################################################
*/

/* ###################################################################################################
 *                     I N I T I A L I Z E   G L O B A L S
 * ###################################################################################################
 */
void initializeGlobals() {
  // >>>>>>>>>>>>>   Set defaults for meterData.
  for (int ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++) {
    meterData[ii].pulseTotal = 0;
    meterData[ii].pulseSubTotal = 0;
    meterData[ii].pulseTimeStamp = 0;
    meterData[ii].pulseLength = 0;
    meterData[ii].pulse_per_kWh = private_default_pulse_per_kWh[ii];

    // >>>>>>>>>>    Set flag for publishing HA configuration   <<<<<<<<<<<<< 
    configurationPublished[ii] = false;
  }

  // >>>>>>>>>>>>>   Set globals for MQTT Device and Client   <<<<<<<<<<<<<<<<<<
  uint8_t mac[6];
  WiFi.macAddress(mac);

  char macStr[13] = {0};
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  mqttDeviceNameWithMac = String(MQTT_DEVICE_NAME + macStr);
  mqttClientWithMac = String(MQTT_CLIENT + macStr);
}


/* ###################################################################################################
 *                     B Y T E    S E T
 * ###################################################################################################
*/
/*
 * Description
 * Sets (writes a '1' to) a number of bits of a returned byte value.
 * 
 * Syntax
 * byte byteVariable = bitSet(NumberOfBitsToBeSet);
 * 
 * Parameters
 * NumberOfBitsToBeSet: the numeric variable for the nuymber of bits to be set,
 * starting at the least-significant (rightmost) bit.
 * 
 * Returns
 * byte
 * 
 * Example
 * bitSet(5), returnes '0b00011111'
 */
byte byteset(u_int8_t numberOfBitsToSet) {
  byte byteVariable = 0;
  for ( int ii = 0; ii < numberOfBitsToSet; ii++) {
    bitSet(byteVariable, ii);
  }
  return byteVariable;
}

/* ###################################################################################################
 *                  P U B L I S H   S K E T C H   V E R S I O N
 * ###################################################################################################
 *  As the sketch will esecute in silence, one way to se which version of the SW is running will be
 *  to subscribe to topic defined as (MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_SKTECH_VERSION)
 *  on the MQTT broker, configured in the privateConfig.h file.
 *  This will return the value of (SKETCH_VERSION)
 */
void publish_sketch_version() {  // Publish only once at every reboot.
  String versionTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_SKTECH_VERSION);
  String versionMessage = String(SKETCH_VERSION);
  mqttClient.publish(versionTopic.c_str(), versionMessage.c_str(), RETAINED);
  versionTopic_NOT_Published = false;
}

/* ###################################################################################################
 *                  P U B L I S H   S T A T U S M E S S A G E
 * ###################################################################################################
 *  Send status messages to MQTT
 * 
*/
void publishStatuMessage(String statusMessage) {  // Publish only once at every reboot.
  String statusTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + MQTT_STATUS);
  mqttClient.publish(statusTopic.c_str(), statusMessage.c_str(), RETAINED);
}

/* ###################################################################################################
 *      G E T   I R Q  P I N   r e f e r e n c e  f r o m   t o p i c   s t r i n g
 * ###################################################################################################
 * 
 * IRQ pin reference is expected to be les than 2 digits!
 */
byte getIRQ_PIN_reference(char* topic) {
  unsigned int startIndex = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/").length();
  int i = 0;
  uint8_t result = 0;
  while(topic[startIndex + i] != '/' && i < 2) {  // IRQ pin reference les than 2 digits!
    result = result * 10 + (topic[startIndex+i]-'0');
    i++;
  }
  return result;
}

/* ###################################################################################################
 *                         U P D A T E     G O O G L E     S H E E T S
 * ###################################################################################################
 * Ideas taken from:
 * https://iotdesignpro.com/articles/esp32-data-logging-to-google-sheets-with-google-scripts
 *  
 */
void updateGoogleSheets( boolean powerOn) {
  // >>>>>>>>>>>>>   Create data-URL string for HTTP request   <<<<<<<<<<<<<<<<<<
  String urlData = "/exec?meterData=";

  for ( int IRQ_PIN_index = 0; IRQ_PIN_index < PRIVATE_NO_OF_CHANNELS; IRQ_PIN_index++)
    urlData += String( float(meterData[IRQ_PIN_index].pulseTotal) / float(meterData[IRQ_PIN_index].pulse_per_kWh), 2) + ",";
  
  for ( int IRQ_PIN_index = 0; IRQ_PIN_index < PRIVATE_NO_OF_CHANNELS; IRQ_PIN_index++) {
    urlData += String( float(meterData[IRQ_PIN_index].pulseSubTotal) / float(meterData[IRQ_PIN_index].pulse_per_kWh), 2);
    if ( IRQ_PIN_index < PRIVATE_NO_OF_CHANNELS - 1)
      urlData += String(",");
  }

  if ( powerOn == true)
    urlData += String(",PowerUp");

  // >>>>>>>>>>>>>   Create URL for HTTP request   <<<<<<<<<<<<<<<<<<
  String urlFinal = "https://script.google.com/macros/s/" + PRIVATE_GOOGLE_SCRIPT_ID + urlData;

  // >>>>>>>>>>>>>   Make HTTP request to google sheet  <<<<<<<<<<<<<<<<<<
  if ( PRIVATE_UPDATE_GOOGLE_SHEET) {
    HTTPClient http;
    http.begin(urlFinal.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    // >>>>>>>>>>>>>   getting response from google sheet  <<<<<<<<<<<<<<<<<<
    if (int httpCode = http.GET()) {
      String httpMessage = http.getString();
      String statusMessage = String( String("HTTP Status Code: ") + httpCode + "HTTP Message: " + httpMessage);
      publishStatuMessage( statusMessage);
    }
    //---------------------------------------------------------------------
    http.end();
  }
}

/*
 * ###################################################################################################
 *                       U p d a t e   T o t a l s   f r o m   M Q T T
 * ###################################################################################################
 */;/*
 * Input: 
 * - Globals: totalsSetForPin. This byte variable is changed by the 'mqttCallback' function.
 *   The MQTT Callback function will set the bits in the byte totalsSetForPin
 *   when the value for a specific total and subtotal is set.
 *   'updateTotalsFromMQTT' will compare totalsSetForPin to a bitmask and set the 
 *   return value to true, if all bits in totalsSetForPin is set.
 * Return:
 * - True, when all data_t.pulseTotal and data_t.pulseSubTotal entries in the data_t sructure has been updated
 *   with values published to totalTopic and subtotalTopic
 * 
 *  This function is part of the process to restore configuration data , Totals and subTotals after a restart.
 *  When tehes data can be read from SD memory, this function is no longer needed.
 
*/
bool updateTotalsFromMQTT() {
  bool allTotalsUpdated = false;

  // >>>>>>>>>>>>>>>>  Subscribe to MQTT MQTT_SUFFIX_VALUES  <<<<<<<<<<<<<<<<<<<<<<
  String valueTopic = String( MQTT_PREFIX + mqttDeviceNameWithMac + "/+" + MQTT_SUFFIX_VALUES);
  mqttClient.subscribe(valueTopic.c_str(), 1);

  //  >>>>>>>>>>>>>>>>>>>>>   Process incomming MQTT messages   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
  for ( int ii = 0; ii < NO_OF_MQTT_ATTEMPTS & !allTotalsUpdated; ii++) {
    mqttClient.loop();
    if( totalsSetForPin == byteset(PRIVATE_NO_OF_CHANNELS)) {
      allTotalsUpdated = true;
    }
  }

  // >>>>>>>>>>>>>> Unsubscribe valueTopic <<<<<<<<<<<<<<<<<<<<<<<<<
  mqttClient.unsubscribe(valueTopic.c_str());

  return allTotalsUpdated;
}

/*
 * ###################################################################################################
 *              P U B L I S H   M Q T T   E N E R G Y   C O N F I G U R A T I O N
 * ###################################################################################################
 * 
 * component can take the values: "sensor" or "number"
 * device_class can take the values "energy" or "power"
*/
/*  Eksempel på Topic og Payload for MQTT sensor integration, deviceClass = "energy":
 *  Where : Component = MQTT_SENSOR_COMPONENT and PIN_reference = 0
 *
 *  energyTopic: homeassistant/sensor/energy/meter_0/config
 *  energyMessage:
 *  {
 *    "name": "Subtotal",
 *    "state_topic": "homeassistant/energy/meter_0/state",
 *    "device_class": "energy",
 *    "unit_of_measurement": "kWh",
 *    "unique_id": "Subtotal_meter_0",
 *    "value_template": "{{ value_json.Subtotal}}",
 *    "device":{
 *      "identifiers":[
 *        "meter_0"
 *      ],
 *      "name": "Energi - Varmepumpe SMO 40 E1"
 *    }
 *  }
 */
/*  Eksempel på Topic og Payload for MQTT sensor integration, deviceClass = "power":
 *  Where : Component = MQTT_SENSOR_COMPONENT and PIN_reference = 0
 *
 *  energyTopic: homeassistant/sensor/energy/meter_0/config
 *  energyMessage:
 *  {
 *    "name": "Forbrug",
 *    "state_topic": "homeassistant/energy/meter_0/state",
 *    "device_class": "power",
 *    "unit_of_measurement": "W",
 *    "unique_id": "Forbrug_meter_0",
 *    "value_template": "{{ value_json.Forbrug}}",
 *    "device":{
 *      "identifiers":[
 *        "meter_0"
 *      ],
 *      "name": "Energi - Varmepumpe SMO 40 E1"
 *     }
 *  }
 */
/*  Eksempel på Topic og Payload for MQTT Numner integration, deviceClass = "energy":
 *  Where : Component = MQTT_NUMBER_COMPONENT and PIN_reference = 0
 *
 *  energyTopic: homeassistant/number/energy/meter_0/config
 *  energyMessage:
 *  {
 *    "command_topic" : "energy/monitor_ESP32_48E72997D320/threshold",
 *    "command_template" : {"Total": {{ value }} },
 *    "max" : "99999.99",
 *    "min" : "0.0",
 *    "name": "Total",
 *    "state_topic": "homeassistant/energy/meter_0/state",
 *    "device_class": "energy",
 *    "unit_of_measurement": "kWh",
 *    "unique_id": "Total_meter_0",
 *    "value_template": "{{ value_json.Total}}",
 *    "device":{
 *      "identifiers":[
 *        "meter_0"
 *      ],
 *      "name": "Energi - Varmepumpe SMO 40 E1"
 *    }
 *  }
 * 
 */
/*  Eksempel på Topic og Payload for opdatering af state (Værdier)
Topic_ homeassistant/energy/meter_0/state
Payload:
{
	"Subtotal" : "123",
  "Forbrug" : "456",
  "Total" : "789"
}
 */
void publishMqttEnergyConfigJson( String component, String entityName, String unitOfMesurement, String deviceClass, u_int8_t PIN_reference) {
  uint8_t payload[1024];
  JsonDocument doc;
  String energyMeter = String( private_energyMeters[PIN_reference]);

  if ( component == MQTT_NUMBER_COMPONENT & deviceClass == MQTT_ENERGY_DEVICECLASS) {
    doc["command_topic"] = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + PIN_reference + MQTT_SUFFIX_TOTAL_TRESHOLD);
    doc["command_template"] = String("{\"" + entityName + "\": {{ value }} }");
    doc["max"] = 99999.99;
    doc["min"] = 0.0;
    doc["step"] = 0.01;
  }
  doc["name"] = entityName;
  doc["state_topic"] = String(MQTT_DISCOVERY_PREFIX + MQTT_PREFIX + MQTT_PREFIX_DEVICE + PIN_reference + MQTT_SUFFIX_STATE);
  doc["availability_topic"] = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE);
  doc["payload_available"] = "True";
  doc["payload_not_available"] = "False";
  doc["device_class"] = deviceClass;
  doc["unit_of_measurement"] = unitOfMesurement;
  doc["unique_id"] = String(entityName + "_" + MQTT_PREFIX_DEVICE + PIN_reference);
  doc["qos"] = 0;

  if ( component == MQTT_SENSOR_COMPONENT & deviceClass == MQTT_POWER_DEVICECLASS)
    doc["value_template"] = String("{{ value_json." + entityName + "}}");
  else 
    doc["value_template"] = String("{{ value_json." + entityName + " | round(2)}}");

  JsonObject device = doc["device"].to<JsonObject>();

  device["identifiers"][0] = String(MQTT_PREFIX_DEVICE + PIN_reference);
  device["name"] = String("Energi - " + energyMeter);

  size_t length = serializeJson(doc, payload);
  String energyTopic = String( MQTT_DISCOVERY_PREFIX + component + "/" + deviceClass + "/" + MQTT_PREFIX_DEVICE + PIN_reference + "/config");

  mqttClient.publish(energyTopic.c_str(), payload, length, UNRETAINED);
}

/*
 * ###################################################################################################
 *                       P U B L I S H   M Q T T   C O N F I G U R A T I O N S  
 * ###################################################################################################
*/
void publishMqttConfigurations( uint8_t device) {

  publishMqttEnergyConfigJson(MQTT_SENSOR_COMPONENT, MQTT_SENSOR_ENERG_ENTITYNAME, "kWh", MQTT_ENERGY_DEVICECLASS, device);
  publishMqttEnergyConfigJson(MQTT_SENSOR_COMPONENT, MQTT_SENSOR_POWER_ENTITYNAME, "W", MQTT_POWER_DEVICECLASS, device);
  publishMqttEnergyConfigJson(MQTT_NUMBER_COMPONENT, MQTT_NUMBER_ENERG_ENTITYNAME, "kWh", MQTT_ENERGY_DEVICECLASS, device);

  configurationPublished[device] = true;
}

/*
 * ###################################################################################################
 *                       P U B L I S H   S E N S O R   J S O N
 * ###################################################################################################
*/
/*  Eksempel på Topic og Payload for opdatering af state (Værdier)
Topic_ homeassistant/energy/meter_0/state
Payload:
{
	"Subtotal" : "123",
  "Forbrug" : "456",
  "Total" : "789"
} 
*/
void publishSensorJson( long powerConsumption, uint8_t IRQ_PIN_index) {
  uint8_t payload[256];
  JsonDocument doc;

  doc[MQTT_SENSOR_ENERG_ENTITYNAME] = float(meterData[IRQ_PIN_index].pulseSubTotal) / float(meterData[IRQ_PIN_index].pulse_per_kWh);
  doc[MQTT_SENSOR_POWER_ENTITYNAME] = powerConsumption;
  doc[MQTT_NUMBER_ENERG_ENTITYNAME] = float(meterData[IRQ_PIN_index].pulseTotal) / float(meterData[IRQ_PIN_index].pulse_per_kWh);

  size_t length = serializeJson(doc, payload);
  String sensorTopic = String(MQTT_DISCOVERY_PREFIX + MQTT_PREFIX + MQTT_PREFIX_DEVICE + IRQ_PIN_index + MQTT_SUFFIX_STATE);

  mqttClient.publish(sensorTopic.c_str(), payload, length, UNRETAINED);
}

/*
 * ###################################################################################################
 *                       P U B L I S H   T O T A L S  
 * ###################################################################################################
*/
/*  Eksempel på Topic og Payload for publisering af totals, subtotals og pulseTimeCorrection
 *  Topic: energy/monitor_ESP32_48E72997D320/1/values
 *  Payload:
 *  {
 *    "Subtotal" : 123,
 *    "Total" : 789,
 *    "pulscorr" : 25
 *  }
*/
/*  Alternative construction using serializeJson 
*/
void publishTotalsJson( u_int8_t PIN_reference) {
  uint8_t payload[256];
  JsonDocument doc;

  doc[MQTT_NUMBER_ENERG_ENTITYNAME] = meterData[PIN_reference].pulseTotal;
  doc[MQTT_SENSOR_ENERG_ENTITYNAME] = meterData[PIN_reference].pulseSubTotal;
  doc[MQTT_PULSTIME_CORRECTION] = pulseTimeCorrection;

  size_t length = serializeJson(doc, payload);
  String valueTopic = String( MQTT_PREFIX + mqttDeviceNameWithMac + "/" + PIN_reference + MQTT_SUFFIX_VALUES);

  mqttClient.publish(valueTopic.c_str(), payload, length, RETAINED);
}

/*
 * ###################################################################################################
 *                       M Q T T   C A L L B A C K  
 * ###################################################################################################
 */;/*
 * For more information about MQTT, visit: https://mqtt.org/
 * For more information about pubsubclient, mqttCallback, visit:
 * - https://pubsubclient.knolleary.net/
 * - https://pubsubclient.knolleary.net/api
 * - https://arduinojson.org
 * The function gets a IRQ pin reference number from the topic and
 * handles messages with the folowing suffixes:
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;                         // 
  byte IRQ_PIN_reference = 0;
  String topicString = String(topic);

  if( topicString.startsWith(MQTT_PREFIX)) {
    IRQ_PIN_reference = getIRQ_PIN_reference(topic);
  }

  if( topicString.endsWith(MQTT_SUFFIX_VALUES)) {
    deserializeJson(doc, payload, length);
    meterData[IRQ_PIN_reference].pulseTotal += long(doc[MQTT_NUMBER_ENERG_ENTITYNAME]);
    meterData[IRQ_PIN_reference].pulseSubTotal += long(doc[MQTT_SENSOR_ENERG_ENTITYNAME]);
    pulseTimeCorrection = long(doc[MQTT_PULSTIME_CORRECTION]);
    bitSet(totalsSetForPin, IRQ_PIN_reference);
    publishMqttConfigurations( IRQ_PIN_reference);
  } else
  if ( topicString.endsWith(MQTT_SUFFIX_TOTAL_TRESHOLD)) {
    deserializeJson(doc, payload, length);
    meterData[IRQ_PIN_reference].pulseTotal = long(float(doc[MQTT_NUMBER_ENERG_ENTITYNAME]) * float(private_default_pulse_per_kWh[IRQ_PIN_reference]));
    long watt_consumption = 0;
    publishTotalsJson( IRQ_PIN_reference);
    publishSensorJson( watt_consumption, IRQ_PIN_reference);
  } else
  /* Set pulse time correction in milliseconds. Done by:
   * Publish: {"pulscorr" : 25}
   * To topic: energy/monitor_ESP32_48E72997D320/config
   */
  if ( topicString.endsWith(MQTT_SUFFIX_CONFIG)) {
    deserializeJson(doc, payload, length);
    pulseTimeCorrection = long(doc[MQTT_PULSTIME_CORRECTION]);
  } else
  /* Publish totals, subtotals to google sheets and reset subtotals. Done by
   * Publish: true
   * To topic: energy/monitor_ESP32_48E72997D320/subtotal_reset
   */
  if ( topicString.endsWith(MQTT_SUFFIX_SUBTOTAL_RESET)) {
    updateGoogleSheets( false);
    for ( int ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++) {
      meterData[ii].pulseSubTotal = 0;
    }
  } else
  if ( topicString.endsWith(MQTT_STATUS)) {
    for (int ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++) {
      configurationPublished[ii] = false;
    } 
  }
}

/*
 * ###################################################################################################
 *                   S T O R E   I R Q    P I N  
 * ###################################################################################################
 */;/*
 *  ISR handler function
 *  The function wil receive a reference to i specific BIT in the 8 bit variable 'IRQ_PINs_stored'
 *  and set the BIT using the  bitSet() function.
 * 
 * Because the function is part of the ISR functionality it is defined as IRAM_ATTR
*/
void IRAM_ATTR store_IRQ_PIN(u_int8_t BIT_Reference) {
  millsTimeStamp[BIT_Reference] = millis();
  bitSet(IRQ_PINs_stored, BIT_Reference);
}

/*
 * ###################################################################################################
 *                  I S R    Functions
 * ###################################################################################################
*/;/*
 *  IRS - Define an ISR for each Interrupt channel, which call the common interrupt handler function.
*/
void IRAM_ATTR Ext_INT1_ISR() {
  store_IRQ_PIN( 0);
}
void IRAM_ATTR Ext_INT2_ISR() {
  store_IRQ_PIN( 1);
}
void IRAM_ATTR Ext_INT3_ISR() {
  store_IRQ_PIN( 2);
}
void IRAM_ATTR Ext_INT4_ISR() {
  store_IRQ_PIN( 3);
}
void IRAM_ATTR Ext_INT5_ISR() {
  store_IRQ_PIN( 4);
}
void IRAM_ATTR Ext_INT6_ISR() {
  store_IRQ_PIN( 5);
}
void IRAM_ATTR Ext_INT7_ISR() {
  store_IRQ_PIN( 6);
}
void IRAM_ATTR Ext_INT8_ISR() {
  store_IRQ_PIN( 7);
}

/*
 * ###################################################################################################
 * ###################################################################################################
 * ###################################################################################################
 *                       S E T U P      B e g i n
 * ###################################################################################################
 * ###################################################################################################
 * ###################################################################################################
 */
void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);             // Initialize build in LED           
  digitalWrite(LED_BUILTIN, HIGH);          // Turn ON LED to indicate startup

  
  initializeGlobals();

  // Initialize Interrupt pins
  for ( u_int8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++) {
    pinMode(channelPin[ii], INPUT_PULLUP);
  }

  // arm interrupt. Create a functioncall for each interrupt pin
  attachInterrupt(private_Metr1_GPIO, Ext_INT1_ISR, FALLING);
  attachInterrupt(private_Metr2_GPIO, Ext_INT2_ISR, FALLING);
  attachInterrupt(private_Metr3_GPIO, Ext_INT3_ISR, FALLING);
  attachInterrupt(private_Metr4_GPIO, Ext_INT4_ISR, FALLING);
  attachInterrupt(private_Metr5_GPIO, Ext_INT5_ISR, FALLING);
  attachInterrupt(private_Metr6_GPIO, Ext_INT6_ISR, FALLING);
  attachInterrupt(private_Metr7_GPIO, Ext_INT7_ISR, FALLING);
  attachInterrupt(private_Metr8_GPIO, Ext_INT8_ISR, FALLING);

  mqttClient.setServer(PRIVATE_MQTT_SERVER.c_str(), PRIVATE_MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  digitalWrite(LED_BUILTIN, LOW);           // Turn OFF LED before entering loop
}

/*
 * ###################################################################################################
 * ###################################################################################################
 * ###################################################################################################
 *                       L O O P     B e g i n
 * ###################################################################################################
 * ###################################################################################################
 * ###################################################################################################
 */
void loop() {

// >>>>>>>>>>>>>>>>>>>>>>>>   Connect to WiFi if not connected    <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  if (WiFi.status() != WL_CONNECTED and millis() > WiFiConnectAttempt + WiFiConnectPostpone) {

    digitalWrite(LED_BUILTIN, HIGH);  // Turn ON the LED to indicate WL connections is lost.

    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(PRIVATE_WIFI_SSID.c_str(), PRIVATE_WIFI_PASS.c_str());

    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      WiFiConnectAttempt = millis();
      WiFiConnectPostpone = WIFI_CONNECT_POSTPONE;
      esp32Connected = false;
    } else {
      /*¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
      *    Arduino basicOTA  impementation
      *¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
      // Port defaults to 3232
      // ArduinoOTA.setPort(3232);

      // Hostname defaults to esp3232-[MAC]
      // ArduinoOTA.setHostname("myesp32");

      // No authentication by default
      // ArduinoOTA.setPassword("admin");

      // Password can be set with it's md5 value as well
      // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
      // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
      */

      ArduinoOTA
        .onStart([]() {
          String type;
          if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
          else // U_SPIFFS
            type = "filesystem";

          // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
          Serial.println("Start updating " + type);
        })
        .onEnd([]() {
          Serial.println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
          Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
          Serial.printf("Error[%u]: ", error);
          if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
          else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
          else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
          else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
          else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });

      ArduinoOTA.begin();

      /*¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
      *    E N D      Arduino basicOTA  impementation
      *¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
      */

    }
  }
// >>>>>>>>>>>>>>>>>>>>>>>>   E N D  Connect to WiFi if not connected    <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>    IF  Connected to WiFi -> Connect to MQTT broker    <<<<<<<<<<<<<<<<<<<<<<<<<
  if (WiFi.status() == WL_CONNECTED) {
    if ( !LED_Toggled) {
      digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off to indicate WL is connected
    }

    ArduinoOTA.handle();
    
    // >>>>>>>>>>>>>>>>>>> Connect to MQTT Broker <<<<<<<<<<<<<<<<<<<<<<<<<<
    if (!mqttClient.connected()) {
      String will = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE);
      if (mqttClient.connect(mqttClientWithMac.c_str(), PRIVATE_MQTT_USER.c_str(), PRIVATE_MQTT_PASS.c_str(), will.c_str(), 1, RETAINED, "False") ) {
        esp32Connected = true;

        //   >>>>>>>>>>>>>>>>>>>>>   publish will and SKETCH_VERSION   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if (versionTopic_NOT_Published) // Publish only once at every reboot.
          publish_sketch_version();

        //   >>>>>>>>>>>>>>>>>>>>>>   Update Totals from MQTT  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if ( !totalsUpdatedFromMQTT) {
          totalsUpdatedFromMQTT = true;  // To prevent Meter Totals from getting updated from MQTT more than once at every restart

          updateTotalsFromMQTT();  
  
          //   >>>>>>>>>>>>>>>>>>   Update Google sheets and set powerUp  <<<<<<<<<<<<<<<<<<<<<<<<<
          updateGoogleSheets( true);
        }

        //   >>>>>>>>>>>>>>>>>>>>  Subscribe to Set Totals and Reset Suttotals topic  <<<<<<<<<<<<<<<<<
        String totalSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+" + MQTT_SUFFIX_TOTAL_TRESHOLD);
        mqttClient.subscribe(totalSetTopic.c_str(), 1);

        String subTotalSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_SUFFIX_SUBTOTAL_RESET);
        mqttClient.subscribe(subTotalSetTopic.c_str(), 1);

        String configSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_SUFFIX_CONFIG);
        mqttClient.subscribe(configSetTopic.c_str(), 1);

        String statusSetTopic = String(MQTT_DISCOVERY_PREFIX + MQTT_STATUS);
        mqttClient.subscribe(statusSetTopic.c_str(), 1);

        mqttClient.publish(will.c_str(), (const uint8_t *)"True", 4, RETAINED);
        
      } else 
        esp32Connected = false;
    }
  }
  // >>>>>>>>>>>>>>>>>>>  E N D IF  Connected to WiFi -> Connect to MQTT broker  <<<<<<<<<<<<<<<<<<<<<<<<<<

  // >>> Process incomming messages and maintain connection to the server
  if ( esp32Connected) {
    if(!mqttClient.loop())
      esp32Connected = false;
  }
  //  <<< END Process incomming messages

  // >>>>>>>>>>>>>>>>>>>>   Publis meterData (If any)   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  if ( IRQ_PINs_stored > 0) {                      // If IRQ has occoured IRQ_PINs_store will be > 0.
    uint8_t pinMask = 0b00000001;                             // Set pinMask to start check if the Least significant Bit (LSB) is set ( = 1)

    // >>>>>>>>>>>>>>>>  Tuggle LED pin if not toggled allready  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    if ( !LED_Toggled) {
      digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
      LED_Toggled = true;
      LED_toggledAt = millis();
    }

    for( int IRQ_PIN_index = 0; IRQ_PIN_index < PRIVATE_NO_OF_CHANNELS; IRQ_PIN_index++) { // Iterate through all bits in the byte IRQ_PINs_stored.
      /*
        *  If bit in IRQ_PINs_stored matches the bit set in pinMask: Calculate powerconsumption and publist data.
        */
      if( IRQ_PINs_stored & pinMask ) {                       // If bit is set 
        if( esp32Connected and !configurationPublished[IRQ_PIN_index]) {
          publishMqttConfigurations( IRQ_PIN_index);
        }
        //   >>>>>>>>>>>>>>>>>>>>>>>>>>>  Calculate power comsumption   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        /*
          * It does not make sence to calculate consumption when the privious pulse is unkown (0) or when millis() has owerflown and millsTimeStamp
          * is before pulseTimeStamp.
          * When millis overflows, the pulsetime (millsTimeStamp - pulseTimeStamp + pulseTimeCorrection) could ofcause be calculated, but it brings complexity to the code 
          * but only saves one comsumption calculation every 50 days...
          */
        long watt_consumption = 0;
        if ( meterData[IRQ_PIN_index].pulseTimeStamp > 0 && meterData[IRQ_PIN_index].pulseTimeStamp < millsTimeStamp[IRQ_PIN_index]) { 
          watt_consumption = round(((float)(60*60*1000) / 
                                (float)(millsTimeStamp[IRQ_PIN_index] - meterData[IRQ_PIN_index].pulseTimeStamp + pulseTimeCorrection)) 
                                / (float)meterData[IRQ_PIN_index].pulse_per_kWh * 1000);

          meterData[IRQ_PIN_index].pulseLength = millsTimeStamp[IRQ_PIN_index] - meterData[IRQ_PIN_index].pulseTimeStamp + pulseTimeCorrection;
        }

        //   >>>>>>>>>>>>>>>>>>>>>>>>>>>  Update meterData and publish totals   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
        if ( watt_consumption < MAX_COMSUMPTION) {
          meterData[IRQ_PIN_index].pulseTimeStamp = millsTimeStamp[IRQ_PIN_index];
          meterData[IRQ_PIN_index].pulseTotal++;
          meterData[IRQ_PIN_index].pulseSubTotal++;
          
          if ( esp32Connected) {
            publishTotalsJson( IRQ_PIN_index);
            publishSensorJson( watt_consumption, IRQ_PIN_index);
          }
        }

        bitClear(IRQ_PINs_stored, IRQ_PIN_index);                 // Set the bit back to 0, ready for next IRQ 
      } 
      pinMask <<= 1;                                              // Left shift the bit mask to check the next bit
    }  
  }

  /* >>>>>>>>>>>>>>>>>>    Pulse time check  <<<<<<<<<<<<<<<<<<<<<<<<<<

   * If a bit in IRQ_PINs_stored is set or being set during the pulse time check, further pulse time check's will be skipped
   * because publishing meterData has priority.  
   * If no bit's are set, check if then pulseTimeStamp indicate that comsumption has been reduced or stopped.
   * 
   * The assumption is: If a pulse has been missing for double the previous pulse period, comsumption has dropped
   * and a new fictive comsumptino is published based on the double of the previous pulse period.
   * This will goes on, until the calculated fictive comsumptino becomes less than the consumption, defined in MIN_CONSUMPTION.
   */
  unsigned long timeStamp = millis();
  if ( GlobalIRQ_PIN_index >= PRIVATE_NO_OF_CHANNELS)
    GlobalIRQ_PIN_index = 0;
  while ( IRQ_PINs_stored == 0 && GlobalIRQ_PIN_index < PRIVATE_NO_OF_CHANNELS) {
    // At startup pulseTimeStamp will be 0 ==> Comsumptino unknown ==> No need to recalculation
    // During exeution mills will overflow ==> Complicates calculation ==> Skip recalculation
    if ( meterData[GlobalIRQ_PIN_index].pulseLength > 0 && meterData[GlobalIRQ_PIN_index].pulseTimeStamp < timeStamp) {
      if (  meterData[GlobalIRQ_PIN_index].pulseTimeStamp + ( 2 * meterData[GlobalIRQ_PIN_index].pulseLength) < timeStamp ) {
        
        long watt_consumption = round(((float)(60*60*1000) / 
                                  (float)(timeStamp - meterData[GlobalIRQ_PIN_index].pulseTimeStamp + pulseTimeCorrection)) 
                                  / (float)meterData[GlobalIRQ_PIN_index].pulse_per_kWh * 1000);

        if ( watt_consumption < MIN_CONSUMPTION) {
          watt_consumption = 0;
          meterData[GlobalIRQ_PIN_index].pulseLength = 0;
        }
        if ( esp32Connected)
          publishSensorJson( -1 * watt_consumption, GlobalIRQ_PIN_index);

        meterData[GlobalIRQ_PIN_index].pulseLength *= 2;
      }
    }
    GlobalIRQ_PIN_index++;
  }
  
  /* Tuggle LED Pin if tuggled and BLIP time has passed
  */
  if ( LED_Toggled and millis() > LED_toggledAt + BLIP) {
    digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
    LED_Toggled = false;
    LED_toggledAt = 0;
  }
 
}