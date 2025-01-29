#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <privateConfig.h>
#include "SD.h"
#include "SPI.h"
#include "time.h"

#define SKETCH_VERSION "Esp32 MQTT interface for Carlo Gavazzi energy meter - V4.2.0"

/*
 * This is an Esp32 MQTT interface for up till eight Carlo Gavazzi energy meters type
 * EM23 DIN and Type EM111.
 * 
 * The interface will publish data to Home Assistant (HA) and Google sheets (GS).
 *
 * The interface will publish kWh (Totals), subtotals and a calculated power consumption to a 
 * MQTT broker for each energy meter.
 * The MQTT topic and payload will be compliant with the (HA) integration
 * MQTT sensor and number integration's.
 * 
 * Subtotals is a periodic count, resat every time ‘true’ is published to the topic: 
 * ‘energy/monitor_ESP32_48E72997D320/subtotal_reset’ at the MQTT broker connected.
 * or
 * once every day, as specified in the privateConfig.h file.for when Google sheets are to be updated
 * 
 * Upon reset of subtotals, the interface will publish kWh and subtotals to GS through
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
 * Latest status can be seen by publis to topic:
 * energy/monitor_ESP32_48E72997D320/status
 * LED indications:
 * 
 * 
 * )** pulse-counts in the number of pulses counted by the energy meter. It is calculated by the kWh,
 * shown on the energy meter multiplied be the number of pulses per kWh for the energy meter. 
 *  Pulses per kWh can be found in the documentation for the energy meter.
 * )*** ‘Energy meter number’ is a number 0..7 for the channel, on which the energy meter is connected.
 * The relation between energy meter and channel number is defined in the privateConfig.h file. 
 */ 

/* Version history:
 *  1.0.0   Initial production version.
 *  2.0.0   Using MQTT broker as a storage for configuration data and energy meter counts, read/write from/to SD memory card has been implemnted.
 *          Struct data_t re-defined and split up to config_t, meta_t and data_t for optimal file read / write funktionality.
 *          Publishing totals, Subtotals and pulscorrections to MQTT has been replaced be write to SD Memory card.
 *  2.0.1   MQTT Reconnect bugfix
 *  2.0.2   serial removed as it is not needed.
 *  3.0.0   Publish data to GS based on epoch time and time collected from a time server
 *  3.0.1   BUGFIX: Interrupt ISR were attached to a non configured pin, which could cause an un-handled IRQ, which then cause infinite loop.
 *  3.1.0   Adding DEBUG compilor switch to investigate SD issue.
 *          -  Code re-arranged to comply with VS Code standard.
 *          -  Need to re-think the Idea for not using directories. The SD library uses short 8.3 names for files. This might be an issue.
 *             Data file sets now placed in directories.
 *          -  Startup delay for 10 sec. introduced, before accessinmg the SD Card. This is to prevent SD Card issues while connecting the device to power.
 *             While onnecting the device to power will in moast cases cause multible power failures (dis-connections), before the power is stable.
 *          -  Test time stamp for IRQ. If Last IRQ is less 500 millis, IRQ is ignored. 
 *  4.0.0   Firmware modified to match hardware. 
 *          -  Pull up is no longer required, as the new hardware inclcudes hex schmidt-triggers.
 *          -  IRQ's are now tgriggered my a raising pulse.
 *          -  Status LED is lit when GPIO pin is LOW
 * 4.1.0    BUGFIX:
 *          - Issue: Stopped updating Google Sheets after power outage. 
 *            Sketch runs, Values published to MQTT, Sketch handles MQTT requests. Updating GS does not happen!
 *            - Investigate millis() overrun.
 *              millis() overrun will cause publsh to GS to stop. Implementing esp_timer_get_time() to return a timestamp in secunds til solve the overrun issue.
 *              Changed the other usage of millis() as timestamps or changed to code to handle millis() overrun.
 *          - Issue: PowerUp message only added once
 *            Changed logic to keep adding PowerUp message until successful call Google Sheets and adding WiFiReconnect message un WiFi reconnect.
 *          - Response for http.begin(urlFinal.c_str()) cal to GS is collected by httpCode = http.GET() but never tested for success.
 *            Test response for "200": httpCode = http.GET() = 200
 *          - Issue: Temporary FIX to prevent more than one IQR to be registeret.
 *            Temporary fix in function 'void IRAM_ATTR store_IRQ_PIN(u_int8_t BIT_Reference)' removed after HEX Schmidt triggers are implemented in the hardware
 * 4.1.1    BUGFIX:
 *         - Issue: Not all Home Assistant (HA) configurations are published. #8
 *         - Issue: "Forbrug" is published as 0 (zero) #9
 * 4.2.0    Enhancements:
 *        - Issue: Stop running when SC card fails #3
 *          
 * Boot analysis:
 * Esp32 MQTT interface for Carlo Gavazzi energy meter - V2.0.0
 * Time to set pinmode and led: 0
 * Time to initialise globals and arm ISR: 2
 * Time to initialise SD: 345 /  10
 * Time to read configuration 7
 * Time to initialise test structure version  0
 * Time to read datafiles 1611
 * Time to write new configuration file: 327 / 31
 * Time to write new data: 4162
 * ### Failed connection to WiFi ########
 *    Time to connect to WiFi: 155
 *    Time to wait for failed connection to WiFi: 2501
 *    Time to connect to WiFi: 31
 *    Time to wait for failed connection to WiFi: 1
 *    Time to connect to WiFi: 18
 *    Time to wait for failed connection to WiFi: 0
 * ###  Sucessfull connection to WiFi, Failed connect to MQTT  ###
 *    Time to connect to WiFi: 155
 *    Time to wait for successfull connection to WiFi: 2001
 * Time to start OTA: 18
 * Time to failed connect to MQTT: 3006
 * Time to failed connect to MQTT: 3006
 * 
 * Suggestions to tune the boot process: 
 * - Look into the use of SdFat library and the use of fixed CS instead of SD library, this might speed up the SD Card operations.
 *   Might be an issue if a new config file is required (Changes done to the configuraion will require a scan throuh all existing files)
 * - Give the process time to handle incomming IRQ's between WiFi connect and MQTT connect. (DONE ny ignoring connet attempts if IRQ's available)
 * 
 *  Future versions:
 *  
 * 
 *  - Ideas: as writes to SD for each Energy meter count will be intensive, implement a method to write thise data to diffenret locations.  
 *           Considder to implement rename of data file instead of creating new datafiles and keeping track of old to avoid re-use of SD memory location.
 *           If NTP Server will give date and time, datafiles can be renamed to date and time and the sam data file names can be used. 
 *  - Finetune MQTT_MAX_PACKET_SIZE in platformio.ini. Definition: MQTT_MAX_PACKET_SIZE < MQTT_MAX_HEADER_SIZE + 2 + strlen(topic) + payload length
 *    where MQTT_MAX_HEADER_SIZE is defined as 5.
 *  - Reduce boot time to reduce loss of Energy meter pulses. Might require use of SdFat library  instead of SD library.
 *  - Imlement ESP.restart() if required. Consider to implement EEPROM.write() https://randomnerdtutorials.com/esp32-flash-memory/
 *    during call to ESP.resart(). Writes for every update will kill flash memory. 
 *  - Configurable parameters - Configuration thrount MQTT:
 *    - Google Sheet ID
 */

/*
 * ######################################################################################################################################
 * ######################################################################################################################################
 *                       V  A  R  I  A  B  L  E      D  E  F  I  N  A  I  T  O  N  S
 * ######################################################################################################################################
 * ######################################################################################################################################
 */


#define CONFIGURATON_VERSION 5
/* WiFi and MQTT connect attempt issues. 
 * IRQ's will be registrated, but the counters for will not be updated during the calls to WiFi and MQTT connect. If more than one pulse
 * from then same meter arrives, it will be lost if these calls takes up too much time. Setting a long connect postpone will reduce the loss
 * of pulses
*/
#define WIFI_CONNECT_POSTPONE 30        // Number of seconds between WiFi connect attempts, when WiFi.begin fails to connect.
#define MQTT_CONNECT_POSTPONE 30        // Number of seconds between MQTT connect dattempts, when MQTT connect fails to connect.
#define BLIP 100                        // Time in milliseconds the LED will blink
#define MIN_CONSUMPTION 25              // Define the minimum powerconsumption published before publishing 0
                                        // See '>>>    Pulse time check  <<<' in the last part of the loop() for further details.
#define RETAINED true                   // Used in MQTT puplications. Can be changed during development and bugfixing.
#define UNRETAINED false
#define MAX_NO_OF_CHANNELS 8
#define MAX_NUMBER_OF_WRITES 65500      // Number of writes made to data file / SD Card, before new set of datafiles will be used (MAX 2^16)

/* Configurable MQTT difinitions
 * These definitions can be changed to suitable nanes.
 * Chagens might effect other inntegrations and configurations in HA!
 */

const String  MQTT_CLIENT = "Carlo-Gavazzi-Energy-Meters_";       // mqtt client_id prefix. Will be suffixed with Esp32 mac to make it unique

const String  MQTT_PREFIX                   = "energy/";              // include tailing '/' in prefix!
const String  MQTT_DEVICE_NAME              = "monitor_ESP32_";       // only alfanumeric and no '/'
const String  MQTT_DISCOVERY_PREFIX         = "homeassistant/";       // include tailing '/' in discovery prefix!
const String  MQTT_PREFIX_DEVICE            = "meter_";
const String  MQTT_ONLINE                   = "/online"; 
const String  MQTT_SENSOR_ENERG_ENTITYNAME  = "Subtotal";  // name dislayed in HA device. No special chars, no spaces
const String  MQTT_SENSOR_POWER_ENTITYNAME  = "Forbrug";   // name dislayed in HA device. No special chars, no spaces
const String  MQTT_NUMBER_ENERG_ENTITYNAME  = "Total";     // name dislayed in HA device. No special chars, no spaces
const String  MQTT_PULSTIME_CORRECTION      = "pulscorr";
const String  MQTT_SKTECH_VERSION           = "/sketch_version";
const String  MQTT_SUFFIX_STATE             = "/state";
const String  MQTT_SUFFIX_CONSUMPTION       = "/watt_consumption";
// MQTT Subscription topics
const String  MQTT_SUFFIX_TOTAL_TRESHOLD    = "/threshold";
const String  MQTT_SUFFIX_SUBTOTAL_RESET    = "/subtotal_reset";
const String  MQTT_SUFFIX_CONFIG            = "/config";
const String  MQTT_SUFFIX_STATUS            = "status";

/*  None configurable MQTT definitions
 *  These definitions are all defined in 'HomeAssistand -> MQTT' and cannot be changed.
*/
const String  MQTT_SENSOR_COMPONENT         = "sensor";
const String  MQTT_NUMBER_COMPONENT         = "number";
const String  MQTT_ENERGY_DEVICECLASS       = "energy";
const String  MQTT_POWER_DEVICECLASS        = "power";

/*
 * File configurations
 * There will be one configuration file and a data file for each energy meter connected (Number of energy meters is defined in privateConfig.h).
 * As writes to data files will occour on every pulse registered, writes to the SD Card will be intensive.
 * To prevent SD Card failures, a new set of datafiles will be created at every reboot or for every 65.500** pulses counted.
 * Each data file set will be created in a directory, which will have an ending number 0 - 2^16 (65536 the max for a uint16_t)
 * Data files will have an ending number 0 - (MAX_NO_OF_CHANNELS - 1).
 * 
 * ** 65.500 pulses will be equal tp 65,50kWh registered on a Carlo Gavazzi type EM111 energy meter.
 * 
 */
const String CONFIGURATION_FILENAME = "/config.cfg ";   // Filenames has to start with '/'
const String DATAFILESET_POSTFIX    = "/fs_v2-";           // Will bee the directory name
const String FILENAME_POSTFIX       = "/df-";           // Leading '/'. 
const String FILENAME_SUFFIX        = ".dat";           //

/*
 * Time server configuration
 */

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

String mqttDeviceNameWithMac;
String mqttClientWithMac;
String errorMessages[] = {
                          "0 SD Card not initialized", 			        // Error index 0
                          "1 open / Creating configuration file", 	// Error index 1
                          "2 writing configuration file", 	      	// Error index 2
                          "3 creating directory for data files",    // Error index 3
                          "4 open / creating data files", 	      	// Error index 4
                          "5 writing data files", 		            	// Error index 5
                          "6 Failed to update number of writes", 		// Error index 6
                          "7 SD operation too slow" 	              // Error index 7
                         };

int errorIndex = 0;
int previousErrorIndex = 0;

uint16_t blip = BLIP;
uint8_t GlobalIRQ_PIN_index = 0;
uint16_t numberOfWrites = 0;
uint8_t GoogleSheetMessageIndex = 1;  // Setting message in GS to PowerUp

// Define array of GPIO pin numbers used for IRQ.
const uint8_t channelPin[MAX_NO_OF_CHANNELS] = {private_Metr1_GPIO,private_Metr2_GPIO,private_Metr3_GPIO,private_Metr4_GPIO,
                                                private_Metr5_GPIO,private_Metr6_GPIO,private_Metr7_GPIO,private_Metr8_GPIO};  

bool configurationPublished[PRIVATE_NO_OF_CHANNELS];  // a flag for publishing the configuration to HA if required.
bool esp32Connected = false;                          // Is true, when connected to WiFi and MQTT Broker
bool LED_ToggledState = false; 
bool LED_Invertred = false;
bool SD_Failed = false;                               // Set to true if SD Card fails

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Define structure for configuration
struct config_t
  {
    int structureVersion;
    unsigned long pulseTimeCorrection;      // Used to calibrate the calculated consumption.
    uint16_t dataFileSetNumber;               // In which data file set ("directory") the data files will be located.
    uint16_t  pulse_per_kWh[PRIVATE_NO_OF_CHANNELS];       // Number of pulses as defined for each energy meter
  } interfaceConfig;

// Define stgructure for meta data
struct meta_t
  {
    unsigned long pulseTimeStamp;   // Stores timestamp, Usec to calculate millis bewteen pulses ==> Calsulate consupmtion.
    unsigned long pulseLength;      // Sorees time between pulses. Used to publish 0 to powerconsumption, when pulses stops arriving == poser comsumptino reduced
  } metaData[PRIVATE_NO_OF_CHANNELS];

// Define structure for energy meter counters
struct data_t
  {
    unsigned long pulseTotal;                // For counting total number of pulses on each Channel
    unsigned long pulseSubTotal;             // For counting number of pulses within a period 
  } meterData[PRIVATE_NO_OF_CHANNELS];

/* Wariables to handle connect postpones and length of LED blinks*/
unsigned long WiFiConnectAttempt = 0;   // Timestamp when an attempt to connect to WiFi were done
unsigned long MQTTConnectAttempt = 0;   // Timestamp when an attempt to connect to MQtT were done

unsigned long WiFiConnectPostpone = 0;  // Millisecunds between each attempt to connect to WiFi.
unsigned long MQTTConnectPostpone = 0;  // Millisecunds between each attempt to connect to MQTT.

unsigned long timeLastCheckedAt;        // Timestamp for when epoch time was checked.
unsigned long secondsToNextTimeCheck;    // Number of seconds to next epoch time check.

unsigned long LED_toggledAt = 0;        // Timestamp when an IRQ tuggels the LED

volatile unsigned long millsTimeStamp[PRIVATE_NO_OF_CHANNELS];  // Used by the ISR to store exactly when an interrupt occoured. 
                                                                // Used to calculate consuption.
volatile uint8_t  IRQ_PINs_stored = 0b00000000;   // Used by the ISR to register which energy meter caused an interrupt.
/*
 * ##################################################################################################
 * ##################################################################################################
 * ##################################################################################################
 * ##########################   f u n c t i o n    d e c l a r a t i o n s  #########################
 * ##################################################################################################
 * ##################################################################################################
 * ##################################################################################################
 */
void writeConfigData();
void writeMeterDataFile( uint8_t);
void writeMeterData(uint8_t);
void setConfigurationDefaults();
void initializeGlobals();
void publish_sketch_version();
void publishStatusMessage(String);
byte getIRQ_PIN_reference(char*);
bool updateGoogleSheets( uint8_t);
unsigned long getsecondsToNextTimeCheck();
unsigned long sec();
void publishMqttEnergyConfigJson( String, String, String, String, u_int8_t);
void publishMqttConfigurations( uint8_t);
void publishSensorJson( long, uint8_t);
void mqttCallback(char*, byte*, unsigned int);
void IRAM_ATTR store_IRQ_PIN(u_int8_t);
void IRAM_ATTR Ext_INT1_ISR();
void IRAM_ATTR Ext_INT2_ISR();
void IRAM_ATTR Ext_INT3_ISR();
void IRAM_ATTR Ext_INT4_ISR();
void IRAM_ATTR Ext_INT5_ISR();
void IRAM_ATTR Ext_INT6_ISR();
void IRAM_ATTR Ext_INT7_ISR();
void IRAM_ATTR Ext_INT8_ISR();
/*
 * ###################################################################################################
 * ###################################################################################################
 * ###################################################################################################
 * ###################   S E T U P      B e g i n     ################################################
 * ###################################################################################################
 * ###################################################################################################
 * ###################################################################################################
 */
void setup() {
/*
 * To prevent SD Cart failures caused by power interruption during mounting, make a delay to wait
 * for a steady power supply.
 */
  delay( 2000);

  pinMode(LED_BUILTIN, OUTPUT);             // Initialize build in LED           
  digitalWrite(LED_BUILTIN, LOW);          // Turn ON LED to indicate startup

  // Initialize Interrupt pins
  for ( u_int8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++)
  {
    pinMode(channelPin[ii], INPUT);
  }

  // arm interrupt. Create a functioncall for each interrupt pin
  if ( PRIVATE_NO_OF_CHANNELS >= 1)
    attachInterrupt(private_Metr1_GPIO, Ext_INT1_ISR, RISING);
  if ( PRIVATE_NO_OF_CHANNELS >= 2)
    attachInterrupt(private_Metr2_GPIO, Ext_INT2_ISR, RISING);
  if ( PRIVATE_NO_OF_CHANNELS >= 3)
    attachInterrupt(private_Metr3_GPIO, Ext_INT3_ISR, RISING);
  if ( PRIVATE_NO_OF_CHANNELS >= 4)
    attachInterrupt(private_Metr4_GPIO, Ext_INT4_ISR, RISING);
  if ( PRIVATE_NO_OF_CHANNELS >= 5)
    attachInterrupt(private_Metr5_GPIO, Ext_INT5_ISR, RISING);
  if ( PRIVATE_NO_OF_CHANNELS >= 6)
    attachInterrupt(private_Metr6_GPIO, Ext_INT6_ISR, RISING);
  if ( PRIVATE_NO_OF_CHANNELS >= 7)
    attachInterrupt(private_Metr7_GPIO, Ext_INT7_ISR, RISING);
  if ( PRIVATE_NO_OF_CHANNELS >= 8)
    attachInterrupt(private_Metr8_GPIO, Ext_INT8_ISR, RISING);

  initializeGlobals();

  mqttClient.setServer(PRIVATE_MQTT_SERVER.c_str(), PRIVATE_MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

/*
 * Read configuration and energy meter data from SD memoory
 */
  if(!SD.begin(5))
  {
    SD_Failed = true;
    bitSet(errorIndex, 0);        // 0 SD Card not initialized
  }

  if ( !SD_Failed )  // Reading configuration 
  {
    String configFilename = String(CONFIGURATION_FILENAME);
    File structFile = SD.open(configFilename, FILE_READ);
    if ( structFile)
    {
      structFile.read((uint8_t *)&interfaceConfig, sizeof(interfaceConfig)/sizeof(uint8_t));
      structFile.close();
    }
  }

  // Check if new configuration and datafiles are required
  if ( interfaceConfig.structureVersion != (CONFIGURATON_VERSION * 100) + PRIVATE_NO_OF_CHANNELS)
  {
    setConfigurationDefaults();
  }

  // Check if new datafileser (directory) is required
  String dirname = String (DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber));
  if ( !SD.exists(dirname))
  {
    if ( !SD.mkdir( dirname))
    {
      SD_Failed = true;
      bitSet(errorIndex, 3);
    }
  }

  String filename = String ( DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber) + FILENAME_POSTFIX + String("writes") + FILENAME_SUFFIX);
  File f = SD.open(filename, FILE_READ);
  if ( f)
  {
    if ( f.read((uint8_t *)&numberOfWrites, sizeof(numberOfWrites)/sizeof(uint8_t)) != sizeof(numberOfWrites)/sizeof(uint8_t))
      numberOfWrites = 0;
   f.close();
  }
  else
    numberOfWrites = 0;

  // Reading datafiles.
  for ( uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++)
  {
    String filename = String (DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber) +\
                              FILENAME_POSTFIX + String(ii) + FILENAME_SUFFIX);
    File structFile = SD.open(filename, FILE_READ);
    if ( structFile && structFile.read((uint8_t *)&meterData[ii], sizeof(meterData[ii])/sizeof(uint8_t)) !=\
          sizeof(meterData[ii])/sizeof(uint8_t)) 
    {
      meterData[ii].pulseTotal = 0;
      meterData[ii].pulseSubTotal = 0;
    }
    structFile.close();
  }

  digitalWrite(LED_BUILTIN, HIGH);           // Turn OFF LED before entering loop
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
void loop() 
{
  // >>>>>>>>>>>>>>>>>>>>>>>>   Connect to WiFi if not connected    <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // Ingore WiFi connect if IRQ's are present.
  if ( IRQ_PINs_stored == 0 and \
       WiFi.status() != WL_CONNECTED and sec() > WiFiConnectAttempt + WiFiConnectPostpone)
  {

    /* Turn Build in LED (LED) ON, when not connected to WiFi.
     * The LED is also used as indicator for interrupt activity (Pules registrered). To combine this
     * with the LED being ON when no WiFi connected, an interrupt activity will tuggle the LED and after
     * a short time (BLIP) the LED will be tuggled again. This way the LED will either turn ON or turn off
     * for a short time, depending on WiFi connection.
     * SO: In order to turn the led ON when not connectged to WiFi, the LED has to be se ON or Off depending
     * on status set by the interrupt activity.
    */
    if (LED_Invertred == false)
    {
      digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
      LED_Invertred = true;
    }
    
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(PRIVATE_WIFI_SSID.c_str(), PRIVATE_WIFI_PASS.c_str());

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
      WiFiConnectAttempt = sec();
      WiFiConnectPostpone = WIFI_CONNECT_POSTPONE;
      blip = 10 * BLIP;        // Make a long blip (LED Flash) to indicate no connection to MQTT broker
      esp32Connected = false;
    } else {
      WiFiConnectAttempt = 0;   // In case WiFi is lost, attempt to reconnect immediatly. 
      WiFiConnectPostpone = 0;

      // Init Unix epoch time
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

      // Init timecheck and post to Google Sheets
      secondsToNextTimeCheck = getsecondsToNextTimeCheck();
      timeLastCheckedAt = sec();
      if (PRIVATE_UPDATE_GOOGLE_SHEET)
      {
       if (updateGoogleSheets( GoogleSheetMessageIndex))
        GoogleSheetMessageIndex = 2;
      }  

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
        .onStart([]()
        {
          String type;
          if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
          else // U_SPIFFS
            type = "filesystem";

          // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
          //  Serial.println("Start updating " + type);                                                      ############################################
        })
        .onEnd([]()
        {
          // Serial.println("\nEnd");                                                                         ############################################
        })
        .onProgress([](unsigned int progress, unsigned int total)
        {
          // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));                                  ############################################
        });
        /*
        .onError([](ota_error_t error)
        {
          Serial.printf("Error[%u]: ", error);
          if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
          else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
          else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
          else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
          else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });
        */

      ArduinoOTA.begin();

      /*¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
      *    E N D      Arduino basicOTA  impementation
      *¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
      */

    }
  }
  // >>>>>>>>>>>>>>>>>>>>>>>>   E N D  Connect to WiFi if not connected    <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  // >>>>>>>>>>>>>>>>>>>>   Connect to MQTT broker IF  Connected to WiFi   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // Ignore connect to MQTT if IRQ's are present-
  if ( IRQ_PINs_stored == 0 and WiFi.status() == WL_CONNECTED)
  {
    if (LED_Invertred == true)
    {
      digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
      LED_Invertred = false;
    }

    ArduinoOTA.handle();
    
    // >>>>>>>>>>>>>>>>>>> Connect to MQTT Broker <<<<<<<<<<<<<<<<<<<<<<<<<<
    if (!mqttClient.connected() and sec() > MQTTConnectAttempt + MQTTConnectPostpone )
    {
      String will = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE);

      if ( mqttClient.connect( mqttClientWithMac.c_str(), PRIVATE_MQTT_USER.c_str(), 
                               PRIVATE_MQTT_PASS.c_str(), will.c_str(), 1, RETAINED, "False"))
      {
        MQTTConnectAttempt = 0;
        MQTTConnectPostpone = 0;
        blip = BLIP;        // Make a short blip (LED Flash) to indicate normal activity.
        esp32Connected = true;

        //   >>>>>>>>>>>>>>>>>>>>>   publish will and SKETCH_VERSION   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        publish_sketch_version();

        //   >>>>>>>>>>>>>>>>>>>>  Subscribe to Set Totals and Reset Suttotals topic  <<<<<<<<<<<<<<<<<
        String totalSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+" + MQTT_SUFFIX_TOTAL_TRESHOLD);
        mqttClient.subscribe(totalSetTopic.c_str(), 1);

        String subTotalSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_SUFFIX_SUBTOTAL_RESET);
        mqttClient.subscribe(subTotalSetTopic.c_str(), 1);

        String configSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_SUFFIX_CONFIG);
        mqttClient.subscribe(configSetTopic.c_str(), 1);

        String statusSetTopic = String(MQTT_DISCOVERY_PREFIX + MQTT_SUFFIX_STATUS);
        mqttClient.subscribe(statusSetTopic.c_str(), 1);

        mqttClient.publish(will.c_str(), (const uint8_t *)"True", 4, RETAINED);
        
      }
      else
      {
        MQTTConnectAttempt = sec();
        MQTTConnectPostpone = MQTT_CONNECT_POSTPONE;
        blip = 10 * BLIP;        // Make a long blip (LED Flash) to indicate no connection to MQTT broker
        esp32Connected = false;
      }
    }
  }
  // >>>>>>>>>>>>>>>>>>>  E N D    Connect to MQTT broker IF  Connected to WiFi <<<<<<<<<<<<<<<<<<<<<<<<<<

  // >>> Process incomming messages and maintain connection to the server
  if ( esp32Connected)
  {
    if(!mqttClient.loop()) {
      blip = 10 * BLIP;        // Make a long blip (LED Flash) to indicate no connection to MQTT broker
      esp32Connected = false;
    }
  }
  //  <<< END Process incomming messages

  // >>>>>>>>>>>>>>>>>>>>   Publis meterData (If any)   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  if ( IRQ_PINs_stored > 0)
  {                                  // If IRQ has occoured IRQ_PINs_store will be > 0.
    uint8_t pinMask = 0b00000001;    // Set pinMask to start check if the Least significant Bit (LSB) is set ( = 1)

    // >>>>>>>>>>>>>>>>  Tuggle LED pin if not toggled allready  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    if ( !LED_ToggledState)
    {
       digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
      LED_ToggledState = true;
      LED_toggledAt = millis();
    }

    // Iterate through all bits in the byte IRQ_PINs_stored.
    for( uint8_t IRQ_PIN_index = 0; IRQ_PIN_index < PRIVATE_NO_OF_CHANNELS; IRQ_PIN_index++)
    {
      // Publish configuration to MQTT broker if not allready done.
      if( esp32Connected and !configurationPublished[IRQ_PIN_index])
      {
        publishMqttConfigurations( IRQ_PIN_index);
      }
      /*
        *  If bit in IRQ_PINs_stored matches the bit set in pinMask: Calculate powerconsumption and publist data.
        */
      if( IRQ_PINs_stored & pinMask)                       // If bit is set 
      {
        //   >>>>>>>>>>>>>>>>>>>>>>>>>>>  Calculate power comsumption   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        /*
          * It does not make sence to calculate consumption when the privious pulse is unkown (0) or 
          * when millis() has owerflown and millsTimeStamp
          * is before pulseTimeStamp.
          * When millis overflows, the pulsetime (millsTimeStamp - pulseTimeStamp + pulseTimeCorrection) 
          * could ofcause be calculated, but it brings complexity to the code 
          * but only saves one comsumption calculation every 50 days...
          */
        long watt_consumption = 0;
        if ( metaData[IRQ_PIN_index].pulseTimeStamp > 0 && metaData[IRQ_PIN_index].pulseTimeStamp < millsTimeStamp[IRQ_PIN_index])
        { 
          watt_consumption = round(((float)(60*60*1000) / 
                                    (float)(millsTimeStamp[IRQ_PIN_index] - metaData[IRQ_PIN_index].pulseTimeStamp +
                                    interfaceConfig.pulseTimeCorrection)) / 
                                    (float)interfaceConfig.pulse_per_kWh[IRQ_PIN_index] * 1000);

          metaData[IRQ_PIN_index].pulseLength = millsTimeStamp[IRQ_PIN_index] - metaData[IRQ_PIN_index].pulseTimeStamp +
                                                interfaceConfig.pulseTimeCorrection;
        }

        //   >>>>>>>>>>>>>>>>>>>>>>>>>>>  Update meterData and publish totals   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
        metaData[IRQ_PIN_index].pulseTimeStamp = millsTimeStamp[IRQ_PIN_index];
        meterData[IRQ_PIN_index].pulseTotal++;
        meterData[IRQ_PIN_index].pulseSubTotal++;

        if ( !SD_Failed )
        {
          writeMeterData( IRQ_PIN_index);
        }

        if ( esp32Connected) {
          publishSensorJson( watt_consumption, IRQ_PIN_index);
        }

        bitClear(IRQ_PINs_stored, IRQ_PIN_index);                 // Set the bit back to 0, ready for next IRQ 
      } 
      pinMask <<= 1;                                              // Left shift the bit mask to check the next bit
    }

  }

  /* >>>>>>>>>>>>>>>>>>    Pulse time check  <<<<<<<<<<<<<<<<<<<<<<<<<<

   * If a bit in IRQ_PINs_stored is set or being set during the pulse time check, further pulse 
   * time check's will be skipped because publishing meterData has priority.  
   * If no bit's are set, check if then pulseTimeStamp indicate that comsumption has been 
   * reduced or stopped.
   * 
   * The assumption is: If a pulse has been missing for double the previous pulse period, 
   * comsumption has dropped and a new fictive comsumptino is published based on the double 
   * of the previous pulse period.
   * This will goes on, until the calculated fictive comsumptino becomes less than the consumption, 
   * defined in MIN_CONSUMPTION.
   */
  unsigned long timeStamp = millis();
  if ( GlobalIRQ_PIN_index >= PRIVATE_NO_OF_CHANNELS)
    GlobalIRQ_PIN_index = 0;

  while ( IRQ_PINs_stored == 0 && GlobalIRQ_PIN_index < PRIVATE_NO_OF_CHANNELS)
  {
    // Publish configuration to MQTT broker if not allready done.
    if( esp32Connected and !configurationPublished[GlobalIRQ_PIN_index])
    {
      publishMqttConfigurations( GlobalIRQ_PIN_index);
    }

    // At startup pulseTimeStamp will be 0 ==> Comsumptino unknown ==> No need to recalculation
    if ( metaData[GlobalIRQ_PIN_index].pulseLength > 0 )
    {
      // During exeution mills will overflow ==> Complicates calculation ==> Skip recalculation
      if ( metaData[GlobalIRQ_PIN_index].pulseTimeStamp > timeStamp)
      {
        long watt_consumption = 0;
        metaData[GlobalIRQ_PIN_index].pulseLength = 0;
        if ( esp32Connected)
          publishSensorJson( -1 * watt_consumption, GlobalIRQ_PIN_index);
      }
      else
      {  
        if (  metaData[GlobalIRQ_PIN_index].pulseTimeStamp + 
              ( 2 * metaData[GlobalIRQ_PIN_index].pulseLength) < timeStamp )
        {
          long watt_consumption = round(((float)(60*60*1000) / 
                                    (float)(timeStamp - metaData[GlobalIRQ_PIN_index].pulseTimeStamp + 
                                    interfaceConfig.pulseTimeCorrection)) /
                                    (float)interfaceConfig.pulse_per_kWh[GlobalIRQ_PIN_index] * 1000);

          if ( watt_consumption < MIN_CONSUMPTION) {
            watt_consumption = 0;
            metaData[GlobalIRQ_PIN_index].pulseLength = 0;
          }
          if ( esp32Connected)
            publishSensorJson( -1 * watt_consumption, GlobalIRQ_PIN_index);

          metaData[GlobalIRQ_PIN_index].pulseLength *= 2;
        }
      }
    }
    /* Tuggle LED Pin if tuggled and BLIP time has passed or millis() has overrun.
    */
    if ( LED_ToggledState )
    {
      if ( millis() > LED_toggledAt + blip or millis() < LED_toggledAt)
      {
        digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
        LED_ToggledState = false;
        LED_toggledAt = 0;
      }
    }

    GlobalIRQ_PIN_index++;
  }

  /* >>>>>>>>>>>>>>>>>>>>>>>>>>> time check to scheculed Google update <<<<<<<<<<<<<<<<<<< */
  if (sec() > timeLastCheckedAt + secondsToNextTimeCheck)
  {
    timeLastCheckedAt = sec();
    secondsToNextTimeCheck = getsecondsToNextTimeCheck();
  }

  if ( secondsToNextTimeCheck == 0)
  {
    secondsToNextTimeCheck = 60; // To prevent another secondsToNextTimeCheck == 0
                                 // Postpone a minute before getting next secondsToNextTimeCheck 
    if (PRIVATE_UPDATE_GOOGLE_SHEET and WiFi.status() == WL_CONNECTED)
      updateGoogleSheets( 0);
    for ( uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++)
    {
      meterData[ii].pulseSubTotal = 0;
      if ( !SD_Failed )
        writeMeterData( ii);
    }
  }

  if ( errorIndex != previousErrorIndex)
  {
    publish_sketch_version();
    previousErrorIndex = errorIndex;
  } 
}
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
 *               W R I T E   C O N F I G   D A T A
 * ###################################################################################################
 */
void writeConfigData()
{
  String configFilename = String(CONFIGURATION_FILENAME);
  File structFile = SD.open(configFilename, FILE_WRITE);
  if (structFile)
  {
    structFile.seek(0);
    if ( structFile.write((uint8_t *)&interfaceConfig, sizeof(interfaceConfig)) != sizeof(interfaceConfig))
    {
      SD_Failed = true;
      bitSet(errorIndex, 2);
    }
    structFile.close();
  } 
  else
  {
    SD_Failed = true;
    bitSet(errorIndex, 1);
  }
}
/* ###################################################################################################
 *               W R I T E   M E T E R   D A T A   T O   F I L E
 * ###################################################################################################
 */
void writeMeterDataFile( uint8_t datafileNumber)
{
  String filename = String ( DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber) + 
                             FILENAME_POSTFIX + String(datafileNumber) + FILENAME_SUFFIX);
  File structFile = SD.open(filename, FILE_WRITE);
  if (!structFile)
  {
    SD_Failed = true;
    bitSet(errorIndex, 4);
  } else
  {
    structFile.seek(0);
    if ( structFile.write((uint8_t *)&meterData[datafileNumber], sizeof(meterData[datafileNumber])) != sizeof(meterData[datafileNumber]))
    {
      SD_Failed = true;
      bitSet(errorIndex, 5);
    }
    structFile.close();
  }
}

/* ###################################################################################################
 *               W R I T E   M E T E R   D A T A
 * ###################################################################################################
 */
void writeMeterData(uint8_t datafileNumber)
{
  if ( numberOfWrites++ >  MAX_NUMBER_OF_WRITES)
  {
    interfaceConfig.dataFileSetNumber++;
    String dirname = String (DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber));
    if ( !SD.mkdir( dirname))
    {
      SD_Failed = true;
      bitSet(errorIndex, 3);
    } else
    {
      writeConfigData();
      numberOfWrites = 0;
      for ( uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++)
        writeMeterDataFile(ii);
    }
  } else
  {
    writeMeterDataFile(datafileNumber);
  }

  String filename = String ( DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber) + 
                             FILENAME_POSTFIX + String("writes") + FILENAME_SUFFIX);
  File writesFile = SD.open(filename, FILE_WRITE);
  if (writesFile)
  {
    writesFile.seek(0);
    writesFile.write( (uint8_t *)&numberOfWrites, sizeof(numberOfWrites)/sizeof(uint8_t));
    writesFile.close();
  }
  else
  {
    SD_Failed = true;
    bitSet(errorIndex, 6);
  }
}

/* ###################################################################################################
 *               S E T    C O N F I G U R A T I O N    D E F A U L T S
 * ###################################################################################################
 * This function is called when:
 * - a new SD Card is present
 * - Changes has been done to the 'config_t' structure
 * - when the number of channels (PRIVATE_NO_OF_CHANNELS) has been changed.
 * The function will delite the old configuration file and create a new one.
 * It will skip all existing data file sets.
 * - int structureVersion;
 * - unsigned long pulseTimeCorrection;      // Used to calibrate the calculated consumption.
 * - uint16_t dataFileSetNumber;            // In which data file set ("directory") the data files will be located.
 * - uint16_t  pulse_per_kWh[PRIVATE_NO_OF_CHANNELS];       // Number of pulses as defined for each energy meter
 */

void setConfigurationDefaults()
{
  interfaceConfig.structureVersion = (CONFIGURATON_VERSION * 100) + PRIVATE_NO_OF_CHANNELS;
  interfaceConfig.pulseTimeCorrection = 0;  // Used to calibrate the calculated consumption.
  
  // Find new data file set for data files == Skip all existing data files.
  uint8_t numberOfFilesExists = 1;
  for ( interfaceConfig.dataFileSetNumber = 0; numberOfFilesExists == 0; interfaceConfig.dataFileSetNumber++)
  {
    numberOfFilesExists = 0;
    if ( !SD_Failed)
    {
      for ( uint8_t iix = 0; iix < PRIVATE_NO_OF_CHANNELS; iix++)
      {
        String dataFileName = String ( DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber) + 
                                       FILENAME_POSTFIX + String(iix) + FILENAME_SUFFIX);
        if (SD.exists(dataFileName))
          numberOfFilesExists++;
      }
    }
  }

  for (uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++)
    interfaceConfig.pulse_per_kWh[ii] = private_default_pulse_per_kWh[ii];    // Number of pulses as defined for each energy meter

  if ( !SD_Failed)
  {
    String filename = String(CONFIGURATION_FILENAME);
    if (SD.exists(filename))
    {
      SD.remove(filename);
    }
    writeConfigData();
  }
}
/* ###################################################################################################
 *                     I N I T I A L I Z E   G L O B A L S
 * ###################################################################################################
 */
void initializeGlobals()
{
  for (uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++)
  {
    metaData[ii].pulseTimeStamp = 0;
    metaData[ii].pulseLength = 0;

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
 *                  P U B L I S H   S K E T C H   V E R S I O N
 * ###################################################################################################
 *  As the sketch will esecute in silence, one way to se which version of the SW is running will be
 *  to subscribe to topic defined as (MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_SKTECH_VERSION)
 *  on the MQTT broker, configured in the privateConfig.h file.
 *  This will return the value of (SKETCH_VERSION)
 */
void publish_sketch_version()   // Publish only once at every reboot.
{  
  IPAddress ip = WiFi.localIP();
  String versionTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_SKTECH_VERSION);
  String versionMessage = String(SKETCH_VERSION + String("\nConnected to SSID: \'") +\
                                  String(PRIVATE_WIFI_SSID) + String("\' at: ") +\
                                  String(ip[0]) + String(".") +\
                                  String(ip[1]) + String(".") +\
                                  String(ip[2]) + String(".") +\
                                  String(ip[3]));

  if ( SD_Failed)
  {
    versionMessage += String("\n");
    uint8_t errorIndexMask = 0b00000001;
    for ( uint8_t ii = 0; ii < 8; ii++)
    {
      if ( errorIndex & errorIndexMask)
        versionMessage += String("\nError: ") + errorMessages[ii];
      errorIndexMask <<= 1;
    }
  }  

  mqttClient.publish(versionTopic.c_str(), versionMessage.c_str(), RETAINED);
}

/* ###################################################################################################
 *                  P U B L I S H   S T A T U S M E S S A G E
 * ###################################################################################################
 *  Send status messages to MQTT
 * 
*/
void publishStatusMessage(String statusMessage) 
{
  String statusTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + MQTT_SUFFIX_STATUS);
  mqttClient.publish(statusTopic.c_str(), statusMessage.c_str(), RETAINED);
}

/* ###################################################################################################
 *      G E T   I R Q  P I N   r e f e r e n c e  f r o m   t o p i c   s t r i n g
 * ###################################################################################################
 * 
 * IRQ pin reference is expected to be les than 2 digits!
 */
byte getIRQ_PIN_reference(char* topic)
{
  uint16_t startIndex = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/").length();
  uint16_t i = 0;
  uint8_t result = 0;
  while(topic[startIndex + i] != '/' && i < 2)  // IRQ pin reference les than 2 digits!
  {
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
bool updateGoogleSheets( uint8_t messageIndex)
{
  int httpCode = 0;
  // >>>>>>>>>>>>>   Create data-URL string for HTTP request   <<<<<<<<<<<<<<<<<<
  String urlData = "/exec?meterData=";

  for ( uint8_t IRQ_PIN_index = 0; IRQ_PIN_index < PRIVATE_NO_OF_CHANNELS; IRQ_PIN_index++)
    urlData += String( float(meterData[IRQ_PIN_index].pulseTotal) / float(interfaceConfig.pulse_per_kWh[IRQ_PIN_index]), 2) + ",";
  
  for ( uint8_t IRQ_PIN_index = 0; IRQ_PIN_index < PRIVATE_NO_OF_CHANNELS; IRQ_PIN_index++)
  {
    urlData += String( float(meterData[IRQ_PIN_index].pulseSubTotal) / float(interfaceConfig.pulse_per_kWh[IRQ_PIN_index]), 2);
    if ( IRQ_PIN_index < PRIVATE_NO_OF_CHANNELS - 1)
      urlData += String(",");
  }

  if ( messageIndex == 1)
    urlData += String(",PowerUp");
  if ( messageIndex == 2)
    urlData += String(",WiFiReconnect");
  
  if ( SD_Failed)
    urlData += String(",SD-Error");
  

  // >>>>>>>>>>>>>   Create URL for HTTP request   <<<<<<<<<<<<<<<<<<
  String urlFinal = "https://script.google.com/macros/s/" + PRIVATE_GOOGLE_SCRIPT_ID + urlData;

  // >>>>>>>>>>>>>   Make HTTP request to google sheet  <<<<<<<<<<<<<<<<<<
  HTTPClient http;
  http.begin(urlFinal.c_str());
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  // >>>>>>>>>>>>>   getting response from google sheet  <<<<<<<<<<<<<<<<<<
  if ( httpCode = http.GET())
  {
    String httpMessage = http.getString();
    String statusMessage = String( String("HTTP Status Code: ") + httpCode + " HTTP Message: " + httpMessage);
    publishStatusMessage( statusMessage);
  }
  //---------------------------------------------------------------------
  http.end();

  if ( httpCode = 200)
    return true;
  else
    return false;
}
/* ###################################################################################################
 *          G E T   M I L L I S   T O   N E X T    T I M E   C H E C K
 * ###################################################################################################

/*
 * Then function is used to schedule a piece of program code at a time defined by the two definitions:
 * #define SCHEDULE_MINUTE  mm
 * #define SCHEDULE_HOUR  hh
 * 
 * getsecondsToNextTimeCheck() will return the number om sec() diveded by two, counted from the time 
 * it is called to the time defined.
 * Doing it this way reduce the number of calls to the timeserver.
 * 
 * getsecondsToNextTimeCheck() returns 0 (zero) when it's called at the time defined.
 * 
 * Usage:
 * #include "time.h"
 * 
 * #define SCHEDULE_MINUTE  mm
 * #define SCHEDULE_HOUR  hh
 * 
 * const char* ntpServer = "pool.ntp.org";
 * const long  gmtOffset_sec = 3600;
 * const int   daylightOffset_sec = 3600;
 * 
 * 
 * setup() {
 *   timeLastCheckedAt = sec();
 *   secondsToNextTimeCheck = getsecondsToNextTimeCheck();
 * }
 * 
 * loop() {
 *  if (sec() > timeLastCheckedAt + secondsToNextTimeCheck){
 *    timeLastCheckedAt = sec();
 *    secondsToNextTimeCheck = getsecondsToNextTimeCheck();
 *  }
 *  if ( secondsToNextTimeCheck == 0 ) {
 *    secondsToNextTimeCheck = 60  // Snooze one minute to avoid multible calls within the same minute
 *     
 *     * Code to be executred
 *      
 *   }
 * 
 */
unsigned long getsecondsToNextTimeCheck()
{
  unsigned long timeToNextTimecheckInSeconcs;
  struct tm timeinfo;
  long dogn = 0;
  int sc_min = SCHEDULE_MINUTE;
  int sc_hour = SCHEDULE_HOUR;

  if(!getLocalTime(&timeinfo))
  {
    return -1;
  }

  if ( timeinfo.tm_hour == sc_hour and timeinfo.tm_min == sc_min)
  {
    timeToNextTimecheckInSeconcs = 0;
  } 
  else
  {
    if (((sc_hour * 60 * 60) + (sc_min * 60)) < ((timeinfo.tm_hour * 60 * 60) + (timeinfo.tm_min * 60) + timeinfo.tm_sec))
    {
      dogn = 24 * 60 * 60;
    }
    timeToNextTimecheckInSeconcs = ((((sc_hour * 60 * 60) + (sc_min * 60) + dogn) - ((timeinfo.tm_hour * 60 * 60) +\
                                   (timeinfo.tm_min * 60) + timeinfo.tm_sec)) /
                                   2)  + 15;
  }

  return timeToNextTimecheckInSeconcs;
}

/*
 * ###################################################################################################
 *              S E C   -   S Y S T E M T I M E    I N   S E C U N D S
 * ###################################################################################################
 * 
 * sec()
 * 
 * [Time]
 *
 * Description
 * 
 * Returns the number of secunds passed since the board began running the current program. 
 * This number will overflow (go back to zero), after approximately 136 years.
 * 
 * Syntax
 * 
 * time = sec()
 * 
 * Parameters
 * 
 * None
 * 
 * Returns
 * 
 * Number of seconds passed since the program started. Data type: unsigned long.
 * 
 * NOTE: The function is based on the ESP Timer (High Resolution Timer), the function esp_timer_get_time() returns a 64 bit value in microseconds.
 * 
 * 
 * Example Code
 * This example code prints on the serial port the number of seconds passed since the board started running the code itself.
 
 unsigned long myTime;
 unsigned long sec();
 
void setup() {
  Serial.begin(9600);
}
void loop() {
  Serial.print("Time: ");
  myTime = sec();

  Serial.println(myTime); // prints time since program started
  delay(1000);          // wait a second so as not to send massive amounts of data
}
 *
 */
unsigned long sec()
{
  int64_t m = esp_timer_get_time();
  unsigned long s = m / 1000000;
  return s;
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
void publishMqttEnergyConfigJson( String component, String entityName, String unitOfMesurement, String deviceClass, u_int8_t PIN_reference)
{
  uint8_t payload[1024];
  JsonDocument doc;
  String energyMeter = String( private_energyMeters[PIN_reference]);

  if ( component == MQTT_NUMBER_COMPONENT & deviceClass == MQTT_ENERGY_DEVICECLASS)
  {
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
void publishSensorJson( long powerConsumption, uint8_t IRQ_PIN_index)
{
  uint8_t payload[256];
  JsonDocument doc;

  doc[MQTT_SENSOR_ENERG_ENTITYNAME] = float(meterData[IRQ_PIN_index].pulseSubTotal) / float(interfaceConfig.pulse_per_kWh[IRQ_PIN_index]);
  doc[MQTT_SENSOR_POWER_ENTITYNAME] = powerConsumption;
  doc[MQTT_NUMBER_ENERG_ENTITYNAME] = float(meterData[IRQ_PIN_index].pulseTotal) / float(interfaceConfig.pulse_per_kWh[IRQ_PIN_index]);

  size_t length = serializeJson(doc, payload);
  String sensorTopic = String(MQTT_DISCOVERY_PREFIX + MQTT_PREFIX + MQTT_PREFIX_DEVICE + IRQ_PIN_index + MQTT_SUFFIX_STATE);

  mqttClient.publish(sensorTopic.c_str(), payload, length, UNRETAINED);
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
void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  JsonDocument doc;                         // 
  byte IRQ_PIN_reference = 0;
  String topicString = String(topic);

  if( topicString.startsWith(MQTT_PREFIX))
  {
    IRQ_PIN_reference = getIRQ_PIN_reference(topic);
  }

  if ( topicString.endsWith(MQTT_SUFFIX_TOTAL_TRESHOLD))
  {
    deserializeJson(doc, payload, length);
    meterData[IRQ_PIN_reference].pulseTotal = long(float(doc[MQTT_NUMBER_ENERG_ENTITYNAME]) * float(interfaceConfig.pulse_per_kWh[IRQ_PIN_reference]));
    long watt_consumption = 0;
    publishSensorJson( watt_consumption, IRQ_PIN_reference);
  }
  else if ( topicString.endsWith(MQTT_SUFFIX_CONFIG))
  {
    /* Set pulse time correction in milliseconds. Done by:
    * Publish: {"pulscorr" : 25}
    * To topic: energy/monitor_ESP32_48E72997D320/config
    */
  
    deserializeJson(doc, payload, length);
    if ( doc.containsKey( MQTT_PULSTIME_CORRECTION))
    {
      interfaceConfig.pulseTimeCorrection = long(doc[MQTT_PULSTIME_CORRECTION]);
    }
    writeConfigData();
  }
  else if ( topicString.endsWith(MQTT_SUFFIX_SUBTOTAL_RESET))
  {
    /* Publish totals, subtotals to GS and reset subtotals. Done by
    * Publish: true
    * To topic: energy/monitor_ESP32_48E72997D320/subtotal_reset
    */
    
    if (PRIVATE_UPDATE_GOOGLE_SHEET)
      updateGoogleSheets( 0);
    for ( uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++)
    {
      meterData[ii].pulseSubTotal = 0;
      if ( !SD_Failed ) 
        writeMeterData( ii);
    }
    
  }
  else if ( topicString.endsWith(MQTT_SUFFIX_STATUS))
  {
    for (uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++)
    {
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
void IRAM_ATTR store_IRQ_PIN(u_int8_t BIT_Reference)
{
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
void IRAM_ATTR Ext_INT1_ISR()
{
  store_IRQ_PIN( 0);
}
void IRAM_ATTR Ext_INT2_ISR()
{
  store_IRQ_PIN( 1);
}
void IRAM_ATTR Ext_INT3_ISR()
{
  store_IRQ_PIN( 2);
}
void IRAM_ATTR Ext_INT4_ISR()
{
  store_IRQ_PIN( 3);
}
void IRAM_ATTR Ext_INT5_ISR()
{
  store_IRQ_PIN( 4);
}
void IRAM_ATTR Ext_INT6_ISR()
{
  store_IRQ_PIN( 5);
}
void IRAM_ATTR Ext_INT7_ISR()
{
  store_IRQ_PIN( 6);
}
void IRAM_ATTR Ext_INT8_ISR()
{
  store_IRQ_PIN( 7);
}
