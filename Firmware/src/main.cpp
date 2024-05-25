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
 * LED indications:
 * 
 * 
 * )** pulse-counts in the number of pulses counted by the energy meter. It is calculated by the kWh,
 * shown on the energy meter multiplied be the number of pulses per kWh for the energy meter. 
 *  Pulses per kWh can be found in the documentation for the energy meter.
 * )*** ‘Energy meter number’ is a number 0..7 for the channel, on which the energy meter is connected.
 * The relation between energy meter and channel number is defined in the privateConfig.h file. 
 */ 

#define SKETCH_VERSION "Esp32 MQTT interface for Carlo Gavazzi energy meter - V3.1.0"

// #define DEBUG true
// #define SD_DEBUG true

/* Version history:
 *  1.0.0   Initial production version.
 *  2.0.0   Using MQTT broker as a storage for configuration data and energy meter counts, read/write from/to SD memory card has been implemnted.
 *          Struct data_t re-defined and split up to config_t, meta_t and data_t for optimal file read / write funktionality.
 *          Publishing totals, Subtotals and pulscorrections to MQTT has been replaced be write to SD Memory card.
 *  2.0.1   MQTT Reconnect bugfix
 *  2.0.2   serial removed as it is not needed.
 *  3.0.0   Publish data to Google Sheets based on epoch time and time collected from a time server
 *  3.0.1   BUGFIX: Interrupt ISR were attached to a non configured pin, which could cause an un-handled IRQ, which then cause infinite loop.
 *  3.1.0   Adding DEBUG compilor switch to investigate SD issue.
 *          -  Code re-arranged to comply with VS Code standard.
 *          -  Need to re-think the Idea for not using directories. The SD library uses short 8.3 names for files. This might be an issue.
 *             Data file sets now placed in directories.
 *          -  Startup delay for 10 sec. introduced, before accessinmg the SD Card. This is to prevent SD Card issues while connecting the device to power.
 *             While onnecting the device to power will in moast cases cause multible power failures (dis-connections), before the power is stable.
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
 */

/*
 * ######################################################################################################################################
 * ######################################################################################################################################
 *                       V  A  R  I  A  B  L  E      D  E  F  I  N  A  I  T  O  N  S
 * ######################################################################################################################################
 * ######################################################################################################################################
 */
#define CONFIGURATON_VERSION 4
/* WiFi and MQTT connect attempt issues. 
 * IRQ's will be registrated, but the counters for will not be updated during the calls to WiFi and MQTT connect. If more than one pulse
 * from then same meter arrives, it will be lost if these calls takes up too much time. Setting a long connect postpone will reduce the loss
 * of pulses
*/
#define WIFI_CONNECT_POSTPONE 30000     // Number of millis between WiFi connect attempts, when WiFi.begin fails to connect.
#define MQTT_CONNECT_POSTPONE 30000     // Number of millisecunds between MQTT connect dattempts, when MQTT connect fails to connect.
#define BLIP 100                        // Time in milliseconds the LED will blink
#define MAX_COMSUMPTION 2200            // Maximum of Watt comsumptino to be registrated. A software method to prevent fake pulses
#define MIN_CONSUMPTION 25              // Define the minimum powerconsumption published before publishing 0
                                        // See '>>>    Pulse time check  <<<' in the last part of the loop() for further details.
#define RETAINED true                   // Used in MQTT puplications. Can be changed during development and bugfixing.
#define UNRETAINED false
#define MAX_NO_OF_CHANNELS 8
#define NUMBER_OF_WRITES 65500          // Number of writes made to data file, before new set of datafiles will be used (MAX 2^16)

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
const String DATAFILESET_POSTFIX    = "/fs-";           // Will bee the directory name
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

uint16_t blip = BLIP;
uint8_t GlobalIRQ_PIN_index = 0;
uint16_t numberOfWrites = 0;

// Define array of GPIO pin numbers used for IRQ.
const uint8_t channelPin[MAX_NO_OF_CHANNELS] = {private_Metr1_GPIO,private_Metr2_GPIO,private_Metr3_GPIO,private_Metr4_GPIO,
                                                private_Metr5_GPIO,private_Metr6_GPIO,private_Metr7_GPIO,private_Metr8_GPIO};  

bool configurationPublished[PRIVATE_NO_OF_CHANNELS];  // a flag for publishing the configuration to HA if required.
bool esp32Connected = false;                          // Is true, when connected to WiFi and MQTT Broker
bool LED_Toggled = false; 
bool LED_Invertred = false;
bool gsheetUpdated = false;

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
unsigned long millisToNextTimeCheck;    // Number of millis to next epoch time check.


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
void indicateError( uint8_t);
void writeConfigData();
void writeMeterDataFile( uint8_t);
void writeMeterData(uint8_t);
void setConfigurationDefaults();
void initializeGlobals();
void publish_sketch_version();
void publishStatusMessage(String);
byte getIRQ_PIN_reference(char*);
void updateGoogleSheets( boolean);
unsigned long getMillisToNextTimeCheck();
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

                                                                                                      #ifdef DEBUG
                                                                                                        void breakpoint();
                                                                                                        void printDirectory( File, int);
                                                                                                      #endif
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
                                                                                                      #ifdef DEBUG
                                                                                                        Serial.begin( 115200);
                                                                                                        while (!Serial) {
                                                                                                          ;  // Wait for serial connectionsbefore proceeding
                                                                                                        }
                                                                                                        Serial.println(SKETCH_VERSION);
                                                                                                        Serial.println("Hit [Enter] to start!");
                                                                                                        while (!Serial.available()) {
                                                                                                          ;  // In order to prevent unattended execution, wait for [Enter].
                                                                                                        }
                                                                                                        while (Serial.available()) {
                                                                                                          char c = Serial.read();  // Empty input buffer.
                                                                                                        }
                                                                                                      #endif
  pinMode(LED_BUILTIN, OUTPUT);             // Initialize build in LED           
  digitalWrite(LED_BUILTIN, HIGH);          // Turn ON LED to indicate startup

  // Initialize Interrupt pins
  for ( u_int8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++) {
    pinMode(channelPin[ii], INPUT_PULLUP);
  }

  // arm interrupt. Create a functioncall for each interrupt pin
  if ( PRIVATE_NO_OF_CHANNELS >= 1)
    attachInterrupt(private_Metr1_GPIO, Ext_INT1_ISR, FALLING);
  if ( PRIVATE_NO_OF_CHANNELS >= 2)
    attachInterrupt(private_Metr2_GPIO, Ext_INT2_ISR, FALLING);
  if ( PRIVATE_NO_OF_CHANNELS >= 3)
    attachInterrupt(private_Metr3_GPIO, Ext_INT3_ISR, FALLING);
  if ( PRIVATE_NO_OF_CHANNELS >= 4)
    attachInterrupt(private_Metr4_GPIO, Ext_INT4_ISR, FALLING);
  if ( PRIVATE_NO_OF_CHANNELS >= 5)
    attachInterrupt(private_Metr5_GPIO, Ext_INT5_ISR, FALLING);
  if ( PRIVATE_NO_OF_CHANNELS >= 6)
    attachInterrupt(private_Metr6_GPIO, Ext_INT6_ISR, FALLING);
  if ( PRIVATE_NO_OF_CHANNELS >= 7)
    attachInterrupt(private_Metr7_GPIO, Ext_INT7_ISR, FALLING);
  if ( PRIVATE_NO_OF_CHANNELS >= 8)
    attachInterrupt(private_Metr8_GPIO, Ext_INT8_ISR, FALLING);

  initializeGlobals();

  mqttClient.setServer(PRIVATE_MQTT_SERVER.c_str(), PRIVATE_MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

/*
 * Read configuration and energy meter data from SD memoory
 * To prevent SD Cart failures caused by power interruption during mounting, make a delay to wait
 * for a steady power supply
 */
  delay( 10000);
                                                                                                      #ifdef DEBUG
                                                                                                        Serial.println("Initialize SD.");
                                                                                                      #endif
  if(!SD.begin(5)){
    indicateError(1);
  }
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.println("Files on SD Card: ");
                                                                                                        File root;
                                                                                                        root = SD.open("/");
                                                                                                        printDirectory( root, 0);
                                                                                                        root.close();
                                                                                                        Serial.println( "Done!");
                                                                                                        breakpoint();
                                                                                                      #endif

  // Reading configuration 
  String configFilename = String(CONFIGURATION_FILENAME);
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.print("Open configuration file: ");
                                                                                                        Serial.println( configFilename);
                                                                                                      #endif
  File structFile = SD.open(configFilename, FILE_READ);
  if ( structFile) {
    structFile.read((uint8_t *)&interfaceConfig, sizeof(interfaceConfig)/sizeof(uint8_t));
    structFile.close();
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.print("Configuration file read. Structure version: ");
                                                                                                        Serial.println( interfaceConfig.structureVersion);
                                                                                                        Serial.print("Current version: ");
                                                                                                        Serial.println( (CONFIGURATON_VERSION * 100) + PRIVATE_NO_OF_CHANNELS);
                                                                                                        Serial.print("Datafile set number: ");
                                                                                                        Serial.println( interfaceConfig.dataFileSetNumber);
                                                                                                      #endif
  }

  // Check if new configuration and datafiles are required
  if ( interfaceConfig.structureVersion != (CONFIGURATON_VERSION * 100) + PRIVATE_NO_OF_CHANNELS) {
    setConfigurationDefaults();
  }

// Reading datafiles.
  uint16_t numberOfDatafilesRead = 0;
  for ( uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++) {
    String filename = String (DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber) + FILENAME_POSTFIX + String(ii) + FILENAME_SUFFIX);
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.print("Attempt to open datafile: ");
                                                                                                        Serial.print( filename);
                                                                                                      #endif
    structFile = SD.open(filename, FILE_READ);
    if ( structFile) {
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.println(". Succeeded.");
                                                                                                        Serial.print("Attempt to read datafile.");
                                                                                                      #endif
      if (structFile.read((uint8_t *)&meterData[ii], sizeof(meterData[ii])/sizeof(uint8_t)) == sizeof(meterData[ii])/sizeof(uint8_t)) {
         numberOfDatafilesRead++;
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.println(". Succeeded.");
                                                                                                      #endif
      } else {
        meterData[ii].pulseTotal = 0;
        meterData[ii].pulseSubTotal = 0;
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.println(". Failed.");
                                                                                                      #endif
      }
      structFile.close();
    } else {
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.println(". Failed.");
                                                                                                      #endif
      meterData[ii].pulseTotal = 0;
      meterData[ii].pulseSubTotal = 0;
    }
  }

  // IF any datafiles in the current datafileset exists, create new datafileset
  if (numberOfDatafilesRead > 0)
  {
    interfaceConfig.dataFileSetNumber++;
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.println("Creating new dataset.");
                                                                                                      #endif
  }
                                                                                                      #ifdef SD_DEBUG
                                                                                                        else
                                                                                                        {
                                                                                                          Serial.println("Reuse existing dataset.");
                                                                                                        }
                                                                                                        
                                                                                                      #endif
  writeConfigData();
  // Create directory for data file set.
  String dirname = String (DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber));
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.print("Creating direectory: ");
                                                                                                        Serial.println( dirname);
                                                                                                      #endif
  if ( !SD.mkdir( dirname))
  {
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.print("FAILED to create direectory: ");
                                                                                                        Serial.println( dirname);
                                                                                                      #endif
    indicateError(4);
  }

  for ( uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++) {
    writeMeterData( ii);
  }
                                                                                                      #ifdef SD_DEBUG
                                                                                                        Serial.println("Files on SD Card: ");
                                                                                                        root = SD.open("/");
                                                                                                        printDirectory( root, 0);
                                                                                                        root.close();
                                                                                                        Serial.println( "Done!");
                                                                                                        breakpoint();
                                                                                                      #endif
                                                                                                      #ifdef DEBUG
                                                                                                        Serial.println("SD Initialized.");
                                                                                                        breakpoint();
                                                                                                      #endif
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
// Ingore WiFi connect if IRQ's are present.
  if ( IRQ_PINs_stored == 0 and \
       WiFi.status() != WL_CONNECTED and millis() > WiFiConnectAttempt + WiFiConnectPostpone) {

    /* Turn Build in LED (LED) ON, when not connected to WiFi.
     * The LED is also used as indicator for interrupt activity (Pules registrered). To combine this
     * with the LED being ON when no WiFi connected, an interrupt activity will tuggle the LED and after
     * a short time (BLIP) the LED will be tuggled again. This way the LED will either turn ON or turn off
     * for a short time, depending on WiFi connection.
     * SO: In order to turn the led ON when not connectged to WiFi, the LED has to be se ON or Off depending
     * on status set by the interrupt activity.
    */

    if (LED_Invertred == false) {
      digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
      LED_Invertred = true;
    }
    
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(PRIVATE_WIFI_SSID.c_str(), PRIVATE_WIFI_PASS.c_str());

    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      WiFiConnectAttempt = millis();
      WiFiConnectPostpone = WIFI_CONNECT_POSTPONE;
      blip = 10 * BLIP;        // Make a long blip (LED Flash) to indicate no connection to MQTT broker
      esp32Connected = false;
    } else {
      WiFiConnectAttempt = 0;   // In case WiFi is lost, attempt to reconnect immediatly. 
      WiFiConnectPostpone = 0;

      // Init epoch time
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

      millisToNextTimeCheck = getMillisToNextTimeCheck();
      timeLastCheckedAt = millis();
      if (PRIVATE_UPDATE_GOOGLE_SHEET and !gsheetUpdated) {
        bool AddPowerON = true;
        gsheetUpdated = true;
        updateGoogleSheets( AddPowerON);
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
        .onStart([]() {
          String type;
          if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
          else // U_SPIFFS
            type = "filesystem";

          // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
          //  Serial.println("Start updating " + type);                                                      ############################################
        })
        .onEnd([]() {
          // Serial.println("\nEnd");                                                                         ############################################
        })
        .onProgress([](unsigned int progress, unsigned int total) {
          // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));                                  ############################################
        });
        /*
        .onError([](ota_error_t error) {
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

// >>>>>>>>>>>>>>>>>>>>    IF  Connected to WiFi -> Connect to MQTT broker    <<<<<<<<<<<<<<<<<<<<<<<<<
// Ignore connect to MQTT if IRQ's are present-
  if ( IRQ_PINs_stored == 0 and WiFi.status() == WL_CONNECTED) {

    if (LED_Invertred == true) {
      digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
      LED_Invertred = false;
    }

    ArduinoOTA.handle();
    
    // >>>>>>>>>>>>>>>>>>> Connect to MQTT Broker <<<<<<<<<<<<<<<<<<<<<<<<<<
    if (!mqttClient.connected() and millis() > MQTTConnectAttempt + MQTTConnectPostpone ) {
      String will = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE);

      if ( mqttClient.connect( mqttClientWithMac.c_str(), PRIVATE_MQTT_USER.c_str(), PRIVATE_MQTT_PASS.c_str(), will.c_str(), 1, RETAINED, "False")) {

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
        
      } else {
        MQTTConnectAttempt = millis();
        MQTTConnectPostpone = MQTT_CONNECT_POSTPONE;
        blip = 10 * BLIP;        // Make a long blip (LED Flash) to indicate no connection to MQTT broker
        esp32Connected = false;
      }
    }
  }
  // >>>>>>>>>>>>>>>>>>>  E N D IF  Connected to WiFi -> Connect to MQTT broker  <<<<<<<<<<<<<<<<<<<<<<<<<<

  // >>> Process incomming messages and maintain connection to the server
  if ( esp32Connected) {
    if(!mqttClient.loop()) {
      blip = 10 * BLIP;        // Make a long blip (LED Flash) to indicate no connection to MQTT broker
      esp32Connected = false;
    }
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

    for( uint8_t IRQ_PIN_index = 0; IRQ_PIN_index < PRIVATE_NO_OF_CHANNELS; IRQ_PIN_index++) { // Iterate through all bits in the byte IRQ_PINs_stored.
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
        if ( metaData[IRQ_PIN_index].pulseTimeStamp > 0 && metaData[IRQ_PIN_index].pulseTimeStamp < millsTimeStamp[IRQ_PIN_index]) { 
          watt_consumption = round(((float)(60*60*1000) / 
                                (float)(millsTimeStamp[IRQ_PIN_index] - metaData[IRQ_PIN_index].pulseTimeStamp + interfaceConfig.pulseTimeCorrection)) 
                                / (float)interfaceConfig.pulse_per_kWh[IRQ_PIN_index] * 1000);

          metaData[IRQ_PIN_index].pulseLength = millsTimeStamp[IRQ_PIN_index] - metaData[IRQ_PIN_index].pulseTimeStamp + interfaceConfig.pulseTimeCorrection;
        }

        //   >>>>>>>>>>>>>>>>>>>>>>>>>>>  Update meterData and publish totals   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
        if ( watt_consumption < MAX_COMSUMPTION) {
          metaData[IRQ_PIN_index].pulseTimeStamp = millsTimeStamp[IRQ_PIN_index];
          meterData[IRQ_PIN_index].pulseTotal++;
          meterData[IRQ_PIN_index].pulseSubTotal++;
          
          writeMeterData( IRQ_PIN_index);
          if ( esp32Connected) {
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
    if ( metaData[GlobalIRQ_PIN_index].pulseLength > 0 && metaData[GlobalIRQ_PIN_index].pulseTimeStamp < timeStamp) {
      if (  metaData[GlobalIRQ_PIN_index].pulseTimeStamp + ( 2 * metaData[GlobalIRQ_PIN_index].pulseLength) < timeStamp ) {
        
        long watt_consumption = round(((float)(60*60*1000) / 
                                  (float)(timeStamp - metaData[GlobalIRQ_PIN_index].pulseTimeStamp + interfaceConfig.pulseTimeCorrection)) 
                                  / (float)interfaceConfig.pulse_per_kWh[GlobalIRQ_PIN_index] * 1000);

        if ( watt_consumption < MIN_CONSUMPTION) {
          watt_consumption = 0;
          metaData[GlobalIRQ_PIN_index].pulseLength = 0;
        }
        if ( esp32Connected)
          publishSensorJson( -1 * watt_consumption, GlobalIRQ_PIN_index);

        metaData[GlobalIRQ_PIN_index].pulseLength *= 2;
      }
    }
  /* Tuggle LED Pin if tuggled and BLIP time has passed
  */
    if ( LED_Toggled and millis() > LED_toggledAt + blip) {
      digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
      LED_Toggled = false;
      LED_toggledAt = 0;
    }

    GlobalIRQ_PIN_index++;
  }

  /* >>>>>>>>>>>>>>>>>>>>>>>>>>> time check to schecule <<<<<<<<<<<<<<<<<<< */
  if (millis() > timeLastCheckedAt + millisToNextTimeCheck){
    timeLastCheckedAt = millis();
    millisToNextTimeCheck = getMillisToNextTimeCheck();
  }

  if ( millisToNextTimeCheck == 0) {
    millisToNextTimeCheck = 60000; // Postpone a minute to get next millisToNextTimeCheck

    if (PRIVATE_UPDATE_GOOGLE_SHEET and WiFi.status() == WL_CONNECTED)
      updateGoogleSheets( false);
    for ( uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++) {
      meterData[ii].pulseSubTotal = 0;
      writeMeterData( ii);
    }
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
/*
 * Flashing LED to indicate an error
 * 
 */
void indicateError( uint8_t errorNumber)
{
                                                                                                      #ifdef DEBUG
                                                                                                        Serial.println("Error indication called.");
                                                                                                      #endif
  digitalWrite(LED_BUILTIN, HIGH);
  while (true)
  {
    for ( uint8_t ii = 0; ii < errorNumber; ii++)
    {
      digitalWrite(LED_BUILTIN, LOW);
      delay( BLIP);
      digitalWrite(LED_BUILTIN, HIGH);
      delay( BLIP);
    }
    delay( 1000);
  }
}
/* ###################################################################################################
 *               W R I T E   C O N F I G   D A T A
 * ###################################################################################################
 */
void writeConfigData()
{
  String configFilename = String(CONFIGURATION_FILENAME);
                                                                                                      #ifdef DEBUG
                                                                                                        Serial.println("Open config file for writing");
                                                                                                      #endif
  File structFile = SD.open(configFilename, FILE_WRITE);
  if (!structFile)
    indicateError(2);
                                                                                                      #ifdef DEBUG
                                                                                                        Serial.println("Successfull opened");
                                                                                                      #endif
  structFile.seek(0);
  structFile.write((uint8_t *)&interfaceConfig, sizeof(interfaceConfig));
  structFile.close();
}
/* ###################################################################################################
 *               W R I T E   M E T E R   D A T A
 * ###################################################################################################
 */
void writeMeterDataFile( uint8_t datafileNumber)
{
  String filename = String ( DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber) + FILENAME_POSTFIX + String(datafileNumber) + FILENAME_SUFFIX);
                                                                                                      #ifdef DEBUG
                                                                                                        Serial.print("Open datafile: ");
                                                                                                        Serial.print( filename);
                                                                                                        Serial.print(". for writing...");
                                                                                                      #endif
  File structFile = SD.open(filename, FILE_WRITE);
  if (!structFile)
  {
                                                                                                      #ifdef DEBUG
                                                                                                        Serial.println("FAILED!");
                                                                                                      #endif
    indicateError(3);
  }
                                                                                                        #ifdef DEBUG
                                                                                                        Serial.println("Successful.");
                                                                                                      #endif

  structFile.seek(0);
  structFile.write((uint8_t *)&meterData[datafileNumber], sizeof(meterData[datafileNumber]));
  structFile.close(); 
}

void writeMeterData(uint8_t datafileNumber)
{
  if ( numberOfWrites++ >  NUMBER_OF_WRITES)
  {
    interfaceConfig.dataFileSetNumber++;
    String dirname = String (DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber));
    SD.mkdir( dirname);
    writeConfigData();
    for ( uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++)
      writeMeterDataFile(ii);
  } else
    writeMeterDataFile(datafileNumber);
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
 * - uint16_t dataFileSetNumber;                  // In which data file set ("directory") the data files will be located.
 * - uint16_t  pulse_per_kWh[PRIVATE_NO_OF_CHANNELS];       // Number of pulses as defined for each energy meter
 */

void setConfigurationDefaults()
{
  interfaceConfig.structureVersion = (CONFIGURATON_VERSION * 100) + PRIVATE_NO_OF_CHANNELS;
  interfaceConfig.pulseTimeCorrection = 0;  // Used to calibrate the calculated consumption.

  // Find new data file set for data files == Skip all existing data files.
  uint8_t numberOfFilesExists = PRIVATE_NO_OF_CHANNELS;
  for ( uint16_t filesetNumber = 0; numberOfFilesExists == PRIVATE_NO_OF_CHANNELS; filesetNumber++)
  {
    interfaceConfig.dataFileSetNumber = filesetNumber;
    numberOfFilesExists = 0;
    for ( uint8_t iix = 0; iix < PRIVATE_NO_OF_CHANNELS; iix++)
    {
      String dataFileName = String ( DATAFILESET_POSTFIX + String(interfaceConfig.dataFileSetNumber) + FILENAME_POSTFIX + String(iix) + FILENAME_SUFFIX);
      if (SD.exists(dataFileName))
        numberOfFilesExists++;
    }
  }
  for (uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++)
    interfaceConfig.pulse_per_kWh[ii] = private_default_pulse_per_kWh[ii];    // Number of pulses as defined for each energy meter

  String filename = String(CONFIGURATION_FILENAME);
  if (SD.exists(filename))
  {
    SD.remove(filename);
  }
  writeConfigData();
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
void updateGoogleSheets( boolean powerOn)
{
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

  if ( powerOn == true)
    urlData += String(",PowerUp");

  // >>>>>>>>>>>>>   Create URL for HTTP request   <<<<<<<<<<<<<<<<<<
  String urlFinal = "https://script.google.com/macros/s/" + PRIVATE_GOOGLE_SCRIPT_ID + urlData;

  // >>>>>>>>>>>>>   Make HTTP request to google sheet  <<<<<<<<<<<<<<<<<<
  if ( PRIVATE_UPDATE_GOOGLE_SHEET)
  {
    HTTPClient http;
    http.begin(urlFinal.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    // >>>>>>>>>>>>>   getting response from google sheet  <<<<<<<<<<<<<<<<<<
    if (int httpCode = http.GET())
    {
      String httpMessage = http.getString();
      String statusMessage = String( String("HTTP Status Code: ") + httpCode + "HTTP Message: " + httpMessage);
      publishStatusMessage( statusMessage);
    }
    //---------------------------------------------------------------------
    http.end();
  }
}
/* ###################################################################################################
 *          G E T   M I L L I S   T O   N E X T    T I M E   C H E C K
 * ###################################################################################################

/*
 * Then function is used to schedule a piece of program code at a time defined by the two definitions:
 * #define SCHEDULE_MINUTE  mm
 * #define SCHEDULE_HOUR  hh
 * 
 * getMillisToNextTimeCheck() will return the number om millis() diveded by two, counted from the time 
 * it is called to the time defined.
 * Doing it this way reduce the number of calls to the timeserver.
 * 
 * getMillisToNextTimeCheck() returns 0 (zero) when it's called at the time defined.
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
 *   timeLastCheckedAt = millis();
 *   millisToNextTimeCheck = getMillisToNextTimeCheck();
 * }
 * 
 * loop() {
 *  if (millis() > timeLastCheckedAt + millisToNextTimeCheck){
 *    timeLastCheckedAt = millis();
 *    millisToNextTimeCheck = getMillisToNextTimeCheck();
 *  }
 *  if ( millisToNextTimeCheck == 0 ) {
 *    millisToNextTimeCheck = 60000  // Snooze one minute to avoid calls within the same minute
 *     
 *     * Code to be executred
 *      
 *   }
 * 
 */
unsigned long getMillisToNextTimeCheck()
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
                                   2) *1000 + 30000;
  }

  return timeToNextTimecheckInSeconcs;
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
    interfaceConfig.pulseTimeCorrection = long(doc[MQTT_PULSTIME_CORRECTION]);
    writeConfigData();
  }
  else if ( topicString.endsWith(MQTT_SUFFIX_SUBTOTAL_RESET))
  {
    /* Publish totals, subtotals to google sheets and reset subtotals. Done by
    * Publish: true
    * To topic: energy/monitor_ESP32_48E72997D320/subtotal_reset
    */
    
    if (PRIVATE_UPDATE_GOOGLE_SHEET)
      updateGoogleSheets( false);
    for ( uint8_t ii = 0; ii < PRIVATE_NO_OF_CHANNELS; ii++)
    {
      meterData[ii].pulseSubTotal = 0;
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


                                                                                                      #ifdef DEBUG
                                                                                                        void breakpoint()
                                                                                                        {
                                                                                                          while (Serial.available())
                                                                                                          {
                                                                                                            char c = Serial.read();  // Empty input buffer.
                                                                                                          }
                                                                                                          Serial.println("Enter any character and hit [Enter] to contiue!");
                                                                                                          while (!Serial.available())
                                                                                                          {
                                                                                                            ;  // In order to prevent unattended execution, wait for [Enter].
                                                                                                          }
                                                                                                          while (Serial.available())
                                                                                                          {
                                                                                                            char c = Serial.read();  // Empty input buffer.
                                                                                                          }

                                                                                                        }

                                                                                                      #endif

                                                                                                      #ifdef SD_DEBUG
                                                                                                        void printDirectory( File dir, int numTabs)
                                                                                                        {
                                                                                                          while (true)
                                                                                                          {
                                                                                                            File entry =  dir.openNextFile();
                                                                                                            if (! entry)
                                                                                                            {
                                                                                                              // no more files
                                                                                                              break;
                                                                                                            }
                                                                                                            for (uint8_t i = 0; i < numTabs; i++) {
                                                                                                              Serial.print('\t');
                                                                                                            }
                                                                                                            Serial.print(entry.name());
                                                                                                            if (entry.isDirectory())
                                                                                                            {
                                                                                                              Serial.println("/");
                                                                                                              printDirectory(entry, numTabs + 1);
                                                                                                            }
                                                                                                            else
                                                                                                            {
                                                                                                              // files have sizes, directories do not
                                                                                                              Serial.print("\t\t");
                                                                                                              Serial.println(entry.size(), DEC);
                                                                                                            }
                                                                                                            entry.close();
                                                                                                          }
                                                                                                        }
                                                                                                      #endif
