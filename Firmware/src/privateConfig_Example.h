/*
 * Private configuration file for projekt: "Esp32 MQTT interface for Carlo Gavazzi energy meter - V2.0.2"
 * ######################################################################################################################################
 * ######################################################################################################################################
 *                       C  O  N  F  I  G  U  T  A  B  L  E       D  E  F  I  N  I  T  I  O  N  S
 * ######################################################################################################################################
 * ######################################################################################################################################
*/

//How to show a list of Wi-Fi connections: netsh wlan show profiles

const String   PRIVATE_WIFI_SSID    = "ESSID Name";         // wifi ssid
const String   PRIVATE_WIFI_PASS    = "SSID Pasword";     // wifi password

const String   PRIVATE_MQTT_SERVER  = "nnn.nnn.nnn.nnn"; // mqtt server address 

const String   PRIVATE_MQTT_USER    = "";       // mqtt user. Use "" for no username
const String   PRIVATE_MQTT_PASS    = "";       // mqtt password. Use "" for no password
const uint16_t PRIVATE_MQTT_PORT    = 1883;     // mqtt port (default is 1883)

/*
 * Define which GPIO pin numbers are used for IRQ pin / Energy Meter.

 * These eight GPIO's can all be configured as INPUT_PULLUP, which mens that no external componenst are required
 * Do NOT remove any of the definations! If less than eight GPIO pin's are used, keep the defination for the unused
 * GPIO's as it is.
 */
#define private_Metr1_GPIO   4
#define private_Metr2_GPIO   12
#define private_Metr3_GPIO   13
#define private_Metr4_GPIO   14
#define private_Metr5_GPIO   15
#define private_Metr6_GPIO   25
#define private_Metr7_GPIO   26
#define private_Metr8_GPIO   27

/*
 *  Google sheets script id. 
 *  Find the schript ID from Google Apps Script -> Deploy -> Manage Deployments -> (Select Deployment) -> Copy ID part of Web Url.
 *  Example: https://script.google.com/macros/s/2AAU82PQwSI9opXFDJ1CNyhjA0fiJGPXuxYloCgRnfN242lnb2r6SgkkXSUDRdhWijJDq3GyWv/exec
 *  ID =                                        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *  The Google sheets script id entered here is an example!
*/
bool PRIVATE_UPDATE_GOOGLE_SHEET = true;   // Set the flag to true if data are to be added to Google sheets
const String PRIVATE_GOOGLE_SCRIPT_ID = "2AAU82PQwSI9opXFDJ1CNyhjA0fiJGPXuxYloCgRnfN242lnb2r6SgkkXSUDRdhWijJDq3GyWv";

/*
 * Define when Google sheets are to be updated
 */
#define SCHEDULE_MINUTE  0
#define SCHEDULE_HOUR  0

// Number of energy meters connected. 
// Be sure to modify the values for next variable (private_default_pulse_per_kWh).
#define PRIVATE_NO_OF_CHANNELS 8

// Define default number of pulses per kWh for each energy meter.

uint16_t private_default_pulse_per_kWh[PRIVATE_NO_OF_CHANNELS] = {1000,1000,1000,1000,1000,100,100,100};

char * private_energyMeters[] = {
           (char*) "Name precented in Home Assistant for energy meter connected to Metr1_GPIO",
           (char*) "Name precented in Home Assistant for energy meter connected to Metr2_GPIO",
           (char*) "Name precented in Home Assistant for energy meter connected to Metr3_GPIO",
           (char*) "Name precented in Home Assistant for energy meter connected to Metr4_GPIO",
           (char*) "Name precented in Home Assistant for energy meter connected to Metr5_GPIO",
           (char*) "Name precented in Home Assistant for energy meter connected to Metr6_GPIO",
           (char*) "Name precented in Home Assistant for energy meter connected to Metr7_GPIO",
           (char*) "Name precented in Home Assistant for energy meter connected to Metr8_GPIO",
};
