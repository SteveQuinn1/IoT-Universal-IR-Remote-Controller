// Copyright 2017 Steve Quinn
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
// Requires MQTT PubSubClient Library found here:                 https://github.com/knolleary/pubsubclient
// Requires BounceI2C Library found here:                         https://github.com/SteveQuinn1/BounceI2C
// Requires Adafruit DHT Library found here:                      https://github.com/adafruit/DHT-sensor-library
// Requires BH1750 Library found here:                            https://github.com/claws/BH1750
// Requires ESP8266WiFi Library found here:                       https://github.com/ekstrand/ESP8266wifi/
// Requires Arduino IDE support for ESP8266 found here:           http://esp8266.github.io/Arduino/versions/2.2.0-rc1/doc/installing.html
// Requires SD file system support found in Arduino Core or here: https://github.com/esp8266/Arduino/blob/master/cores/esp8266/FS.h
// Requires ESP8266WebServer Library found here:                  http://esp8266.github.io/Arduino/versions/2.2.0-rc1/doc/installing.html
// Requires ESP8266mDNS Library found here:                       http://esp8266.github.io/Arduino/versions/2.2.0-rc1/doc/installing.html
//
// Overview of software
// --------------------
// Using DHT22 (Temp and Humidity) sensor, PCF8574P (I2C I/O Extender), BH1750 (Ambient Light) sensor and ATMega328P-PU IR remote device simulator
//
// Compiled for ESP8266-12E on Arduino IDE 1.6.9
// 
// Inclusive of the above the software supports MQTT over WiFi and allows for the following;
// 1. Setting led statuses
// 2. Reading software version
// 3. Reading local, Temperature, Humidity, Heat Index, Ambient light level.
//
//
// Maintenance History
// 24/06/17 : 1     Inherits from MQTTWiFiAll16_8.ino (stable IoT framework). First creation of file. Inital test works with, IR_Remote_Sim3.ino/IR_Remote_Sim_Test3.ino
// 25/06/17 : 2     Added improved packed Union to work with IR_Remote_Sim4.ino and successfully live tested on TV, Sound Bar and Sky HD box.
// 25/06/17 : 3     Added JSON file access capablity.
// 15/07/17 : 4     Removed JSON file access capability (library far too bloated and a PITA to use, not inclusive, pat on the back to the developer for creating unintelligible C++), and added IRCodes.h extra header file.
// 20/07/17 : 5     Added linked list handler and Value/Alias read routine
// 22/07/17 : 6     Debugging stack dump on string parser. Linked list handler and Value/Alias read routine now bulletproof. Used visual studio to debug.
// 27/08/17 : 7     First integration of IR Device handling code, not compiled yet, need to remove circular buffer handlers and I/P handler 'checkInputs' no longer required.
// 27/08/17 : 8     Gave up on integrated state control for both commandBuffer and IR Device handlers and opted to have separate state machines and make simpler implementation instead.
// 28/08/17 : 9     Completed code for irRemoteCmdTopic, everything compiles and it now supports two separate state machines eSYSTEM_STATUS, eSYSTEM_EVENT, eIR_DEVICE_STATUS, eIR_DEVICE_EVENT
// 29/08/17 : 10    Added debug code for state machines. irRemoteCmdTopic now works. Needs tidying up. Now need to work on Part Raw and Raw.
// 29/08/17 : 11    Added debug code for state machines. irRemoteCmdPartRawTopic and irRemoteCmdRawTopic now work. Still needs tidying up.
// 03/09/17 : 12    irRemoteCmdTopic, irRemoteCmdPartRawTopic and irRemoteCmdRawTopic tested.
// 03/09/17 : 13    Added ambient light sensor. Tidied up. Now needs arse kicking out of the testing.
// 27/09/17 : 14    Version for automated testing. Added handler for Err No 14 : Compound number exceeds max digits. Kicked the arse out of the testing using Python test script engine
// 30/09/17 : 15    Added handler for Err No 15. Button repeats out of bounds 0 or > 255, and Err No. 16 Delay Between Button Repeats out of bounds 0 or > 65535. Impacts irRemoteCmdPartRawTopic 
// 21/10/17 : 16    Extended reset delay for IR remote device to 2 seconds and ensured all debug tracing can be removed without causing a crash. Increased delay in 'void doIRDeviceUpdate(void)' from 10mS to 20mS after call to Wire.endTransmission();
// 11/09/18 : 17    Fixed fileRead bool type issue.
// 17/04/21 : 18    Updated IRCodes.h for IR control of SkyQ box and added uint32_t fix for 32bit raw data
//
// Start up sequence
// -----------------
// Unit starts up in STA_AP mode and looks for SD file SECURITY_PARAMETERS_FILE. If the file doesn't exist the IoT device switches to AP mode, 
// starts a web server and initialises an mDNS service. The device then broadcasts the SSID AP_NETWORK_SSID_DEFAULT + DeviceMACAddress. To connect 
// to this APnetwork use the following password AP_NETWORK_PASSWORD_DEFAULT once connected enter the following URL into your browser nDNSHostName + '.local'
// The IoT device will then serve up a configuration web page allowing new sensor network security parameters to be submitted.
// If the SD file is present the information in this file is used by the IoT device to connect to the sensor network. The device then stays in STA_AP 
// mode until it has connected. Once connected to the sensor network the IoT device switches to STA mode.
// If the information in the SECURITY_PARAMETERS_FILE does not allow connection to the sensor network it is possible to connect to the sensors APnetwork as above,
// only now you must enter the IP address 192.168.4.1. This is due to a flaw in the mDNS library of the ESP8266. When in STA_AP mode mDNS service
// will not work.
// Once the device has connected to the sensor network it attempts to connect to the MQTT broker which it expects at the following address MQTT_BROKER_IP_DEFAULT
// and port MQTT_BROKER_PORT_DEFAULT. If the IoT device exceeds mqtt_broker_connection_attempts it will re-initialise as if no SECURITY_PARAMETERS_FILE were present.
// Once connected to the MQTT broker, if the connection to the broker is lost the system re-initialises.
// If mqtt_broker_connection_attempts=0 the IoT device will continue to attempt an MQTT Broker connection.
//
// To give a visual indication of the above connection states, the IoT device will flash the local led as follows.
// 1. When no onboard configuration file is present SECURITY_PARAMETERS_FILE 1 quick flash.
// 2. When attempting to connect to a given WiFi network 2 quick flashes in succession.
// 3. Once a WiFi n/w connection has been achieved. Whilst attempting to connect to an MQTT Broker the led will be on continuously.
// 4. Once WiFi n/w and MQTT Broker connections are in place the led will extinguish.
//
//
// Configuration files
// Here there are five text files named 'alivals.txt', 'calvals.txt', 'secvals.txt', and 'index.htm' + some other crap such that a little 'IoT' icon appears in the browser address bar.
// Copy these to the root directory of the system SD card.
//
// 'calvals.txt' contains seven entries. These values are exposed for read write via MQTT topics.
// - 1st Calibration zero offset for Temperature a float
// - 2nd Calibration scale factor for Temperature a float
// - 3rd Calibration zero offset for Humidity a float
// - 4th Calibration scale factor for Humidity a float
// - 5th Value is the reporting strategy value. 0 = Send and update when a change is detected in the monitored value. 1...60 = report back the monitored value every 'n' minutes
//
// 'alivals.txt' contains the pairing of an Alias for a given TV channel and the corresponding numerical remote control button press
// In the form 'Alias,Value'. Value must be numeric. Lines in this text file can also be commented. A comment is preceeded by a '#'. For example
// BBC1,101 # This is the remote control button sequence for a Sky HD Statellite box
// BBC2,102 # This is another remote control button sequence for a Sky HD Statellite box
//
// 'secvals.txt' contains six entries. These values are write only via MQTT topics.
// - 1st MQTT Broker IP Address. In dotted decimal form AAA.BBB.CCC.DDD
// - 2nd MQTT Broker Port. In Integer form.
// - 3rd MQTT Broker connection attempts to make before switching from STA mode to AP mode. In Integer form. 
// - 4th WiFi Network SSID. In free form text.
// - 5th WiFi Network Password. In free form text.
// - 6th WiFi Network connection attempts to make before switching from STA mode to AP mode. In Integer form. // NOTE this is not implemented
//
//
// 'index.htm'
// Contains web page served up when the device can't connect to the Network using the password held in the 'secvals.txt' file
//
// Arduino IDE programming parameters.
// 
// From Tools Menu
// Board: 'Generic ESP8266 Module'
// Flash Mode: 'DIO'
// Flash Size: '4M (3M SPIFFS)'
// Debug Port: 'Disabled'
// Debug Level: 'None'
// Reset Method: 'ck'
// Flash Frequency '40MHz'
// CPU Frequency '80 MHz'
// Upload Speed: '115200'
// 

   
//..#define DEBUG_GENERAL      // Undefine this for general debug information via the serial port. Note, you must deploy with this undefined as your network security parameters will be sent to serial port
//#define DEBUG_WEB          // Undefine this for comprehensive debug information on web interface via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_MDNS         // Undefine this for comprehensive debug information on mDNS support via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_TIMER        // Undefine this for timer debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_SD           // Undefine this for SD debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_IR_TIMER     // Undefine this for IR_TIMER debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_VALIDATION   // Undefine this for validation debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_STATE_CHANGE // Undefine this for 'eIRDEVICESTATE' State Change debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_LEDFLASH     // Undefine this for 'eLEDFLASHSTATE' State Change debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_SECVALS      // Undefine this for MQTT SecVals Topic debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_PARMGRAB     // Undefine this for parmGrab function debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_INPUTS       // Undefine this for debugging system input functionality. Requires DEBUG_GENERAL.  
//#define DEBUG_PARSER       // Undefine this for logging StringParser function. Requires DEBUG_GENERAL.  
//.#define DEBUG_ALVAL        // Undefine this for read Alias/Value function. Requires DEBUG_GENERAL.  
//.#define DEBUG_REM_COMMANDS // Undefine this for remote command debug. Requires DEBUG_GENERAL.  
//..#define DEBUG_IRD_COMMANDS // Undefine this to see the commands issued to the IR Device. Requires DEBUG_GENERAL.  
//..#define DEBUG_IRD_CALLBACK // Undefine this to debug the IR Device callback function. Requires DEBUG_GENERAL.  
//.#define DEBUG_IRDEV_TIMER  // Undefine this to get debug on the IR Device specific timers. Requires DEBUG_GENERAL.
//.#define DEBUG_SYSTEM_STATES_EVENTS // Undefine this to debug the doRemoteCommandUpdate function. Requires DEBUG_GENERAL.
//.#define DEBUG_IR_DEVICE_STATES_EVENTS // Undefine this to debug the doIRDeviceUpdate function. Requires DEBUG_GENERAL.

#include "IRCodes.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <DHT.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <BounceI2C.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <stdarg.h>
#include <malloc.h>
#include <ctype.h>
#include <BH1750.h>


#define TO_UPPER 0xDF
#define MAKE_UPPER_CASE(c)     ((c) & TO_UPPER)
#define SET_BIT(p,whichBit)    ((p) |=  (1    << (whichBit)))
#define CLEAR_BIT(p,whichBit)  ((p) &= ~((1)  << (whichBit)))
#define TOGGLE_BIT(p,whichBit) ((p) ^=  (1    << (whichBit)))
#define BIT_IS_SET(p,whichBit) ((p) &   (1    << (whichBit)))

#define SD_FILE_READ_MODE  FILE_READ
//#define SD_FILE_WRITE_MODE FILE_WRITE
#define SD_FILE_WRITE_MODE (O_WRITE | O_CREAT | O_TRUNC)


#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

// Define I2C pins on ESP8266-12E
#define I2C_SDA       4               // s/w defined I2C lines on the ESP8266-12E
#define I2C_SCL       5               // s/w defined I2C lines on the ESP8266-12E



#define CALIBRATION_PARAMETERS_FILE             ((char *)("/calvals.txt"))
#define SECURITY_PARAMETERS_FILE                ((char *)("/secvals.txt"))
#define ALIAS_VALUE_INPUT_FILE                  ((char *)("/alivals.txt"))
#define LOWER_REPORTING_STRATEGY_VALUE          0  // Report any change
#define UPPER_REPORTING_STRATEGY_VALUE          60 // Send updates every hour
#define TEMPERATURE_CALIBRATION_OFFSET_DEFAULT  ((float)0.0)
#define TEMPERATURE_CALIBRATION_SCALE_DEFAULT   ((float)1.0)
#define HUMIDITY_CALIBRATION_OFFSET_DEFAULT     ((float)0.0) 
#define HUMIDITY_CALIBRATION_SCALE_DEFAULT      ((float)1.0) 
#define REPORTING_STRATEGY_DEFAULT              LOWER_REPORTING_STRATEGY_VALUE
#define DEFAULT_SENSOR_INTERVAL                 10000 // Sensor read interval in Milliseconds



// I/O Defines
const int IRDeviceBusyPin  = 0;     // Reads busy state of IR Device
const int lightPin0        = 7;     // Controls system LED0 (System)
const int lightPin1        = 6;     // Controls system LED1 (IR Device Busy)
const int IRDeviceResetPin = 5;     // Used to Reset the IR Device

#define NXP_PCF8574P_ADDRESS 0x3E
#define NXP_PCF8574P_INPUT_MASK 0b00001111 // P7 = O/P System Led0, P6 = O/P System Led1, P5 = O/P Reset IR Device, P4, P3, P2, P1, P0 = I/P IR Device Busy
// create an instance of the bounce class
BounceI2C IRDeviceBusy = BounceI2C(NXP_PCF8574P_ADDRESS, NXP_PCF8574P_INPUT_MASK);
BounceI2C port = BounceI2C(NXP_PCF8574P_ADDRESS, NXP_PCF8574P_INPUT_MASK);



// For BH1750 Ambient light sensor
BH1750 ambientLightSensor;
unsigned long ambient_light_sensor_old = 0;                        // Previous ALS reading
unsigned long ambient_light_sensor_new = 0;                        // New ALS reading
unsigned long previousALSMillis        = 0;                        // previous time value, so routine is non blocking
unsigned long readIntervalALS          = DEFAULT_SENSOR_INTERVAL;  // interval at which to read sensor, in milli seconds (10secs)
boolean sendALSUpdate                  = false;  
unsigned long displayALSCurrentValue   = 0;
String strAmbientLightLevel("");



// For DHT22 Temperature Humidity Sensor
#define DHTPIN 2        // The digital o/p pin we're connected to. GPIO2
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

// Values read from Temperature/Humidity sensor
float temp_c_old;                 // Temperature in Centigrade, earlier reading
float temp_c_new;                 // Temperature in Centigrade, latest reading
float humidity_old;               // Relative humidity in %age, earlier reading
float humidity_new;               // Relative humidity in %age, latest reading
float hic_old;                    // Heat Index in Centigrade, earlier reading
float hic_new;                    // Heat Index in Centigrade, latest reading
unsigned long previousMillis = 0; // previous time value, so routine is non blocking
unsigned long readIntervalTempHumi = DEFAULT_SENSOR_INTERVAL;  // interval at which to read Temp/Humi sensor, in milli seconds
float tempCalOffset                = TEMPERATURE_CALIBRATION_OFFSET_DEFAULT;
float tempCalScale                 = TEMPERATURE_CALIBRATION_SCALE_DEFAULT;
float humCalOffset                 = HUMIDITY_CALIBRATION_OFFSET_DEFAULT;
float humCalScale                  = HUMIDITY_CALIBRATION_SCALE_DEFAULT;



// Temperature and Humidity readings
boolean newTemperatureDataFromWifi = true;
boolean newHumidityDataFromWifi = true;
char tmpTempStr[20+1] = "00.00";
char tmpTempStrOld[20+1] = "00.00";
char tmpHumidStr[20+1] = "00.00";
char tmpHumidStrOld[20+1] = "00.00";
//String tmpHICStr[20+1];
//String tmpHICStrOld[20+1];


int     reportingStrategy = REPORTING_STRATEGY_DEFAULT;
boolean sendTHUpdate      = false;                      // Used to trigger Temp/Humidity update
boolean bBrokerPresent    = false;


#define MQTT_VERSION MQTT_VERSION_3_1

#define MQTT_BROKER_IP_STRING_MAX_LEN           30
#define MQTT_BROKER_IP_DEFAULT                  "192.168.1.44"
#define MQTT_BROKER_PORT_DEFAULT                ((int)1883)
#define STA_NETWORK_SSID_DEFAULT                "IRREMSIM"
#define STA_NETWORK_PASSWORD_DEFAULT            "PASSWORD"
#define AP_NETWORK_SSID_DEFAULT                 "IRREMSIM"
#define AP_NETWORK_PASSWORD_DEFAULT             "PASSWORD"
#define NETWORK_CONNECTION_ATTEMPTS_DEFAULT     ((int)10)
#define MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT ((int)10)
#define NETWORK_SSID_STRING_MAX_LEN             32  
#define NETWORK_PASSWORD_STRING_MAX_LEN         40
#define CONNECTION_ATTEMPTS_MAX                 100
#define CONNECTION_ATTEMPTS_MIN                 0

char   mqtt_broker_ip[MQTT_BROKER_IP_STRING_MAX_LEN];
int    mqtt_broker_port;
String macStrForAPSSID;
char   sta_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
char   sta_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
char   ap_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
char   ap_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
int    network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
int    mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
#ifdef DEBUG_GENERAL
int    conDotCountNW   = 0; // Used to print a NW connecting dot each 500ms
int    conDotCountMQTT = 0; // Used to print a MQTT connecting dot each 500ms
#endif



// Topic to publish to, to request this device publish the status of its local software version (Generic Device, MAC Addr, Filename.ino). Caution : This a THSen Broadcast message.
const char* swVerTopic = "WFD/IRTHSen/SwVer/Command";

char swVerThisDeviceTopic[50];

// Topic to subscribe to, to receive publication of this device's software version. In form (Generic Device, MAC Addr, Filename.ino)
const char* swVerConfirmTopic = "WFD/IRTHSen/SwVer/Confirm";

// Topic to publish to, to control the status of this device's local led state
const char* lightTopic0 = "WFD/IRTHSen/1/Led/Command/1";

// Topic to publish to, to control the status of this device's local led state
const char* lightTopic1 = "WFD/IRTHSen/1/Led/Command/2";

// Topic to subscribe to, to receive confirmation that this device has recieved a Led control command
const char* lightConfirmTopic0 = "WFD/IRTHSen/1/Led/Confirm/1";

// Topic to subscribe to, to receive confirmation that this device has recieved a Led control command
const char* lightConfirmTopic1 = "WFD/IRTHSen/1/Led/Confirm/2";

// Topic to subscribe to, to receive publication of the status of this devices local Ambient Light Status
const char* ambientLightSensorTopic = "WFD/IRTHSen/1/ALSStatus/1";

// Topic to subscribe to, to receive publication of the status of this devices local temperature
const char* temperatureTopic = "WFD/IRTHSen/1/TempStatus/1";

// Topic to subscribe to, to receive publication of the status of this devices local humidity
const char* humidityTopic = "WFD/IRTHSen/1/HumdStatus/1";

// Topic to subscribe to, to receive publication of the status of this devices local heat index
const char* heatIndexTopic = "WFD/IRTHSen/1/HeatIndStatus/1";

// Topic to subscribe to, to request this device publish the status of its local RSSI for SSID
const char* rssiTopic = "WFD/IRTHSen/1/RSSILev";

// Topic to subscribe to, to receive publication of the status of this devices  local RSSI in dBm
const char* rssiConfirmTopic = "WFD/IRTHSen/1/RSSILev/Confirm";

// Topic to publish to, to request this device re-format it's local SD filing system. Response; 0 = Done, Error = 1
// Response is sent via 'sdConfirmTopic'
const char* sdInitTopic = "WFD/IRTHSen/1/SD/Init/Command";

// Topic to publish to, to request this device re-read all the values stored in it's local SD filing system (specifically CALIBRATION_PARAMETERS_FILE). Response; 0 = Done, Error = 1
// Response is sent via 'sdConfirmTopic'
const char* sdReadTopic = "WFD/IRTHSen/1/SD/Read/Command";

// Topic to publish to, to request this device store a new Temperature Calibration Zero Offset in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 3
// Temperature command parameter in units of degrees celcius is a float in range -x.x to y.y
// Response is sent via 'sdConfirmTopic'
const char* sdTemperatureZeroOffsetTopic = "WFD/IRTHSen/1/SD/CalVal/Temperature/ZeroOff/Command/1";

// Topic to publish to, to request this device store a new Temperature Calibration Scaling Factor in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 5
// Temperature command parameter, unitless multiplication factor is a float in range 1.0 to x.x
// Response is sent via 'sdConfirmTopic'
const char* sdTemperatureScalingFactorTopic = "WFD/IRTHSen/1/SD/CalVal/Temperature/ScaleFact/Command/1";

// Topic to publish to, to request this device store a new Humidity Calibration Zero Offset in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 2
// Humidity command parameter in units of percent is a float in range -x.x to y.y
// Response is sent via 'sdConfirmTopic'
const char* sdHumidityZeroOffsetTopic = "WFD/IRTHSen/1/SD/CalVal/Humidity/ZeroOff/Command/1";

// Topic to publish to, to request this device store a new Humidity Calibration Scaling Factor in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 4
// Humidity command parameter, unitless multiplication factor is a float in range 1.0 to x.x
// Response is sent via 'sdConfirmTopic'
const char* sdHumidityScalingFactorTopic = "WFD/IRTHSen/1/SD/CalVal/Humidity/ScaleFact/Command/1";

// Topic to publish to, to request this device to publish the value of a given entry in the CALIBRATION_PARAMETERS_FILE
// SD Send command parameter = 1..n, where n is the value in the CALIBRATION_PARAMETERS_FILE file to return. It effectively 
// represents a given line number and responds in freeform text 
// Response is sent via 'sdConfirmTopic'
// Temperature Zero Offset  = 1
// Temperature Scale Factor = 2
// Humidity Zero Offset     = 3
// Humidity Scale Factor    = 4
// Reporting Strategy       = 5
//
const char* sdSendTopic = "WFD/IRTHSen/1/SD/Send";

// Topic to publish to, to request this device store new Security Values in it's local SD filing system. 
// Responses; 
// 0  = Done, 
// 1  = Failed to open SECURITY_PARAMETERS_FILE for write, 
// 6  = MQTT Broker IP address malformed,
// 7  = MQTT Broker Port number invalid,
// 8  = Network SSID or Network Password Wrong length,
// 9  = MQTT Broker Connection Attempts number invalid,
// 10 = MQTT Broker Connection Attempts out of range,
// 11 = Network Connection Attempts number invalid,
// 12 = Network Connection Attempts out of range,
// 13 = One or more items in the parameter string is missing
// Parameter is in the following form 'BrokerIPAddress,BrokerPort,MQTTBrokerConnectionAttempts,NetworkSSID,NetworkPassword,NetworkConnectionAttempts'
// Where;
// BrokerIPAddress : AAA.BBB.CCC.DDD dotted decimal form
// BrokerPort : Integer form. Typically 1883 for Mosquitto MQTT Broker
// MQTTBrokerConnectionAttempts : CONNECTION_ATTEMPTS_MIN ... CONNECTION_ATTEMPTS_MAX. 0 is a special case meaning keep retrying
// NetworkSSID : Free form text
// NetworkPassword : Free form text
// NetworkConnectionAttempts : Integer form. can be any value as this field is not implemented.
// 
// Response is sent via 'sdConfirmTopic'
const char* sdNewSecValsTopic  = "WFD/IRTHSen/1/SD/SecVals";

// Topic to subscribe to, to receive publication of response that a given SD command has received and executed 
const char* sdConfirmTopic = "WFD/IRTHSen/1/SD/Conf";

// Topic to publish to, to request this device control the reporting strategy 
// Reporting Strategy command parameter 0..60, where n is the value in minutes to send a temperature/humidity update. The value 0 means send whenever there is a change. Response; 0 = Done, Error in range = 1
const char* reportingStrategyTopic = "WFD/IRTHSen/1/RepStrat";

// Topic to publish a pass_response that a given Reporting Strategy command has received and exe
// 1 = NOT IMPLEMENTED. Error Malformed Command (,which parameter), cuted
const char* reportingStrategyConfirmTopic = "WFD/IRTHSen/1/RepStrat/Conf";

// IR Command : Topic to publish to, to request this device store and execute a remote control command. 
// Responses;
// 0 = Done, 
// 1 = NOT IMPLEMENTED. Error Malformed Command (,which parameter), 
// 2 = Busy, 
// 3 = Wrong number of parameters, 
// 4 = Null Parameter string, "4,3" - 3 Parms, "4,4" - 4 Parms
// 5 = Numerical Device index out of bounds, 
// 6 = Textual Device not recognised, 
// 7 = Remote Control Button String not recognised in this topic payload, 
// 8 = Compound function name not recognised in this topic payload, 
// 9 = Unable to find Alias in this topic payload, 
// 10 = Command Buffer full/overflow, 
// 11 = Reset Timer Timeout. eSYSTEM_STATUS_CBBusy, IR Device reset time has now expired.
// 12 = Busy Timer Timeout. eSYSTEM_STATUS_CBBusy, IR Device has been busy for too long, probably locked up, reset it.
// 13 = Command Buffer Inactivity Timer Timeout. eSYSTEM_STATUS_CBBuffering
// 14 = Compound number exceeds max digits
//
// IR Remote command is in the form; 'Device,Function,[Parameter],FreshData'. 
// Note 
// 'Parameter' is optional and only used with compound commands. To date it is only used with the key word 'Chan' for channel
// 'FreshData' Allows up to MAX_IR_COMMANDS to be sent to the IR Device at one go. If FreshData = 0, IR commands are stored (stacked) in the IR Command buffer, When FreshData = 1
// all previously stored commands including this one are sent to the IR Device simulator.
// Parameter count = 3 or 4 (if compound parmeter is used)
//
// Device (note the device can also be a number 0 = Sky, 1 = SonySB, 2 = SonyTV)
// ------
// 'Sky', Satellite
// 'SonySB', Audio device
// 'SonyTV', Television
// 
// Function (these are the respective remote control button names)
// --------
// 'Chan', Compound command, this requires a Parameter value
// 'Mute'
// 'OnOff'
// 'Select'
// 'Backup'
// ...
// 
// Parameter (if this is non numerical then the text is matched with the contents of alivals.txt)
// ---------
// '1'
// '10'
// '321'
// 'Alibi'
// 'BBC'
// 'ITV'
//
// FreshData
// ---------
// 0...1, 0 = More data to come, 1 = Complete IR Data transfer.
//
// Example payloads, of single executable commands. 
// 'Sky,Chan,BBC,1'
// 'Sky,Chan,101,1'
// 'SonySB,Mute,1'
// Example payloads, of single executable commands which are stacked before execution. Turn TV on, Sky box on, Sound bar on, turn to channel 101 on Sky (BBC1)
// 'SonyTV,OnOff,0'
// 'Sky,OnOff,0'
// 'SonySB,OnOff,0'
// 'Sky,Chan,101,1'
// 
// Response is sent via 'iRConfirmTopic'
const char* irRemoteCmdTopic = "WFD/IRTHSen/1/Rem/Command/1";


// IR Command Part Raw : Topic to publish to, to request this device store and execute a remote control command. 
// Responses;
// 0 = Done, 
// 1 = NOT IMPLEMENTED. Error Malformed Command (,which parameter), 
// 2 = Busy, 
// 3 = Wrong number of parameters, 
// 4 = Null Parameter string, 
// 5 = Numerical Device index out of bounds, 
// 6 = Textual Device not recognised, 
// 7 = Remote Control Button String not recognised in this topic payload, 
// 10 = Command Buffer full/overflow, 
// 11 = Reset Timer Timeout. eSYSTEM_STATUS_CBBusy 
// 12 = Busy Timer Timeout. eSYSTEM_STATUS_CBBusy 
// 13 = Command Buffer Inactivity Timer Timeout. eSYSTEM_STATUS_CBBuffering
// 15 = Button repeats out of bounds 0 or > 255. 
// 16 = Delay Between Button Repeats out of bounds 0 or > 65535.
//
// IR Remote command is in the form; 'Device,Function,bButtonRepeats,ui16DelayBetweenButtonRepeats,FreshData'. 
// Parameter count = 5
//
// Device
// ------
// As above
// 
// Function (these are the respective remote control button names)
// --------
// 'Mute'
// 'OnOff'
// 'Select'
// 'Backup'
// ...
// 
// Parameter
// ---------
// As above
//
// bButtonRepeats
// --------------
// Byte value 1...255, representing the number of simulated button repeats required
//
// ui16DelayBetweenButtonRepeats
// -----------------------------
// Unsigned Int value 1...65535, representing the delay in mS between simulated button repeats
//
// FreshData
// ---------
// As above
//
// Example payloads, of single executable commands. 
// 'Sky,1,3,400,1' # Results in the simulation of button '1' on the Sky remote being pressed 3 times with 400mS between each press
// 'SonySB,SndUp,4,500,1' # Results in the simulation of button 'Sound Up' on the Sony Sound Bar remote being pressed 4 times with 500mS between each press
// Example payloads, of single executable commands which are stacked before execution. Select Input on Sony TV, move choice up twice and select this input source
// 'SonyTV,SelIP,1,400,0'
// 'SonyTV,Up,2,400,0'
// 'SonyTV,Sel,1,400,1'
//
// Response is sent via 'iRConfirmTopic'
const char* irRemoteCmdPartRawTopic = "WFD/IRTHSen/1/Rem/Command/PartRaw/1";


// IR Command Raw : Topic to publish to, to request this device store and execute a remote control command. 
// Responses;
// 0 = Done, 
// 1 = Error Malformed Command (,which parameter), 
// 2 = Busy, 
// 3 = Wrong number of parameters, 
// 4 = Null Parameter string, 
// 10 = Command Buffer full/overflow, 
// 11 = Reset Timer Timeout. eSYSTEM_STATUS_CBBusy 
// 12 = Busy Timer Timeout. eSYSTEM_STATUS_CBBusy 
// 13 = Command Buffer Inactivity Timer Timeout. eSYSTEM_STATUS_CBBuffering
//
// IR Remote command is in the form; 'bEncoding,ui32Data,bNumberOfBitsInTheData,bPulseTrainRepeats,bDelayBetweenPulseTrainRepeats,bButtonRepeats,ui16DelayBetweenButtonRepeats,bFreshData'. 
// Parameter count = 8
//
// bEncoding (can be a hex [0x0 ... 0x4] or decimal value)
// ---------
// Byte value 0...4, representing what IR encoding scheme is used. [RC6=0, SONY=1, SAMSUNG=2, NEC=3, LG=4]
//
// ui32Data (can be a hex [0x00000000 ... 0xFFFFFFFF] or decimal value)
// --------
// Unsigned 32bit Int, representing the data to be sent, as stream of 1s and 0s. 00000000 ... FFFFFFFF
//
// bNumberOfBitsInTheData (can be a hex [0x00 ... 0x20] or decimal value)
// ----------------------
// Byte value  1...32, representing which of the 32 bits of the 'ui32Data' value are to be considered as data to be sent to the IR Transmitter. Different encoding schemes require different lengths of bit streams.
//
// bPulseTrainRepeats (can be a hex [0x00 ... 0xff] or decimal value)
// ------------------
// Byte value 1...255, representing the number of times a given pulse train is repeated. This is typically different depending upon the button being pressed. It simulates the duration of the button press. ie. to turn the Sky box on it typically takes 3 repeats
//
// bDelayBetweenPulseTrainRepeats (can be a hex [0x00 ... 0xff] or decimal value)
// ------------------------------
// Byte value 1...255, representing the time in mS between each pulse train repeat.
//
// bButtonRepeats (can be a hex [0x00 ... 0xff] or decimal value)
// --------------
// Byte value 1...255, representing the number of simulated button repeats required
//
// ui16DelayBetweenButtonRepeats (can be a hex [0x0000 ... 0xffff] or decimal value)
// -----------------------------
// Unsigned Int value 1...65535, representing the delay in mS between simulated button repeats
//
// FreshData (can be a hex [0x0 ... 0x1] or decimal value)
// ---------
// 0...1, 0 = More data to come, 1 = Complete IR Data transfer.
//
//
// Example payloads, of single executable commands. 
// SonySB - Mute
// 1,0x140C,15,4,22,1,0,1
// Sky - Select
// 0,0xC05C5C,24,2,124,1,400,1
//
// Example payloads, of single executable commands which are stacked before execution. Unmute Sony Sound Bar (or Mute, it's a toggle), hit select on Sky remote and turn Sony TV on (or off, it's also a toggle)
// SonySB - Mute
// 1,0x140C,15,4,22,1,0,0
// Sky - Select
// 0,0xC05C5C,24,2,124,1,400,0
// SonyTV - OnOff
// 1,0xA90,12,4,26,1,400,1
//
// Response is sent via 'iRConfirmTopic'
const char* irRemoteCmdRawTopic = "WFD/IRTHSen/1/Rem/Command/Raw/1";


// Topic to subscribe to, to receive publication of response that a given IR command has received
const char* iRConfirmTopic = "WFD/IRTHSen/1/Rem/Conf";



// This line is here to cure the following compiler error;
//
extern  void callback(char* topic, byte* payload, unsigned int length);
//
//  XXXXXXXXX:35: error: 'callback' was not declared in this scope
//
//  PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);
//
// At the time of writing this code it was a known issue with Arduino IDE 1.6.8 and the 2.1.0 esp code this sketch fails to compile:
// TITLE : Function not declared in this scope #1881 
// URL   : https://github.com/esp8266/Arduino/issues/1881
//


const char* subscriptionsArray[] = {swVerThisDeviceTopic, swVerTopic, lightTopic0, lightTopic1, rssiTopic, sdInitTopic, sdReadTopic, sdHumidityZeroOffsetTopic, sdHumidityScalingFactorTopic, sdTemperatureZeroOffsetTopic, sdTemperatureScalingFactorTopic, sdSendTopic, reportingStrategyTopic, sdNewSecValsTopic, irRemoteCmdTopic, irRemoteCmdPartRawTopic, irRemoteCmdRawTopic};
int maxSubscriptions = 0;

#define WiFiConnected (WiFi.status() == WL_CONNECTED)
WiFiClient wifiClient;
PubSubClient MQTTclient(wifiClient);

String clientName;
const char* THIS_GENERIC_DEVICE = "esp8266";
String swVersion;


// Struct to hold a single timer instance
typedef struct tsTimerInstance {
  void (*tmrcallback)(void);   // Function called when timing period exceeded
  boolean bRunning;            // Flag, set with timer running
  unsigned long ulTimerPeriod; // Timing period in milliseconds
  unsigned long ulStartValue;  // Grab of value from millis() when timer was started, used to calculate elapsed time
} TimerInstance;

#define TOTAL_TIME_IN_MILLISECONDS(H, M, S) ((unsigned long)((((unsigned long)H)*60UL*60UL*1000UL)+(((unsigned long)M)*60UL*1000UL)+(((unsigned long)S)*1000UL)))
#define MAX_TIMERS                      5
#define PERIODIC_UPDATE_TIMER           0
#define LED_FLASH_TIMER                 1
#define IR_DEVICE_RESET_TIMER           2
#define IR_DEVICE_BUSY_TIMER            3
#define COMMAND_BUFFER_INACTIVITY_TIMER 4

TimerInstance stiTimerArray[MAX_TIMERS];  // Array for holding all the active timer instances


#define FILE_VAR_INSTANCE_TYPE_STRING 0
#define FILE_VAR_INSTANCE_TYPE_FLOAT  1
#define FILE_VAR_INSTANCE_TYPE_INT    2
#define FILE_VAR_INSTANCE_TYPE_BOOL   3

typedef struct tsFileVarInstance {
  int iVarType;  
  void *ptrVar;
} FileVarInstance;


FileVarInstance SecurityVarArray[] = 
{
  {FILE_VAR_INSTANCE_TYPE_STRING, (void *)mqtt_broker_ip                  },
  {FILE_VAR_INSTANCE_TYPE_INT,    (void *)&mqtt_broker_port               },
  {FILE_VAR_INSTANCE_TYPE_INT,    (void *)&mqtt_broker_connection_attempts},
  {FILE_VAR_INSTANCE_TYPE_STRING, (void *)sta_network_ssid                },
  {FILE_VAR_INSTANCE_TYPE_STRING, (void *)sta_network_password            },
  {FILE_VAR_INSTANCE_TYPE_INT,    (void *)&network_connection_attempts    }
};



FileVarInstance CalibrationVarArray[] = 
{
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&tempCalOffset     },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&tempCalScale      },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&humCalOffset      },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&humCalScale       },
  {FILE_VAR_INSTANCE_TYPE_INT,   (void *)&reportingStrategy }
};



typedef enum {
   eIRDEVICESTATE_INIT         = 0,
   eIRDEVICESTATE_NO_CONFIG    = 1,
   eIRDEVICESTATE_PENDING_NW   = 2,
   eIRDEVICESTATE_PENDING_MQTT = 3,
   eIRDEVICESTATE_ACTIVE       = 4
} eIRDEVICESTATE;

eIRDEVICESTATE eIRDEVICESTATE_STATE = eIRDEVICESTATE_INIT;

const char* nDNSHostName = "IRREMSIM";

ESP8266WebServer server(80);
static bool hasSD = false;
//IPAddress APIPAddress (192,168,1,1);
//IPAddress APNWMask (255,255,255,0);
IPAddress tmpAPIPAddress;


#define LED_FLASH_PERIOD   500 // Time of flash period in mS
#define FLASH_SEQUENCE_MAX 11  // The maximum number of definable states a flash sequence can have

typedef enum {
   eLEDFLASH_NO_CONFIG    = 0,
   eLEDFLASH_PENDING_NW   = 1,
   eLEDFLASH_PENDING_MQTT = 2,
   eLEDFLASH_OFF          = 3,
   eLEDFLASH_SEQUENCE_END = 4
} eLEDFLASHSTATE;

eLEDFLASHSTATE eLEDFLASHSTATE_STATE = eLEDFLASH_OFF;
int iFlashSequenceIndex = 0;

char cFlashProfiles[][FLASH_SEQUENCE_MAX] = {
  "1000000000",  // No Config
  "1010000000",  // Pending NW
  "1111111111",  // Pending MQTT
  "0000000000",  // Off
};

#ifdef DEBUG_STATE_CHANGE
char printStateChangeBuf[200];
const char *SensorStates[]= {
  "eIRDEVICESTATE_INIT",
  "eIRDEVICESTATE_NO_CONFIG",
  "eIRDEVICESTATE_PENDING_NW",
  "eIRDEVICESTATE_PENDING_MQTT",
  "eIRDEVICESTATE_ACTIVE"
};

char *printStateChange(eIRDEVICESTATE ThisState, eIRDEVICESTATE NextState, const char *InThisFunction)
{
    printStateChangeBuf[0] = 0x00;
    sprintf(printStateChangeBuf,"State Change %s => %s : Within '%s'",SensorStates[ThisState],SensorStates[NextState],InThisFunction);
    return printStateChangeBuf;
}

#define SHOW_UPDATED_STATE(t,n,f) Serial.println(printStateChange((t),(n),(f)))
#endif

#ifdef DEBUG_LEDFLASH
char printLedStateChangeBuf[200];
const char *LedStates[]= {
  "eLEDFLASH_NO_CONFIG",
  "eLEDFLASH_PENDING_NW",
  "eLEDFLASH_PENDING_MQTT",
  "eLEDFLASH_OFF",
  "eLEDFLASH_SEQUENCE_END"
};

char *printLedStateChange(eLEDFLASHSTATE ThisState, eLEDFLASHSTATE NextState, const char *InThisFunction)
{
    printLedStateChangeBuf[0] = 0x00;
    sprintf(printLedStateChangeBuf,"LED State Change %s => %s : Within '%s'",LedStates[ThisState],LedStates[NextState],InThisFunction);
    return printLedStateChangeBuf;
}

#define SHOW_UPDATED_LED_STATE(t,n,f) Serial.println(printLedStateChange((t),(n),(f)))
#endif

  
// Alias/Value Instances Linked List Handler
#define MAX_ALIAS_STRING            (30 + 1)
#define MAX_VALUE_STRING            (30 + 1)
#define ALIAS_VALUE_SEPARATOR       ((char *)",")
#define PAYLOAD_SEPARATOR           ((char *)",")
#define ALIAS_VALUE_COMMENT_CHAR    ((char)'#')
#define PARAMETER_SEPARATOR         ((char)',')
#define MAX_PARM_STRING_LENGTH      100
#define COMPOUND_COMMAND_IDENTIFIER ((char *)"Chan")
#define MAX_BUTTON_PRESSES          20 // Same as the button press buffer, MAX_SEQUENCES in IR_Remote_Sim4.ino
#define MAX_DECODED_BUTTONS         5  // This buffer is used to temporarily store a single decoded topic payload
#define REMOTE_BUTTON_DIGITS        3  // This is the maximum number of numerical digits which can be entered in one go. ie for sky 000 ... 999

typedef struct  {
  uint16_t ui16ButtonIndex;
  bool bFreshData;
} sButtonPressType;

typedef struct AliasValueInstance {
    char   strAlias[MAX_ALIAS_STRING];
    char   strValue[MAX_VALUE_STRING];
    struct AliasValueInstance *next;
    struct AliasValueInstance *previous;
} sAliasValueInstance;

sAliasValueInstance *ptrHeadOfAliasValueInstances = NULL;
sAliasValueInstance *tmpAliasValueInstancePtr     = NULL;
boolean bValueAliasPresent = false;


typedef enum {
   eSYSTEM_STATUS_CBEmpty =     0,
   eSYSTEM_STATUS_CBBuffering = 1,
   eSYSTEM_STATUS_CBBusy =      2
} eSYSTEM_STATUS_TYPE;

volatile eSYSTEM_STATUS_TYPE eSYSTEM_STATUS     = eSYSTEM_STATUS_CBEmpty;
volatile uint8_t ui8CommandBufferHeadPointer = 0;


#define COMMAND_BUFFER_INACTIVITY_PERIOD 11000 // The maximum period allowed between feeding new button presses and not signalling end of sequence with bFrshData flag, in mS

typedef enum {
   eSYSTEM_EVENT_None                     = 0,
   eSYSTEM_EVENT_Got_Data_No_FD_Flag      = 1,
   eSYSTEM_EVENT_Got_Data_And_FD_Flag     = 2,
   eSYSTEM_EVENT_Command_Buffer_Overflow  = 3,
   eSYSTEM_EVENT_Command_Buffer_Timeout   = 4,
   eSYSTEM_EVENT_IR_Device_Done_Sending   = 5,
   eSYSTEM_EVENT_IR_Device_Reset_Timeout  = 6
} eSYSTEM_EVENT_TYPE;

volatile eSYSTEM_EVENT_TYPE eSYSTEM_EVENT = eSYSTEM_EVENT_None;


typedef enum {
   eIR_DEVICE_STATUS_Idle         = 0,
   eIR_DEVICE_STATUS_TXing        = 1,
   eIR_DEVICE_STATUS_Transferring = 2,
   eIR_DEVICE_STATUS_Resetting    = 3
} eIR_DEVICE_STATUS_TYPE;

volatile eIR_DEVICE_STATUS_TYPE eIR_DEVICE_STATUS = eIR_DEVICE_STATUS_Idle;

#define IR_DEVICE_RESET_PERIOD           2000   // Time allowed for IR Device to reset in mS
#define IR_DEVICE_BUSY_PERIOD            20000 // Maximum period IR Device is allowed to be busy before a crash is supected, in mS

typedef enum {
   eIR_DEVICE_EVENT_None                   = 0,
   eIR_DEVICE_EVENT_Send_Command           = 1,
   eIR_DEVICE_EVENT_Last_Command_Sent      = 2,
   eIR_DEVICE_EVENT_Busy_Timeout           = 3,
   eIR_DEVICE_EVENT_Reset_Timeout          = 4,
   eIR_DEVICE_EVENT_IR_Device_Done_Sending = 5
} eIR_DEVICE_EVENT_TYPE;

volatile eIR_DEVICE_EVENT_TYPE eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_None;
volatile uint8_t ui8CommandBufferPointer = 0;


typedef struct __attribute((__packed__)) {
  byte bEncoding;
  uint32_t ui32Data;
  byte bNumberOfBitsInTheData;
  byte bPulseTrainRepeats;
  byte bDelayBetweenPulseTrainRepeats;
  byte bButtonRepeats;
  uint16_t ui16DelayBetweenButtonRepeats;
  byte bFreshData;
} registerAllocationType;

typedef struct __attribute((__packed__)) {  
  unsigned char ucDataArray[sizeof(registerAllocationType)];
} dataArrayType;  

typedef union {
  dataArrayType da;
  registerAllocationType ra;
} dataArrayRegisterAllocationUnionType;

#define MAX_BUFFER sizeof(registerAllocationType)


dataArrayRegisterAllocationUnionType commandBuffer[MAX_BUTTON_PRESSES];

#define SLAVE_ADDR 9  // Slave address of IR Device


#ifdef DEBUG_SYSTEM_STATES_EVENTS
char printSystemStatesEventsBuf[200];
const char *SystemStates[]= {
   "eSYSTEM_STATUS_CBEmpty",
   "eSYSTEM_STATUS_CBBuffering",
   "eSYSTEM_STATUS_CBBusy"
};

const char *SystemEvents[]= {
   "eSYSTEM_EVENT_None",
   "eSYSTEM_EVENT_Got_Data_No_FD_Flag",
   "eSYSTEM_EVENT_Got_Data_And_FD_Flag",
   "eSYSTEM_EVENT_Command_Buffer_Overflow",
   "eSYSTEM_EVENT_Command_Buffer_Timeout",
   "eSYSTEM_EVENT_IR_Device_Done_Sending",
   "eSYSTEM_EVENT_IR_Device_Reset_Timeout"
};

char *printSystemStateEvent(eSYSTEM_STATUS_TYPE ThisState, eSYSTEM_EVENT_TYPE ThisEvent, const char *InThisFunction)
{
    printSystemStatesEventsBuf[0] = 0x00;
    sprintf(printSystemStatesEventsBuf,"State %s/Event %s : Within '%s'",SystemStates[ThisState],SystemEvents[ThisEvent],InThisFunction);
    return printSystemStatesEventsBuf;
}

#define SHOW_SYSTEM_STATE_EVENT(s,e,f) Serial.println(printSystemStateEvent((s),(e),(f)))
//#else
//#define SHOW_SYSTEM_STATE_EVENT(s,e,f) 
#endif


#ifdef DEBUG_IR_DEVICE_STATES_EVENTS
char printIRDeviceStatesEventsBuf[200];
const char *IRDeviceStates[]= {
   "eIR_DEVICE_STATUS_Idle",
   "eIR_DEVICE_STATUS_TXing",
   "eIR_DEVICE_STATUS_Transferring",
   "eIR_DEVICE_STATUS_Resetting"   
};

const char *IRDeviceEvents[]= {
   "eIR_DEVICE_EVENT_None",
   "eIR_DEVICE_EVENT_Send_Command",
   "eIR_DEVICE_EVENT_Last_Command_Sent",
   "eIR_DEVICE_EVENT_Busy_Timeout",
   "eIR_DEVICE_EVENT_Reset_Timeout",
   "eIR_DEVICE_EVENT_IR_Device_Done_Sending"
};

char *printIRDeviceStateEvent(eIR_DEVICE_STATUS_TYPE ThisState, eIR_DEVICE_EVENT_TYPE ThisEvent, const char *InThisFunction)
{
    printIRDeviceStatesEventsBuf[0] = 0x00;
    sprintf(printIRDeviceStatesEventsBuf,"State %s/Event %s : Within '%s'",IRDeviceStates[ThisState],IRDeviceEvents[ThisEvent],InThisFunction);
    return printIRDeviceStatesEventsBuf;
}

#define SHOW_IR_DEVICE_STATE_EVENT(s,e,f) Serial.println(printIRDeviceStateEvent((s),(e),(f)))
//#else
//#define SHOW_IR_DEVICE_STATE_EVENT(s,e,f)  
#endif



//##############################################
//###                                        ###
//###          Function Declarations         ###
//###                                        ###
//##############################################
void doRemoteCommandUpdate(void);
void doIRDeviceUpdate(void);
void checkTemperatureAndHumidity(void);
void checkALSValue(void);
void grabParm(char **ptrToParmString, String *recipientString);
int fileWrite(File f, FileVarInstance *fviArray, int iTotalParametersToWrite);
int fileRead(File f, FileVarInstance *fviArray, int iTotalParametersToRead);
void readCalibrationValues();
void readNetworkSecurityParameters();
void connectMQTT();
String macToStr(const uint8_t* mac, boolean addColons);
void timer_create(int iTimerNumber, unsigned long ulTimerPeriod, void (*callbackfn)(void));
void timer_update(void);
void timer_start(int iTimerNumber);
void timer_stop(int iTimerNumber);
void timer_reset(int iTimerNumber);
boolean timer_isRunning(int iTimerNumber);
void timer_change_period(int iTimerNumber, unsigned long ulTimerPeriod);
void ledFlashTimerCallback(void);
void periodicUpdateTimerCallback(void);
void irDeviceResetTimerCallback(void);
void irDeviceBusyTimerCallback(void);
void commandBufferInactivityTimerCallback(void);
void returnOK(String mess);
void returnFail(String msg);
bool loadFromSD(String path);
void handleNetworkConfig();
void handleNotFound();
boolean isFloat(String tString);
bool isHex(char *str);
boolean isValidNumber(String str);
bool isValidIpv4Address(char *st);
char *fmtDouble(double val, byte precision, char *buf, unsigned bufLen);
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen, byte width);
int extractAliasValue(char *strAliasValue, char *strAlias, char *strValue);
int parseString(char *delimiter, char *source, ...);
void stripComments(char *strInput);
void stripSpaces(char *strInput);
bool isNumeric(const char *str);
int parmCount(char *strParameterString);
char *stristr(const char *haystack, const char *needle);
int findRemote(remoteControlType *remoteControls, int iMaxRemotes, char *cRemoteName);
int findRemoteButton(remoteControlButtonType *remoteControlButtonCollection, uint16_t iMaxRemoteButtons, char *cButtonName);
int findAliasValue(sAliasValueInstance *ptrHeadOfAliasValueInstancesX, char *cAliasName, char **cValueName);
void readAliasValueInput(void);
sAliasValueInstance * addItem(sAliasValueInstance **Head, const char *sAlias, const char *sValue);
boolean aliasValueListEnd(sAliasValueInstance *Head, sAliasValueInstance *Tail);
boolean aliasValueListBegining(sAliasValueInstance *Head, sAliasValueInstance *Tail);
sAliasValueInstance * aliasValueListGetNext(sAliasValueInstance **tmpPtr);
sAliasValueInstance * aliasValueListGetPrevious(sAliasValueInstance **tmpPtr);




void setup() {
  // Start the serial line for debugging
  // This is enabled or the TX/RX port will require 10K pull ups to stop oscillations of the I/Ps which makes the ESP8266-01 pull more current and can crash
  //#ifdef DEBUG_GENERAL
  Serial.begin(115200);
  delay(100);
  //#endif

  // Set up I2C
  Wire.begin(I2C_SDA,I2C_SCL);  

  // Initialise BH1750FVI I2C Ambient Light Intensity Sensor
  ambient_light_sensor_old  = 0;                                          
  ambient_light_sensor_new  = 0;                                
  previousALSMillis         = 0;                      
  readIntervalALS           = DEFAULT_SENSOR_INTERVAL;
  displayALSCurrentValue    = 0;
  strAmbientLightLevel      = "";
  ambientLightSensor.begin();

  // Initialise DHT sensor
  dht.begin();
  humidity_old         = 0.0;
  humidity_new         = 0.0;
  temp_c_old           = 0.0;  
  temp_c_new           = 0.0;  
  hic_old              = 0.0;     
  hic_new              = 0.0; 
  humCalOffset         = HUMIDITY_CALIBRATION_OFFSET_DEFAULT;
  humCalScale          = HUMIDITY_CALIBRATION_SCALE_DEFAULT;
  tempCalOffset        = TEMPERATURE_CALIBRATION_OFFSET_DEFAULT;
  tempCalScale         = TEMPERATURE_CALIBRATION_SCALE_DEFAULT;
  reportingStrategy    = REPORTING_STRATEGY_DEFAULT;
  readIntervalTempHumi = DEFAULT_SENSOR_INTERVAL;

  // Initialise input signals
  IRDeviceBusy.attach(IRDeviceBusyPin);
  IRDeviceBusy.interval(50);
  IRDeviceBusy.update(); // Read IR Device Busy status and check for HIGH or LOW 

  // Initialize the system LEDs as an output and set to HIGH (off), and IR Device Reset line inactive
  port.digitalWrite(lightPin0, HIGH);  
  port.digitalWrite(lightPin1, HIGH);  
  port.digitalWrite(IRDeviceResetPin, LOW);  
  
  // Generate client name based on MAC address
  clientName = THIS_GENERIC_DEVICE;
  clientName += '-';
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac,true);
  macStrForAPSSID = macToStr(mac,false);
  macStrForAPSSID.trim();
  sprintf(swVerThisDeviceTopic,"WFD/%s/SwVer/Command",macToStr(mac, true).c_str());

  swVersion = THIS_GENERIC_DEVICE;
  swVersion += ',';
  swVersion += macToStr(mac,true);
  swVersion += ',';
  swVersion += __FILENAME__;
  #ifdef DEBUG_GENERAL  
  Serial.print("Client Name : ");
  Serial.println(clientName);
  Serial.print("SW Version : ");
  Serial.println(swVersion);
  #endif

  // Set up default security parameters. If all else fails so this device can become an AP.
  strcpy(ap_network_ssid,AP_NETWORK_SSID_DEFAULT);
  strcat(ap_network_ssid,macStrForAPSSID.c_str());
  strcpy(ap_network_password,AP_NETWORK_PASSWORD_DEFAULT);
  strcpy(mqtt_broker_ip, MQTT_BROKER_IP_DEFAULT);
  mqtt_broker_port = MQTT_BROKER_PORT_DEFAULT;
  mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
  strcpy(sta_network_ssid, STA_NETWORK_SSID_DEFAULT);
  strcpy(sta_network_password, STA_NETWORK_PASSWORD_DEFAULT);
  network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
  bBrokerPresent = true;

  // Set up MQTT auto topic subscription
  maxSubscriptions = sizeof(subscriptionsArray)/sizeof(const char*);  

  if (SD.begin(SS))
  {
    #ifdef DEBUG_GENERAL
    Serial.println("SD initialisation completed ok");
    #endif
  } else {
    #ifdef DEBUG_GENERAL
    Serial.println("SD initialisation failed! Card not responding");
    #endif
  }  
  delay(2000);
  
  // Try to read the calibration values file. If missing this will set defaults
  readCalibrationValues();
  // Try to read the security paramaters file. If missing this will set the ssid and p/w for the AP
  readNetworkSecurityParameters();

  // Read in Alias Value Input
  ptrHeadOfAliasValueInstances = NULL;
  tmpAliasValueInstancePtr     = NULL;
  bValueAliasPresent           = false;
  readAliasValueInput();

  // Set up initial conditions
  eIRDEVICESTATE_STATE = eIRDEVICESTATE_INIT;
  WiFi.mode(WIFI_OFF);

  // Set up IR Device buffers
  eSYSTEM_STATUS     = eSYSTEM_STATUS_CBEmpty;
  eSYSTEM_EVENT      = eSYSTEM_EVENT_None;
  ui8CommandBufferHeadPointer = 0;

  eIR_DEVICE_STATUS = eIR_DEVICE_STATUS_Idle;
  eIR_DEVICE_EVENT  = eIR_DEVICE_EVENT_None;
  ui8CommandBufferPointer = 0;

  // Set up timers
  sendTHUpdate = false;
  sendALSUpdate = false;
  timer_create(PERIODIC_UPDATE_TIMER, TOTAL_TIME_IN_MILLISECONDS(0, reportingStrategy, 0), periodicUpdateTimerCallback);   
  if (reportingStrategy>0)
    timer_start(PERIODIC_UPDATE_TIMER);

  iFlashSequenceIndex = 0;
  eLEDFLASHSTATE_STATE = eLEDFLASH_OFF;
  timer_create(LED_FLASH_TIMER, LED_FLASH_PERIOD, ledFlashTimerCallback);   
  //timer_create(LED_FLASH_TIMER, TOTAL_TIME_IN_MILLISECONDS(0, LED_FLASH_PERIOD, 0), ledFlashTimerCallback);   
  timer_start(LED_FLASH_TIMER);

  timer_create(IR_DEVICE_RESET_TIMER,           IR_DEVICE_RESET_PERIOD,           irDeviceResetTimerCallback);   
  timer_create(IR_DEVICE_BUSY_TIMER,            IR_DEVICE_BUSY_PERIOD,            irDeviceBusyTimerCallback);   
  timer_create(COMMAND_BUFFER_INACTIVITY_TIMER, COMMAND_BUFFER_INACTIVITY_PERIOD, commandBufferInactivityTimerCallback);   
 
  delay(2000);
}



void loop(){
  timer_update();                // Update timers
  doRemoteCommandUpdate();       // Check for completed IR commands
  doIRDeviceUpdate();            // Process any IR Device actions
  checkALSValue();               // Monitor Ambient Light Levels
  checkTemperatureAndHumidity(); // Read Temp and Humidity sensor
  //MDNS.update();                 // Check for any mDNS queries and send responses

  switch (eIRDEVICESTATE_STATE) {
    case eIRDEVICESTATE_INIT : //
           WiFi.mode(WIFI_OFF);
           yield();
           //delay(1000);
           if ((SD.exists(SECURITY_PARAMETERS_FILE)) && (bBrokerPresent)) {
              eIRDEVICESTATE_STATE = eIRDEVICESTATE_PENDING_NW;
              eLEDFLASHSTATE_STATE = eLEDFLASH_PENDING_NW;
              #ifdef DEBUG_STATE_CHANGE
              SHOW_UPDATED_STATE(eIRDEVICESTATE_INIT,eIRDEVICESTATE_PENDING_NW,"loop");
              #endif
              WiFi.mode(WIFI_AP_STA);
              delay(1000);
              // Read the security paramaters file. 
              readNetworkSecurityParameters();
              // Start STA wifi subsystem
              WiFi.begin((const char *)sta_network_ssid, (const char *)sta_network_password);
              #ifdef DEBUG_GENERAL
              Serial.println("Switching to AP_STA Mode. SecVals Found");
              Serial.print("Connecting to "); Serial.println(sta_network_ssid);
              conDotCountNW = 0;  
              #endif
           } else {
              #ifdef DEBUG_STATE_CHANGE
              SHOW_UPDATED_STATE(eIRDEVICESTATE_INIT,eIRDEVICESTATE_NO_CONFIG,"loop");
              #endif
              eIRDEVICESTATE_STATE = eIRDEVICESTATE_NO_CONFIG;
              eLEDFLASHSTATE_STATE = eLEDFLASH_NO_CONFIG;
              WiFi.mode(WIFI_AP);
              delay(1000);
              #ifdef DEBUG_GENERAL
              if (bBrokerPresent)
                Serial.println("Switching to AP Mode. No SecVals found");
              else
                Serial.println("Switching to AP Mode. No MQTT Broker found");
              #endif
           }

           // Start AP wifi subsystem
           WiFi.encryptionType(ENC_TYPE_WEP);
           //WiFi.softAPConfig(APIPAddress,APIPAddress,APNWMask);
           WiFi.softAP((const char *)ap_network_ssid, (const char *)ap_network_password);
          
           // Late binding for MQTT client
           MQTTclient.setServer((const char *)mqtt_broker_ip, mqtt_broker_port); 
           MQTTclient.setCallback(callback);
           hasSD = true;

           tmpAPIPAddress = WiFi.softAPIP();
           #ifdef DEBUG_GENERAL
           Serial.print("AP IP address: "); Serial.println(tmpAPIPAddress);
           #endif    
          
           //if (MDNS.begin(nDNSHostName, APIPAddress)) {
           if (MDNS.begin(nDNSHostName)) {
             MDNS.addService("http", "tcp", 80);
             #ifdef DEBUG_MDNS
             Serial.println("MDNS responder started");
             Serial.print("You can now connect to http://");
             Serial.print(nDNSHostName);
             Serial.println(".local");
             #endif
           } else {
             #ifdef DEBUG_MDNS
             Serial.println("MDNS responder failed to start");
             #endif
           }

           // Set up HTTP server
           server.on("/0", HTTP_GET, handleNetworkConfig);
           server.onNotFound(handleNotFound);
           server.begin();
           #ifdef DEBUG_WEB
           Serial.println("HTTP server started");
           #endif
           break;

    case eIRDEVICESTATE_NO_CONFIG  : // Run only as an access point to allow the user to reconfigure to new network
           server.handleClient();
           yield();
           //delay(10); 
           break;

    case eIRDEVICESTATE_PENDING_NW : // Run as an access point to allow the user to reconfigure to new network and as a station trying to connnect to NW
           server.handleClient();
           yield();
           //delay(10); 
           if (WiFiConnected) {
              // Start wifi subsystem
              //WiFi.mode(WIFI_STA);  // Switch off access point
              //#ifdef DEBUG_GENERAL
              //Serial.println();
              //Serial.println("Switching to STA Mode. Now WiFi is connected.");
              //#endif
              //WiFi.begin((const char *)sta_network_ssid, (const char *)sta_network_password);
              eIRDEVICESTATE_STATE = eIRDEVICESTATE_PENDING_MQTT;
              eLEDFLASHSTATE_STATE = eLEDFLASH_PENDING_MQTT;

              #ifdef DEBUG_STATE_CHANGE
              SHOW_UPDATED_STATE(eIRDEVICESTATE_PENDING_NW,eIRDEVICESTATE_PENDING_MQTT,"loop");
              #endif
              
              //print out some more debug once connected
              #ifdef DEBUG_GENERAL
              Serial.println("WiFi connected");  
              Serial.print("IP address: ");
              Serial.println(WiFi.localIP());
              #endif
           } else {
              #ifdef DEBUG_GENERAL
              if (conDotCountNW > 50)
                  conDotCountNW = 0;
              if (conDotCountNW == 0)
                Serial.print(".");
              conDotCountNW++;  
              #endif
           }
           break;
    
    case eIRDEVICESTATE_PENDING_MQTT : // Try to connect to MQTT Broker
           readCalibrationValues();
           connectMQTT();
           break;
    
    case eIRDEVICESTATE_ACTIVE : // Run as a WiFi client in active mode
           // Reconnect if connection is lost
           if (!MQTTclient.connected()) {
            #ifdef DEBUG_STATE_CHANGE
            Serial.println();
            Serial.println("Switching to AP_STA Mode. As MQTT has disconnected.");
            #endif
            WiFi.mode(WIFI_AP_STA);
            delay(1000);
            WiFi.encryptionType(ENC_TYPE_WEP);
            //WiFi.softAPConfig(APIPAddress,APIPAddress,APNWMask);
            WiFi.softAP((const char *)ap_network_ssid, (const char *)ap_network_password);
            
            tmpAPIPAddress = WiFi.softAPIP();
            #ifdef DEBUG_GENERAL
            Serial.print("AP IP address: "); Serial.println(tmpAPIPAddress);
            #endif    
            connectMQTT();
           } else //maintain MQTT connection
            MQTTclient.loop();
        
           // Delay to allow ESP8266 WIFI functions to run
           yield();
           //delay(10); 
           break;
  }
}



void doRemoteCommandUpdate(void)
{
  // This is called in each main loop pass. 

  /*
  // Handle asynchronous event.
  IRDeviceBusy.update(); // Poll busy line from IR Device
  #ifdef DEBUG_INPUTS
  Serial.println("Inputs");
  #endif  
  if(IRDeviceBusy.read() == HIGH){
    #ifdef DEBUG_INPUTS
    Serial.println("IR Dev Idle");
    #endif
    port.digitalWrite(lightPin1, HIGH);
  } else {
    #ifdef DEBUG_INPUTS
    Serial.println("IR Dev Busy");
    #endif
    port.digitalWrite(lightPin1, LOW);
  }

  if((IRDeviceBusy.read() == LOW) && (IRDeviceBusy.rose()))
  {
    eSYSTEM_EVENT = eSYSTEM_EVENT_IR_Device_Done_Sending;
  }
  */
  
  if (eSYSTEM_EVENT != eSYSTEM_EVENT_None)
  {
    #ifdef DEBUG_REM_COMMANDS
    Serial.println("In doRemoteCommandUpdate");
    #endif
    #ifdef DEBUG_SYSTEM_STATES_EVENTS
    SHOW_SYSTEM_STATE_EVENT(eSYSTEM_STATUS,eSYSTEM_EVENT,"In doRemoteCommandUpdate");
    #endif
    #ifdef DEBUG_IR_DEVICE_STATES_EVENTS
    SHOW_IR_DEVICE_STATE_EVENT(eIR_DEVICE_STATUS,eIR_DEVICE_EVENT,"In doRemoteCommandUpdate");
    #endif
    switch (eSYSTEM_EVENT) {
      case eSYSTEM_EVENT_None : 
             // Should never get here
             break;
      case eSYSTEM_EVENT_Got_Data_No_FD_Flag :  // Handelled in callback for speed of response
            switch (eSYSTEM_STATUS) {
                case eSYSTEM_STATUS_CBEmpty : 
                       break;
                case eSYSTEM_STATUS_CBBuffering : 
                       break;
                case eSYSTEM_STATUS_CBBusy : 
                       break;
            }
            break;
      case eSYSTEM_EVENT_Got_Data_And_FD_Flag : 
            switch (eSYSTEM_STATUS) {
                case eSYSTEM_STATUS_CBEmpty : 
                       eSYSTEM_STATUS = eSYSTEM_STATUS_CBBusy;
                       // Start transfer and send
                       ui8CommandBufferPointer = 0;
                       eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_Send_Command;
                       //timer_start(IR_DEVICE_BUSY_TIMER);
                       break;
                case eSYSTEM_STATUS_CBBuffering : 
                       timer_stop(COMMAND_BUFFER_INACTIVITY_TIMER);
                       eSYSTEM_STATUS = eSYSTEM_STATUS_CBBusy;
                       // Start transfer and send
                       ui8CommandBufferPointer = 0;
                       eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_Send_Command;
                       //timer_start(IR_DEVICE_BUSY_TIMER);
                       break;
                case eSYSTEM_STATUS_CBBusy : 
                       break;
            }
            break;
      case eSYSTEM_EVENT_Command_Buffer_Overflow : // Handelled in callback for speed of response
            switch (eSYSTEM_STATUS) {
                case eSYSTEM_STATUS_CBEmpty : 
                       break;
                case eSYSTEM_STATUS_CBBuffering : 
                       break;
                case eSYSTEM_STATUS_CBBusy : 
                       break;
            }
            break;
      case eSYSTEM_EVENT_Command_Buffer_Timeout : 
            switch (eSYSTEM_STATUS) {
                case eSYSTEM_STATUS_CBEmpty : 
                       break;
                case eSYSTEM_STATUS_CBBuffering : 
                       if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
                         MQTTclient.publish(iRConfirmTopic, "13"); // 13 = Command Buffer Inactivity Timer Timeout. eSYSTEM_STATUS_CBBuffering
                       ui8CommandBufferHeadPointer = 0; 
                       eSYSTEM_STATUS = eSYSTEM_STATUS_CBEmpty;
                       break;
                case eSYSTEM_STATUS_CBBusy : 
                       break;
            }
            break;
      case eSYSTEM_EVENT_IR_Device_Done_Sending : 
            switch (eSYSTEM_STATUS) {
                case eSYSTEM_STATUS_CBEmpty : 
                       break;
                case eSYSTEM_STATUS_CBBuffering : 
                       break;
                case eSYSTEM_STATUS_CBBusy : 
                       if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
                         MQTTclient.publish(iRConfirmTopic, "0"); // 0 = Done
                       ui8CommandBufferHeadPointer = 0; 
                       eSYSTEM_STATUS = eSYSTEM_STATUS_CBEmpty;
                       break;
            }
            break;
      case eSYSTEM_EVENT_IR_Device_Reset_Timeout : 
            switch (eSYSTEM_STATUS) {
                case eSYSTEM_STATUS_CBEmpty : 
                       break;
                case eSYSTEM_STATUS_CBBuffering : 
                       break;
                case eSYSTEM_STATUS_CBBusy : 
                       ui8CommandBufferHeadPointer = 0; 
                       eSYSTEM_STATUS = eSYSTEM_STATUS_CBEmpty;
                       break;
            }
            break;
    }
    eSYSTEM_EVENT = eSYSTEM_EVENT_None;
    #ifdef DEBUG_SYSTEM_STATES_EVENTS
    SHOW_SYSTEM_STATE_EVENT(eSYSTEM_STATUS,eSYSTEM_EVENT,"Out doRemoteCommandUpdate");  
    #endif
    #ifdef DEBUG_IR_DEVICE_STATES_EVENTS
    SHOW_IR_DEVICE_STATE_EVENT(eIR_DEVICE_STATUS,eIR_DEVICE_EVENT,"Out doRemoteCommandUpdate");
    #endif
  }
}



void doIRDeviceUpdate(void)
{
  // This is called in each main loop pass. 

  // Handle asynchronous event.
  IRDeviceBusy.update(); // Poll busy line from IR Device
  #ifdef DEBUG_INPUTS
  Serial.println("Inputs");
  #endif  
  if(IRDeviceBusy.read() == HIGH){
    #ifdef DEBUG_INPUTS
    Serial.println("IR Dev Idle");
    #endif
    port.digitalWrite(lightPin1, HIGH);
  } else {
    #ifdef DEBUG_INPUTS
    Serial.println("IR Dev Busy");
    #endif
    port.digitalWrite(lightPin1, LOW);
  }

  if((IRDeviceBusy.read() == HIGH) && (IRDeviceBusy.rose()))
  {
    #ifdef DEBUG_REM_COMMANDS
    Serial.println("In doIRDeviceUpdate - Caught rising edge");
    #endif
    eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_IR_Device_Done_Sending;
  }
  
  if (eIR_DEVICE_EVENT != eIR_DEVICE_EVENT_None)
  {
    #ifdef DEBUG_REM_COMMANDS
    Serial.println("In doIRDeviceUpdate");
    #endif
    #ifdef DEBUG_SYSTEM_STATES_EVENTS
    SHOW_SYSTEM_STATE_EVENT(eSYSTEM_STATUS,eSYSTEM_EVENT,"In doIRDeviceUpdate");
    #endif
    #ifdef DEBUG_IR_DEVICE_STATES_EVENTS
    SHOW_IR_DEVICE_STATE_EVENT(eIR_DEVICE_STATUS,eIR_DEVICE_EVENT,"In doIRDeviceUpdate");
    #endif
    switch (eIR_DEVICE_EVENT) {
      case eIR_DEVICE_EVENT_None : 
             // Should never get here
             break;
      case eIR_DEVICE_EVENT_Send_Command : 
            switch (eIR_DEVICE_STATUS) {
                case eIR_DEVICE_STATUS_Idle : 
                        Wire.beginTransmission(SLAVE_ADDR); 
                        Wire.write(commandBuffer[ui8CommandBufferPointer].da.ucDataArray,MAX_BUFFER);
                        Wire.endTransmission();
                        delay(20);
                        #ifdef DEBUG_IRD_COMMANDS
                          Serial.print("ui8CommandBufferPointer          : "); Serial.println(ui8CommandBufferPointer);
                          Serial.print("  bEncoding                      : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bEncoding);
                          Serial.print("  ui32Data                       : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.ui32Data);
                          Serial.print("  bNumberOfBitsInTheData         : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bNumberOfBitsInTheData);
                          Serial.print("  bPulseTrainRepeats             : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bPulseTrainRepeats);
                          Serial.print("  bDelayBetweenPulseTrainRepeats : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bDelayBetweenPulseTrainRepeats);
                          Serial.print("  bButtonRepeats                 : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bButtonRepeats);
                          Serial.print("  ui16DelayBetweenButtonRepeats  : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.ui16DelayBetweenButtonRepeats);
                          Serial.print("  bFreshData                     : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bFreshData?"True":"False");
                        #endif
                        ui8CommandBufferPointer++;
                        eIR_DEVICE_STATUS = eIR_DEVICE_STATUS_Transferring;
                       break;
                case eIR_DEVICE_STATUS_TXing : 
                       break;
                case eIR_DEVICE_STATUS_Transferring : 
                        Wire.beginTransmission(SLAVE_ADDR); 
                        Wire.write(commandBuffer[ui8CommandBufferPointer].da.ucDataArray,MAX_BUFFER);
                        Wire.endTransmission();
                        delay(20);
                        #ifdef DEBUG_IRD_COMMANDS
                          Serial.print("ui8CommandBufferPointer          : "); Serial.println(ui8CommandBufferPointer);
                          Serial.print("  bEncoding                      : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bEncoding);
                          Serial.print("  ui32Data                       : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.ui32Data);
                          Serial.print("  bNumberOfBitsInTheData         : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bNumberOfBitsInTheData);
                          Serial.print("  bPulseTrainRepeats             : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bPulseTrainRepeats);
                          Serial.print("  bDelayBetweenPulseTrainRepeats : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bDelayBetweenPulseTrainRepeats);
                          Serial.print("  bButtonRepeats                 : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bButtonRepeats);
                          Serial.print("  ui16DelayBetweenButtonRepeats  : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.ui16DelayBetweenButtonRepeats);
                          Serial.print("  bFreshData                     : "); Serial.println(commandBuffer[ui8CommandBufferPointer].ra.bFreshData?"True":"False");
                        #endif
                        ui8CommandBufferPointer++;
                       break;
                case eIR_DEVICE_STATUS_Resetting : 
                       break;
            }
            if (ui8CommandBufferPointer >= ui8CommandBufferHeadPointer)
              eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_Last_Command_Sent;
            else
              eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_Send_Command;
            break;
      case eIR_DEVICE_EVENT_Last_Command_Sent : 
            switch (eIR_DEVICE_STATUS) {
                case eIR_DEVICE_STATUS_Idle : 
                       break;
                case eIR_DEVICE_STATUS_TXing : 
                       break;
                case eIR_DEVICE_STATUS_Transferring : 
                       timer_start(IR_DEVICE_BUSY_TIMER);
                       eIR_DEVICE_STATUS = eIR_DEVICE_STATUS_TXing;
                       break;
                case eIR_DEVICE_STATUS_Resetting : 
                       break;
            }
            eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_None;
            break;
      case eIR_DEVICE_EVENT_Busy_Timeout : 
            switch (eIR_DEVICE_STATUS) {
                case eIR_DEVICE_STATUS_Idle : 
                       break;
                case eIR_DEVICE_STATUS_TXing : 
                       port.digitalWrite(IRDeviceResetPin, HIGH); 
                       if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
                         MQTTclient.publish(iRConfirmTopic, "12"); // Busy Timer Timeout
                       //ui8CommandBufferHeadPointer = 0; 
                       timer_start(IR_DEVICE_RESET_TIMER); 
                       eIR_DEVICE_STATUS = eIR_DEVICE_STATUS_Resetting;
                       delay(100);
                       port.digitalWrite(IRDeviceResetPin, LOW); 
                       break;
                case eIR_DEVICE_STATUS_Transferring : 
                       break;
                case eIR_DEVICE_STATUS_Resetting : 
                       break;
            }
            eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_None;
            break;
      case eIR_DEVICE_EVENT_Reset_Timeout : 
            switch (eIR_DEVICE_STATUS) {
                case eIR_DEVICE_STATUS_Idle : 
                       break;
                case eIR_DEVICE_STATUS_TXing : 
                       break;
                case eIR_DEVICE_STATUS_Transferring : 
                       break;
                case eIR_DEVICE_STATUS_Resetting : 
                       if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
                         MQTTclient.publish(iRConfirmTopic, "11"); // Reset Timer Timeout
                       ui8CommandBufferHeadPointer = 0; 
                       eSYSTEM_EVENT = eSYSTEM_EVENT_IR_Device_Reset_Timeout;
                       eIR_DEVICE_STATUS = eIR_DEVICE_STATUS_Idle;
                       break;
            }
            eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_None;
            break;
      case eIR_DEVICE_EVENT_IR_Device_Done_Sending : 
            switch (eIR_DEVICE_STATUS) {
                case eIR_DEVICE_STATUS_Idle : 
                       break;
                case eIR_DEVICE_STATUS_TXing : 
                       timer_stop(IR_DEVICE_BUSY_TIMER); 
                       ui8CommandBufferHeadPointer = 0; 
                       ui8CommandBufferPointer = 0;
                       eSYSTEM_EVENT = eSYSTEM_EVENT_IR_Device_Done_Sending;
                       eIR_DEVICE_STATUS = eIR_DEVICE_STATUS_Idle;
                       break;
                case eIR_DEVICE_STATUS_Transferring : 
                       break;
                case eIR_DEVICE_STATUS_Resetting : 
                       break;
            }
            eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_None;
            break;
    }
    #ifdef DEBUG_SYSTEM_STATES_EVENTS
    SHOW_SYSTEM_STATE_EVENT(eSYSTEM_STATUS,eSYSTEM_EVENT,"Out doIRDeviceUpdate");
    #endif
    #ifdef DEBUG_IR_DEVICE_STATES_EVENTS
    SHOW_IR_DEVICE_STATE_EVENT(eIR_DEVICE_STATUS,eIR_DEVICE_EVENT,"Out doIRDeviceUpdate");
    #endif
  }
}



void checkTemperatureAndHumidity(void)
{
  String s1, s2, s3;

  // Wait at least 10 seconds between measurements.
  // if the difference between the current time and last time you read
  // the sensor is bigger than the interval you set, read the sensor
  // Works better than delay for things happening elsewhere also
  unsigned long currentMillis = millis();
   
  if (reportingStrategy == 0) {
    if(currentMillis - previousMillis >= readIntervalTempHumi) {
      // save the last time you read the sensor 
      previousMillis = currentMillis;   
    
      // Reading temperature for humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
      humidity_new = dht.readHumidity();          // Read humidity (percent)
      humidity_new *= humCalScale;
      humidity_new += humCalOffset;
      temp_c_new = dht.readTemperature();     // Read temperature as Centigrade
      temp_c_new *= tempCalScale;
      temp_c_new += tempCalOffset;
      hic_new = dht.computeHeatIndex(temp_c_new, humidity_new, false);   // Compute heat index in Celsius 
      // Check if any reads failed and exit early (to try again).

      if (isnan(humidity_new) || isnan(temp_c_new)) {
        #ifdef DEBUG_GENERAL
        Serial.println("Failed to read from DHT sensor!");
        #endif
        return;
      }
     
      #ifdef DEBUG_GENERAL
      if ((temp_c_new != temp_c_old)     ||
          (humidity_new != humidity_old) ||
          (hic_new != hic_old)) {
        Serial.println("Rep Strat ==0");
        Serial.print("Humidity: ");
        Serial.print(humidity_new);
        Serial.print(" %\t");
        Serial.print("Temperature: ");
        Serial.print(temp_c_new);
        Serial.print(" *C\t");
        Serial.print("Heat index: ");
        Serial.print(hic_new);
        Serial.println(" *C");
      } else {
        Serial.println("Rep Strat ==0");
        Serial.println("No change to T H or HI");
      }
      #endif

      if (temp_c_new != temp_c_old)
      {
        s1 = String(temp_c_new);
        if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
          MQTTclient.publish(temperatureTopic, s1.c_str());  
        temp_c_old = temp_c_new;
      }
  
      if (humidity_new != humidity_old)
      {
        s1 = String(humidity_new);
        if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
          MQTTclient.publish(humidityTopic, s1.c_str());        
        humidity_old = humidity_new;
      }
  
      if (hic_new != hic_old)
      {
        s1 = String(hic_new);
        if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
          MQTTclient.publish(heatIndexTopic, s1.c_str());        
        hic_old = hic_new;
      }
    }
  } else  {
    if (sendTHUpdate == true) {
      #ifdef DEBUG_GENERAL
      Serial.println("Rep Strat <> 0");
      #endif
      
      sendTHUpdate = false;
      humidity_new = dht.readHumidity();          // Read humidity (percent)
      temp_c_new = dht.readTemperature();     // Read temperature as Centigrade
      if (isnan(humidity_new) || isnan(temp_c_new)) {
        #ifdef DEBUG_GENERAL
        Serial.println("Failed to read from DHT sensor !");
        #endif
      }
      
      if (!isnan(temp_c_new)) {
        temp_c_new *= tempCalScale;
        temp_c_new += tempCalOffset;
        if (temp_c_new != temp_c_old)
        {
          s1 = String(temp_c_new);
          if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
            MQTTclient.publish(temperatureTopic, s1.c_str());        
          temp_c_old = temp_c_new;
        }
      } else {
        s1 = String(temp_c_old);
        if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
          MQTTclient.publish(temperatureTopic, s1.c_str());        
      }
      
      if (!isnan(humidity_new)) {
        humidity_new *= humCalScale;
        humidity_new += humCalOffset;
        if (humidity_new != humidity_old)
        {
          s1 = String(humidity_new);
          if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
            MQTTclient.publish(humidityTopic, s1.c_str());        
          humidity_old = humidity_new;
        }
      } else {
        s1 = String(humidity_old);
        if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
          MQTTclient.publish(humidityTopic, s1.c_str());        
      }
      
      if ((!isnan(humidity_new)) && (!isnan(temp_c_new))) {
        hic_new = dht.computeHeatIndex(temp_c_new, humidity_new, false);   // Compute heat index in Celsius 
        if (hic_new != hic_old)
        {
          s1 = String(hic_new);
          if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
            MQTTclient.publish(heatIndexTopic, s1.c_str());        
          hic_old = hic_new;
        }
      } else {
          s1 = String(hic_old);
          if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
            MQTTclient.publish(heatIndexTopic, s1.c_str());        
      }
      #ifdef DEBUG_GENERAL    
      Serial.print("Humidity: ");
      Serial.print(humidity_new);
      Serial.print(" %\t");
      Serial.print("Temperature: ");
      Serial.print(temp_c_new);
      Serial.print(" *C\t");
      Serial.print("Heat index: ");
      Serial.print(hic_new);
      Serial.println(" *C");
      #endif
    }
  }
}



void checkALSValue(void)
{
  unsigned long currentMillis = millis();  
  #ifdef DEBUG_BACKLIGHT
  //Serial.println("In checkALSValue()");
  #endif
  
  displayALSCurrentValue = (unsigned long) ambientLightSensor.readLightLevel();
  strAmbientLightLevel  = String(displayALSCurrentValue);
  ambient_light_sensor_new = displayALSCurrentValue;
  
  if (reportingStrategy == 0) {   
    if(currentMillis - previousALSMillis >= readIntervalALS) {
      // save the last time you read the sensor 
      previousALSMillis = currentMillis;   

      #ifdef DEBUG_GENERAL
      if (ambient_light_sensor_new != ambient_light_sensor_old) {
        Serial.println("ALS Rep Strat ==0");
        Serial.print(ambient_light_sensor_new);
        Serial.println("Lux");
        } else {
          Serial.println("ALS Rep Strat ==0");
          Serial.println("No change to ALS");
        }      
       #endif

      if (ambient_light_sensor_new != ambient_light_sensor_old)
      {
        if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
          MQTTclient.publish(ambientLightSensorTopic, strAmbientLightLevel.c_str());        
        ambient_light_sensor_old = ambient_light_sensor_new;
      }
    }
  } else {
    if (sendALSUpdate == true) {
      sendALSUpdate = false;
      #ifdef DEBUG_GENERAL
      Serial.println("ALS Rep Strat <> 0");
      #endif

      if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
        MQTTclient.publish(ambientLightSensorTopic, strAmbientLightLevel.c_str());        
          
      #ifdef DEBUG_GENERAL
      Serial.print("ALS : ");
      Serial.print(ambient_light_sensor_new);
      Serial.println(" Lux");
      #endif
    }
  }
  ambient_light_sensor_old = ambient_light_sensor_new;
}



void callback(char* topic, byte* payload, unsigned int length) {
  int Hour, Min;
  int Day, Month, Year;

  //convert topic to string to make it easier to work with
  String topicStr = topic; 
  char tmpCharBuf[length+1];
  int tmpInt = 0;

  for (int i=0;i<length;i++) 
    tmpCharBuf[i] = ((char)payload[i]);
  tmpCharBuf[length] = 0x00;
  
  //Print out some debugging info
  #ifdef DEBUG_GENERAL  
  Serial.println("Callback update.");
  Serial.print("Topic: ");
  Serial.println(topicStr);
  Serial.print("Payload: ");
  Serial.println(tmpCharBuf);
  #endif

  //turn light0 off if the payload is '0' and publish to the MQTT server a confirmation message
  if (strcmp(lightTopic0,topic)== 0) {
    if(payload[0] == '1'){ //turn the light on if the payload is '1' and publish the confirmation 
      port.digitalWrite(lightPin0, LOW);
      MQTTclient.publish(lightConfirmTopic0, "On");
    } else if (payload[0] == '0'){ //turn the light off if the payload is '0' and publish the confirmation
      port.digitalWrite(lightPin0, HIGH);
      MQTTclient.publish(lightConfirmTopic0, "Off");
    } else {
      MQTTclient.publish(lightConfirmTopic0, "Err");
    }
    return;
  }

  //turn light1 off if the payload is '0' and publish to the MQTT server a confirmation message
  if (strcmp(lightTopic1,topic)== 0) {
    if(payload[0] == '1'){ //turn the light on if the payload is '1' and publish the confirmation 
      port.digitalWrite(lightPin1, LOW);
      MQTTclient.publish(lightConfirmTopic1, "On");
    } else if (payload[0] == '0'){ //turn the light off if the payload is '0' and publish the confirmation
      port.digitalWrite(lightPin1, HIGH);
      MQTTclient.publish(lightConfirmTopic1, "Off");
    } else {
      MQTTclient.publish(lightConfirmTopic1, "Err");
    }
    return;
  }

  if (strcmp(swVerTopic,topic)== 0) {
    MQTTclient.publish(swVerConfirmTopic, swVersion.c_str());
    return;
  }  

  if (strcmp(swVerThisDeviceTopic,topic)== 0) {
    MQTTclient.publish(swVerConfirmTopic, swVersion.c_str());
    return;
  }  

  // handle RSSI topic, send MQTT confirmation via rssiConfirmTopic  
  if (strcmp(rssiTopic,topic)== 0) {
    int32_t rssi = WiFi.RSSI();
    sprintf(tmpCharBuf,"%ld",rssi);
    MQTTclient.publish(rssiConfirmTopic, tmpCharBuf);
    return;
  }    

/*
  // SD Handlers
  // Re-initialise the filing system
  if (strcmp(sdInitTopic,topic)== 0) {
    if (SD.format())
      MQTTclient.publish(sdConfirmTopic, "0");
    else
      MQTTclient.publish(sdConfirmTopic, "1");
    return;
  }  
*/
  // Re-read all stored calibration values
  if (strcmp(sdReadTopic,topic)== 0) {
    String s;
    // open file for readting
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_READ_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      fileRead(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  

  // Write Humidity calibration value to cal file and update local Humidity Cal offset
  if (strcmp(sdHumidityZeroOffsetTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpHumidityCalVal = tmpCharBuf;
    if (!isFloat(tmpHumidityCalVal))
    {
      MQTTclient.publish(sdConfirmTopic, "2");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      humCalOffset = tmpHumidityCalVal.toFloat();
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  

  // Write Humidity calibration value to cal file and update local Humidity Cal scaling factor
  if (strcmp(sdHumidityScalingFactorTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpHumidityCalVal = tmpCharBuf;
    if (!isFloat(tmpHumidityCalVal))
    {
      MQTTclient.publish(sdConfirmTopic, "4");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      humCalScale = tmpHumidityCalVal.toFloat();
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  
    
  // Write Temperature calibration value to cal file and update local Temperature Cal offset
  if (strcmp(sdTemperatureZeroOffsetTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpTemperatureCalVal = tmpCharBuf;
    if (!isFloat(tmpTemperatureCalVal))
    {
      MQTTclient.publish(sdConfirmTopic, "3");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      tempCalOffset = tmpTemperatureCalVal.toFloat();
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  

  // Write Temperature calibration value to cal file and update local Temperature Cal scaling factor
  if (strcmp(sdTemperatureScalingFactorTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpTemperatureCalVal = tmpCharBuf;
    if (!isFloat(tmpTemperatureCalVal))
    {
      MQTTclient.publish(sdConfirmTopic, "5");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      tempCalScale = tmpTemperatureCalVal.toFloat();
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  

  // Write Reporting Strategy value to cal file and update local Reporting Strategy variable
  if (strcmp(reportingStrategyTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpReportingStrategyVal = tmpCharBuf;
    #ifdef DEBUG_SD
    Serial.println(tmpReportingStrategyVal);
    #endif
    int tmpVal = tmpReportingStrategyVal.toInt();
    if ((tmpVal < LOWER_REPORTING_STRATEGY_VALUE) || (tmpVal > UPPER_REPORTING_STRATEGY_VALUE))
    {
      MQTTclient.publish(reportingStrategyConfirmTopic, "2");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(reportingStrategyConfirmTopic, "1");
      return;
    } else {
      reportingStrategy=tmpVal;
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      if (reportingStrategy == 0)
        timer_stop(PERIODIC_UPDATE_TIMER);
      else {
        timer_change_period(PERIODIC_UPDATE_TIMER, TOTAL_TIME_IN_MILLISECONDS(0, reportingStrategy, 0));  
        timer_start(PERIODIC_UPDATE_TIMER);                      
      }
      sendTHUpdate = false;
      f.close();
      MQTTclient.publish(reportingStrategyConfirmTopic, "0");
      return;
    }
  }

  // Query a stored calibration value and publish this value
  if (strcmp(sdSendTopic,topic)== 0) {
    String s;
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_READ_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "Err no file");
      return;
    } else {
      //int x = os_sscanf(tmpCharBuf, "%d", &tmpInt);
      tmpInt = String(tmpCharBuf).toInt();
      int iMaxCalParms = (int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance));
      for (int i = 0; i <= iMaxCalParms; i++){
        s=f.readStringUntil('\n');
        s.trim();
        if (i == (tmpInt-1)) {
          MQTTclient.publish(sdConfirmTopic, s.c_str());
          return;
        }
        if (f.position()>= f.size()) break;
      }
      f.close();
      MQTTclient.publish(sdConfirmTopic, "Err Parm Not Found");
      return;
    }
  }   

  // Write new network security values to file and restart IoT device
  if (strcmp(sdNewSecValsTopic,topic)== 0) {
    char  *StrPtr = tmpCharBuf;
    char   tmp_mqtt_broker_ip[MQTT_BROKER_IP_STRING_MAX_LEN];
    int    tmp_mqtt_broker_port;
    int    tmp_mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
    char   tmp_sta_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
    char   tmp_sta_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
    int    tmp_network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
    String strMQTTBrokerIPAddress;
    String strMQTTBrokerPort;
    String strMQTTBrokerConnectionAttempts;
    String strNetworkSSID;
    String strNetworkPassword;
    String strNetworkConnectionAttempts;

    grabParm(&StrPtr,&strMQTTBrokerIPAddress);
    grabParm(&StrPtr,&strMQTTBrokerPort);
    grabParm(&StrPtr,&strMQTTBrokerConnectionAttempts);
    grabParm(&StrPtr,&strNetworkSSID);
    grabParm(&StrPtr,&strNetworkPassword);
    grabParm(&StrPtr,&strNetworkConnectionAttempts);

/*    
    //sscanf(tmpCharBuf,"%s,%d,%d,%s,%s,%d",tmp_mqtt_broker_ip,&tmp_mqtt_broker_port,&tmp_mqtt_broker_connection_attempts,tmp_sta_network_ssid,tmp_sta_network_password,&tmp_network_connection_attempts);
    os_sprintf(tmpCharBuf,"%s,%d,%d,%s,%s,%d",tmp_mqtt_broker_ip,&tmp_mqtt_broker_port,&tmp_mqtt_broker_connection_attempts,tmp_sta_network_ssid,tmp_sta_network_password,&tmp_network_connection_attempts);
    strMQTTBrokerIPAddress = tmp_mqtt_broker_ip;
    strMQTTBrokerPort = tmp_mqtt_broker_port;
    strMQTTBrokerConnectionAttempts = tmp_mqtt_broker_connection_attempts;
    strNetworkSSID = tmp_sta_network_ssid;
    strNetworkPassword = tmp_sta_network_password;
    strNetworkConnectionAttempts = tmp_network_connection_attempts;
*/    
    #ifdef DEBUG_SECVALS
    Serial.print("SecValsMQTTBrokerIPAddress : "); Serial.println(strMQTTBrokerIPAddress);
    Serial.print("SecValsMQTTBrokerPort : "); Serial.println(strMQTTBrokerPort);
    Serial.print("SecValsMQTTBrokerConnectionAttempts : "); Serial.println(strMQTTBrokerConnectionAttempts);
    Serial.print("SecValsSTANetworkSSID : "); Serial.println(strNetworkSSID);
    Serial.print("SecValsSTANetworkPassword : "); Serial.println(strNetworkPassword);
    Serial.print("SecValsNetworkConnectionAttempts : "); Serial.println(strNetworkConnectionAttempts);
    #endif

    strMQTTBrokerIPAddress.trim();
    strMQTTBrokerPort.trim();
    strMQTTBrokerConnectionAttempts.trim();
    strNetworkSSID.trim();
    strNetworkPassword.trim();
    strNetworkConnectionAttempts.trim();

    if ((strMQTTBrokerIPAddress.length()          == 0) || 
        (strMQTTBrokerPort.length()               == 0) || 
        (strMQTTBrokerConnectionAttempts.length() == 0) || 
        (strNetworkSSID.length()                  == 0) || 
        (strNetworkPassword.length()              == 0) || 
        (strNetworkConnectionAttempts.length()    == 0)) {
      MQTTclient.publish(sdConfirmTopic, "13");
      return;
    }
    
    strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
    if (! isValidIpv4Address((char *)strMQTTBrokerIPAddress.c_str())) {
        MQTTclient.publish(sdConfirmTopic, "6");
        return;
    } else {
      //strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
      if (! isValidNumber(strMQTTBrokerPort)) {
        MQTTclient.publish(sdConfirmTopic, "7");
        return;
      } else {
        tmp_mqtt_broker_port = strMQTTBrokerPort.toInt();
        if (((strNetworkSSID.length() == 0)     || (strNetworkSSID.length() >= NETWORK_SSID_STRING_MAX_LEN)) || 
            ((strNetworkPassword.length() == 0) || (strNetworkPassword.length() >= NETWORK_PASSWORD_STRING_MAX_LEN))) {
            MQTTclient.publish(sdConfirmTopic, "8");
            return;
        } else {
          strcpy(tmp_sta_network_ssid,strNetworkSSID.c_str());
          strcpy(tmp_sta_network_password,strNetworkPassword.c_str());
  
          if (! isValidNumber(strMQTTBrokerConnectionAttempts)) {
            MQTTclient.publish(sdConfirmTopic, "9");
            return;
          } else {
            tmp_mqtt_broker_connection_attempts = strMQTTBrokerConnectionAttempts.toInt();
            if ((tmp_mqtt_broker_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_mqtt_broker_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
              MQTTclient.publish(sdConfirmTopic, "10");
              return;
            } else {
              if (! isValidNumber(strNetworkConnectionAttempts)) {
                MQTTclient.publish(sdConfirmTopic, "11");
                return;
              } else {
                tmp_network_connection_attempts = strNetworkConnectionAttempts.toInt();
                if ((tmp_network_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_network_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
                  MQTTclient.publish(sdConfirmTopic, "12");
                  return;
                } else {
                  strcpy(mqtt_broker_ip,tmp_mqtt_broker_ip);
                  mqtt_broker_port = tmp_mqtt_broker_port;
                  mqtt_broker_connection_attempts = tmp_mqtt_broker_connection_attempts;
                  strcpy(sta_network_ssid,tmp_sta_network_ssid);
                  strcpy(sta_network_password,tmp_sta_network_password);
                  network_connection_attempts = tmp_network_connection_attempts;
                  // Save new network parameters
                  File f = SD.open(SECURITY_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
                  if (!f) {
                    MQTTclient.publish(sdConfirmTopic, "1");
                    return;
                  } else {
                    fileWrite(f, SecurityVarArray,(int)(sizeof(SecurityVarArray)/sizeof(tsFileVarInstance)));
                    f.close();
                    MQTTclient.publish(sdConfirmTopic, "0");
                    bBrokerPresent = true;
                    #ifdef DEBUG_STATE_CHANGE
                    SHOW_UPDATED_STATE(eIRDEVICESTATE_STATE,eIRDEVICESTATE_INIT,"callback, sdNewSecValsTopic");
                    #endif
                    eIRDEVICESTATE_STATE = eIRDEVICESTATE_INIT;
                    #ifdef DEBUG_SECVALS
                    Serial.print("SecValsMQTTBrokerIPAddress : "); Serial.println(mqtt_broker_ip);
                    Serial.print("SecValsMQTTBrokerPort : "); Serial.println(mqtt_broker_port);
                    Serial.print("SecValsMQTTBrokerConnectionAttempts : "); Serial.println(mqtt_broker_connection_attempts);
                    Serial.print("SecValsSTANetworkSSID : "); Serial.println(sta_network_ssid);
                    Serial.print("SecValsSTANetworkPassword : "); Serial.println(sta_network_password);
                    Serial.print("SecValsNetworkConnectionAttempts : "); Serial.println(network_connection_attempts);
                    #endif
                    return;
                  }      
                }
              }
            }
          }
        }
      }
    }
  }  

  if (strcmp(irRemoteCmdTopic,topic)== 0) {
    if ((eSYSTEM_EVENT  == eSYSTEM_EVENT_None)       &&
        (eSYSTEM_STATUS == eSYSTEM_STATUS_CBEmpty)   || 
        (eSYSTEM_STATUS == eSYSTEM_STATUS_CBBuffering))
    {
      uint16_t ui16ParmCount = 0;
      stripSpaces(tmpCharBuf);
      ui16ParmCount = parmCount(tmpCharBuf);
      #ifdef DEBUG_IRD_CALLBACK      
        Serial.println("irRemoteCmdTopic handler");
      #endif
      if ((ui16ParmCount >= 3) && (ui16ParmCount <= 4))
      {
        uint16_t ui16ParmsRead = 0;
        int iRemoteControlsIndex = 0;
        int iRemoteControlsButonIndex = 0;
        int iFindAliasValueReturn = 0;
        char strDevice[MAX_PARM_STRING_LENGTH];
        char strFunction[MAX_PARM_STRING_LENGTH];
        char strParameter[MAX_PARM_STRING_LENGTH];
        char strFreshData[MAX_PARM_STRING_LENGTH];
        sButtonPressType buttonPressArray[MAX_DECODED_BUTTONS];
        uint16_t ui16ButtonPressCount = 0;

        strDevice[0] = '\0';
        strFunction[0] = '\0';
        strParameter[0] = '\0';
        strFreshData[0] = '\0';

        if (ui16ParmCount == 3)
        {
          ui16ParmsRead = parseString(PAYLOAD_SEPARATOR, tmpCharBuf, strDevice, strFunction, strFreshData);
          if ((strlen(strDevice) == 0) || (strlen(strFunction) == 0) || (strlen(strFreshData) == 0)) {
            MQTTclient.publish(iRConfirmTopic, "4,3"); // Null Parameter string in this topic payload
            return;
          }
        } else { // Otherwise it has to be 4
          ui16ParmsRead = parseString(PAYLOAD_SEPARATOR, tmpCharBuf, strDevice, strFunction, strParameter, strFreshData);
          if ((strlen(strDevice) == 0) || (strlen(strFunction) == 0) || (strlen(strParameter) == 0) || (strlen(strFreshData) == 0)) {
            MQTTclient.publish(iRConfirmTopic, "4,4"); // Null Parameter string in this topic payload
            return;
          }
        }

        #ifdef DEBUG_IRD_CALLBACK      
          Serial.print("tmpCharBuf : "); Serial.print(tmpCharBuf); Serial.print(", strDevice : "); Serial.print(strDevice); Serial.print(", strFunction : "); Serial.print(strFunction); Serial.print(", strParameter : "); Serial.print(strParameter); Serial.print(", strFreshData : "); Serial.println(strFreshData);
        #endif

        // Get device index into remoteControls array
        if (isNumeric(strDevice))
        {
          iRemoteControlsIndex = atoi(strDevice);
          if ((iRemoteControlsIndex < eREMOTE_CONTROL_CODING_SCHEME_MIN) || (iRemoteControlsIndex > eREMOTE_CONTROL_CODING_SCHEME_MAX)) {
            MQTTclient.publish(iRConfirmTopic, "5"); // Numerical Device index out of bounds in this topic payload
            return;
          }
        } else {
          iRemoteControlsIndex = findRemote(remoteControls, maxRemoteControls, strDevice);
          if (iRemoteControlsIndex < 0) {
            MQTTclient.publish(iRConfirmTopic, "6"); // Textual Device not recognised in this topic payload
            return;
          }
        }

        #ifdef DEBUG_IRD_CALLBACK      
          Serial.print("iRemoteControlsIndex : "); Serial.println(iRemoteControlsIndex); 
        #endif

        if (ui16ParmCount == 3) { // Handle payload with 3 parameters.
          iRemoteControlsButonIndex = findRemoteButton(remoteControls[iRemoteControlsIndex].remoteControlButtonCollection, remoteControls[iRemoteControlsIndex].sizeOfRemoteControlButtonCollection, strFunction);
          if (iRemoteControlsButonIndex < 0) {
            MQTTclient.publish(iRConfirmTopic, "7"); // Remote Control Button String not recognised in this topic payload
            return;
          }
          // Found button
          ui16ButtonPressCount = 1;
          buttonPressArray[0].ui16ButtonIndex = iRemoteControlsButonIndex;
          buttonPressArray[0].bFreshData = (strFreshData[0]=='1'?true:false);
        } else { // Handle payload with 4 parameters. 
          if (stristr(strFunction,COMPOUND_COMMAND_IDENTIFIER) > 0) {
            if (!isNumeric(strParameter)) // This is an Alias, Alibi, BBC2 etc.
            {
              char *ptrValue = NULL;
     
              iFindAliasValueReturn = findAliasValue(ptrHeadOfAliasValueInstances, strParameter, &ptrValue);
              if (iFindAliasValueReturn < 0) {
                MQTTclient.publish(iRConfirmTopic, "9"); // Unable to find Alias in this topic payload
                return;
              }
              //strcpy_s(strValue,ptrValue);
              unsigned int index = 0;
              char strTmp[2];
              for(; index < strlen(ptrValue); index++)
              {
                strTmp[0] = ptrValue[index]; strTmp[1] = '\0';
                buttonPressArray[index].ui16ButtonIndex = findRemoteButton(remoteControls[iRemoteControlsIndex].remoteControlButtonCollection, remoteControls[iRemoteControlsIndex].sizeOfRemoteControlButtonCollection, strTmp);
                buttonPressArray[index].bFreshData = false;
              }
              buttonPressArray[index-1].bFreshData = (strFreshData[0]=='1'?true:false);
              ui16ButtonPressCount = index;
            } else { // This is a Compound number 1, 10, 101 etc.
              //strcpy_s(strValue,strParameter);
              
              unsigned int index = 0;
              char strTmp[2];
              if (strlen(strParameter) > REMOTE_BUTTON_DIGITS) {
                MQTTclient.publish(iRConfirmTopic, "14"); // Compound number exceeds max digits
                return;
              }
              
              for(; index < strlen(strParameter); index++)
              {
                strTmp[0] = strParameter[index]; strTmp[1] = '\0';
                buttonPressArray[index].ui16ButtonIndex = findRemoteButton(remoteControls[iRemoteControlsIndex].remoteControlButtonCollection, remoteControls[iRemoteControlsIndex].sizeOfRemoteControlButtonCollection, strTmp);
                buttonPressArray[index].bFreshData = false;
              }
              buttonPressArray[index-1].bFreshData = (strFreshData[0]=='1'?true:false);
              ui16ButtonPressCount = index;
            }
          } else {
            MQTTclient.publish(iRConfirmTopic, "8"); // Compound function name not recognised in this topic payload (only looking for the key word 'Chan')
            return;
          }
        }

        // Stack commands into command buffer 
        if ((MAX_BUTTON_PRESSES - ui8CommandBufferHeadPointer)> ui16ButtonPressCount)
        {
          int index = 0;
          for(; index < ui16ButtonPressCount; index++)
          {
            memcpy(&commandBuffer[ui8CommandBufferHeadPointer], &((remoteControls[iRemoteControlsIndex].remoteControlButtonCollection[buttonPressArray[index].ui16ButtonIndex]).bEncoding),sizeof(registerAllocationType));
            commandBuffer[ui8CommandBufferHeadPointer].ra.bFreshData = buttonPressArray[index].bFreshData;
            ui8CommandBufferHeadPointer++;
          }
          
          if (buttonPressArray[index-1].bFreshData) {
            #ifdef DEBUG_IRD_CALLBACK      
              Serial.println("eSYSTEM_EVENT_Got_Data_And_FD_Flag"); 
            #endif
            timer_stop(COMMAND_BUFFER_INACTIVITY_TIMER);
            eSYSTEM_EVENT = eSYSTEM_EVENT_Got_Data_And_FD_Flag;
          } else { // eSYSTEM_EVENT_Got_Data_No_FD_Flag
            if (eSYSTEM_STATUS == eSYSTEM_STATUS_CBEmpty) {
                timer_start(COMMAND_BUFFER_INACTIVITY_TIMER);
                eSYSTEM_STATUS = eSYSTEM_STATUS_CBBuffering;
                #ifdef DEBUG_IRD_CALLBACK      
                  Serial.println("eSYSTEM_STATUS_CBBuffering"); 
                #endif
            } else {
               timer_reset(COMMAND_BUFFER_INACTIVITY_TIMER); 
              #ifdef DEBUG_IRD_CALLBACK      
                Serial.println("Data stacked"); 
              #endif
            }
            //eSYSTEM_EVENT = eSYSTEM_EVENT_Got_Data_No_FD_Flag;
          }

          #ifdef DEBUG_IRD_CALLBACK      
            Serial.println("Decoded Button"); 
            Serial.print("ui8CommandBufferHeadPointer : "); Serial.println(ui8CommandBufferHeadPointer); 
            Serial.print("ui16ButtonPressCount : "); Serial.println(ui16ButtonPressCount); 
          #endif
        } else {
          eSYSTEM_EVENT = eSYSTEM_EVENT_Command_Buffer_Overflow;
          MQTTclient.publish(iRConfirmTopic, "10"); // Command Buffer full/overflow
          ui8CommandBufferHeadPointer = 0; 
          timer_stop(COMMAND_BUFFER_INACTIVITY_TIMER);          
          eSYSTEM_EVENT = eSYSTEM_EVENT_None;
          eSYSTEM_STATUS = eSYSTEM_STATUS_CBEmpty;
          return;
        }
      } else {
        MQTTclient.publish(iRConfirmTopic, "3"); // Wrong number of parameters for this topic payload
        return;
      }
    } else {
      MQTTclient.publish(iRConfirmTopic, "2"); // Busy
      return;
    }
    return;
 }

  if (strcmp(irRemoteCmdPartRawTopic,topic)== 0) {
    if ((eSYSTEM_EVENT  == eSYSTEM_EVENT_None)       &&
        (eSYSTEM_STATUS == eSYSTEM_STATUS_CBEmpty)   || 
        (eSYSTEM_STATUS == eSYSTEM_STATUS_CBBuffering))
    {
      uint16_t ui16ParmCount = 0;
      stripSpaces(tmpCharBuf);
      ui16ParmCount = parmCount(tmpCharBuf);
      #ifdef DEBUG_IRD_CALLBACK      
        Serial.println("irRemoteCmdPartRawTopic handler");
      #endif
      if (ui16ParmCount == 5)
      {
        uint16_t ui16ParmsRead = 0;
        int iRemoteControlsIndex = 0;
        int iRemoteControlsButonIndex = 0;
        int iFindAliasValueReturn = 0;
        char strDevice[MAX_PARM_STRING_LENGTH];
        char strFunction[MAX_PARM_STRING_LENGTH];
        char strParameter[MAX_PARM_STRING_LENGTH];
        char strButtonRepeats[MAX_PARM_STRING_LENGTH];
        char strDelayBetweenButtonRepeats[MAX_PARM_STRING_LENGTH];
        char strFreshData[MAX_PARM_STRING_LENGTH];
        sButtonPressType buttonPressArray[MAX_DECODED_BUTTONS];
        uint16_t ui16ButtonPressCount = 0;

        strDevice[0] = '\0';
        strFunction[0] = '\0';
        strParameter[0] = '\0';
        strButtonRepeats[0]  = '\0';
        strDelayBetweenButtonRepeats[0] = '\0';
        strFreshData[0] = '\0';

        ui16ParmsRead = parseString(PAYLOAD_SEPARATOR, tmpCharBuf, strDevice, strFunction, strButtonRepeats, strDelayBetweenButtonRepeats, strFreshData);
        #ifdef DEBUG_IRD_CALLBACK      
          Serial.print("ui16ParmsRead : "); Serial.println(ui16ParmsRead); 
        #endif
        if ((strlen(strDevice) == 0) || (strlen(strFunction) == 0) || (strlen(strButtonRepeats) == 0) || (strlen(strDelayBetweenButtonRepeats) == 0) || (strlen(strFreshData) == 0)) {
          MQTTclient.publish(iRConfirmTopic, "4"); // Null Parameter string in this topic payload
          return;
        }

        #ifdef DEBUG_IRD_CALLBACK      
          Serial.print("tmpCharBuf : ");                     Serial.print(tmpCharBuf); 
          Serial.print(", strDevice : ");                    Serial.print(strDevice); 
          Serial.print(", strFunction : ");                  Serial.print(strFunction); 
          Serial.print(", strButtonRepeats : ");             Serial.print(strButtonRepeats); 
          Serial.print(", strDelayBetweenButtonRepeats : "); Serial.print(strDelayBetweenButtonRepeats);
          Serial.print(", strFreshData : ");                 Serial.println(strFreshData);
        #endif

        // Get device index into remoteControls array
        if (isNumeric(strDevice))
        {
          iRemoteControlsIndex = atoi(strDevice);
          if ((iRemoteControlsIndex < eREMOTE_CONTROL_CODING_SCHEME_MIN) || (iRemoteControlsIndex > eREMOTE_CONTROL_CODING_SCHEME_MAX)) {
            MQTTclient.publish(iRConfirmTopic, "5"); // Numerical Device index out of bounds in this topic payload
            return;
          }
        } else {
          iRemoteControlsIndex = findRemote(remoteControls, maxRemoteControls, strDevice);
          if (iRemoteControlsIndex < 0) {
            MQTTclient.publish(iRConfirmTopic, "6"); // Textual Device not recognised in this topic payload
            return;
          }
        }

        #ifdef DEBUG_IRD_CALLBACK      
          Serial.print("iRemoteControlsIndex : "); Serial.println(iRemoteControlsIndex); 
        #endif

        iRemoteControlsButonIndex = findRemoteButton(remoteControls[iRemoteControlsIndex].remoteControlButtonCollection, remoteControls[iRemoteControlsIndex].sizeOfRemoteControlButtonCollection, strFunction);
        if (iRemoteControlsButonIndex < 0) {
          MQTTclient.publish(iRConfirmTopic, "7"); // Remote Control Button String not recognised in this topic payload
          return;
        }
        // Found button
        ui16ButtonPressCount = 1;
        buttonPressArray[0].ui16ButtonIndex = iRemoteControlsButonIndex;
        buttonPressArray[0].bFreshData = (strFreshData[0]=='1'?true:false);

        // Stack command into command buffer 
        if ((MAX_BUTTON_PRESSES - ui8CommandBufferHeadPointer)> ui16ButtonPressCount)
        {
          String sbButtonRepeats(strButtonRepeats);
          String sui16DelayBetweenButtonRepeats(strDelayBetweenButtonRepeats);

          if ((sbButtonRepeats.toInt()==0) or (sbButtonRepeats.toInt() > 255)) {
            MQTTclient.publish(iRConfirmTopic, "15"); // Button repeats out of bounds 0 or > 255.
            return;
          }

          if ((sui16DelayBetweenButtonRepeats.toInt()==0) or (sui16DelayBetweenButtonRepeats.toInt() > 65535)) {
            MQTTclient.publish(iRConfirmTopic, "16"); // Delay Between Button Repeats out of bounds 0 or > 65535.
            return;
          }
          memcpy(&commandBuffer[ui8CommandBufferHeadPointer], &((remoteControls[iRemoteControlsIndex].remoteControlButtonCollection[buttonPressArray[0].ui16ButtonIndex]).bEncoding),sizeof(registerAllocationType));

          commandBuffer[ui8CommandBufferHeadPointer].ra.bButtonRepeats = (byte) sbButtonRepeats.toInt();
          commandBuffer[ui8CommandBufferHeadPointer].ra.ui16DelayBetweenButtonRepeats = (uint16_t) sui16DelayBetweenButtonRepeats.toInt();
          commandBuffer[ui8CommandBufferHeadPointer].ra.bFreshData = buttonPressArray[0].bFreshData;
          ui8CommandBufferHeadPointer++;
          
          if (buttonPressArray[0].bFreshData) {
            #ifdef DEBUG_IRD_CALLBACK      
              Serial.println("eSYSTEM_EVENT_Got_Data_And_FD_Flag"); 
            #endif
            timer_stop(COMMAND_BUFFER_INACTIVITY_TIMER);
            eSYSTEM_EVENT = eSYSTEM_EVENT_Got_Data_And_FD_Flag;
          } else { // eSYSTEM_EVENT_Got_Data_No_FD_Flag
            if (eSYSTEM_STATUS == eSYSTEM_STATUS_CBEmpty) {
                timer_start(COMMAND_BUFFER_INACTIVITY_TIMER);
                eSYSTEM_STATUS = eSYSTEM_STATUS_CBBuffering;
                #ifdef DEBUG_IRD_CALLBACK      
                  Serial.println("eSYSTEM_STATUS_CBBuffering"); 
                #endif
            } else {
               timer_reset(COMMAND_BUFFER_INACTIVITY_TIMER); 
              #ifdef DEBUG_IRD_CALLBACK      
                Serial.println("Data stacked"); 
              #endif
            }
            //eSYSTEM_EVENT = eSYSTEM_EVENT_Got_Data_No_FD_Flag;
          }

          #ifdef DEBUG_IRD_CALLBACK      
            Serial.println("Decoded Button"); 
            Serial.print("ui8CommandBufferHeadPointer : "); Serial.println(ui8CommandBufferHeadPointer); 
            Serial.print("ui16ButtonPressCount : "); Serial.println(ui16ButtonPressCount); 
          #endif
        } else {
          eSYSTEM_EVENT = eSYSTEM_EVENT_Command_Buffer_Overflow;
          MQTTclient.publish(iRConfirmTopic, "10"); // Command Buffer full/overflow
          ui8CommandBufferHeadPointer = 0; 
          timer_stop(COMMAND_BUFFER_INACTIVITY_TIMER);          
          eSYSTEM_EVENT = eSYSTEM_EVENT_None;
          eSYSTEM_STATUS = eSYSTEM_STATUS_CBEmpty;
          return;
        }
      } else {
        MQTTclient.publish(iRConfirmTopic, "3"); // Wrong number of parameters for this topic payload
        return;
      }
    } else {
      MQTTclient.publish(iRConfirmTopic, "2"); // Busy
      return;
    }
    return;
 }

  if (strcmp(irRemoteCmdRawTopic,topic)== 0) {
    if ((eSYSTEM_EVENT  == eSYSTEM_EVENT_None)       &&
        (eSYSTEM_STATUS == eSYSTEM_STATUS_CBEmpty)   || 
        (eSYSTEM_STATUS == eSYSTEM_STATUS_CBBuffering))
    {
      uint16_t ui16ParmCount = 0;
      stripSpaces(tmpCharBuf);
      ui16ParmCount = parmCount(tmpCharBuf);
      #ifdef DEBUG_IRD_CALLBACK      
        Serial.println("irRemoteCmdRawTopic handler");
      #endif
      if (ui16ParmCount == 8)
      {
        bool bFreshDataFlag = false;
        uint16_t ui16ParmsRead = 0;
        int iRemoteControlsIndex = 0;
        int iRemoteControlsButonIndex = 0;
        char strEncoding[MAX_PARM_STRING_LENGTH];
        char strData[MAX_PARM_STRING_LENGTH];
        char strNumberOfBitsInTheData[MAX_PARM_STRING_LENGTH];
        char strPulseTrainRepeats[MAX_PARM_STRING_LENGTH];
        char strDelayBetweenPulseTrainRepeats[MAX_PARM_STRING_LENGTH];
        char strButtonRepeats[MAX_PARM_STRING_LENGTH];
        char strDelayBetweenButtonRepeats[MAX_PARM_STRING_LENGTH];
        char strFreshData[MAX_PARM_STRING_LENGTH];
        sButtonPressType buttonPressArray[MAX_DECODED_BUTTONS];
        byte bEncoding;
        uint32_t ui32Data;
        byte bNumberOfBitsInTheData;
        byte bPulseTrainRepeats;
        byte bDelayBetweenPulseTrainRepeats;
        byte bButtonRepeats;
        uint16_t ui16DelayBetweenButtonRepeats;
        byte bFreshData;
        
        strEncoding[0] = '\0';
        strData[0] = '\0';
        strNumberOfBitsInTheData[0] = '\0';
        strPulseTrainRepeats[0] = '\0';
        strDelayBetweenPulseTrainRepeats[0] = '\0';
        strButtonRepeats[0] = '\0';
        strDelayBetweenButtonRepeats[0] = '\0';
        strFreshData[0] = '\0';

        
        ui16ParmsRead = parseString(PAYLOAD_SEPARATOR, tmpCharBuf, strEncoding, strData, strNumberOfBitsInTheData, strPulseTrainRepeats, strDelayBetweenPulseTrainRepeats, strButtonRepeats, strDelayBetweenButtonRepeats, strFreshData);
        #ifdef DEBUG_IRD_CALLBACK      
          Serial.print("ui16ParmsRead : "); Serial.println(ui16ParmsRead); 
        #endif
        if ((strlen(strEncoding) == 0) || (strlen(strData) == 0) || (strlen(strNumberOfBitsInTheData) == 0) || (strlen(strPulseTrainRepeats) == 0) || (strlen(strDelayBetweenPulseTrainRepeats) == 0) || (strlen(strButtonRepeats) == 0) || (strlen(strDelayBetweenButtonRepeats) == 0) || (strlen(strFreshData) == 0)) {
          MQTTclient.publish(iRConfirmTopic, "4"); // Null Parameter string in this topic payload
          return;
        }

        #ifdef DEBUG_IRD_CALLBACK      
          Serial.print("tmpCharBuf : ");                         Serial.print(tmpCharBuf); 
          Serial.print(", strEncoding : ");                      Serial.print(strEncoding); 
          Serial.print(", strData : ");                          Serial.print(strData); 
          Serial.print(", strNumberOfBitsInTheData : ");         Serial.print(strNumberOfBitsInTheData); 
          Serial.print(", strPulseTrainRepeats : ");             Serial.print(strPulseTrainRepeats); 
          Serial.print(", strDelayBetweenPulseTrainRepeats : "); Serial.print(strDelayBetweenPulseTrainRepeats); 
          Serial.print(", strButtonRepeats : ");                 Serial.print(strButtonRepeats); 
          Serial.print(", strDelayBetweenButtonRepeats : ");     Serial.print(strDelayBetweenButtonRepeats);
          Serial.print(", strFreshData : ");                     Serial.println(strFreshData);
        #endif

        // Encoding
        if (!(isHex(strEncoding) || isNumeric(strEncoding))) {
          MQTTclient.publish(iRConfirmTopic, "1,1");
          return;
        } else {
          if (isHex(strEncoding))
          {
            char *ptr;
            bEncoding = (byte) strtol(strEncoding, &ptr, 16);
          } else { // Must be numerical
            String sbEncoding(strEncoding);
            bEncoding = (byte) sbEncoding.toInt();
          }
          #ifdef DEBUG_IRD_CALLBACK      
            Serial.print("bEncoding : "); Serial.println(bEncoding); 
          #endif
        }

        // Data
        if (!(isHex(strData) || isNumeric(strData))) {
          MQTTclient.publish(iRConfirmTopic, "1,2");
          return;
        } else {
          if (isHex(strData))
          {
            char *ptr;
            ui32Data = (uint32_t) strtoll(strData, NULL, 16);
            //ui32Data = (uint32_t) strtol(strData, &ptr, 16);
          } else { // Must be numerical
            String sui32Data(strData);
            ui32Data = (uint32_t) sui32Data.toInt();
          }
          #ifdef DEBUG_IRD_CALLBACK      
            Serial.print("ui32Data : "); Serial.println(ui32Data); 
          #endif
        }
          
        // Number Of Bits In The Data
        if (!(isHex(strNumberOfBitsInTheData) || isNumeric(strNumberOfBitsInTheData))) {
          MQTTclient.publish(iRConfirmTopic, "1,3");
          return;
        } else {
          if (isHex(strNumberOfBitsInTheData))
          {
            char *ptr;
            bNumberOfBitsInTheData = (byte) strtol(strNumberOfBitsInTheData, &ptr, 16);
          } else { // Must be numerical
            String sbNumberOfBitsInTheData(strNumberOfBitsInTheData);
            bNumberOfBitsInTheData = (byte) sbNumberOfBitsInTheData.toInt();
          }
          #ifdef DEBUG_IRD_CALLBACK      
            Serial.print("bNumberOfBitsInTheData : "); Serial.println(bNumberOfBitsInTheData); 
          #endif
        }
          
        // Pulse Train Repeats  
        if (!(isHex(strPulseTrainRepeats) || isNumeric(strPulseTrainRepeats))) {
          MQTTclient.publish(iRConfirmTopic, "1,4");
          return;
        } else {
          if (isHex(strPulseTrainRepeats))
          {
            char *ptr;
            bPulseTrainRepeats = (byte) strtol(strPulseTrainRepeats, &ptr, 16);
          } else { // Must be numerical
            String sbPulseTrainRepeats(strPulseTrainRepeats);
            bPulseTrainRepeats = (byte) sbPulseTrainRepeats.toInt();
          }
          #ifdef DEBUG_IRD_CALLBACK      
            Serial.print("bPulseTrainRepeats : "); Serial.println(bPulseTrainRepeats); 
          #endif
        }
        
        // Delay Between Pulse Train Repeats
        if (!(isHex(strDelayBetweenPulseTrainRepeats) || isNumeric(strDelayBetweenPulseTrainRepeats))) {
          MQTTclient.publish(iRConfirmTopic, "1,5");
          return;
        } else {
          if (isHex(strDelayBetweenPulseTrainRepeats))
          {
            char *ptr;
            bDelayBetweenPulseTrainRepeats = (byte) strtol(strDelayBetweenPulseTrainRepeats, &ptr, 16);
          } else { // Must be numerical
            String sbDelayBetweenPulseTrainRepeats(strDelayBetweenPulseTrainRepeats);
            bDelayBetweenPulseTrainRepeats = (byte) sbDelayBetweenPulseTrainRepeats.toInt();
          }
          #ifdef DEBUG_IRD_CALLBACK      
            Serial.print("bDelayBetweenPulseTrainRepeats : "); Serial.println(bDelayBetweenPulseTrainRepeats); 
          #endif
        }

        // Button Repeats
        String sbButtonRepeats(strButtonRepeats);
        if ((!(isHex(strButtonRepeats) || isNumeric(strButtonRepeats))) or ((sbButtonRepeats.toInt()==0) or (sbButtonRepeats.toInt() > 255))) {
          MQTTclient.publish(iRConfirmTopic, "1,6");
          return;
        } else {
          if (isHex(strButtonRepeats))
          {
            char *ptr;
            bButtonRepeats = (byte) strtol(strButtonRepeats, &ptr, 16);
          } else { // Must be numerical
            String sbButtonRepeats(strButtonRepeats);
            bButtonRepeats = (byte) sbButtonRepeats.toInt();
          }
          #ifdef DEBUG_IRD_CALLBACK      
            Serial.print("bButtonRepeats : "); Serial.println(bButtonRepeats); 
          #endif
        }

        // Delay Between Button Repeats
        if (!(isHex(strDelayBetweenButtonRepeats) || isNumeric(strDelayBetweenButtonRepeats))) {
          MQTTclient.publish(iRConfirmTopic, "1,7");
          return;
        } else {
          if (isHex(strDelayBetweenButtonRepeats))
          {
            char *ptr;
            ui16DelayBetweenButtonRepeats = (uint16_t) strtol(strDelayBetweenButtonRepeats, &ptr, 16);
          } else { // Must be numerical
            String sui16DelayBetweenButtonRepeats(strDelayBetweenButtonRepeats);
            ui16DelayBetweenButtonRepeats = (uint16_t) sui16DelayBetweenButtonRepeats.toInt();
          }
          #ifdef DEBUG_IRD_CALLBACK      
            Serial.print("ui16DelayBetweenButtonRepeats : "); Serial.println(ui16DelayBetweenButtonRepeats); 
          #endif
        }

        // Fresh Data
        if (!isNumeric(strFreshData)) {
          MQTTclient.publish(iRConfirmTopic, "1,8");
          return;
        }
        
        // Stack command into command buffer 
        if ((MAX_BUTTON_PRESSES - ui8CommandBufferHeadPointer)> 1)
        {
          commandBuffer[ui8CommandBufferHeadPointer].ra.bEncoding = bEncoding;
          commandBuffer[ui8CommandBufferHeadPointer].ra.ui32Data = ui32Data;
          commandBuffer[ui8CommandBufferHeadPointer].ra.bNumberOfBitsInTheData = bNumberOfBitsInTheData;
          commandBuffer[ui8CommandBufferHeadPointer].ra.bPulseTrainRepeats = bPulseTrainRepeats;
          commandBuffer[ui8CommandBufferHeadPointer].ra.bDelayBetweenPulseTrainRepeats = bDelayBetweenPulseTrainRepeats;
          commandBuffer[ui8CommandBufferHeadPointer].ra.bButtonRepeats = bButtonRepeats;
          commandBuffer[ui8CommandBufferHeadPointer].ra.ui16DelayBetweenButtonRepeats = ui16DelayBetweenButtonRepeats;
          commandBuffer[ui8CommandBufferHeadPointer].ra.bFreshData = (strFreshData[0]=='1'?1:0);
          bFreshDataFlag = strFreshData[0]=='1'?true:false;
          ui8CommandBufferHeadPointer++;
          
          if (bFreshDataFlag) {
            bFreshDataFlag = false;
            #ifdef DEBUG_IRD_CALLBACK      
              Serial.println("eSYSTEM_EVENT_Got_Data_And_FD_Flag"); 
            #endif
            timer_stop(COMMAND_BUFFER_INACTIVITY_TIMER);
            eSYSTEM_EVENT = eSYSTEM_EVENT_Got_Data_And_FD_Flag;
          } else { // eSYSTEM_EVENT_Got_Data_No_FD_Flag
            if (eSYSTEM_STATUS == eSYSTEM_STATUS_CBEmpty) {
                timer_start(COMMAND_BUFFER_INACTIVITY_TIMER);
                eSYSTEM_STATUS = eSYSTEM_STATUS_CBBuffering;
                #ifdef DEBUG_IRD_CALLBACK      
                  Serial.println("eSYSTEM_STATUS_CBBuffering"); 
                #endif
            } else {
               timer_reset(COMMAND_BUFFER_INACTIVITY_TIMER); 
              #ifdef DEBUG_IRD_CALLBACK      
                Serial.println("Data stacked"); 
              #endif
            }
            //eSYSTEM_EVENT = eSYSTEM_EVENT_Got_Data_No_FD_Flag;
          }

          #ifdef DEBUG_IRD_CALLBACK      
            Serial.print("ui8CommandBufferHeadPointer : "); Serial.println(ui8CommandBufferHeadPointer); 
          #endif
        } else {
          eSYSTEM_EVENT = eSYSTEM_EVENT_Command_Buffer_Overflow;
          MQTTclient.publish(iRConfirmTopic, "10"); // Command Buffer full/overflow
          ui8CommandBufferHeadPointer = 0; 
          timer_stop(COMMAND_BUFFER_INACTIVITY_TIMER);          
          eSYSTEM_EVENT = eSYSTEM_EVENT_None;
          eSYSTEM_STATUS = eSYSTEM_STATUS_CBEmpty;
          return;
        }
      } else {
        MQTTclient.publish(iRConfirmTopic, "3"); // Wrong number of parameters for this topic payload
        return;
      }
    } else {
      MQTTclient.publish(iRConfirmTopic, "2"); // Busy
      return;
    }
    return;
 }
}



void grabParm(char **ptrToParmString, String *recipientString){
  #ifdef DEBUG_PARMGRAB
  Serial.print("**ptrToParmString : "); 
  #endif
  while (**ptrToParmString)
  {
    #ifdef DEBUG_PARMGRAB
    Serial.print(**ptrToParmString);
    #endif
    *recipientString += **ptrToParmString;
    (*ptrToParmString)++;
    if ((**ptrToParmString=='\0') || (**ptrToParmString==','))
    {
      if (**ptrToParmString==',')
        (*ptrToParmString)++;
      #ifdef DEBUG_PARMGRAB
      Serial.println();
      #endif
      return;
    }
  }
}



int fileWrite(File f, FileVarInstance *fviArray, int iTotalParametersToWrite) {
    String s;
    for (int i = 0; i < iTotalParametersToWrite; i++){
      switch (fviArray[i].iVarType){
        case FILE_VAR_INSTANCE_TYPE_STRING :
                f.println((char *)(fviArray[i].ptrVar));
                break;
        case FILE_VAR_INSTANCE_TYPE_FLOAT :
                char tmpStr[10];
                dtostrf(*((float *)(fviArray[i].ptrVar)),5,2,tmpStr); // dtostrf(FLOAT,WIDTH,PRECSISION,BUFFER);
                s = tmpStr;
                s.trim();
                f.println(s.c_str());
                //f.println(tmpStr);
                break;
        case FILE_VAR_INSTANCE_TYPE_INT :
                f.println(*((int *)(fviArray[i].ptrVar)));
                break;
        case FILE_VAR_INSTANCE_TYPE_BOOL :
                f.println( ((*((int *)(fviArray[i].ptrVar)))?"1":"0") );
                break;
        default :
                return 1;
      }
  }
  return 0;
}



int fileRead(File f, FileVarInstance *fviArray, int iTotalParametersToRead) {
    String s;
    for (int i = 0; i < iTotalParametersToRead; i++){
      s=f.readStringUntil('\n');
      s.trim();
      switch (fviArray[i].iVarType){
        case FILE_VAR_INSTANCE_TYPE_STRING :
                strcpy((char *)(fviArray[i].ptrVar),s.c_str());
                break;
        case FILE_VAR_INSTANCE_TYPE_FLOAT :
                *((float *)(fviArray[i].ptrVar)) = s.toFloat();
                break;
        case FILE_VAR_INSTANCE_TYPE_INT :
                *((int *)(fviArray[i].ptrVar)) = s.toInt();
                break;
        case FILE_VAR_INSTANCE_TYPE_BOOL :
                *((bool *)(fviArray[i].ptrVar)) = (s.toInt()==0?false:true);
                break;
        default : // Unknown data type
                return 1;
      }
  }
  return 0; // Successful completion
}



void readCalibrationValues()
{
  // open file for reading
  String s;
  File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_READ_MODE);
  if (!f) {
    tempCalOffset = TEMPERATURE_CALIBRATION_OFFSET_DEFAULT;
    tempCalScale  = TEMPERATURE_CALIBRATION_SCALE_DEFAULT;
    humCalOffset  = HUMIDITY_CALIBRATION_OFFSET_DEFAULT;
    humCalScale   = HUMIDITY_CALIBRATION_SCALE_DEFAULT;
    reportingStrategy = REPORTING_STRATEGY_DEFAULT;
    #ifdef DEBUG_GENERAL
    Serial.println("Failed to read SD Cal Vals");
    #endif
  } else {
    fileRead(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
    f.close();
  }
  #ifdef DEBUG_GENERAL
  Serial.println("readCalibrationValues");
  s=tempCalOffset;
  Serial.print("Temp Cal Offset : ");    Serial.println(s);
  s=tempCalScale;
  Serial.print("Temp Cal Scale : ");    Serial.println(s);
  s=humCalOffset;
  Serial.print("Humi Cal Offset : ");    Serial.println(s);
  s=humCalScale;
  Serial.print("Humi Cal Scale : ");    Serial.println(s);
  s=reportingStrategy;
  Serial.print("Reporting Strategy : "); Serial.println(s);
  #endif
}



void readNetworkSecurityParameters(){
  // open file for reading
  String s;
  File f = SD.open(SECURITY_PARAMETERS_FILE, SD_FILE_READ_MODE);
  if (!f) {
    strcpy(mqtt_broker_ip, MQTT_BROKER_IP_DEFAULT);
    mqtt_broker_port = MQTT_BROKER_PORT_DEFAULT;
    mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
    strcpy(sta_network_ssid, STA_NETWORK_SSID_DEFAULT);
    strcpy(sta_network_password, STA_NETWORK_PASSWORD_DEFAULT);
    network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
    #ifdef DEBUG_GENERAL
    Serial.println("Failed to read SD Sec Vals. Using defaults");
    #endif
  } else {
    fileRead(f, SecurityVarArray,(int)(sizeof(SecurityVarArray)/sizeof(tsFileVarInstance)));
    f.close();
  }
  strcpy(ap_network_ssid,AP_NETWORK_SSID_DEFAULT);
  strcat(ap_network_ssid,macStrForAPSSID.c_str());
  strcpy(ap_network_password,AP_NETWORK_PASSWORD_DEFAULT);
  #ifdef DEBUG_GENERAL
  Serial.println("readNetworkSecurityParameters");
  Serial.print("Broker IP : ");            Serial.println(mqtt_broker_ip);
  Serial.print("Broker Port : ");          Serial.println(mqtt_broker_port);
  Serial.print("Max MQTT Conn Atmpts : "); Serial.println(mqtt_broker_connection_attempts);
  Serial.print("STA SSID : ");             Serial.println(sta_network_ssid);
  Serial.print("STA PW : ");               Serial.println(sta_network_password);
  Serial.print("Max NW Conn Atmpts : ");   Serial.println(network_connection_attempts);
  Serial.print("AP SSID : ");              Serial.println(ap_network_ssid);
  Serial.print("AP PW : ");                Serial.println(ap_network_password);
  #endif
}



void connectMQTT() {
  int connection_counts = 0;
  eIRDEVICESTATE tmpeIRDEVICESTATE_STATE;
  bBrokerPresent = true;
  #ifdef DEBUG_GENERAL
  conDotCountMQTT = 0;
  #endif
  tmpeIRDEVICESTATE_STATE = eIRDEVICESTATE_STATE; // Record the state connectMQTT was entered from. 
  // Make sure we are connected to WIFI before attemping to reconnect to MQTT
  eLEDFLASHSTATE_STATE = eLEDFLASH_PENDING_MQTT;
  timer_update(); // Update timers
  if(WiFi.status() == WL_CONNECTED){
    // Loop until we're reconnected to the MQTT server
    #ifdef DEBUG_STATE_CHANGE
    SHOW_UPDATED_STATE(eIRDEVICESTATE_STATE,eIRDEVICESTATE_PENDING_MQTT,"connectMQTT");
    #endif
    eIRDEVICESTATE_STATE = eIRDEVICESTATE_PENDING_MQTT;
    #ifdef DEBUG_GENERAL
    Serial.print("Attempting MQTT connection");
    #endif
    while (!MQTTclient.connected()) {
      #ifdef DEBUG_GENERAL
      if (conDotCountMQTT > 50)
          conDotCountMQTT = 0;
      if (conDotCountMQTT == 0)
        Serial.print(".");
      conDotCountMQTT++;  
      #endif
    
      timer_update(); // Update timers
      server.handleClient();      
      //if connected, subscribe to the topic(s) we want to be notified about
      if (MQTTclient.connect((char*) clientName.c_str())) {
        // Start wifi subsystem
        WiFi.mode(WIFI_STA);  // Switch off access point and turn into station only
        //WiFi.begin((const char *)sta_network_ssid, (const char *)sta_network_password);
        #ifdef DEBUG_GENERAL
        Serial.println();
        Serial.println("Switching to STA Mode. Now MQTT is connected.");
        #endif
        MQTTclient.publish(swVerConfirmTopic, swVersion.c_str());        
        makeSubscriptions();
        #ifdef DEBUG_STATE_CHANGE
        SHOW_UPDATED_STATE(eIRDEVICESTATE_STATE,eIRDEVICESTATE_ACTIVE,"connectMQTT");
        #endif
        eIRDEVICESTATE_STATE = eIRDEVICESTATE_ACTIVE;
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
      } else { //otherwise print failed for debugging
        #ifdef DEBUG_GENERAL
        Serial.println("\tFailed."); 
        #endif
        //abort();
      }
      
      if(WiFi.status() != WL_CONNECTED) { // Catches a lost NW whilst looking for MQTT broker
        #ifdef DEBUG_STATE_CHANGE
        SHOW_UPDATED_STATE(eIRDEVICESTATE_STATE,eIRDEVICESTATE_INIT,"connectMQTT WiFi lost");
        #endif
        eIRDEVICESTATE_STATE = eIRDEVICESTATE_INIT;
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
        return;
      }
      
      if (eIRDEVICESTATE_STATE == eIRDEVICESTATE_INIT) // Catches the state where device is hung in MQTT pending mode with mqtt_broker_connection_attempts==0 and user sets new config via handleNetworkConfig
      {
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
        timer_update();
        return;
      }  
      
      if ((connection_counts >= mqtt_broker_connection_attempts) && (mqtt_broker_connection_attempts > 0))
      {
        #ifdef DEBUG_STATE_CHANGE
        SHOW_UPDATED_STATE(eIRDEVICESTATE_STATE,eIRDEVICESTATE_INIT,"connectMQTT con count");
        #endif
        eIRDEVICESTATE_STATE = eIRDEVICESTATE_INIT;
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
        timer_update();
        if (tmpeIRDEVICESTATE_STATE == eIRDEVICESTATE_ACTIVE)
          bBrokerPresent = true; // Force programme to go back to eIRDEVICESTATE_INIT state if MQTT Broker conn lost after having connected to the nw and broker at least once
        else
          bBrokerPresent = false; // Force programme to go to eIRDEVICESTATE_NO_CONFIG State if after MQTT connection attempts made and never having made an MQTT on this nw before
        return;
      }
      
      if (mqtt_broker_connection_attempts > 0)
        connection_counts++;
      yield();  
      //delay(10);
    }
  } else { // catches a lost NW as the cause for an MQTT broker connection failure
    #ifdef DEBUG_STATE_CHANGE
    SHOW_UPDATED_STATE(eIRDEVICESTATE_STATE,eIRDEVICESTATE_INIT,"connectMQTT no WiFi at start");
    #endif
    eIRDEVICESTATE_STATE = eIRDEVICESTATE_INIT;
    eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
  }
}



void makeSubscriptions(void)
{
// Fixes these : https://github.com/knolleary/pubsubclient/issues/141
//             : https://github.com/knolleary/pubsubclient/issues/98
  for (int index = 0; index < maxSubscriptions; index++)
  {
    MQTTclient.subscribe(subscriptionsArray[index]);
    for (int i=0;i<10;i++) {
      MQTTclient.loop();
      yield();
      //delay(10);
    }
  }
}



//generate unique name from MAC addr
String macToStr(const uint8_t* mac, boolean addColons){

  String result;

  for (int i = 0; i < 6; ++i) {
    if ((mac[i] & 0xF0) == 0)
      result += String(0, HEX); // stop suppression of leading zero
    result += String(mac[i], HEX);

    if (addColons && (i < 5)){
      result += ':';
    }
  }
  return result;
}



void timer_create(int iTimerNumber, unsigned long ulTimerPeriod, void (*callbackfn)(void))
{
  if (iTimerNumber <= MAX_TIMERS)
  {
    stiTimerArray[iTimerNumber].tmrcallback = callbackfn;
    stiTimerArray[iTimerNumber].bRunning = false;
    stiTimerArray[iTimerNumber].ulTimerPeriod = ulTimerPeriod;
    stiTimerArray[iTimerNumber].ulStartValue = 0;
    #ifdef DEBUG_TIMER
    Serial.print(F("T Create, TNum : "));
    Serial.print(iTimerNumber);
    Serial.print(F(", TPeriod : "));
    Serial.println(ulTimerPeriod);
    #endif
  }
}



void timer_update(void)
{
  unsigned long ulCurrentTime = millis();
  unsigned long ulElapsedTime = 0;
  
  for (int iIndex = 0; iIndex < MAX_TIMERS; iIndex++)
  {
    if (stiTimerArray[iIndex].bRunning)
    {
      ulElapsedTime = ulCurrentTime - stiTimerArray[iIndex].ulStartValue;
      /* // Argh! twos complement arithmetic, I hate it...
      if (ulCurrentTime < stiTimerArray[iIndex].ulStartValue) // Cater for UL counter wrap ~every day
        ulElapsedTime = ulCurrentTime - stiTimerArray[iIndex].ulStartValue;
      else  
        ulElapsedTime = ulCurrentTime + (ULONG_MAX - stiTimerArray[iIndex].ulStartValue);
      */
      #ifdef DEBUG_TIMER
      Serial.print(F("T Up, TNum : "));
      Serial.print(iIndex);
      Serial.print(F(", T Elapsed : "));
      Serial.println(ulElapsedTime);
      #endif
        
      if (ulElapsedTime >= stiTimerArray[iIndex].ulTimerPeriod)
      {
        stiTimerArray[iIndex].bRunning = false;
        stiTimerArray[iIndex].tmrcallback();
      }
    }
  }
}



void timer_start(int iTimerNumber)
{
  if (iTimerNumber <= MAX_TIMERS)
  {
    stiTimerArray[iTimerNumber].ulStartValue = millis();
    stiTimerArray[iTimerNumber].bRunning = true;
    #ifdef DEBUG_TIMER
    Serial.print(F("T Start , TNum : "));
    Serial.print(iTimerNumber);
    Serial.print(F(", TStart : "));
    Serial.println(stiTimerArray[iTimerNumber].ulStartValue);
    #endif
  }
}



void timer_stop(int iTimerNumber)
{
  if (iTimerNumber <= MAX_TIMERS)
    stiTimerArray[iTimerNumber].bRunning = false;
  #ifdef DEBUG_TIMER
  Serial.print(F("T Stop : "));
  Serial.println(iTimerNumber);
  #endif
}



void timer_reset(int iTimerNumber)
{
  if (iTimerNumber <= MAX_TIMERS)
    stiTimerArray[iTimerNumber].ulStartValue = millis();
  #ifdef DEBUG_TIMER
  Serial.print(F("T Reset : "));
  Serial.println(iTimerNumber);
  #endif
}



boolean timer_isRunning(int iTimerNumber)
{
  return stiTimerArray[iTimerNumber].bRunning;
}



void timer_change_period(int iTimerNumber, unsigned long ulTimerPeriod)
{
  boolean bTmpRunning;
  if (iTimerNumber <= MAX_TIMERS)
  {
    bTmpRunning = stiTimerArray[iTimerNumber].bRunning;
    stiTimerArray[iTimerNumber].bRunning = false;
    stiTimerArray[iTimerNumber].ulTimerPeriod = ulTimerPeriod;
    stiTimerArray[iTimerNumber].bRunning = bTmpRunning;
    #ifdef DEBUG_TIMER
    Serial.print(F("T Change Period, TNum : "));
    Serial.print(iTimerNumber);
    Serial.print(F(", TPeriod : "));
    Serial.println(ulTimerPeriod);
    #endif
  }
}


  
void ledFlashTimerCallback(void)
{
  // This is called if the led flash timer has timed out. 
  #ifdef DEBUG_TIMER
  Serial.println("In ledFlashTimerCallback()");
  #endif


  #ifdef DEBUG_LEDFLASH
  SHOW_UPDATED_LED_STATE(eLEDFLASHSTATE_STATE,eLEDFLASHSTATE_STATE,"ledFlashTimerCallback");
  Serial.print("Led Flash : ");
  Serial.print(cFlashProfiles[eLEDFLASHSTATE_STATE][iFlashSequenceIndex]);
  Serial.print(", Led Flash Ind : ");
  Serial.print(iFlashSequenceIndex);
  Serial.print(", Led Flash State : ");
  Serial.println(eLEDFLASHSTATE_STATE);
  #endif

  switch (eLEDFLASHSTATE_STATE){
    case eLEDFLASH_NO_CONFIG    :
    case eLEDFLASH_PENDING_NW   :
    case eLEDFLASH_PENDING_MQTT :
        if (cFlashProfiles[eLEDFLASHSTATE_STATE][iFlashSequenceIndex] == '1')
          port.digitalWrite(lightPin0, LOW); // Led on
        else  
          port.digitalWrite(lightPin0, HIGH); // Led off
        break;
        
    case eLEDFLASH_SEQUENCE_END : 
        port.digitalWrite(lightPin0, HIGH); // Led off
        eLEDFLASHSTATE_STATE = eLEDFLASH_OFF;
        break;
        
    case eLEDFLASH_OFF : 
        iFlashSequenceIndex = 0;
        break;
        
    default : 
        break;
  }

  iFlashSequenceIndex++;
  if (iFlashSequenceIndex >= (FLASH_SEQUENCE_MAX-1))
    iFlashSequenceIndex = 0;

  //if (eLEDFLASHSTATE_STATE != eLEDFLASH_OFF)
    timer_start(LED_FLASH_TIMER);
  #ifdef DEBUG_SD
  Serial.println("In ledFlashTimerCallback");
  #endif
}



void periodicUpdateTimerCallback(void)
{
  // This is called if the Periodic Update timer has timed out. 
  #ifdef DEBUG_TIMER
  Serial.println("In periodicUpdateTimerCallback()");
  #endif

  sendTHUpdate = true;
  sendALSUpdate = true;  
  timer_start(PERIODIC_UPDATE_TIMER);
  #ifdef DEBUG_SD
  Serial.println("In periodicUpdateTimerCallback");
  #endif
}



void irDeviceResetTimerCallback(void)
{
  // This is called if the IR Device Reset Timer has timed out. 
  #ifdef DEBUG_TIMER
  Serial.println("In irDeviceResetTimerCallback()");
  #endif
  #ifdef DEBUG_IRDEV_TIMER
  Serial.println("In irDeviceResetTimerCallback()");
  #endif

  #ifdef DEBUG_IR_DEVICE_STATES_EVENTS
  SHOW_IR_DEVICE_STATE_EVENT(eIR_DEVICE_STATUS,eIR_DEVICE_EVENT,"In irDeviceResetTimerCallback");
  #endif
  switch (eIR_DEVICE_STATUS) {
      case eIR_DEVICE_STATUS_Idle : 
             break;
      case eIR_DEVICE_STATUS_TXing : 
             break;
      case eIR_DEVICE_STATUS_Transferring : 
             break;
      case eIR_DEVICE_STATUS_Resetting : 
             eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_Reset_Timeout;
             break;
  }
  #ifdef DEBUG_IR_DEVICE_STATES_EVENTS
  SHOW_IR_DEVICE_STATE_EVENT(eIR_DEVICE_STATUS,eIR_DEVICE_EVENT,"Out irDeviceResetTimerCallback");
  #endif
 
  #ifdef DEBUG_IR_TIMER
  Serial.println("In irDeviceResetTimerCallback");
  #endif
}



void irDeviceBusyTimerCallback(void)
{
  // This is called if the IR Device Busy Timer has timed out. 
  #ifdef DEBUG_TIMER
  Serial.println("In irDeviceBusyTimerCallback()");
  #endif
  #ifdef DEBUG_IRDEV_TIMER
  Serial.println("In irDeviceBusyTimerCallback()");
  #endif

  #ifdef DEBUG_IR_DEVICE_STATES_EVENTS
  SHOW_IR_DEVICE_STATE_EVENT(eIR_DEVICE_STATUS,eIR_DEVICE_EVENT,"In irDeviceBusyTimerCallback");
  #endif
  switch (eIR_DEVICE_STATUS) {
      case eIR_DEVICE_STATUS_Idle : 
             break;
      case eIR_DEVICE_STATUS_TXing : 
             eIR_DEVICE_EVENT = eIR_DEVICE_EVENT_Busy_Timeout;
             break;
      case eIR_DEVICE_STATUS_Transferring : 
             break;
      case eIR_DEVICE_STATUS_Resetting : 
             break;
  }
  #ifdef DEBUG_IR_DEVICE_STATES_EVENTS
  SHOW_IR_DEVICE_STATE_EVENT(eIR_DEVICE_STATUS,eIR_DEVICE_EVENT,"Out irDeviceBusyTimerCallback");
  #endif
    
  #ifdef DEBUG_IR_TIMER
  Serial.println("In irDeviceBusyTimerCallback");
  #endif
}



void commandBufferInactivityTimerCallback(void)
{
  // This is called if the Command Buffer Inactivity Timer has timed out. 
  #ifdef DEBUG_TIMER
  Serial.println("In commandBufferInactivityTimerCallback()");
  #endif
  #ifdef DEBUG_IRDEV_TIMER
  Serial.println("In commandBufferInactivityTimerCallback()");
  #endif

  #ifdef DEBUG_SYSTEM_STATES_EVENTS
  SHOW_SYSTEM_STATE_EVENT(eSYSTEM_STATUS,eSYSTEM_EVENT,"In commandBufferInactivityTimerCallback");
  #endif
  switch (eSYSTEM_STATUS) {
    case eSYSTEM_STATUS_CBEmpty : 
           break;
    case eSYSTEM_STATUS_CBBuffering : 
           eSYSTEM_EVENT = eSYSTEM_EVENT_Command_Buffer_Timeout;
           break;
    case eSYSTEM_STATUS_CBBusy : 
           break;
  }
  #ifdef DEBUG_SYSTEM_STATES_EVENTS
  SHOW_SYSTEM_STATE_EVENT(eSYSTEM_STATUS,eSYSTEM_EVENT,"Out commandBufferInactivityTimerCallback");
  #endif
  
  #ifdef DEBUG_IR_TIMER
  Serial.println("In commandBufferInactivityTimerCallback");
  #endif
}



void returnOK(String mess) {
  #ifdef DEBUG_WEB
  Serial.println("returnOK");  
  #endif
  if (mess.length() > 0)
    server.send(200, "text/html", mess);
  else  
    server.send(200, "text/plain", "");
}



void returnFail(String msg) {
  #ifdef DEBUG_WEB
  Serial.println("returnFail");  
  #endif
  server.send(500, "text/plain", msg + "\r\n");
}



bool loadFromSD(String path){
  String dataType = "text/plain";
  #ifdef DEBUG_WEB
  Serial.println("loadFromSD");  
  #endif
  if(path.endsWith("/")) path += "index.htm";

  if(path.endsWith(".src")) path = path.substring(0, path.lastIndexOf("."));
  else if(path.endsWith(".htm")) dataType = "text/html";
  else if(path.endsWith(".css")) dataType = "text/css";
  else if(path.endsWith(".js")) dataType = "application/javascript";
  else if(path.endsWith(".json")) dataType = "application/json";
  else if(path.endsWith(".png")) dataType = "image/png";
  else if(path.endsWith(".gif")) dataType = "image/gif";
  else if(path.endsWith(".jpg")) dataType = "image/jpeg";
  else if(path.endsWith(".ico")) dataType = "image/x-icon";
  else if(path.endsWith(".xml")) dataType = "text/xml";
  else if(path.endsWith(".pdf")) dataType = "application/pdf";
  else if(path.endsWith(".zip")) dataType = "application/zip";
  else if(path.endsWith(".png")) dataType = "image/png";

  File dataFile = SD.open(path.c_str(),SD_FILE_READ_MODE);

  if (!dataFile)
    return false;

  if (server.hasArg("download")) dataType = "application/octet-stream";

  if (server.streamFile(dataFile, dataType) != dataFile.size()) {
    #ifdef DEBUG_WEB
    Serial.println("Sent less data than expected!");
    #endif
  }

  dataFile.close();
  return true;
}



void handleNetworkConfig()
{
  String pass_response;
  String fail_response;
  char tmp_mqtt_broker_ip[MQTT_BROKER_IP_STRING_MAX_LEN];
  int  tmp_mqtt_broker_port;
  int  tmp_mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
  char tmp_sta_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
  char tmp_sta_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
  int  tmp_network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
  //char tmp_ap_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
  //char tmp_ap_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];

  pass_response  = "<html>";
  pass_response += "  <head>";
  pass_response += "   <title>Form submitted</title>";
  pass_response += " </head>";
  pass_response += " <body>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Sensor Configuration Home Page </b> </font></p>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif'>New configuration details now submitted</font></p>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif'><a href='index.htm'>Return to main page</a></font></p>";
  pass_response += " </body>";
  pass_response += "</html>";  

  fail_response  = "<html>";
  fail_response += "  <head>";
  fail_response += "   <title>Form not submitted</title>";
  fail_response += " </head>";
  fail_response += " <body>";
  fail_response += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Sensor Configuration Home Page </b> </font></p>";
  fail_response += "   <p><font face='Helvetica, Arial, sans-serif'>Return to main page and re-submit details</font></p>";
  fail_response += "   <p><font face='Helvetica, Arial, sans-serif'><a href='index.htm'>Return to main page</a></font></p>";
  fail_response += " </body>";
  fail_response += "</html>";  

  #ifdef DEBUG_WEB
  Serial.println("handleNetworkConfig");
  #endif
  String strMQTTBrokerIPAddress=server.arg("MQTTBrokerIPAddress");
  String strMQTTBrokerPort=server.arg("MQTTBrokerPort");
  String strMQTTBrokerConnectionAttempts=server.arg("MQTTBrokerConnectionAttempts");
  String strNetworkSSID=server.arg("NetworkSSID");
  String strNetworkPassword=server.arg("NetworkPassword");
  String strNetworkConnectionAttempts=server.arg("NetworkConnectionAttempts");

  strMQTTBrokerIPAddress.trim();
  strMQTTBrokerPort.trim();
  strMQTTBrokerConnectionAttempts.trim();
  strNetworkSSID.trim();
  strNetworkPassword.trim();
  strNetworkConnectionAttempts.trim();

  strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
  if (! isValidIpv4Address((char *)strMQTTBrokerIPAddress.c_str())) {
    returnOK(fail_response);
  } else {
    //strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
    if (! isValidNumber(strMQTTBrokerPort)) {
      returnOK(fail_response);
    } else {
      tmp_mqtt_broker_port = strMQTTBrokerPort.toInt();
      if (((strNetworkSSID.length() == 0)     || (strNetworkSSID.length() >= NETWORK_SSID_STRING_MAX_LEN)) || 
          ((strNetworkPassword.length() == 0) || (strNetworkPassword.length() >= NETWORK_PASSWORD_STRING_MAX_LEN))) {
        returnOK(fail_response);
      } else {
        strcpy(tmp_sta_network_ssid,strNetworkSSID.c_str());
        strcpy(tmp_sta_network_password,strNetworkPassword.c_str());

        if (! isValidNumber(strMQTTBrokerConnectionAttempts)) {
          returnOK(fail_response);
        } else {
          tmp_mqtt_broker_connection_attempts = strMQTTBrokerConnectionAttempts.toInt();
          if ((tmp_mqtt_broker_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_mqtt_broker_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
            returnOK(fail_response);
          } else {
            if (! isValidNumber(strNetworkConnectionAttempts)) {
              returnOK(fail_response);
            } else {
              tmp_network_connection_attempts = strNetworkConnectionAttempts.toInt();
              if ((tmp_network_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_network_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
                returnOK(fail_response);
              } else {
                strcpy(mqtt_broker_ip,tmp_mqtt_broker_ip);
                mqtt_broker_port = tmp_mqtt_broker_port;
                mqtt_broker_connection_attempts = tmp_mqtt_broker_connection_attempts;
                strcpy(sta_network_ssid,tmp_sta_network_ssid);
                strcpy(sta_network_password,tmp_sta_network_password);
                network_connection_attempts = tmp_network_connection_attempts;
                bBrokerPresent = true;
                // Save new network parameters
                File f = SD.open(SECURITY_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
                if (f) {
                  fileWrite(f, SecurityVarArray,(int)(sizeof(SecurityVarArray)/sizeof(tsFileVarInstance)));
                  f.close();
                }      
                returnOK(pass_response);
                #ifdef DEBUG_STATE_CHANGE
                SHOW_UPDATED_STATE(eIRDEVICESTATE_STATE,eIRDEVICESTATE_INIT,"handleNetworkConfig");
                #endif
                eIRDEVICESTATE_STATE = eIRDEVICESTATE_INIT;
              }
            }
          }
        }
      }
    }
  }
 
  #ifdef DEBUG_WEB
  Serial.print("MQTTBrokerIPAddress : "); Serial.println(mqtt_broker_ip);
  Serial.print("MQTTBrokerPort : "); Serial.println(mqtt_broker_port);
  Serial.print("MQTTBrokerConnectionAttempts : "); Serial.println(mqtt_broker_connection_attempts);
  Serial.print("STANetworkSSID : "); Serial.println(sta_network_ssid);
  Serial.print("STANetworkPassword : "); Serial.println(sta_network_password);
  Serial.print("NetworkConnectionAttempts : "); Serial.println(network_connection_attempts);
  #endif
  return;
}



/*
 * http://www.esp8266.com/viewtopic.php?f=29&t=2153
 * 
Processing arguments of GET and POST requests is also easy enough. Let's make our sketch turn a led on or off depending on the value of a request argument.
http://<ip address>/led?state=on will turn the led ON
http://<ip address>/led?state=off will turn the led OFF
CODE: SELECT ALL
server.on("/led", []() {
  String state=server.arg("state");
  if (state == "on") digitalWrite(13, LOW);
  else if (state == "off") digitalWrite(13, HIGH);
  server.send(200, "text/plain", "Led is now " + state);
});
- See more at: http://www.esp8266.com/viewtopic.php?f=29&t=2153#sthash.7O0kU5VW.dpuf
 */

void handleNotFound(){
  #ifdef DEBUG_WEB
  Serial.println("handleNotFound");
  #endif
  if(hasSD && loadFromSD(server.uri())) return;
  String message = "SD Not Detected\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " NAME:"+server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  #ifdef DEBUG_WEB
  Serial.print(message);
  #endif
}



boolean isFloat(String tString) {
  String tBuf;
  boolean decPt = false;
  
  if(tString.charAt(0) == '+' || tString.charAt(0) == '-') tBuf = &tString[1];
  else tBuf = tString;  

  for(int x=0;x<tBuf.length();x++)
  {
    if(tBuf.charAt(x) == '.') {
      if(decPt) return false;
      else decPt = true;  
    }    
    else if(tBuf.charAt(x) < '0' || tBuf.charAt(x) > '9') return false;
  }
  return true;
}



bool isHex(char *str)
{
  const char hexarray[] = "0123456789aAbBcCdDeEfFxX";
  bool bFlag = false;
  if (strlen(str)>0)
  {
    if ((stristr((const char *)str, (const char *) "0x") != NULL) || (stristr((const char *)str, (const char *) "0X") != NULL)) {
      char *ptr = str;
      while (*ptr)
      {
        bFlag = false;
        for(int index = 0; index < (int)strlen(hexarray); index++)
        {
          if ((!bFlag) && (*ptr == hexarray[index]))
          {
            bFlag = true;
            break;
          }
        }
        if (!bFlag)
          return bFlag;
        ptr++;
      }
    }
  }
  return bFlag;
}



boolean isValidNumber(String str){
   for(byte i=0;i<str.length();i++)
   {
      if(!isDigit(str.charAt(i))) return false;
   }
   return true;
} 



bool isValidIpv4Address(char *st)
{
    int num, i, len;
    char *ch;

    //counting number of quads present in a given IP address
    int quadsCnt=0;

    #ifdef DEBUG_VALIDATION
    Serial.print("Split IP: ");
    Serial.println(st);
    #endif
    len = strlen(st);

    //  Check if the string is valid
    if(len<7 || len>15)
        return false;

    ch = strtok(st, ".");

    while (ch != NULL) 
    {
        quadsCnt++;
        #ifdef DEBUG_VALIDATION
        Serial.print("Quald ");
        Serial.print(quadsCnt);
        Serial.print(" is ");
        Serial.println(ch);
        #endif

        num = 0;
        i = 0;

        //  Get the current token and convert to an integer value
        while(ch[i]!='\0')
        {
            num = num*10;
            num = num+(ch[i]-'0');
            i++;
        }

        if(num<0 || num>255)
        {
            #ifdef DEBUG_VALIDATION
            Serial.println("Not a valid ip");
            #endif
            return false;
        }

        if( (quadsCnt == 1 && num == 0) || (quadsCnt == 4 && num == 0))
        {
            #ifdef DEBUG_VALIDATION
            Serial.print("Not a valid ip, quad: ");
            Serial.print(quadsCnt);
            Serial.print(" AND/OR quad: ");
            Serial.print(quadsCnt);
            Serial.println(" is zero");
            #endif
            return false;
        }

        ch = strtok(NULL, ".");
    }

    //  Check the address string, should be n.n.n.n format
    if(quadsCnt!=4)
    {
        return false;
    }

    //  Looks like a valid IP address
    return true;
}



#if defined(DEBUG_GENERAL)
//
// Format a floating point value with number of decimal places.
// The 'precision' parameter is a number from 0 to 6 indicating the desired decimal places.
// The 'buf' parameter points to a buffer to receive the formatted string.  This must be
// sufficiently large to contain the resulting string.  The buffer's length may be
// optionally specified.  If it is given, the maximum length of the generated string
// will be one less than the specified value.
//
// example: fmtDouble(3.1415, 2, buf); // produces 3.14 (two decimal places)
//
char *fmtDouble(double val, byte precision, char *buf, unsigned bufLen)
{
  char *tmpBufPtr = buf;
  
 if (!buf || !bufLen)
   return NULL;

 // limit the precision to the maximum allowed value
 const byte maxPrecision = 6;
 if (precision > maxPrecision)
   precision = maxPrecision;

 if (--bufLen > 0)
 {
   // check for a negative value
   if (val < 0.0)
   {
     val = -val;
     *buf = '-';
     bufLen--;
   }

   // compute the rounding factor and fractional multiplier
   double roundingFactor = 0.5;
   unsigned long mult = 1;
   for (byte i = 0; i < precision; i++)
   {
     roundingFactor /= 10.0;
     mult *= 10;
   }

   if (bufLen > 0)
   {
     // apply the rounding factor
     val += roundingFactor;

     // add the integral portion to the buffer
     unsigned len = fmtUnsigned((unsigned long)val, buf, bufLen, precision);
     buf += len;
     bufLen -= len;
   }

   // handle the fractional portion
   if ((precision > 0) && (bufLen > 0))
   {
     *buf++ = '.';
     if (--bufLen > 0)
       buf += fmtUnsigned((unsigned long)((val - (unsigned long)val) * mult), buf, bufLen, precision);
   }
 }

 // null-terminate the string
 *buf = '\0';
 return tmpBufPtr;
}



//
// Produce a formatted string in a buffer corresponding to the value provided.
// If the 'width' parameter is non-zero, the value will be padded with leading
// zeroes to achieve the specified width.  The number of characters added to
// the buffer (not including the null termination) is returned.
//
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen, byte width)
{
 if (!buf || !bufLen)
   return(0);

 // produce the digit string (backwards in the digit buffer)
 char dbuf[10];
 unsigned idx = 0;
 while (idx < sizeof(dbuf))
 {
   dbuf[idx++] = (val % 10) + '0';
   if ((val /= 10) == 0)
     break;
 }

 // copy the optional leading zeroes and digits to the target buffer
 unsigned len = 0;
 byte padding = (width > idx) ? width - idx : 0;
 char c = '0';
 while ((--bufLen > 0) && (idx || padding))
 {
   if (padding)
     padding--;
   else
     c = dbuf[--idx];
   *buf++ = c;
   len++;
 }

 // add the null termination
 *buf = '\0';
 return(len);
}
#endif



int extractAliasValue(char *strAliasValue, char *strAlias, char *strValue)
{
  int iParmCount = 0;
  if ((strlen(strAliasValue) > (MAX_ALIAS_STRING + 1 + MAX_VALUE_STRING)) || (strlen(strAliasValue) < 3))
    return iParmCount;
  iParmCount = parseString(ALIAS_VALUE_SEPARATOR, strAliasValue, strAlias, strValue);
  if (!isNumeric(strValue))
    return (int)0;
  #ifdef DEBUG_PARSER
  {
    char tmpStr[200];
    sprintf(tmpStr,"extractAliasCompound : [%s,%s], PCount %d",strAlias,strValue,iParmCount);
    Serial.println(tmpStr);
  }
  #endif
  return iParmCount;
}



int parseString(char *delimiter, char *source, ...)
{
   va_list arg_ptr;
   int argCount = 0;
   int args = 0;
   char *srcPtr = source;
   char *destPtr;

   if (strlen(source) > 1) {
     va_start(arg_ptr, source);
     while(*srcPtr)
     {
       destPtr = va_arg(arg_ptr, char *);
       while((*srcPtr) && (*srcPtr != *delimiter))
       {
        *destPtr++ = *srcPtr++;
        *destPtr = '\0';
       }
       if ((*srcPtr == *delimiter) || (*srcPtr == '\0')){
         if (*srcPtr == *delimiter) srcPtr++;
         argCount++;
       }
     }
     va_end(arg_ptr);
   }
   return argCount;
}



// This is an in place algorithm, requires no duplicated stack space
// The routine will strip spaces from the supplied string and remove 
// everything to the right of the ALIAS_VALUE_COMMENT_CHAR
void stripComments(char *strInput)
{
   char *srcPtr  = strInput;
   char *destPtr = strInput;
   bool bCommentFound = false;

   if (strlen(strInput) > 0) {
     while(*srcPtr && !bCommentFound)
     {
       if (*srcPtr == ALIAS_VALUE_COMMENT_CHAR){
        *destPtr = '\0';
        bCommentFound = true;
       }
     
       if ((!bCommentFound) && (*srcPtr != ' ')){
        *destPtr++ = *srcPtr;
       }
       srcPtr++;
     }
     *destPtr = '\0';
   }
}



// This is an in place algorithm, requires no duplicated stack space
// The routine will strip spaces from the supplied string
void stripSpaces(char *strInput)
{
   char *srcPtr  = strInput;
   char *destPtr = strInput;

   if (strlen(strInput) > 0) {
     while(*srcPtr)
     {
       if (*srcPtr != ' ')
        *destPtr++ = *srcPtr;
       srcPtr++;
     }
     *destPtr = '\0';
   }
}



// Determines if a string is a representation of a decimal number
// Ripped from here; https://stackoverflow.com/questions/17292545/how-to-check-if-the-input-is-a-number-or-not-in-c
bool isNumeric(const char *str) 
{
    while(*str != '\0')
    {
        if(*str < '0' || *str > '9')
            return false;
        str++;
    }
    return true;
}



int parmCount(char *strParameterString)
{
  int iPSepCount = 0;
  char *ptrSource = strParameterString;
  if (strlen(strParameterString) > 4)  // Account for minimum parameters of 3, meaning at least 5 chars (3 parameters separated by 2 commas. X,Y,Z)
  {
    while(*ptrSource)
    {
      if ((*ptrSource++) == PARAMETER_SEPARATOR)
        iPSepCount++;
    }
    if (iPSepCount < 2) 
      iPSepCount = 0;
    else
      iPSepCount += 1; // Now becomes the number of parameters. ie Parm Sep + 1.
  }
  return iPSepCount;
}



// Case insensitive string search.
// Ripped from here and tweaked; https://stackoverflow.com/questions/27303062/strstr-function-like-that-ignores-upper-or-lower-case
char *stristr(const char *haystack, const char *needle) {
    int c = tolower((unsigned char)*needle);
    if (c == '\0')
        return (char *)haystack;
    for (; *haystack; haystack++) {
        if (tolower((unsigned char)*haystack) == c) {
            for (uint16_t i = 0;;) {
                if (needle[++i] == '\0')
                    return (char *)haystack;
                if (tolower((unsigned char)haystack[i]) != tolower((unsigned char)needle[i]))
                    break;
            }
        }
    }
    return NULL;
}



int findRemote(remoteControlType *remoteControls, int iMaxRemotes, char *cRemoteName)
{
  for (int index = 0; index < iMaxRemotes; index++)
  {
    //if (stristr((const char *) (*(remoteControls + index)->sRemoteName), (const char *)cRemoteName))
    if (stristr((const char *) (remoteControls[index].sRemoteName), (const char *)cRemoteName) && 
      (strlen((const char *) (remoteControls[index].sRemoteName)) == strlen((const char *)cRemoteName)))
      return index;
    yield();
  }
  return (int)-1;
}



int findRemoteButton(remoteControlButtonType *remoteControlButtonCollection, uint16_t iMaxRemoteButtons, char *cButtonName)
{
  for (int index = 0; index < iMaxRemoteButtons; index++)
  {
    //if (stristr((const char *) (*(remoteControlButtonCollection + index)->sButtonName), (const char *)sButtonName))
    if (stristr((const char *) (remoteControlButtonCollection[index].sButtonName), (const char *)cButtonName) &&
      (strlen((const char *) (remoteControlButtonCollection[index].sButtonName)) == strlen((const char *)cButtonName)))
      return index;
    yield();
  }
  return (int)-1;
}



int findAliasValue(sAliasValueInstance *ptrHeadOfAliasValueInstancesX, char *cAliasName, char **cValueName)
{
  int iIndex = 0;

  sAliasValueInstance *tmpPtr = ptrHeadOfAliasValueInstances;
  do {
      if ((stristr((const char *) (tmpPtr->strAlias), (const char *)cAliasName)) &&
         (strlen((const char *) (tmpPtr->strAlias)) == strlen((const char *)cAliasName)))
      {
        *cValueName = tmpPtr->strValue;
        return iIndex;
      }
      iIndex++;
      aliasValueListGetNext(&tmpPtr);
      yield();
  } while (!aliasValueListBegining(ptrHeadOfAliasValueInstances,tmpPtr));
  return (int)-1;
}



void readAliasValueInput(void){
  char strAlias[MAX_ALIAS_STRING * 2], strValue[MAX_VALUE_STRING *2];
  char strAliasValue[(MAX_ALIAS_STRING * 2) + 1 + (MAX_VALUE_STRING * 2)];
  String sAliasValue;
  bool bOk = true;
  int iExtractAliasValue = 0;
  int index = 0;
  bValueAliasPresent = false;
  File f = SD.open(ALIAS_VALUE_INPUT_FILE, SD_FILE_READ_MODE);
  if (!f) {
    #ifdef DEBUG_GENERAL
    Serial.println("Failed to read Alias/Value Input Vals.");
    #endif
    return;
  } else {
      #ifdef DEBUG_ALVAL
      Serial.println("readAliasValueInput");
      Serial.print("From : "); Serial.println(ALIAS_VALUE_INPUT_FILE);
      #endif
      index = 0;  // For debug, total entries.
      bOk = true;
      while ((f.position() < f.size()) && (bOk))
      {
        sAliasValue=f.readStringUntil('\n'); sAliasValue.trim(); // Read in A/V and remove trailing and leading spaces
        if (sAliasValue.length() > (MAX_ALIAS_STRING + 1 + MAX_VALUE_STRING)) // Truncate if the string is bigger the max allowable size
          sAliasValue.remove(MAX_ALIAS_STRING + 1 + MAX_VALUE_STRING);
        if (sAliasValue.length() > 3) { // At least 'A,V'
          strcpy(strAliasValue,sAliasValue.c_str());
          stripComments(strAliasValue);
          if (strlen(strAliasValue) > 3) {
          
            iExtractAliasValue = extractAliasValue(strAliasValue, strAlias, strValue);
            
            #ifdef DEBUG_ALVAL
            Serial.print("strAlias/strValue : '"); Serial.print(sAliasValue); Serial.print("','"); Serial.print(strAlias); Serial.print("','"); Serial.print(strValue); Serial.println("'");
            #endif
    
            if (iExtractAliasValue == 2) {
              if (((strlen(strAlias) > 0) && (strlen(strAlias) <= (MAX_ALIAS_STRING))) &&
                  ((strlen(strValue) > 0) && (strlen(strValue) <= (MAX_VALUE_STRING))) &&
                  (addItem(&ptrHeadOfAliasValueInstances, strAlias, strValue) != NULL)) {
                  index++;
                  bValueAliasPresent = true;
              } else 
                  bOk = false;
              
              #ifdef DEBUG_ALVAL
              Serial.println(sAliasValue);
              Serial.println(strAlias);
              Serial.println(strValue);
              #endif
            }
            yield();
          }          
        }
      }
      f.close();
  }
  
  #ifdef DEBUG_ALVAL
  {
    sAliasValueInstance *tmpPtr = ptrHeadOfAliasValueInstances;
    int x = 1;
    Serial.print("Total Entries # : "); Serial.println(index);
    do {
      Serial.print("Entry #   : "); Serial.println(x);
      Serial.print("Alias     : "); Serial.println(tmpPtr->strAlias);
      Serial.print("Value     : "); Serial.println(tmpPtr->strValue);
      x++;
      aliasValueListGetNext(&tmpPtr);
      yield();
    } while (!aliasValueListBegining(ptrHeadOfAliasValueInstances,tmpPtr));
  }
  #endif
}



sAliasValueInstance * addItem(sAliasValueInstance **Head, const char *sAlias, const char *sValue)
{
  sAliasValueInstance *Tail = (sAliasValueInstance *) malloc(sizeof(sAliasValueInstance));
  sAliasValueInstance *ptrSITmp;
  if (!(Tail == NULL)) {
    strcpy(Tail->strAlias,sAlias);
    strcpy(Tail->strValue,sValue);
    if (*Head == NULL) {  // Uninitialised head pointer
      Tail->next     = Tail;
      Tail->previous = Tail;
      *Head = Tail;
    } else {            // Entry already exists in the list, add to the end of the list
      ptrSITmp = (*Head)->previous;
      ptrSITmp->next = Tail;
      Tail->next = *Head;
      Tail->previous = ptrSITmp;
      (*Head)->previous = Tail;
    }
  } else {  // No memory available to allocate
    return ((sAliasValueInstance *)NULL);
  }
  return (sAliasValueInstance *) Tail;
}



boolean aliasValueListEnd(sAliasValueInstance *Head, sAliasValueInstance *Tail)
{
  return (((Head == Tail->next) && (Head != Tail)) || ((Head == Tail->next) && (Head == Tail->previous) && (Head == Tail)));
}



boolean aliasValueListBegining(sAliasValueInstance *Head, sAliasValueInstance *Tail)
{
  return (Head == Tail);
}



sAliasValueInstance * aliasValueListGetNext(sAliasValueInstance **tmpPtr)
{
  *tmpPtr = (*tmpPtr)->next;
  return (*tmpPtr);
}



sAliasValueInstance * aliasValueListGetPrevious(sAliasValueInstance **tmpPtr)
{
  *tmpPtr = (*tmpPtr)->previous;
  return (*tmpPtr);
}

