#include <AsyncMqttClient.h>
#include <WiFi.h>

#define ON 1
#define OFF 0
#define JETS_RUNNING 2

#define HEATER_PIN 6
#define PUMP1_LOW_PIN 9
#define PUMP1_HIGH_PIN 10
#define PUMP2_LOW_PIN 11
#define PUMP2_HIGH_PIN 12
#define CIRCULATION_PUMP_PIN 7
#define OZONE_PIN 8

//Spa light pin (using PWM for brightness control)
#define LIGHTS_PIN 47
const int channel = 0;
const int freq = 5000;
const int resolution = 8;
//Spa light PWM brightness levels (0-255)
#define lightLow 50
#define lightMedium 140
#define lightHigh 220 

#define FLOW_SENSOR_PIN 41
#define ONE_WIRE_BUS_WATER_PIN 21
#define ONE_WIRE_BUS_HIGH_LIMIT_PIN 5

// FastLED setup for built-in LED WIFI status indication
#define LED_PIN 48 // For the built-in WS2812 LED
#define NUM_LEDS 1
#define BRIGHTNESS 20

const int HIGH_LIMIT = 119;
int ERROR_STATE = 0; 
// 0 All systems active
// 1 ERROR: No flow detected from circulation pump!
// 2 ERROR: Flow before circulation pump turned on!
// 3 ERROR: High limit temperature exceeded! Heater turned off!
// 4 ERROR: Temperature sensor fault (disconnected or out of range)! Heater turned off!

// Valid sensor reading range in Fahrenheit; anything outside is treated as a bad reading
const float SENSOR_MIN_VALID_F = 32.0;
const float SENSOR_MAX_VALID_F = 140.0;
const int SENSOR_MAX_BAD_READS = 3;
const int SENSOR_STABILIZE_ATTEMPTS = 30;  // ~30s at boot before giving up with error 4   // consecutive bad reads before latching error 4
const unsigned long NO_FLOW_ERROR_DELAY = 5000;  // no flow must persist this long (ms) before latching error 1

int TEMP_SETPOINT = 71; // Desired temperature in Fahrenheit
const float TEMP_HYSTERESIS = 0.5; // Hysteresis in Fahrenheit

const char* WIFI_SSID = "DropItLikeItsAHotspot";
const char* WIFI_PASSWORD = "flintbomb";
const char* WIFI_HOSTNAME = "spa-pack";
const unsigned long WIFI_RETRY_INTERVAL = 30*1000UL;     // force a fresh connect attempt this often while down
const unsigned long WIFI_REBOOT_TIMEOUT = 15*60*1000UL;  // restart if WiFi down this long (only when jets/light are off)
const int HTTP_PORT = 80;
IPAddress local_IP(10, 69, 69, 111);       // Set your desired static IP
IPAddress gateway(10, 69, 69, 1);         // Set your router's IP address
IPAddress subnet(255, 255, 255, 0);        // Set your subnet mask
IPAddress primaryDNS(8, 8, 8, 8);          // Optional: Set a primary DNS server
IPAddress secondaryDNS(8, 8, 4, 4);        // Optional: Set a secondary DNS server


#define MQTT_HOST IPAddress(10, 69, 69, 100)
#define MQTT_PORT 1883
const char* MQTT_USER = "mqtt_login";
const char* MQTT_PASSWORD = "flint";

//mqtt topics
const char* waterTempTopic = "spa/water_temp";
const char* setpointTempTopic = "spa/setpoint_temp";
const char* upButtonTopic = "spa/up_button";
const char* downButtonTopic = "spa/down_button";
const char* jet1buttonTopic = "spa/jet1_button"; 
const char* jet2buttonTopic = "spa/jet2_button";
const char* fixALeakSetTopic = "spa/fix_a_leak/set";      // payload ON/OFF starts/stops the Fix-A-Leak program
const char* fixALeakStateTopic = "spa/fix_a_leak/state";
const char* fixALeakDiscoveryTopic = "homeassistant/switch/spa_pack/fix_a_leak/config";  // Home Assistant MQTT discovery
// retired 12 hour select; cleared on connect so Home Assistant removes it
const char* oldJets12hDiscoveryTopic = "homeassistant/select/spa_pack/jets_12h/config";
const char* oldJets12hStateTopic = "spa/jets_12h_state";
const char* lightButtonTopic = "spa/light_button";  
const char* heaterStateTopic = "spa/heater_state";
const char* jet1StateTopic = "spa/jet1_state";
const char* jet2StateTopic = "spa/jet2_state";
const char* lightStateTopic = "spa/light_state";
const char* highLimitTopic = "spa/high_limit";
const char* errorStateTopic = "spa/status";

//Filtration cycle info
int FILTER_DURATION = 30; // in minutes
int FILTER_START_HOUR = 8; // 8 AM
int FILTER_START_MINUTE = 0;
unsigned long jetMaxRunTime = 30*60*1000UL;    // max time jets run for
// Fix-A-Leak program: both pumps HIGH then LOW, repeating for the full duration
const unsigned long FIX_A_LEAK_HIGH_TIME = 15*60*1000UL;    // 15 min on HIGH
const unsigned long FIX_A_LEAK_LOW_TIME = 105*60*1000UL;    // 1 hr 45 min on LOW
const unsigned long FIX_A_LEAK_DURATION = 12*60*60*1000UL;  // 12 hours total
unsigned long lightMaxRunTime = 60*60*1000UL;  // max time light runs for

//global control variable declarations
int jet1State = 0;
int jet2State = 0;
int lightState = 0;
int heaterState = 0;
float currentTemp = 0.0;
float highLimitTemp = 0.0;
unsigned long wifiLastAttempt = 0;
unsigned long wifiDownSince = 0;
// run timers store start time + duration so they survive millis() rollover
unsigned long jet1StartTime = 0;
unsigned long jet1RunTime = 0;
unsigned long jet2StartTime = 0;
unsigned long jet2RunTime = 0;
unsigned long lightStartTime = 0;
bool filterCycleRunning = false;
bool fixALeakActive = false;
unsigned long fixALeakStartTime = 0;
bool waterSensorOk = true;
bool highLimitSensorOk = true;
int waterSensorBadReads = 0;
int highLimitSensorBadReads = 0;
unsigned long noFlowSince = 0;   // 0 = flow currently detected


void outputPrintln(const String &msg);
void outputPrint(const String &msg);
//void setupWebServer();
void checkCirculationPump();
void checkTemperature();
bool flowDetected();

void reconnect();
void onMqttPublish(uint16_t packetId);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttConnect(bool sessionPresent);
void connectToMqtt();
void connectToWiFi();
void WiFiEvent(WiFiEvent_t event);
void checkWiFi();
void updateOzone();

void checkHighLimitSensor();
void checkWaterTemperature();
void publishCurrentTemperature();
void publishSetpointTemperature();
void publishErrorState();  
void publishHeaterState();
void setJet1State(unsigned long runTime = jetMaxRunTime);
void setJet2State(unsigned long runTime = jetMaxRunTime);
void forceHeaterOff();
void enterErrorState(int code, const String &info);
bool intervalElapsed(unsigned long &lastRun, unsigned long interval);
bool validSensorTemp(float tempF);
void publishFixALeakState();
void startFixALeak();
void stopFixALeak();
void cancelFixALeak();
void checkFixALeak();
void publishDiscovery();
void stabilizeSensors();
void setSpaLight(int brightness);
void checkJetTimers();
void checkFilterCycle();
void getNTPTime();