#include <Arduino.h>
#include "main.h"
#include <AsyncTCP.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h> 
#include <DallasTemperature.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <arduinoJson.h>
#include <deque>
#include <vector>
#include "log_page.h"
#include <FastLED.h>
#include <NTPClient.h>
#include <esp_task_wdt.h>

//TODO
//Pump logic on button press
//send temperature and heater state to mqtt periodically
//format mqtt messages as json

// ERROR INFO STRING
String ERROR_INFO = "";

//initialize temperature smoothing array;
float previousHighLimitTemp[6] = {0};

//NVM Preferances 
Preferences preferences;

// mqtt declarations
AsyncMqttClient mqttClient;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

// live log buffer (shared between tasks, guarded by logMutex)
SemaphoreHandle_t logMutex = NULL;
std::deque<String> logHistory;
uint32_t logSeq = 0;
String logPartial = "";
String pendingCommand = "";

// NTP Client setup for time keeping
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -3600 * 8); // Adjust for your timezone (e.g., -7 hours for PST)

// Setup oneWire instances to communicate with temperature sensors
OneWire oneWire1(ONE_WIRE_BUS_WATER_PIN);
OneWire oneWire2(ONE_WIRE_BUS_HIGH_LIMIT_PIN);

// Pass our oneWire reference to the Dallas Temperature sensor
DallasTemperature waterSensor(&oneWire1);
DallasTemperature highLimitSensor(&oneWire2);

// FastLED setup for built-in LED WIFI status indication
CRGB leds[NUM_LEDS]; // Creates an array to hold the LED data

void setup() {
  Serial.begin(115200);
  logMutex = xSemaphoreCreateMutex();

  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, OFF);
  pinMode(PUMP1_LOW_PIN, OUTPUT);
  digitalWrite(PUMP1_LOW_PIN, OFF); 
  pinMode(PUMP1_HIGH_PIN, OUTPUT);
  digitalWrite(PUMP1_HIGH_PIN, OFF);
  pinMode(PUMP2_LOW_PIN, OUTPUT);
  digitalWrite(PUMP2_LOW_PIN, OFF);
  pinMode(PUMP2_HIGH_PIN, OUTPUT);
  digitalWrite(PUMP2_HIGH_PIN, OFF);
  pinMode(LIGHTS_PIN, OUTPUT);
  digitalWrite(LIGHTS_PIN, OFF);  
  pinMode(CIRCULATION_PUMP_PIN, OUTPUT);
  digitalWrite(CIRCULATION_PUMP_PIN, OFF); 
  pinMode(OZONE_PIN, OUTPUT);
  digitalWrite(OZONE_PIN, OFF);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);

  // Configure Spa Light
  ledcSetup(channel, freq, resolution);
  // Attach the pin to the channel
  ledcAttachPin(LIGHTS_PIN, channel);
   
  // get preferences
  outputPrintln(F("Reading NVM"));
  preferences.begin("states", false);
  TEMP_SETPOINT = preferences.getInt("setPoint", TEMP_SETPOINT);
  FILTER_START_HOUR = preferences.getInt("filterHour", FILTER_START_HOUR);
  FILTER_START_MINUTE = preferences.getInt("filterMinute", FILTER_START_MINUTE);
  FILTER_DURATION = preferences.getInt("filterDuration", FILTER_DURATION);
  outputPrintln("Stored setpoint " + String(TEMP_SETPOINT) + "F, filter " + String(FILTER_START_HOUR) + ":" +
                (FILTER_START_MINUTE < 10 ? "0" : "") + String(FILTER_START_MINUTE) + " for " + String(FILTER_DURATION) + " min");
  preferences.end();

  //Set initial LED state: red until WiFi connects
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  leds[0] = CRGB::Red;
  FastLED.show();
  outputPrintln(String("Reset reason: ") + resetReasonString());
  loadPreviousBootInfo();

  // Start up the DallasTemperature library
  highLimitSensor.begin();
  waterSensor.begin();

  //Activate and check circulation pump, if bad flow sensor or no flow detected sets error state TRUE
  checkCirculationPump(); //Check if circulation pump is running

  //Setup MQTT Server and Callbacks
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);

  //connect to WiFi 
  WiFi.persistent(false);     // don't rewrite flash on every connect
  WiFi.setHostname(WIFI_HOSTNAME);   // must be set before WiFi.mode()
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);       // no modem power save: faster response and reliable OTA
  WiFi.setAutoReconnect(true);
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);     // scan every channel and
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL); // join the strongest access point, not the first one found
  WiFi.onEvent(WiFiEvent);
  connectToWiFi();

  //setup live log page and websocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", LOG_PAGE_HTML);
  });
  server.on("/webserial", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/");   // old WebSerial bookmark
  });
  server.begin();

  //Arduino OTA Setup
    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      outputPrintln("Start updating " + type);
    })
    .onEnd([]() {
      safeRestart("OTA update installed");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      //outputPrintln("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      //outputPrintf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        outputPrintln("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        outputPrintln("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        outputPrintln("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        outputPrintln("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        outputPrintln("End Failed");
      }
    });

  ArduinoOTA.begin();

  // Initialize NTP client
  timeClient.begin();

  //Stabilize temperature sensors
  stabilizeSensors();

  // restart automatically if loop() ever hangs
  esp_task_wdt_init(LOOP_WATCHDOG_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
}

void loop() {
  esp_task_wdt_reset();

  checkWiFi();
  checkMqtt();
  updateStatusLed();
  flushLogs();

  // run commands from the live log page here, not in the network task
  String cmd;
  xSemaphoreTake(logMutex, portMAX_DELAY);
  cmd = pendingCommand;
  pendingCommand = "";
  xSemaphoreGive(logMutex);
  if (cmd.length()) handleCommand(cmd);

  static unsigned long lastDiagnostics = 0;
  if (intervalElapsed(lastDiagnostics, 60000)) publishDiagnostics();

  // update NTP time every 60 seconds (from loop, not a timer/event task)
  static unsigned long lastNtpUpdate = 0;
  if (WiFi.isConnected() && intervalElapsed(lastNtpUpdate, 60000)) getNTPTime();
  //Handle Arduino OTA
  ArduinoOTA.handle();

  //HOT TUB LOGIC **************************************************************************
  
  // send Error state every 30 seconds
  static unsigned long lastErrorPublish = 0;
  if (intervalElapsed(lastErrorPublish, 30000)) publishErrorState();

  // Check high limit sensor
  checkHighLimitSensor();

  // Check water temperature
  checkWaterTemperature();

  //check if jets have been on for 30min and turn off
  checkJetTimers();

  // step the Fix-A-Leak program between HIGH and LOW
  checkFixALeak();

  // If error state is false, check temperature and control heater
  if (ERROR_STATE == 0) {
    
    // update NTP time and cHeck if filter cycle is due to begin
    checkFilterCycle();

    bool flow = flowDetected();
    //only run heater if no jet pumps are on and flow is detected
    if (flow && jet1State == OFF && jet2State == OFF) {
      noFlowSince = 0;
      // check temperature and turn heater on or off as needed
      checkTemperature();
    } else if (jet1State > OFF || jet2State > OFF) {  // LOW or HIGH
      if (heaterState != JETS_RUNNING) {
        heaterState = JETS_RUNNING;
        digitalWrite(HEATER_PIN, OFF);
        publishHeaterState();
        outputPrintln(F("Heater turned off due to pump operation."));
      }
    } else {  // no flow with jets off
      // heater off immediately, latch error only if no flow persists
      forceHeaterOff();
      if (noFlowSince == 0) {
        noFlowSince = millis();
        outputPrintln(F("No flow detected, heater turned off."));
      } else if (millis() - noFlowSince >= NO_FLOW_ERROR_DELAY) {
        digitalWrite(CIRCULATION_PUMP_PIN, OFF);
        enterErrorState(1, "No flow detected from circulation pump");
      }
    }

  }
  updateOzone();

  // END HOT TUB LOGIC ********************************************************************8

}

// ---------------- Logging ----------------
// outputPrint/outputPrintln can be called from any task. Complete lines go into
// a mutex-protected buffer; loop() sends new lines to the live log page and to
// Home Assistant (MQTT), so the network libraries are only touched from loop().
void addLogLine(const String &msg) {
  String line = logTimestamp() + " ";
  if (ERROR_STATE > 0) line += "Error State: ";
  line += msg;
  Serial.println(line);
  if (logMutex == NULL) return;
  xSemaphoreTake(logMutex, portMAX_DELAY);
  logHistory.push_back(line);
  logSeq++;
  while (logHistory.size() > LOG_HISTORY_LINES) logHistory.pop_front();
  xSemaphoreGive(logMutex);
}

void outputPrintln(const String &msg) {
  String line;
  if (logMutex != NULL) xSemaphoreTake(logMutex, portMAX_DELAY);
  line = logPartial + msg;
  logPartial = "";
  if (logMutex != NULL) xSemaphoreGive(logMutex);
  addLogLine(line);
}

void outputPrint(const String &msg) {
  if (logMutex != NULL) xSemaphoreTake(logMutex, portMAX_DELAY);
  logPartial += msg;
  if (logPartial.length() > 200) logPartial = logPartial.substring(0, 200);
  if (logMutex != NULL) xSemaphoreGive(logMutex);
}

// "HH:MM:SS" once NTP has synced, otherwise uptime "+Ns"
String logTimestamp() {
  if (timeClient.isTimeSet()) return timeClient.getFormattedTime();
  return "+" + String(millis() / 1000) + "s";
}

// Called from loop(): push lines added since the last call to websocket clients and MQTT
void flushLogs() {
  static uint32_t sentSeq = 0;
  ws.cleanupClients();
  if (logMutex == NULL) return;

  std::vector<String> fresh;
  xSemaphoreTake(logMutex, portMAX_DELAY);
  uint32_t newCount = logSeq - sentSeq;
  if (newCount > logHistory.size()) newCount = logHistory.size();
  for (size_t i = logHistory.size() - newCount; i < logHistory.size(); i++) fresh.push_back(logHistory[i]);
  sentSeq = logSeq;
  xSemaphoreGive(logMutex);
  if (fresh.empty()) return;

  if (ws.count() > 0) {
    String batch;
    for (auto &l : fresh) { batch += l; batch += '\n'; }
    ws.textAll(batch);
  }
  if (mqttClient.connected()) {
    for (auto &l : fresh) {
      // routine publish chatter stays on the live page only
      if (l.indexOf("Publishing") >= 0) continue;
      String msg = l.length() > 250 ? l.substring(0, 250) : l;
      mqttClient.publish(logTopic, 0, false, msg.c_str());
    }
  }
}

// Commands typed on the live log page
void handleCommand(const String &d) {
  outputPrintln("Command: " + d);
  if (d.startsWith("r")) {
    safeRestart("restart command");
  } else if (d.startsWith("h") || d.startsWith("m")) {
    String numberString = "";
    for (int i = 0; i < d.length(); i++) {
      if (isDigit(d.charAt(i))) numberString += d.charAt(i);
    }
    if (numberString.length() == 0) return;
    int value = numberString.toInt();
    preferences.begin("states", false);
    if (d.startsWith("h") && value < 24) {
      FILTER_START_HOUR = value;
      preferences.putInt("filterHour", FILTER_START_HOUR);
      outputPrintln("Set filter start hour to: " + String(FILTER_START_HOUR));
    } else if (d.startsWith("m") && value < 60) {
      FILTER_START_MINUTE = value;
      preferences.putInt("filterMinute", FILTER_START_MINUTE);
      outputPrintln("Set filter start minute to: " + String(FILTER_START_MINUTE));
    } else {
      outputPrintln(F("Value out of range."));
    }
    preferences.end();
  } else {
    outputPrintln(F("Unknown command. Use r, h<hour>, m<minute>."));
  }
}

// Runs in the network task: send history to new clients, hand commands to loop()
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    String batch;
    xSemaphoreTake(logMutex, portMAX_DELAY);
    for (auto &l : logHistory) { batch += l; batch += '\n'; }
    xSemaphoreGive(logMutex);
    client->text(batch);
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String cmd;
      for (size_t i = 0; i < len && i < 64; i++) cmd += (char)data[i];
      cmd.trim();
      xSemaphoreTake(logMutex, portMAX_DELAY);
      pendingCommand = cmd;
      xSemaphoreGive(logMutex);
    }
  }
}

// Start a fresh connection attempt with the static IP.
// resetRadio fully restarts the WiFi radio first, clearing any stuck driver state.
void connectToWiFi(bool resetRadio) {
  wifiAttempts++;
  if (resetRadio) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
  } else {
    WiFi.disconnect();
  }
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    outputPrintln("STA Failed to configure");
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiLastAttempt = millis();
  outputPrintln(F("Connecting to WiFi..."));
}

// The only place reconnects are started. The driver's auto reconnect gets
// WIFI_RETRY_INTERVAL to recover before we force a fresh attempt, so attempts
// never cut each other off. Reboots if WiFi stays down too long and the
// jets/light aren't running.
void checkWiFi() {
  static unsigned long lastCheck = 0;
  if (!intervalElapsed(lastCheck, 1000)) return;

  // log WiFi events here rather than in the WiFi event task
  static unsigned long lastHistorySave = 0;
  if (wifiDisconnectEvent) {
    wifiDisconnectEvent = false;
    outputPrintln("WiFi disconnected: " + String(wifiReasonName(wifiLastDisconnectReason)) +
                  " (" + String(wifiLastDisconnectReason) + ")");
    // keep a record in flash in case this boot never reconnects (at most every 30s)
    if (millis() - lastHistorySave >= 30000) {
      lastHistorySave = millis();
      saveWifiHistory("not connected");
    }
  }
  if (wifiGotIpEvent) {
    wifiGotIpEvent = false;
    outputPrintln("WiFi connected after " + String(millis() / 1000) + "s uptime, attempt " + String(wifiAttempts) +
                  ", IP " + WiFi.localIP().toString() + ", signal " + String(WiFi.RSSI()) + " dBm");
    if (!wifiConnectedOnce) {
      wifiConnectedOnce = true;
      saveWifiHistory("connected");
    }
    mqttLastAttempt = millis() - MQTT_RETRY_INTERVAL;   // connect MQTT right away
  }

  if (WiFi.isConnected()) {
    wifiDownSince = 0;
    return;
  }
  unsigned long now = millis();
  if (wifiDownSince == 0) {
    wifiDownSince = now;
    // give the driver's own reconnect a short head start before forcing a fresh attempt
    wifiLastAttempt = now - WIFI_RETRY_INTERVAL + WIFI_FIRST_RETRY_DELAY;
  }

  if (now - wifiLastAttempt >= WIFI_RETRY_INTERVAL) {
    outputPrintln(F("WiFi still down, resetting radio and retrying."));
    connectToWiFi(true);
  }
  if (now - wifiDownSince >= WIFI_REBOOT_TIMEOUT && jet1State == OFF && jet2State == OFF && lightState == OFF) {
    safeRestart("WiFi down too long");
  }
}

void connectToMqtt() {
  outputPrintln("Connecting to MQTT...");
  mqttClient.connect();
}

// MQTT reconnects are driven from loop() only (no timer task)
void checkMqtt() {
  if (!WiFi.isConnected() || mqttClient.connected()) return;
  if (millis() - mqttLastAttempt < MQTT_RETRY_INTERVAL) return;
  mqttLastAttempt = millis();
  connectToMqtt();
}

// Status LED: blue = WiFi connected, red = not connected.
// With an error it flashes that colour: 3 quick flashes, pause, then one slow
// flash per error number (e.g. error 2 = 2 slow flashes), long pause, repeat.
void updateStatusLed() {
  static CRGB shown = CRGB(1, 2, 3);   // forces the first write
  CRGB color = WiFi.isConnected() ? CRGB::Blue : CRGB::Red;
  bool on = true;

  if (ERROR_STATE > 0) {
    const unsigned long QUICK = 150, SLOW = 600, GAP = 1000, END_PAUSE = 2500;
    unsigned long quickPart = 3 * 2 * QUICK;
    unsigned long slowPart = (unsigned long)ERROR_STATE * 2 * SLOW;
    unsigned long cycle = quickPart + GAP + slowPart + END_PAUSE;
    unsigned long t = millis() % cycle;
    if (t < quickPart) {
      on = (t / QUICK) % 2 == 0;
    } else if (t < quickPart + GAP) {
      on = false;
    } else if (t < quickPart + GAP + slowPart) {
      on = ((t - quickPart - GAP) / SLOW) % 2 == 0;
    } else {
      on = false;
    }
  }

  CRGB wanted = on ? color : CRGB(CRGB::Black);
  if (wanted != shown) {
    shown = wanted;
    leds[0] = wanted;
    FastLED.show();
  }
}

const char* resetReasonString() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "power on";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "crash";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "loop watchdog (hang)";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_BROWNOUT:  return "brownout (low power)";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_EXT:       return "external reset";
    default:                return "unknown";
  }
}

// Retained JSON on spa/diagnostics so the cause of a dropout is visible afterwards
void publishDiagnostics() {
  if (!mqttClient.connected()) return;
  JsonDocument doc;
  doc["uptime_min"] = millis() / 60000;
  doc["reset_reason"] = resetReasonString();
  doc["rssi"] = WiFi.RSSI();
  doc["wifi_drops"] = wifiDisconnectCount;
  doc["wifi_last_reason"] = wifiLastDisconnectReason;
  doc["mqtt_drops"] = mqttDisconnectCount;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["min_free_heap"] = ESP.getMinFreeHeap();
  doc["error"] = ERROR_STATE;
  doc["prev_restart"] = prevRestartWhy;
  doc["prev_boot_wifi"] = prevBootWifi;
  String output;
  serializeJson(doc, output);
  mqttClient.publish(diagnosticsTopic, 1, true, output.c_str());
}

// Restart after logging why, saying goodbye to MQTT and the WiFi router first
// so the router doesn't hold on to the old connection and refuse the rejoin.
void safeRestart(const String &why) {
  outputPrintln("Restarting: " + why);
  Preferences diag;   // own instance: the shared one is also used from the MQTT task
  diag.begin("diag", false);
  diag.putString("restartWhy", why);
  diag.end();
  saveWifiHistory("restarting");
  flushLogs();
  delay(200);
  mqttClient.disconnect();
  delay(200);
  WiFi.disconnect(true);   // sends a proper disconnect to the router
  delay(500);
  ESP.restart();
}

// WiFi state, attempts and recent disconnect reasons for this boot, saved to flash
void saveWifiHistory(const char* state) {
  String rec = String(state) + " at " + String(millis() / 1000) + "s, attempts " + String(wifiAttempts) +
               ", reset " + resetReasonString();
  uint32_t count = wifiDisconnectCount;
  if (count) {
    rec += ", reasons";
    uint32_t first = count > WIFI_REASON_LOG_SIZE ? count - WIFI_REASON_LOG_SIZE : 0;
    for (uint32_t i = first; i < count; i++) rec += " " + String(wifiReasonLog[i % WIFI_REASON_LOG_SIZE]);
  }
  Preferences diag;   // own instance: the shared one is also used from the MQTT task
  diag.begin("diag", false);
  diag.putString("wifi", rec);
  diag.end();
}

// Read what the previous boot recorded, log it, and clear the restart note
void loadPreviousBootInfo() {
  Preferences diag;   // own instance: the shared one is also used from the MQTT task
  diag.begin("diag", false);
  prevBootWifi = diag.getString("wifi", "none");
  prevRestartWhy = diag.getString("restartWhy", "");
  diag.remove("restartWhy");
  diag.end();
  if (prevRestartWhy.length() == 0) prevRestartWhy = resetReasonString();   // not a planned restart
  outputPrintln("Previous restart: " + prevRestartWhy);
  outputPrintln("Previous boot WiFi: " + prevBootWifi);
}

const char* wifiReasonName(uint8_t reason) {
  switch (reason) {
    case 2:   return "auth expired";
    case 3:   return "router kicked us (auth leave)";
    case 4:   return "assoc expired";
    case 5:   return "router full";
    case 8:   return "assoc leave";
    case 15:  return "4-way handshake timeout";
    case 200: return "beacon timeout (signal lost)";
    case 201: return "no AP found";
    case 202: return "auth failed (password?)";
    case 203: return "assoc failed";
    case 204: return "handshake timeout";
    case 205: return "connection failed";
    default:  return "other";
  }
}

// Runs in the WiFi event task: only set flags here, loop() does the work
void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch(event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        wifiGotIpEvent = true;
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        wifiLastDisconnectReason = info.wifi_sta_disconnected.reason;
        wifiReasonLog[wifiDisconnectCount % WIFI_REASON_LOG_SIZE] = wifiLastDisconnectReason;
        wifiDisconnectCount++;
        wifiDisconnectEvent = true;
        break;
    default:
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  outputPrintln(F("Connected to MQTT."));

  // Subscribe to topics
  mqttClient.subscribe(jet1buttonTopic, 1);
  mqttClient.subscribe(jet2buttonTopic, 1);
  mqttClient.subscribe(fixALeakSetTopic, 1);
  mqttClient.subscribe(upButtonTopic, 1);
  mqttClient.subscribe(downButtonTopic, 1);
  mqttClient.subscribe(lightButtonTopic, 1);
  outputPrintln(F("Subscribed to control topics."));

  // Announce the 12 hour run control to Home Assistant
  publishDiscovery();
  publishFixALeakState();

  // Publish initial setpoint temperature
  publishSetpointTemperature();
  publishCurrentTemperature();
  publishHeaterState();
  publishErrorState();
  publishDiagnostics();
  mqttClient.publish(lightStateTopic, 1, true, "OFF");
  mqttClient.publish(jet1StateTopic, 1, true, "OFF");
  mqttClient.publish(jet2StateTopic, 1, true, "OFF");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  outputPrintln("Disconnected from MQTT, reason " + String(static_cast<uint8_t>(reason)));
  mqttDisconnectCount++;   // checkMqtt() in loop() reconnects
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  // outputPrint("  packetId: ");
  // outputPrintln(packetId);
  // outputPrint("  qos: ");
  // outputPrintln(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {

}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String payloadText;
  for (size_t i = 0; i < len && i < 64; i++) payloadText += payload[i];
  outputPrintln("Received " + String(topic) + ": " + payloadText);
  // outputPrint("  topic: ");
  // outputPrintln(topic);
  // outputPrint("  qos: ");
  // outputPrintln(properties.qos);
  // outputPrint("  dup: ");
  // outputPrintln(properties.dup);
  // outputPrint("  retain: ");
  // outputPrintln(properties.retain);
  //outputPrint("  len: ");
  //outputPrintln(String(len));
  // outputPrint("  index: ");
  // outputPrintln(index);
  // outputPrint("  total: ");
  // outputPrintln(total);
    // Serial.print(".");
    // Serial.print(payload);
    // Serial.print(".");
  //Logic to handle received messages
  if (String(topic) == jet1buttonTopic) {
    cancelFixALeak();   // manual control takes over
    if (strncmp(payload, "OFF",3)  == 0) {
      jet1State = 0;
      setJet1State();
    } else if (strncmp(payload, "LOW",3) == 0) {
      jet1State = 1;
      setJet1State();
    } else if (strncmp(payload, "HIGH",4) == 0) {
      jet1State = 2;
      setJet1State();
    }
  } else if (String(topic) == jet2buttonTopic) {
    cancelFixALeak();   // manual control takes over
    if (strncmp(payload, "OFF",3) == 0) {
      jet2State = 0;
      setJet2State();
    } else if (strncmp(payload, "LOW",3) == 0) {
      jet2State = 1;
      setJet2State();
    } else if (strncmp(payload, "HIGH",4) ==0) {
      jet2State = 2;
      setJet2State();
    }
  } else if (String(topic) == fixALeakSetTopic) {
    if (strncmp(payload, "ON",2) == 0) {
      startFixALeak();
    } else if (strncmp(payload, "OFF",3) == 0) {
      // only touch the pumps if the program is running; manual jets are left alone
      if (fixALeakActive) stopFixALeak();
      else publishFixALeakState();
    }
  } else if (String(topic) == upButtonTopic) {
    // Increase temperature setpoint
    if (TEMP_SETPOINT < 104) {
      TEMP_SETPOINT += 1;
      //Store setpoint in NVM
      preferences.begin("states", false);
      preferences.putInt("setPoint", TEMP_SETPOINT); 
      preferences.end(); 
      //outputPrint(F("Increased temperature setpoint to:"));
      //outputPrintln(String(TEMP_SETPOINT));
      publishSetpointTemperature();
    }
  } else if (String(topic) == downButtonTopic) {
    // Decrease temperature setpoint
    if (TEMP_SETPOINT > 50) {
      TEMP_SETPOINT -= 1;
      //Store Setpoint in NVM
      preferences.begin("states", false);
      preferences.putInt("setPoint", TEMP_SETPOINT); 
      preferences.end(); 
      //outputPrint(F("Decreased temperature setpoint to: "));
      //outputPrintln(String(TEMP_SETPOINT));
      publishSetpointTemperature();
    }
  } else if (String(topic) == lightButtonTopic) {
    // Toggle lights    
    if (strncmp(payload, "LOW",3) == 0) {
      lightState = 1;
      setSpaLight(lightLow); // Set to medium brightness
    } else if (strncmp(payload, "MEDIUM",6) == 0) {
      lightState = 2;
      setSpaLight(lightMedium); // Set to medium brightness
    } else if (strncmp(payload, "HIGH",4) == 0) {
      lightState = 3;
      setSpaLight(lightHigh); // Set to high brightness
    } else if (strncmp(payload, "OFF",3) == 0) {
      lightState = 0;
      setSpaLight(OFF); // Turn off lights
    }
  }
}

void onMqttPublish(uint16_t packetId) {
  // outputPrint("  packetId: ");
  // outputPrintln(packetId);
}

void stabilizeSensors() {
  bool stabil = false;
  int attempts = 0;
  outputPrintln(F("Waiting for temperature sensors..."));
  while (!stabil) {
  if (++attempts > SENSOR_STABILIZE_ATTEMPTS) {
    // give up so loop() (and OTA) still runs; heater stays off
    enterErrorState(4, F("Temperature sensor not responding at boot"));
    return;
  }
  highLimitSensor.requestTemperatures(); // Send the command to get temperatures
  float highLimitTemp = highLimitSensor.getTempFByIndex(0);
  waterSensor.requestTemperatures(); // Send the command to get temperatures
  currentTemp = waterSensor.getTempFByIndex(0);
  if (highLimitTemp == DEVICE_DISCONNECTED_F || currentTemp == DEVICE_DISCONNECTED_F) {
    outputPrintln(F("Error: Could not read temperature data from sensors!"));
    stabil = false;
  } else {
    outputPrintln(F("Sensors stabilized."));
    stabil = true;
  }
  delay(1000);
  } 
}

void checkCirculationPump() {
  // CHeck if circulation pump and flow sensor are working
  // After a restart water can still be moving for a few seconds; wait for it to stop
  unsigned long settleStart = millis();
  while (flowDetected() && millis() - settleStart < BOOT_FLOW_SETTLE_TIME) {
    delay(100);
  }
  if (!flowDetected()) {
    digitalWrite(CIRCULATION_PUMP_PIN, ON);
    delay(5000); // wait 5 seconds to see if flow is detected
    if (!flowDetected()) {
      //digitalWrite(CIRCULATION_PUMP_PIN, OFF);
      outputPrintln(F("ERROR: No flow detected from circulation pump!"));
      ERROR_STATE = 1; // ERROR_STATE = true
    } else {
      outputPrintln(F("Circulation pump and flow sensor are working."));
      ERROR_STATE = 0; // ERROR_STATE = false
    }
  } else {
    outputPrintln(F("ERROR: Bad Flow Sensor - Flow detected before circulation pump activated!"));
    ERROR_STATE = 2; // ERROR_STATE = true
  }
} 

bool flowDetected() {
  if (digitalRead(FLOW_SENSOR_PIN) == LOW) {
    return true;
  } else {
    return false;
  }
}

// Returns true (and resets the timer) once every interval ms; rollover safe
bool intervalElapsed(unsigned long &lastRun, unsigned long interval) {
  unsigned long now = millis();
  if (now - lastRun < interval) return false;
  lastRun = now;
  return true;
}

// DS18B20 returns -196.6F when disconnected and 185F (85C) on a power-on reset
bool validSensorTemp(float tempF) {
  return tempF > SENSOR_MIN_VALID_F && tempF < SENSOR_MAX_VALID_F;
}

void forceHeaterOff() {
  digitalWrite(HEATER_PIN, OFF);
  if (heaterState != OFF) {
    heaterState = OFF;
    outputPrintln(F("Heater OFF"));
    publishHeaterState();
  }
}

void enterErrorState(int code, const String &info) {
  forceHeaterOff();
  ERROR_STATE = code;
  ERROR_INFO = info;
  outputPrintln("ERROR " + String(code) + ": " + info);
  publishErrorState();
}

void checkTemperature() {
  static unsigned long lastRun = 0;
  if (!intervalElapsed(lastRun, 3000)) return; // Check every 3 seconds
  // never heat on a stale or bad reading
  if (!waterSensorOk || !highLimitSensorOk) {
    forceHeaterOff();
    return;
  }
  if (heaterState != ON && currentTemp < (TEMP_SETPOINT - TEMP_HYSTERESIS)) {
    digitalWrite(HEATER_PIN, ON);
    heaterState = ON;
    outputPrintln(F("Heater ON"));
    publishHeaterState();
  } else if (heaterState != OFF && currentTemp > (TEMP_SETPOINT + TEMP_HYSTERESIS)) {
    digitalWrite(HEATER_PIN, OFF);
    heaterState = OFF;
    outputPrintln(F("Heater OFF"));
    publishHeaterState();
  }
}

void checkHighLimitSensor() {
  boolean triggerHighLimit = false;
  
  static unsigned long lastRun = 0;

  if (ERROR_STATE > 0) return; // If already in error state, skip checking
  if (!intervalElapsed(lastRun, 10000)) return; // Check every 10 seconds

  highLimitSensor.requestTemperatures(); // Send the command to get temperatures
  float reading = highLimitSensor.getTempFByIndex(0);
  if (!validSensorTemp(reading)) {
    highLimitSensorOk = false;
    forceHeaterOff();
    highLimitSensorBadReads++;
    outputPrintln("High limit sensor bad reading: " + String(reading));
    if (highLimitSensorBadReads >= SENSOR_MAX_BAD_READS) {
      enterErrorState(4, "High limit sensor fault: " + String(reading, 1));
    }
    return;  // keep bad readings out of the smoothing array
  }
  highLimitSensorOk = true;
  highLimitSensorBadReads = 0;
  highLimitTemp = reading;

  // Shift previous temperatures
  for (int i = 4; i >= 0; i--) {
    previousHighLimitTemp[i + 1] = previousHighLimitTemp[i];
  }
  previousHighLimitTemp[0] = highLimitTemp; // Add current temp to the front of the array
  // Check if current temp exceeds high limit and  is within 10 degrees of the previous 5 readings to avoid false positives from sensor errors
  for (int i = 1; i <= 5; i++) {
    if (highLimitTemp >= HIGH_LIMIT && abs(highLimitTemp - previousHighLimitTemp[i]) < 10) {
      triggerHighLimit = true;
    } else {
      triggerHighLimit = false;
      break; // If any previous reading is not within range, exit the loop
    }
  }

  if (triggerHighLimit) {
    enterErrorState(3, "High limit temperature exceeded: " + String(highLimitTemp, 1));
  }
}

void checkWaterTemperature() {
  static unsigned long lastRun = 0;
  if (!intervalElapsed(lastRun, 15000)) return; // Check every 15 seconds
  waterSensor.requestTemperatures(); // Send the command to get temperatures
  float reading = waterSensor.getTempFByIndex(0);
  if (!validSensorTemp(reading)) {
    // keep last good currentTemp, but don't heat on it
    waterSensorOk = false;
    forceHeaterOff();
    waterSensorBadReads++;
    outputPrintln("Water sensor bad reading: " + String(reading));
    if (ERROR_STATE == 0 && waterSensorBadReads >= SENSOR_MAX_BAD_READS) {
      enterErrorState(4, "Water sensor fault: " + String(reading, 1));
    }
    return;
  }
  waterSensorOk = true;
  waterSensorBadReads = 0;
  currentTemp = reading;
  publishCurrentTemperature();
}

void checkJetTimers() {
  unsigned long currentTime = millis();
  if (jet1State > 0 && currentTime - jet1StartTime >= jet1RunTime) {
    outputPrintln(F("Jet 1 timer expired, turning off Jet 1."));
    jet1State = 0;
    setJet1State();
  }
  if (jet2State > 0 && currentTime - jet2StartTime >= jet2RunTime) {
    outputPrintln(F("Jet 2 timer expired, turning off Jet 2."));
    jet2State = 0;
    setJet2State();
  }
  if (lightState > 0 && currentTime - lightStartTime >= lightMaxRunTime) {
    outputPrintln(F("Light timer expired, turning off lights."));
    lightState = 0;
    setSpaLight(OFF);
  }
}

void getNTPTime() {
  timeClient.update(); // Update time from NTP every 60 seconds
  
  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();

  // log the clock once an hour (and the first time) rather than every minute
  static int lastLoggedHour = -1;
  if (currentHour != lastLoggedHour) {
    lastLoggedHour = currentHour;
    outputPrintln("Clock " + timeClient.getFormattedTime() + ", filter cycle starts " + String(FILTER_START_HOUR) + ":" +
                  (FILTER_START_MINUTE < 10 ? "0" : "") + String(FILTER_START_MINUTE));
  }
}
void checkFilterCycle() {
  static unsigned long filterStartTime = 0;
  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();

  // Check for alarm condition
  // skip if pump 1 is already running so a 12 hour run isn't cut down to the filter time
  if (!filterCycleRunning && jet1State == OFF && currentHour == FILTER_START_HOUR && currentMinute == FILTER_START_MINUTE) {
    filterCycleRunning = true;
    filterStartTime = millis();
    jet1State = 1;
    setJet1State();
  }

  if (filterCycleRunning && (millis() - filterStartTime >= (unsigned long)FILTER_DURATION * 60000)) {
    filterCycleRunning = false;
    jet1State = 0;
    setJet1State();
  }
}

void publishSetpointTemperature() {
  JsonDocument doc;
  doc["temperature"] = String(TEMP_SETPOINT).c_str();
  char output[256];
  serializeJson(doc, output);
  outputPrintln("Publishing setpoint " + String(output));
  mqttClient.publish(setpointTempTopic, 1, true, output);
}

void publishCurrentTemperature() {
  JsonDocument doc;
  char charArray[10];
  doc["temperature"] = String(currentTemp).c_str();
  char output[256];
  serializeJson(doc, output);
  outputPrintln("Publishing water temp " + String(output) + ", high limit " + String(highLimitTemp, 1));
  
  // convert high limit temp to char array
  snprintf(charArray, sizeof(charArray), "%.1f", highLimitTemp);

  // publish to mqtt clients
  mqttClient.publish(highLimitTopic, 1, true, charArray);
  mqttClient.publish(waterTempTopic, 1, true, output);
}

void publishHeaterState() {
  JsonDocument doc;
  if (heaterState == ON) {
    doc["state"] = "ON";
  } else if (heaterState == OFF || heaterState == JETS_RUNNING) {
    doc["state"] = "OFF";
  }
  char output[256];
  serializeJson(doc, output);
  outputPrintln("Publishing heater " + String(output));
  mqttClient.publish(heaterStateTopic, 1, true, output);
}

void publishErrorState() {
  char output[12];
  snprintf(output, sizeof(output), "%d", ERROR_STATE);
  // only log the periodic status when there is an error or it changed
  static int lastLoggedState = -1;
  if (ERROR_STATE != 0 || ERROR_STATE != lastLoggedState) {
    String line = "Publishing status " + String(output);
    if (ERROR_INFO.length()) line += " (" + ERROR_INFO + ")";
    outputPrintln(line);
    lastLoggedState = ERROR_STATE;
  }
  mqttClient.publish(errorStateTopic, 1, true, output);
}

void setJet1State(unsigned long runTime) {
  jet1StartTime = millis();   //restart jet off timer
  jet1RunTime = runTime;
  if (jet1State == 1) {
    digitalWrite(PUMP1_HIGH_PIN, OFF);
    delay(250);
    digitalWrite(PUMP1_LOW_PIN, ON);
    outputPrintln("Pump 1 set to LOW for " + String(runTime / 60000) + " min");
  } else if (jet1State == 2) {
    digitalWrite(PUMP1_LOW_PIN, OFF);
    delay(250);
    digitalWrite(PUMP1_HIGH_PIN, ON);
    outputPrintln("Pump 1 set to HIGH for " + String(runTime / 60000) + " min");
  } else if (jet1State == 0) {
    digitalWrite(PUMP1_HIGH_PIN, OFF);
    digitalWrite(PUMP1_LOW_PIN, OFF);
    outputPrintln(F("Pump 1 turned OFF"));
    filterCycleRunning = false;      // reset filter cycle variable so filter cycle can trigger again
  }
  
  //publish to mqtt
  JsonDocument doc;
  if (jet1State == 1) doc["state"] = "LOW";
   else if (jet1State == 2) doc["state"] = "HIGH";
   else doc["state"] = "OFF";
  char output[256];
  serializeJson(doc, output);
  outputPrintln("Publishing pump 1 " + String(output));
  mqttClient.publish(jet1StateTopic, 1, true, output);
  publishFixALeakState();
}

void setJet2State(unsigned long runTime) {
  jet2StartTime = millis();   //restart jet off timer
  jet2RunTime = runTime;
  if (jet2State == 1) {
    digitalWrite(PUMP2_HIGH_PIN, OFF);
    delay(500);
    digitalWrite(PUMP2_LOW_PIN, ON);
    outputPrintln("Pump 2 set to LOW for " + String(runTime / 60000) + " min");
  } else if (jet2State == 2) {
    digitalWrite(PUMP2_LOW_PIN, OFF);
    delay(500);
    digitalWrite(PUMP2_HIGH_PIN, ON);
    outputPrintln("Pump 2 set to HIGH for " + String(runTime / 60000) + " min");
  } else if (jet2State == 0) {
    digitalWrite(PUMP2_HIGH_PIN, OFF);
    digitalWrite(PUMP2_LOW_PIN, OFF);
    outputPrintln(F("Pump 2 turned OFF"));
  }
  
  //publish to mqtt
  JsonDocument doc;
  if (jet2State == 1) doc["state"] = "LOW";
   else if (jet2State == 2) doc["state"] = "HIGH";
   else doc["state"] = "OFF";
  char output[256];
  serializeJson(doc, output);
  outputPrintln("Publishing pump 2 " + String(output));
  mqttClient.publish(jet2StateTopic, 1, true, output);
  publishFixALeakState();
}

// Fix-A-Leak: both pumps cycle FIX_A_LEAK_HIGH_TIME on HIGH then FIX_A_LEAK_LOW_TIME
// on LOW, repeating until FIX_A_LEAK_DURATION, then both pumps turn off
void startFixALeak() {
  outputPrintln(F("Fix-A-Leak started."));
  filterCycleRunning = false;   // don't let an active filter cycle stop pump 1 early
  fixALeakActive = true;
  fixALeakStartTime = millis();
  jet1State = 0;                // force checkFixALeak() to apply the first step
  jet2State = 0;
  checkFixALeak();
}

void stopFixALeak() {
  outputPrintln(F("Fix-A-Leak stopped."));
  fixALeakActive = false;
  jet1State = 0;
  jet2State = 0;
  setJet1State();
  setJet2State();
}

// a manual jet command ends the program but leaves the pumps to the manual command
void cancelFixALeak() {
  if (!fixALeakActive) return;
  outputPrintln(F("Fix-A-Leak cancelled by manual jet control."));
  fixALeakActive = false;
  publishFixALeakState();
}

void checkFixALeak() {
  if (!fixALeakActive) return;
  unsigned long elapsed = millis() - fixALeakStartTime;
  if (elapsed >= FIX_A_LEAK_DURATION) {
    outputPrintln(F("Fix-A-Leak complete."));
    stopFixALeak();
    return;
  }
  int wanted = (elapsed % (FIX_A_LEAK_HIGH_TIME + FIX_A_LEAK_LOW_TIME)) < FIX_A_LEAK_HIGH_TIME ? 2 : 1;
  if (jet1State != wanted || jet2State != wanted) {
    // jet timers run to the end of the program as a backstop
    unsigned long remaining = FIX_A_LEAK_DURATION - elapsed;
    jet1State = wanted;
    jet2State = wanted;
    setJet1State(remaining);
    setJet2State(remaining);
  }
}

// Ozone runs only with no errors and no Fix-A-Leak
void updateOzone() {
  static int ozoneState = -1;
  int wanted = (ERROR_STATE == 0 && !fixALeakActive) ? ON : OFF;
  if (wanted != ozoneState) {
    ozoneState = wanted;
    digitalWrite(OZONE_PIN, wanted);
    outputPrintln(wanted == ON ? F("Ozone ON") : F("Ozone OFF"));
  }
}

void publishFixALeakState() {
  mqttClient.publish(fixALeakStateTopic, 1, true, fixALeakActive ? "ON" : "OFF");
}

// Home Assistant MQTT discovery for the Fix-A-Leak switch
void publishDiscovery() {
  // remove the retired 12 hour select from Home Assistant
  mqttClient.publish(oldJets12hDiscoveryTopic, 1, true, "");
  mqttClient.publish(oldJets12hStateTopic, 1, true, "");

  JsonDocument doc;
  doc["name"] = "Fix-A-Leak";
  doc["unique_id"] = "spa_pack_fix_a_leak";
  doc["icon"] = "mdi:water-pump";
  doc["command_topic"] = fixALeakSetTopic;
  doc["state_topic"] = fixALeakStateTopic;
  JsonObject device = doc["device"].to<JsonObject>();
  device["identifiers"][0] = "spa_pack";
  device["name"] = "Spa Pack";
  String output;
  serializeJson(doc, output);
  outputPrintln(F("Publishing Home Assistant discovery..."));
  mqttClient.publish(fixALeakDiscoveryTopic, 1, true, output.c_str());

  publishSensorDiscovery("log", "Log", logTopic, "{{ value[:255] }}", "mdi:text-box-outline", NULL);
  publishSensorDiscovery("wifi_signal", "WiFi Signal", diagnosticsTopic, "{{ value_json.rssi }}", "mdi:wifi", "dBm");
  publishSensorDiscovery("uptime", "Uptime", diagnosticsTopic, "{{ value_json.uptime_min }}", "mdi:timer-outline", "min");
  publishSensorDiscovery("reset_reason", "Last Reset Reason", diagnosticsTopic, "{{ value_json.reset_reason }}", "mdi:restart", NULL);
  publishSensorDiscovery("wifi_drops", "WiFi Drops", diagnosticsTopic, "{{ value_json.wifi_drops }}", "mdi:wifi-off", NULL);
}

// One Home Assistant MQTT sensor on the Spa Pack device
void publishSensorDiscovery(const char* id, const char* name, const char* stateTopic, const char* tmpl, const char* icon, const char* unit) {
  JsonDocument doc;
  doc["name"] = name;
  doc["unique_id"] = String("spa_pack_") + id;
  doc["state_topic"] = stateTopic;
  doc["value_template"] = tmpl;
  doc["icon"] = icon;
  if (unit) doc["unit_of_measurement"] = unit;
  if (strcmp(stateTopic, diagnosticsTopic) == 0) doc["entity_category"] = "diagnostic";
  JsonObject device = doc["device"].to<JsonObject>();
  device["identifiers"][0] = "spa_pack";
  device["name"] = "Spa Pack";
  String output;
  serializeJson(doc, output);
  String topic = String("homeassistant/sensor/spa_pack/") + id + "/config";
  mqttClient.publish(topic.c_str(), 1, true, output.c_str());
}

// Set spa light brightness (0-255)
void setSpaLight(int brightness) {
  ledcWrite(channel, brightness);
  outputPrintln("Spa light brightness " + String(brightness));
   
  //publish to mqtt
  JsonDocument doc;
  if (lightState == 1) {
    doc["state"] = "LOW";
    lightStartTime = millis();   //restart light off timer
  } else if (lightState == 2) {
    doc["state"] = "MEDIUM"; 
    lightStartTime = millis();   //restart light off timer
  } else if (lightState == 3) {
    doc["state"] = "HIGH";
    lightStartTime = millis();   //restart light off timer
  } else if (lightState == OFF) {
    doc["state"] = "OFF";
  }

  char output[256];
  serializeJson(doc, output);
  outputPrintln("Publishing light " + String(output));
  mqttClient.publish(lightStateTopic, 1, true, output);
}